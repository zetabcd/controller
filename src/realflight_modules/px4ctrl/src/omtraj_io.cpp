#include <px4ctrl/omtraj_dynamics.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace
{
bool finite(const OmTrajectoryState & x)
{
  return std::isfinite(x.time) && x.position.allFinite() && x.velocity.allFinite() &&
         x.attitude.coeffs().allFinite() && std::abs(x.attitude.norm() - 1) < 1e-6 &&
         std::isfinite(x.thrust_acceleration) && x.thrust_acceleration > 0 &&
         x.body_rate.allFinite();
}
bool validModel(const OmTrajectoryModel & m)
{
  return std::isfinite(m.gravity) && m.gravity > 0 && m.linear_drag.allFinite() &&
         (m.linear_drag.array() >= 0).all() && std::isfinite(m.horizontal_lift) &&
         m.horizontal_lift >= 0;
}
// CSV certifies model consistency; the caller supplies the physical envelope separately.
OmTrajectoryOptions modelAuditOptions(const OmTrajectoryModel & model)
{
  OmTrajectoryOptions o;
  o.model = model;
  o.tracking.enabled = false;
  o.enforce_hover_boundary_input = false;
  o.thrust_acceleration_min = 0;
  o.thrust_acceleration_max = 1e10;
  o.body_rate_max.setConstant(1e10);
  return o;
}
}
OmTrajectoryValidation validateOmTrajectory(
  const OmTrajectoryResult & r,
  const OmTrajectoryOptions & o, const OmWaypoints & waypoints)
{
  OmTrajectoryValidation a;
  if (r.format_version != 2 || r.states.size() < 2 || r.integration_substeps < 1 ||
    r.integration_substeps > 100 || !validModel(r.model) ||
    std::abs(r.states.front().time) > 1e-9 || !std::isfinite(r.total_time) || r.total_time <= 0 ||
    std::abs(r.total_time - r.states.back().time) > 1e-8)
  {
    a.reason = "Invalid v2 model/grid";
    return a;
  }
  double last = -1;
  for (const auto & x:r.states) {
    if (!finite(x) || x.time <= last) {
      a.reason = "Invalid or unordered state";
      return a;
    }
    last = x.time;
  }
  if (std::abs(r.model.gravity - o.model.gravity) > 1e-9 ||
    (r.model.linear_drag - o.model.linear_drag).norm() > 1e-9 ||
    std::abs(r.model.horizontal_lift - o.model.horizontal_lift) > 1e-9)
  {
    a.reason = "Model mismatch";
    return a;
  }
  const double z0 = r.states.front().position.z();
  if (!waypoints.empty() && r.progress_mu.size()) {
    const auto & mu = r.progress_mu;
    const auto & lambda = r.progress_lambda;
    const int n = r.states.size() - 1, m = waypoints.size();
    if (mu.rows() != n || mu.cols() != m || lambda.rows() != n + 1 || lambda.cols() != m ||
      !mu.allFinite() || !lambda.allFinite())
    {
      a.reason = "Invalid CSTC progress matrices";
      return a;
    }
    double violation = std::max(
      {0.0, -mu.minCoeff(), mu.maxCoeff() - 1,
        -lambda.minCoeff(), lambda.maxCoeff() - 1});
    for (int j = 0; j < m; ++j) {
      violation = std::max({violation, std::abs(lambda(0, j) - 1), std::abs(lambda(n, j))});
      for (int k = 0; k < n; ++k) {
        violation = std::max(violation, std::abs(lambda(k + 1, j) - lambda(k, j) + mu(k, j)));
        const double g = (r.states[k].position - waypoints[j].position).squaredNorm() -
          waypoints[j].tolerance * waypoints[j].tolerance;
        a.waypoint_violation = std::max(a.waypoint_violation, mu(k, j) * g);
        if (j + 1 < m) {
          violation = std::max(violation, lambda(k, j) - lambda(k, j + 1));
        }
      }
    }
    if (violation > o.constraint_tolerance || a.waypoint_violation > o.cstc_tolerance) {
      a.reason = "CSTC progress/order audit failed";
      return a;
    }
  }
  for (size_t k = 0; k + 1 < r.states.size(); ++k) {
    const auto & x = r.states[k];
    const auto & y = r.states[k + 1];
    double dt = y.time - x.time;
    const auto coarse = omtraj::integrate(x, omtraj::input(y), dt, r.model, r.integration_substeps);
    const auto fine =
      omtraj::integrate(x, omtraj::input(y), dt, r.model, 2 * r.integration_substeps);
    const auto d = omtraj::difference(fine, y), e = omtraj::difference(fine, coarse);
    a.position_defect = std::max(a.position_defect, d.head<3>().lpNorm<Eigen::Infinity>());
    a.velocity_defect = std::max(a.velocity_defect, d.segment<3>(3).lpNorm<Eigen::Infinity>());
    a.attitude_defect = std::max(a.attitude_defect, d.tail<3>().lpNorm<Eigen::Infinity>());
    a.integration_error = std::max(a.integration_error, e.lpNorm<Eigen::Infinity>());
    const Eigen::Vector4d u0 = omtraj::input(x), u1 = omtraj::input(y), slope = (u1 - u0) / dt;
    int count = std::max(10, int(std::ceil(dt / 0.005)));
    if (count > 100000) {
      a.reason = "Audit interval too large";
      return a;
    }
    double worst = 0, worst_fraction = 0;
    for (int j = 0; j <= count; ++j) {
      const double s = double(j) / count;
      const auto sample = omtraj::integrate(
        x, (1 - s) * u0 + s * u1, s * dt, r.model,
        2 * r.integration_substeps);
      const auto g = omtraj::constraints(sample, slope, o, z0);
      if (!g.allFinite()) {
        a.reason = "Nonfinite continuous dynamics/constraints";
        return a;
      }
      a.constraint_violation = std::max(a.constraint_violation, g.maxCoeff());
      if (g.maxCoeff() > worst) {
        worst = g.maxCoeff();
        worst_fraction = s;
      }
    }
    if (worst > o.constraint_tolerance) {
      a.refinement_points.emplace_back(k, worst_fraction);
    }
  }
  if (o.enforce_hover_boundary_input) {
    for (const auto * x:{&r.states.front(), &r.states.back()}) {
      a.constraint_violation = std::max(
        {a.constraint_violation, x->velocity.norm(), x->body_rate.norm(),
          std::abs(x->thrust_acceleration - r.model.gravity),
          (x->attitude * Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitZ()).norm()});
    }
  }
  // Independently require actual ordered sphere/cone hits, not only small mu*g.
  size_t next = 0;
  for (const auto & w:waypoints) {
    bool hit = false;
    for (size_t k = next; k < r.states.size(); ++k) {
      const auto & x = r.states[k];
      if ((x.position - w.position).norm() <= w.tolerance + o.constraint_tolerance &&
        (x.attitude * Eigen::Vector3d::UnitZ()).z() >=
        std::cos(w.maximum_tilt) - o.constraint_tolerance)
      {
        a.waypoint_times.push_back(x.time);
        next = k + 1;
        hit = true;
        break;
      }
    }
    if (!hit) {
      a.reason = "No ordered waypoint sphere/cone hit";
      a.waypoint_violation = 1;
      return a;
    }
  }
  if (a.position_defect > o.position_tolerance || a.velocity_defect > o.velocity_tolerance ||
    a.attitude_defect > o.attitude_tolerance ||
    a.integration_error >
    0.5 * std::min({o.position_tolerance, o.velocity_tolerance, o.attitude_tolerance}))
  {
    a.reason = "Continuous dynamics audit failed (refine grid/integration or SCP)";
    return a;
  }
  if (a.constraint_violation > o.constraint_tolerance) {
    a.reason = "Dense physical constraint audit failed";
    return a;
  }
  a.valid = true;
  a.reason = "Validated at nodes and dense subinterval samples";
  return a;
}
OmTrajectoryState OmTrajectoryOptimizer::sample(const OmTrajectoryResult & r, double t)
{
  if (!std::isfinite(t) || r.states.size() < 2) {
    throw std::invalid_argument("Invalid omtraj sample");
  }
  if (t <= 0) {
    return r.states.front();
  }
  if (t >= r.total_time) {
    auto end = r.states.back();
    // Only stationary, level endpoints admit this hold. Flight adapters reject others.
    if (end.velocity.norm() < 1e-4 &&
      (end.attitude * Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitZ()).norm() < 1e-4)
    {
      end.velocity.setZero();
      end.thrust_acceleration = r.model.gravity;
      end.body_rate.setZero();
    }
    return end;
  }
  auto upper = std::upper_bound(
    r.states.begin(), r.states.end(), t,
    [](double time, const OmTrajectoryState & x) {return time < x.time;});
  const auto & x = *(upper - 1);
  double s = (t - x.time) / (upper->time - x.time);
  const Eigen::Vector4d u = (1 - s) * omtraj::input(x) + s * omtraj::input(*upper);
  if (r.format_version == 1) {
    // Legacy files remain inspectable, but are never implicitly certified for flight.
    auto y = x;
    y.position = (1 - s) * x.position + s * upper->position;
    y.velocity = (1 - s) * x.velocity + s * upper->velocity;
    y.attitude = x.attitude.slerp(s, upper->attitude);
    omtraj::setInput(y, u);
    y.time = t;
    return y;
  }
  return omtraj::integrate(x, u, t - x.time, r.model, r.integration_substeps);
}
bool saveOmTrajectoryCsv(
  const OmTrajectoryResult & r, const std::string & file,
  std::string * error)
{
  const auto fail = [&](const std::string & s) {
      if (error) {
        *error = s;
      }
      return false;
    };
  const auto o = modelAuditOptions(r.model);
  const auto audit = validateOmTrajectory(r, o);
  if (!r.success || !audit.valid || file.empty()) {
    return fail("Refusing unvalidated trajectory: " + audit.reason);
  }
  std::error_code ec;
  auto parent = std::filesystem::path(file).parent_path();
  if (!parent.empty()) {
    std::filesystem::create_directories(parent, ec);
    if (ec) {
      return fail(ec.message());
    }
  }
  const std::string tmp = file + ".tmp";
  std::ofstream out(tmp);
  if (!out) {
    return fail("Cannot open " + tmp);
  }
  out << std::setprecision(17) << "# px4ctrl_omtraj_v2\n# model " << r.model.gravity << ' '
      << r.model.linear_drag.transpose() << ' ' << r.model.horizontal_lift << ' ' <<
    r.integration_substeps
      << "\n# time,px,py,pz,vx,vy,vz,qw,qx,qy,qz,thrust_acceleration,wx,wy,wz\n";
  for (const auto & x:r.states) {
    out << x.time << ',' << x.position.x() << ',' << x.position.y() << ',' << x.position.z() << ','
        << x.velocity.x() << ',' << x.velocity.y() << ',' << x.velocity.z() << ','
        << x.attitude.w() << ',' << x.attitude.x() << ',' << x.attitude.y() << ',' <<
      x.attitude.z() << ','
        << x.thrust_acceleration << ',' << x.body_rate.x() << ',' << x.body_rate.y() << ',' <<
      x.body_rate.z() << '\n';
  }
  out.close();
  if (!out || std::rename(tmp.c_str(), file.c_str()) != 0) {
    std::remove(tmp.c_str());
    return fail("Cannot atomically save trajectory");
  }
  return true;
}
OmTrajectoryResult loadOmTrajectoryCsv(const std::string & file)
{
  OmTrajectoryResult r;
  r.format_version = 1;
  r.status = "Cannot load " + file;
  std::ifstream in(file);
  if (!in) {
    return r;
  }
  bool model = false;
  std::string line;
  double previous = -1;
  while (std::getline(in, line)) {
    if (line == "# px4ctrl_omtraj_v2") {
      r.format_version = 2;
      continue;
    }
    if (line.rfind("# model ", 0) == 0) {
      std::istringstream s(line.substr(8));
      if (!(s >> r.model.gravity >> r.model.linear_drag.x() >> r.model.linear_drag.y() >>
        r.model.linear_drag.z() >> r.model.horizontal_lift >> r.integration_substeps))
      {
        r.status = "Invalid model header";
        return r;
      }
      model = true;
      continue;
    }
    auto first = line.find_first_not_of(" \t\r");
    if (first == std::string::npos || line[first] == '#') {
      continue;
    }
    std::replace(line.begin(), line.end(), ',', ' ');
    std::istringstream s(line);
    OmTrajectoryState x;
    if (!(s >> x.time >> x.position.x() >> x.position.y() >> x.position.z() >>
      x.velocity.x() >> x.velocity.y() >> x.velocity.z() >> x.attitude.w() >> x.attitude.x() >>
      x.attitude.y() >> x.attitude.z() >> x.thrust_acceleration >>
      x.body_rate.x() >> x.body_rate.y() >> x.body_rate.z()))
    {
      r.status = "Invalid CSV row";
      return r;
    }
    std::string extra;
    if (s >> extra) {
      r.status = "Unexpected CSV column";
      return r;
    }
    if (!finite(x) || x.time <= previous) {
      r.status = "Invalid CSV state/time";
      return r;
    }
    if (!r.states.empty() && r.states.back().attitude.dot(x.attitude) < 0) {
      x.attitude.coeffs() *= -1;
    }
    r.states.push_back(x);
    previous = x.time;
  }
  if (!in.eof() || r.states.size() < 2 || std::abs(r.states.front().time) > 1e-9) {
    r.status = "Invalid CSV grid";
    return r;
  }
  r.total_time = r.states.back().time;
  if (r.format_version == 2) {
    if (!model) {
      r.status = "Missing v2 model";
      return r;
    }
    const auto o = modelAuditOptions(r.model);
    auto audit = validateOmTrajectory(r, o);
    if (!audit.valid) {
      r.status = audit.reason;
      return r;
    }
    r.dynamics_validated = true;
  }
  r.success = true;
  r.status = r.format_version ==
    2 ? "v2 loaded; dynamics revalidated" : "legacy v1 loaded for inspection only";
  // CSV has no optimization history, so never invent a convergence certificate.
  return r;
}
