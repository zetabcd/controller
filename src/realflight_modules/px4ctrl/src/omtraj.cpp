#include <px4ctrl/omtraj_dynamics.h>
#include "omtraj_qp.h"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <numeric>
#include <stdexcept>

namespace
{
using namespace omtraj;
using Clock = std::chrono::steady_clock;
using namespace omtraj::detail;
double seconds(Clock::time_point start)
{
  return std::chrono::duration<double>(Clock::now() - start).count();
}

// Contiguous blocks: state increments, input increments, optional L1 epigraph,
// waypoint progress, progress decrement, and the common time increment.
struct Layout
{
  int n, m, up, vp, lp, mp, tp, size;
  Layout(int N, int M, bool elastic)
  : n(N), m(M), up(9 * (n + 1)), vp(up + 4 * (n + 1)),
    lp(vp + (elastic ? 9 * n : 0)), mp(lp + (n + 1) * m), tp(mp + n * m), size(tp + 1) {}
  int x(int k, int j = 0) const {return 9 * k + j;}
  int u(int k, int j = 0) const {return up + 4 * k + j;}
  int lam(int k, int j) const {return lp + k * m + j;}
  int mu(int k, int j) const {return mp + k * m + j;}
};
struct Iterate
{
  OmStates xs;
  Eigen::MatrixXd lambda, mu;
  double T;
};

// A quintic stop-through-waypoint seed is merely an initial guess; passage nodes
// and all intermediate velocities remain free in the subsequent CSTC problem.
Iterate initialize(
  const OmTrajectoryBoundary & a, const OmTrajectoryBoundary & b,
  const OmWaypoints & wp, const OmTrajectoryOptions & o)
{
  int n = o.intervals, m = wp.size();
  std::vector<Eigen::Vector3d> path{a.position};
  for (const auto & w:wp) {
    path.push_back(w.position);
  }
  path.push_back(b.position);
  std::vector<double> lengths(path.size(), 0);
  for (size_t j = 1; j < path.size(); ++j) {
    lengths[j] = lengths[j - 1] + std::max(0.01, (path[j] - path[j - 1]).norm());
  }
  Iterate z;
  z.T = std::clamp(1.8 * lengths.back() / o.initial_speed, o.minimum_time, o.maximum_time);
  std::vector<int> nodes{0};
  for (int j = 0; j < m; ++j) {
    nodes.push_back(
      std::clamp(
        int(std::lround(n * lengths[j + 1] / lengths.back())),
        nodes.back() + 1, n - (m - j)));
  }
  nodes.push_back(n);
  z.xs.resize(n + 1);
  const double dt = z.T / n;
  const double yaw0 =
    std::atan2(a.attitude.toRotationMatrix()(1, 0), a.attitude.toRotationMatrix()(0, 0));
  const double yaw1 =
    std::atan2(b.attitude.toRotationMatrix()(1, 0), b.attitude.toRotationMatrix()(0, 0));
  for (int k = 0; k <= n; ++k) {
    int seg = std::min(int(std::upper_bound(nodes.begin(), nodes.end(), k) - nodes.begin()) - 1, m);
    double h = (nodes[seg + 1] - nodes[seg]) * dt,
      s = double(k - nodes[seg]) / (nodes[seg + 1] - nodes[seg]);
    const double f = s * s * s * (10 + s * (-15 + 6 * s)), fd = 30 * s * s * (1 - s) * (1 - s) / h,
      fdd = 60 * s * (1 - 3 * s + 2 * s * s) / (h * h);
    auto & x = z.xs[k];
    x.time = k * dt;
    x.position = path[seg] + f * (path[seg + 1] - path[seg]);
    x.velocity = fd * (path[seg + 1] - path[seg]);
    Eigen::Vector3d force = fdd * (path[seg + 1] - path[seg]) + o.model.gravity *
      Eigen::Vector3d::UnitZ();
    Eigen::Vector3d zb = force.normalized();
    double yaw = yaw0 + std::remainder(yaw1 - yaw0, 2 * M_PI) * k / n;
    Eigen::Vector3d heading(-std::sin(yaw), std::cos(yaw), 0), xb = heading.cross(zb).normalized();
    Eigen::Matrix3d R;
    R << xb, zb.cross(xb), zb;
    x.attitude = Eigen::Quaterniond(R).normalized();
    x.thrust_acceleration = force.norm();
  }
  for (int k = 0; k < n; ++k) {
    z.xs[k].body_rate = log(z.xs[k].attitude.conjugate() * z.xs[k + 1].attitude) / dt;
  }
  z.xs.front().position = a.position;
  z.xs.front().velocity = a.velocity;
  z.xs.front().attitude = a.attitude.normalized();
  z.xs.back().position = b.position;
  z.xs.back().velocity = b.velocity;
  z.xs.back().attitude = b.attitude.normalized();
  if (o.enforce_hover_boundary_input) {
    for (int k:{0, n}) {
      z.xs[k].thrust_acceleration = o.model.gravity;
      z.xs[k].body_rate.setZero();
    }
  }
  z.mu = Eigen::MatrixXd::Zero(n, m);
  z.lambda = Eigen::MatrixXd::Zero(n + 1, m);
  for (int j = 0; j < m; ++j) {
    z.mu(nodes[j + 1], j) = 1;
    z.lambda(0, j) = 1;
    for (int k = 0; k < n; ++k) {
      z.lambda(k + 1, j) = z.lambda(k, j) - z.mu(k, j);
    }
  }
  return z;
}
Eigen::VectorXd intervalConstraints(
  const OmTrajectoryState & x, const Eigen::Vector4d & u1,
  double dt, const OmTrajectoryOptions & o, double z0, const std::vector<double> & samples)
{
  const auto u0 = input(x);
  const Eigen::Vector4d slope = (u1 - u0) / dt;
  std::vector<double> values;
  for (double fraction:samples) {
    const auto r = fraction == 0 ? x : integrate(
      x, (1 - fraction) * u0 + fraction * u1,
      fraction * dt, o.model, o.integration_substeps);
    const auto kind = fraction == 0 ? ConstraintSample::All :
      (fraction == 1 ? ConstraintSample::Endpoint : ConstraintSample::Interior);
    const auto g = constraints(r, slope, o, z0, kind);
    values.insert(values.end(), g.data(), g.data() + g.size());
  }
  return Eigen::Map<Eigen::VectorXd>(values.data(), values.size());
}
struct Residual
{
  double dynamics{0}, waypoint{0}, order{0}, physical{0}, merit{0};
};
Residual residual(
  const Iterate & z, const OmWaypoints & wp, const OmTrajectoryOptions & o,
  double sigma, const std::vector<std::vector<double>> & samples)
{
  Residual r;
  r.merit = o.optimize_total_time ? z.T : 0;
  int n = o.intervals, m = wp.size();
  double dt = z.T / n;
  for (int k = 0; k < n; ++k) {
    const V9 d =
      difference(
      integrate(
        z.xs[k], input(
          z.xs[k + 1]), dt, o.model, o.integration_substeps), z.xs[k + 1]);
    r.dynamics = std::max(r.dynamics, d.lpNorm<Eigen::Infinity>());
    r.merit += o.virtual_control_weight * d.lpNorm<1>();
    const auto g = intervalConstraints(
      z.xs[k], input(z.xs[k + 1]), dt, o,
      z.xs.front().position.z(), samples[k]);
    r.physical = std::max(r.physical, g.maxCoeff());
    r.merit += o.virtual_control_weight * g.cwiseMax(0).sum();
    for (int j = 0; j < m; ++j) {
      const double g0 = (z.xs[k].position - wp[j].position).squaredNorm() - wp[j].tolerance *
        wp[j].tolerance;
      const double prod = z.mu(k, j) * g0;
      const double cone =
        z.mu(
        k,
        j) * (std::cos(wp[j].maximum_tilt) - (z.xs[k].attitude * Eigen::Vector3d::UnitZ()).z());
      r.waypoint = std::max({r.waypoint, prod, cone});
      r.merit += o.cstc_merit_weight * (std::max(0.0, prod - sigma) + std::max(0.0, cone - sigma));
      if (j + 1 < m) {
        const double order = z.lambda(k, j) - z.lambda(k, j + 1);
        r.order = std::max(r.order, order);
        r.merit += o.cstc_merit_weight * std::max(0.0, order);
      }
    }
  }
  return r;
}
OmTrajectoryResult resultOf(const Iterate & z, const OmTrajectoryOptions & o)
{
  OmTrajectoryResult r;
  r.states = z.xs;
  r.total_time = z.T;
  r.model = o.model;
  r.integration_substeps = o.integration_substeps;
  r.progress_lambda = z.lambda;
  r.progress_mu = z.mu;
  for (size_t k = 0; k < r.states.size(); ++k) {
    r.states[k].time = z.T * k / (r.states.size() - 1);
  }
  return r;
}
}

OmTrajectoryOptimizer::OmTrajectoryOptimizer(const OmTrajectoryOptions & o)
{
  setOptions(o);
}
void OmTrajectoryOptimizer::setOptions(const OmTrajectoryOptions & o)
{
  const auto positive = [](double v) {return std::isfinite(v) && v > 0;};
  const auto vectorPositive = [&](const Eigen::Vector3d & v) {
      return v.allFinite() && (v.array() > 0).all();
    };
  if (o.intervals < 4 || o.intervals > 1000 || o.max_scp_iterations < 1 ||
    o.integration_substeps < 1 ||
    o.integration_substeps > 100 || !positive(o.minimum_time) || !positive(o.maximum_time) ||
    o.maximum_time < o.minimum_time || !positive(o.initial_speed) || !positive(o.model.gravity) ||
    !o.model.linear_drag.allFinite() || (o.model.linear_drag.array() < 0).any() ||
    !std::isfinite(o.model.horizontal_lift) || o.model.horizontal_lift < 0 ||
    !positive(o.thrust_acceleration_min) ||
    o.thrust_acceleration_max <= o.thrust_acceleration_min ||
    !positive(o.thrust_acceleration_max) || !vectorPositive(o.body_rate_max))
  {
    throw std::invalid_argument("Invalid omtraj grid, model or input bounds");
  }
  for (double v:{o.position_trust_region, o.velocity_trust_region, o.attitude_trust_region,
      o.thrust_trust_region, o.time_trust_region, o.progress_trust_region, o.state_regularization,
      o.progress_regularization, o.virtual_control_weight, o.maximum_virtual_control_weight,
      o.cstc_merit_weight, o.position_tolerance,
      o.velocity_tolerance, o.attitude_tolerance, o.constraint_tolerance, o.cstc_tolerance,
      o.convergence_tolerance})
  {
    if (!positive(v)) {
      throw std::invalid_argument("Invalid omtraj numerical parameter");
    }
  }
  if (!vectorPositive(o.body_rate_trust_region) || o.scp_backtracking_steps < 1 ||
    o.maximum_virtual_control_weight < o.virtual_control_weight ||
    !std::isfinite(o.cstc_relaxation_initial) || o.cstc_relaxation_initial < 0 ||
    !std::isfinite(o.cstc_relaxation_decay) || o.cstc_relaxation_decay <= 0 ||
    o.cstc_relaxation_decay >= 1)
  {
    throw std::invalid_argument("Invalid omtraj continuation/trust parameters");
  }
  const auto & l = o.tracking;
  if (l.enabled &&
    (!positive(l.thrust_min) || !positive(l.thrust_max) || l.thrust_max <= l.thrust_min ||
    !vectorPositive(l.rate_max) || !positive(l.thrust_slew_max) ||
    !vectorPositive(l.angular_acceleration_max) ||
    !std::isfinite(l.thrust_time_constant) || l.thrust_time_constant < 0 ||
    !positive(l.speed_max) || !positive(l.maximum_tilt) || l.maximum_tilt > M_PI ||
    !std::isfinite(l.minimum_relative_altitude) || !positive(l.mass) ||
    !vectorPositive(l.inertia) ||
    !positive(l.arm) || !std::isfinite(l.arm_angle) || !positive(l.torque_to_thrust) ||
    !std::isfinite(l.motor_min) || l.motor_min < 0 || !positive(l.motor_max) ||
    l.motor_max <= l.motor_min))
  {
    throw std::invalid_argument("Invalid omtraj tracking envelope");
  }
  if (l.enabled && l.motor_constraints) {
    (void)allocationInverse(l);
  }
  if (l.enabled && std::max(l.thrust_min, o.thrust_acceleration_min) >=
    std::min(l.thrust_max, o.thrust_acceleration_max))
  {
    throw std::invalid_argument("Empty tracking/paper thrust intersection");
  }
  options_ = o;
}

OmTrajectoryResult OmTrajectoryOptimizer::optimize(
  const OmTrajectoryBoundary & initial,
  const OmTrajectoryBoundary & terminal, const OmWaypoints & waypoints)
{
  const auto start = Clock::now();
  auto o = options_;
  const int n = o.intervals, m = waypoints.size();
  OmTrajectoryResult best;
  best.model = o.model;
  best.integration_substeps = o.integration_substeps;
  const auto validBoundary = [](const OmTrajectoryBoundary & b) {
      return b.position.allFinite() && b.velocity.allFinite() && b.attitude.coeffs().allFinite() &&
             b.attitude.norm() > 1e-8;
    };
  if (!validBoundary(initial) || !validBoundary(terminal) || m >= n) {
    best.status = "Invalid boundary/waypoint count";
    return best;
  }
  for (const auto & w:waypoints) {
    if (!w.position.allFinite() || !std::isfinite(w.tolerance) || w.tolerance <= 0 ||
      !std::isfinite(w.maximum_tilt) || w.maximum_tilt <= 0 || w.maximum_tilt > M_PI)
    {
      best.status = "Invalid waypoint sphere/cone";
      return best;
    }
  }
  if (o.enforce_hover_boundary_input) {
    for (const auto * b:{&initial, &terminal}) {
      if (b->velocity.norm() > 1e-8 ||
        (b->attitude.normalized() * Eigen::Vector3d::UnitZ() - Eigen::Vector3d::UnitZ()).norm() >
        1e-8)
      {
        best.status = "Hover boundaries require zero velocity and level attitude";
        return best;
      }
    }
  }
  Iterate z = initialize(initial, terminal, waypoints, o);
  std::vector<std::vector<double>> samples(n, std::vector<double>{0, 0.5, 1});
  double trust = 1;
  int accepted = 0;
  bool force_restore = false;
  bool virtual_controls = false;
  std::vector<OmTrajectoryIteration> history;
  for (int iter = 0; iter < o.max_scp_iterations; ++iter) {
    const auto step_start = Clock::now();
    double dt = z.T / n;
    double sigma = o.cstc_relaxation_initial * std::pow(o.cstc_relaxation_decay, accepted);
    if (sigma < o.cstc_tolerance * 0.1) {
      sigma = 0;
    }
    const auto before = residual(z, waypoints, o, sigma, samples);
    virtual_controls = force_restore || (virtual_controls && before.dynamics >= 1e-3);
    force_restore = false;
    const bool polish = before.dynamics < 1e-3 && before.waypoint < 1e-3 &&
      (before.dynamics >
      0.5 * std::min({o.position_tolerance, o.velocity_tolerance, o.attitude_tolerance}) ||
      before.waypoint > 0.5 * o.cstc_tolerance || before.physical > 0.5 * o.constraint_tolerance);
    const Layout l(n, m, virtual_controls);
    Qp qp(l.size);
    qp.q(l.tp) = o.optimize_total_time ? 1 : 0;
    for (int i = 0; i < l.up; ++i) {
      qp.h(i) = 2 * o.state_regularization;
    }
    // Increment-only proximal terms vanish at a fixed point: no smoothness or
    // effort cost can compete with the physical objective T.
    for (int i = l.up; i < l.vp; ++i) {
      qp.h(i) = 0.005;
    }
    qp.h(l.tp) = 0.2;
    for (int i = l.mp; i < l.tp; ++i) {
      qp.h(i) = 2 * o.progress_regularization;
    }
    for (int i = l.vp; i < l.lp; ++i) {
      qp.q(i) = o.virtual_control_weight;
    }
    // Manuscript (21)-(25): keep the nominal defect, use delta T (not absolute T).
    // Epigraph t >= |dx_next - J*delta - defect| gives the exact L1 virtual-control cost.
    for (int k = 0; k < n; ++k) {
      const auto x = z.xs[k];
      const auto u0 = input(x), u1 = input(z.xs[k + 1]);
      const V9 defect = difference(
        integrate(
          x, u1, dt, o.model,
          o.integration_substeps), z.xs[k + 1]);
      Eigen::Matrix<double, 9, 18> J;
      const auto g = intervalConstraints(x, u1, dt, o, initial.position.z(), samples[k]);
      Eigen::MatrixXd G(g.size(), 18);
      for (int c = 0; c < 18; ++c) {
        auto a = x;
        Eigen::Vector4d b = u1;
        double h = dt;
        constexpr double eps = 1e-5;
        if (c < 9) {
          V9 d = V9::Zero();
          d(c) = eps;
          a = retract(a, d);
        } else if (c < 13) {
          auto u = u0;
          u(c - 9) += eps;
          setInput(a, u);
        } else if (c < 17) {
          b(c - 13) += eps;
        } else {
          h += eps / n;
        }
        J.col(c) =
          (difference(
            integrate(a, b, h, o.model, o.integration_substeps),
            z.xs[k + 1]) - defect) / eps;
        G.col(c) = (intervalConstraints(a, b, h, o, initial.position.z(), samples[k]) - g) / eps;
      }
      const auto index = [&](int c) {
          return c <
                 9 ? l.x(k, c) : (c < 13 ? l.u(k, c - 9) : (c < 17 ? l.u(k + 1, c - 13) : l.tp));
        };
      // With a nonzero nominal attitude defect, changing the next rotation
      // also changes the log-map chart. Its Jacobian is not the identity.
      Eigen::Matrix3d next_rotation;
      const auto predicted = integrate(x, u1, dt, o.model, o.integration_substeps);
      for (int c = 0; c < 3; ++c) {
        constexpr double eps = 1e-5;
        V9 d = V9::Zero();
        d(6 + c) = eps;
        next_rotation.col(c) = -(difference(predicted, retract(z.xs[k + 1], d)) -
          defect).tail<3>() / eps;
      }
      for (int j = 0; j < 9; ++j) {
        Entries a;
        if (j < 6) {
          a.emplace_back(l.x(k + 1, j), 1);
        } else {
          for (int c = 0; c < 3; ++c) {
            a.emplace_back(l.x(k + 1, 6 + c), next_rotation(j - 6, c));
          }
        }
        for (int c = 0; c < 18; ++c) {
          a.emplace_back(index(c), -J(j, c));
        }
        auto b = a;
        for (auto & v:b) {
          v.second = -v.second;
        }
        if (virtual_controls) {
          a.emplace_back(l.vp + 9 * k + j, -1);
          b.emplace_back(l.vp + 9 * k + j, -1);
          qp.row(a, -inf, defect(j));
          qp.row(b, -inf, -defect(j));
        } else {
          qp.row(a, defect(j), defect(j));
        }
      }
      // Hard physical rows at interval endpoints and midpoint, rechecked densely.
      for (int j = 0; j < g.size(); ++j) {
        Entries a;
        for (int c = 0; c < 18; ++c) {
          a.emplace_back(index(c), G(j, c));
        }
        qp.row(a, -inf, -g(j));
      }
    }
    for (int k:{0, n}) {
      const auto & b = k == 0 ? initial : terminal;
      OmTrajectoryState goal;
      goal.position = b.position;
      goal.velocity = b.velocity;
      goal.attitude = b.attitude.normalized();
      const V9 d = difference(goal, z.xs[k]);
      for (int j = 0; j < 9; ++j) {
        qp.bound(l.x(k, j), d(j), d(j));
      }
      if (o.enforce_hover_boundary_input) {
        Eigen::Vector4d hover;
        hover << o.model.gravity, 0, 0, 0;
        const Eigen::Vector4d du = hover - input(z.xs[k]);
        for (int j = 0; j < 4; ++j) {
          qp.bound(l.u(k, j), du(j), du(j));
        }
      }
    }
    // Manuscript (26)-(32). Linear cumulative progress ordering is sufficient:
    // positive progress of waypoint j+1 requires prior progress of waypoint j.
    // Together with exact CSTC this implies ordered first visits.
    for (int j = 0; j < m; ++j) {
      qp.bound(l.lam(0, j), 1 - z.lambda(0, j), 1 - z.lambda(0, j));
      qp.bound(l.lam(n, j), -z.lambda(n, j), -z.lambda(n, j));
      for (int k = 0; k < n; ++k) {
        double d = z.lambda(k, j) - z.mu(k, j) - z.lambda(k + 1, j);
        qp.row({{l.lam(k + 1, j), 1}, {l.lam(k, j), -1}, {l.mu(k, j), 1}}, d, d);
        const auto e = (z.xs[k].position - waypoints[j].position).eval();
        double g = e.squaredNorm() - waypoints[j].tolerance * waypoints[j].tolerance;
        Entries a{{l.mu(k, j), g}};
        for (int c = 0; c < 3; ++c) {
          a.emplace_back(l.x(k, c), 2 * z.mu(k, j) * e(c));
        }
        qp.row(a, -inf, sigma - z.mu(k, j) * g);
        if (waypoints[j].maximum_tilt < M_PI - 1e-8) {
          const Eigen::Matrix3d R = z.xs[k].attitude.toRotationMatrix();
          double cone = std::cos(waypoints[j].maximum_tilt) - R(2, 2);
          const Eigen::RowVector3d dc = Eigen::Vector3d::UnitZ().transpose() * R * hat(
            Eigen::Vector3d::UnitZ());
          Entries ca{{l.mu(k, j), cone}};
          for (int c = 0; c < 3; ++c) {
            ca.emplace_back(l.x(k, 6 + c), z.mu(k, j) * dc(c));
          }
          qp.row(ca, -inf, sigma - z.mu(k, j) * cone);
        }

      }
      if (j + 1 < m) {
        for (int k = 0; k <= n; ++k) {
          qp.row(
            {{l.lam(k, j), 1}, {l.lam(k, j + 1), -1}}, -inf, z.lambda(k, j + 1) - z.lambda(
              k,
              j));
        }
      }
    }
    for (int k = 0; k <= n; ++k) {
      for (int j = 0; j < 9; ++j) {
        double r = trust *
          (j <
          3 ? o.position_trust_region : (j <
          6 ? o.velocity_trust_region : o.attitude_trust_region));
        qp.bound(l.x(k, j), -r, r);
      }
      for (int j = 0; j < 4; ++j) {
        double r = trust * (j == 0 ? o.thrust_trust_region : o.body_rate_trust_region(j - 1));
        qp.bound(l.u(k, j), -r, r);
      }
      for (int j = 0; j < m; ++j) {
        double r = trust * o.progress_trust_region;
        qp.bound(l.lam(k, j), std::max(-r, -z.lambda(k, j)), std::min(r, 1 - z.lambda(k, j)));
        if (k < n) {
          qp.bound(l.mu(k, j), std::max(-r, -z.mu(k, j)), std::min(r, 1 - z.mu(k, j)));
        }
      }
    }
    for (int i = l.vp; i < l.lp; ++i) {
      qp.bound(i, 0, virtual_controls ? inf : 0);
    }
    double tr = trust * o.time_trust_region * z.T;
    qp.bound(
      l.tp, o.optimize_total_time ? std::max(polish ? 0.0 : -tr, o.minimum_time - z.T) : 0,
      o.optimize_total_time ? std::min(tr, o.maximum_time - z.T) : 0);
    Eigen::VectorXd solution;
    std::string status;
    const auto qstart = Clock::now();
    const double accuracy = std::clamp(
      0.02 * std::max(
        {before.dynamics,
          before.waypoint, before.order, before.physical}), 2e-7, 1e-3);
    bool solved = solveQp(qp, solution, status, accuracy);
    double qtime = seconds(qstart);
    if (!solved) {
      if (!virtual_controls) {
        force_restore = true;
        virtual_controls = true;
        continue;
      }
      best.status = "QP: " + status;
      break;
    }
    bool take = false;
    Iterate next = z;
    double alpha = 1;
    Residual after;
    for (int back = 0; back < o.scp_backtracking_steps; ++back, alpha *= 0.5) {
      next = z;
      next.T += alpha * solution(l.tp);
      for (int k = 0; k <= n; ++k) {
        next.xs[k] = retract(z.xs[k], alpha * solution.segment<9>(l.x(k)));
        setInput(next.xs[k], input(z.xs[k]) + alpha * solution.segment<4>(l.u(k)));
        for (int j = 0; j < m; ++j) {
          next.lambda(k, j) += alpha * solution(l.lam(k, j));
          if (k < n) {
            next.mu(k, j) += alpha * solution(l.mu(k, j));
          }
        }
      }
      after = residual(next, waypoints, o, sigma, samples);
      if (std::isfinite(after.merit) && after.merit <= before.merit + 1e-8) {
        take = true;
        break;
      }
    }
    if (!take) {
      trust *= 0.5;
      if (trust < 1e-4) {
        best.status = "SCP stalled";
        break;
      }
      continue;
    }
    z = std::move(next);
    ++accepted;
    trust = alpha > .99 ? std::min(1.0, trust * 1.3) : std::max(1e-4, trust * 0.7);
    double update =
      std::max(
      (alpha * solution.head(l.vp)).lpNorm<Eigen::Infinity>(),
      std::abs(alpha * solution(l.tp)));
    double virtual_max = virtual_controls ? solution.segment(l.vp, 9 * n).maxCoeff() : 0;
    auto candidate = resultOf(z, o);
    candidate.maximum_update = update;
    candidate.maximum_virtual_control = virtual_max;
    candidate.maximum_dynamics_defect = after.dynamics;
    candidate.maximum_waypoint_residual = after.waypoint;
    candidate.maximum_order_residual = after.order;
    candidate.scp_iterations = iter + 1;
    // Never certify a trajectory from QP status alone. Independent higher-resolution audit.
    if (after.dynamics <
      std::max({o.position_tolerance, o.velocity_tolerance, o.attitude_tolerance}) &&
      after.physical <= o.constraint_tolerance && after.waypoint <= o.cstc_tolerance &&
      after.order <= o.cstc_tolerance)
    {
      const auto audit = validateOmTrajectory(candidate, o, waypoints);
      // Integration error cannot be corrected by solving the same QP again.
      // Refine RK4 without adding optimization variables or loosening tolerances.
      if (audit.integration_error >
        0.5 * std::min({o.position_tolerance, o.velocity_tolerance, o.attitude_tolerance}) &&
        o.integration_substeps < 100)
      {
        o.integration_substeps = std::min(100, 2 * o.integration_substeps);
      }
      for (auto [k, fraction]:audit.refinement_points) {
        if (std::none_of(
            samples[k].begin(), samples[k].end(), [&](double s) {
              return std::abs(s - fraction) < 1e-4;
            }))
        {
          samples[k].push_back(fraction);
          trust = 1.0;
        }
      }
      if (!audit.valid) {
        status += " audit=" + audit.reason + " physical=" + std::to_string(
          audit.constraint_violation) +
          " integration=" + std::to_string(audit.integration_error);
      }
      const auto boundary_valid = [&](const OmTrajectoryState & x, const OmTrajectoryBoundary & b) {
          return (x.position - b.position).lpNorm<Eigen::Infinity>() <= o.position_tolerance &&
                 (x.velocity - b.velocity).lpNorm<Eigen::Infinity>() <= o.velocity_tolerance &&
                 log(b.attitude.conjugate() * x.attitude).lpNorm<Eigen::Infinity>() <=
                 o.attitude_tolerance;
        };
      if (audit.valid && boundary_valid(candidate.states.front(), initial) &&
        boundary_valid(candidate.states.back(), terminal))
      {
        candidate.success = true;
        candidate.dynamics_validated = true;
        candidate.waypoint_times = audit.waypoint_times;
        candidate.status = "validated local SCP solution";
        if (!best.success || candidate.total_time < best.total_time) {
          best = candidate;
        }
      }
    }
    OmTrajectoryIteration entry;
    entry.scp_iteration = iter + 1;
    entry.total_time = z.T;
    entry.maximum_update = update;
    entry.maximum_dynamics_defect = after.dynamics;
    entry.maximum_virtual_control = virtual_max;
    entry.maximum_waypoint_residual = after.waypoint;
    entry.maximum_order_residual = after.order;
    entry.qp_solve_time = qtime;
    entry.step_solve_time = seconds(step_start);
    entry.elapsed_solve_time = seconds(start);
    entry.solver_status = status + (polish ? " correction" : " time") + " alpha=" + std::to_string(
      alpha);
    entry.states = candidate.states;
    if (o.progress_callback) {
      o.progress_callback(entry);
    }
    if (o.store_history) {
      history.push_back(entry);
    }
    const double model_decrease = -qp.q.dot(solution) -
      0.5 * (qp.h.array() * solution.array().square()).sum();
    if (candidate.success && !polish && status.rfind("solved", 0) == 0 && sigma == 0 &&
      !virtual_controls && model_decrease <= o.convergence_tolerance * trust &&
      std::abs(best.total_time - z.T) < o.convergence_tolerance)
    {
      best = candidate;
      best.converged = true;
      break;
    }
    if (update < 0.05 && after.dynamics >= 0.9 * before.dynamics &&
      after.dynamics >
      std::min({o.position_tolerance, o.velocity_tolerance, o.attitude_tolerance}) &&
      o.virtual_control_weight < o.maximum_virtual_control_weight)
    {
      o.virtual_control_weight = std::min(
        o.maximum_virtual_control_weight,
        2 * o.virtual_control_weight);
    }
    if (iter + 1 == o.max_scp_iterations && !best.success) {
      best = candidate;
      best.status = "SCP iteration limit; no validated trajectory";
    }
  }
  best.history = std::move(history);
  best.total_solve_time = seconds(start);
  if (best.success && !best.converged) {
    best.status = "validated feasible incumbent; local SCP not converged";
  }
  if (best.status.empty()) {
    best.status = "No validated trajectory";
  }
  return best;
}
