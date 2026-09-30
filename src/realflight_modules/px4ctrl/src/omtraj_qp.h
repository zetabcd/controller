#pragma once
#include <Eigen/SparseCore>
#include <osqp/osqp.h>
#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <string>
#include <vector>

namespace omtraj::detail
{
using Entries = std::vector<std::pair<int, double>>;
constexpr double inf = OSQP_INFTY;
// Sparse QP rows and algebraic presolve. Screening changes the row identities,
// so each reduced QP gets a fresh factorization rather than stale dual warm starts.
struct Qp
{
  int size;
  std::vector<Eigen::Triplet<double>> entries;
  std::vector<double> lower, upper;
  Eigen::VectorXd h, q;
  explicit Qp(int n)
  : size(n), h(Eigen::VectorXd::Constant(n, 1e-9)), q(Eigen::VectorXd::Zero(n)) {}
  void row(const Entries & a, double lo, double hi)
  {
    int r = lower.size();
    for (auto [c, v]:a) {
      if (std::abs(v) > 1e-10) {
        entries.emplace_back(r, c, v);
      }
    }
    lower.push_back(lo);
    upper.push_back(hi);
  }
  void bound(int i, double lo, double hi) {row({{i, 1}}, lo, hi);}
};
// Substitute fixed variables and merge singleton rows into bounds. This is
// algebraic QP presolve, rebuilt at every SCP step, not a waypoint schedule.
struct ReducedQp
{
  Qp qp;
  Eigen::VectorXd fixed;
  std::vector<int> columns;
  explicit ReducedQp(const Qp & input)
  : qp(0), fixed(Eigen::VectorXd::Zero(input.size))
  {
    std::vector<Entries> rows(input.lower.size());
    for (const auto & a : input.entries) {
      rows[a.row()].emplace_back(a.col(), a.value());
    }
    auto lower = input.lower, upper = input.upper;
    Eigen::VectorXd lo = Eigen::VectorXd::Constant(input.size, -inf), hi = -lo;
    std::vector<bool> known(input.size, false), active(rows.size(), true);
    bool changed = true;
    while (changed) {
      changed = false;
      for (size_t r = 0; r < rows.size(); ++r) {
        if (!active[r]) {
          continue;
        }
        Entries remaining;
        for (auto [c, v] : rows[r]) {
          if (known[c]) {
            lower[r] -= v * fixed(c);
            upper[r] -= v * fixed(c);
          } else {
            remaining.emplace_back(c, v);
          }
        }
        rows[r] = std::move(remaining);
        if (rows[r].size() > 1) {
          continue;
        }
        active[r] = false;
        if (rows[r].empty()) {
          if (lower[r] > 1e-8 || upper[r] < -1e-8) {
            throw std::runtime_error("Inconsistent constant QP row");
          }
          continue;
        }
        auto [c, v] = rows[r].front();
        double a = lower[r] / v, b = upper[r] / v;
        if (v < 0) {
          std::swap(a, b);
        }
        lo(c) = std::max(lo(c), a);
        hi(c) = std::min(hi(c), b);
        if (lo(c) > hi(c) + 1e-8) {
          throw std::runtime_error("Inconsistent QP bounds");
        }
        if (hi(c) - lo(c) <= 1e-10) {
          fixed(c) = 0.5 * (lo(c) + hi(c));
          known[c] = true;
          changed = true;
        }
      }
    }
    std::vector<int> index(input.size, -1);
    for (int c = 0; c < input.size; ++c) {
      if (!known[c]) {
        index[c] = columns.size();
        columns.push_back(c);
      }
    }
    qp = Qp(columns.size());
    for (size_t i = 0; i < columns.size(); ++i) {
      int c = columns[i];
      qp.h(i) = input.h(c);
      qp.q(i) = input.q(c);
      if (lo(c) > -inf || hi(c) < inf) {
        qp.bound(i, lo(c), hi(c));
      }
    }
    for (size_t r = 0; r < rows.size(); ++r) {
      if (!active[r]) {
        continue;
      }
      Entries a;
      double minimum = 0, maximum = 0;
      for (auto [c, v] : rows[r]) {
        a.emplace_back(index[c], v);
        minimum += v * (v > 0 ? lo(c) : hi(c));
        maximum += v * (v > 0 ? hi(c) : lo(c));
      }
      // A linear inequality that cannot become active anywhere in the trust
      // box is redundant for this QP. It is reconsidered next SCP iteration.
      if (minimum >= lower[r] && maximum <= upper[r]) {
        continue;
      }
      qp.row(a, lower[r], upper[r]);
    }
  }
  Eigen::VectorXd expand(const Eigen::VectorXd & x) const
  {
    auto result = fixed;
    for (size_t i = 0; i < columns.size(); ++i) {
      result(columns[i]) = x(i);
    }
    return result;
  }
};
inline bool solveQp(
  const Qp & original, Eigen::VectorXd & x, std::string & status,
  double tolerance)
{
  try {
    ReducedQp reduced(original);
    const auto & qp = reduced.qp;
    if (qp.size == 0) {
      x = reduced.fixed;
      status = "solved by presolve";
      return x.allFinite();
    }
    using Sparse = Eigen::SparseMatrix<OSQPFloat, Eigen::ColMajor, OSQPInt>;
    Sparse A(qp.lower.size(), qp.size), P(qp.size, qp.size);
    A.setFromTriplets(qp.entries.begin(), qp.entries.end());
    A.makeCompressed();
    for (int i = 0; i < qp.size; ++i) {
      P.insert(i, i) = qp.h(i);
    }
    P.makeCompressed();
    OSQPCscMatrix a, p;
    OSQPCscMatrix_set_data(
      &a, A.rows(), A.cols(), A.nonZeros(), A.valuePtr(),
      A.innerIndexPtr(), A.outerIndexPtr());
    OSQPCscMatrix_set_data(
      &p, P.rows(), P.cols(), P.nonZeros(), P.valuePtr(),
      P.innerIndexPtr(), P.outerIndexPtr());
    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = 0;
    settings.polishing = 1;
    settings.eps_abs = tolerance;
    settings.eps_rel = std::min(1e-4, tolerance);
    settings.check_dualgap = 0;
    settings.max_iter = 3000;
    settings.adaptive_rho_interval = 50;
    OSQPSolver * solver = nullptr;
    if (osqp_setup(
        &solver, &p, qp.q.data(), &a, qp.lower.data(), qp.upper.data(), A.rows(),
        A.cols(), &settings))
    {
      if (solver) {
        osqp_cleanup(solver);
      }
      status = "OSQP setup failed";
      return false;
    }
    const int code = osqp_solve(solver);
    const int state = solver->info->status_val;
    status = std::string(solver->info->status) + " it=" + std::to_string(solver->info->iter) +
      " n=" + std::to_string(qp.size) + " rows=" + std::to_string(A.rows()) +
      " prim=" + std::to_string(solver->info->prim_res) + " dual=" + std::to_string(
      solver->info->dual_res);
    const bool solved = !code && solver->solution &&
      (state == OSQP_SOLVED || state == OSQP_SOLVED_INACCURATE ||
      (state == OSQP_MAX_ITER_REACHED && solver->info->prim_res < 1e-3));
    if (solved) {
      x = reduced.expand(Eigen::Map<const Eigen::VectorXd>(solver->solution->x, qp.size));
    }
    osqp_cleanup(solver);
    return solved && x.allFinite();
  } catch (const std::exception & e) {
    status = e.what();
    return false;
  }
}
}
