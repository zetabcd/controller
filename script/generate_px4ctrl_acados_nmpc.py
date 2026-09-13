#!/usr/bin/env python3
"""Generate the acados backend used by px4ctrl's nonlinear MPC facade.

The generated solver uses the same state and input convention as OmMpcControl:
x=[p_ENU,v_ENU,q_wxyz], u=[a_T,omega_body].  a_T is mass-normalized, so the
generated model is independent of vehicle mass; px4ctrl multiplies it by the
configured mass only when publishing total thrust.

Mathematical labels such as [DYN-1] and [ACADOS-COST-1] match the comments in
solver_nmpc_common.cpp and solver_nmpc_acados.cpp.  Generated C files must not
be edited by hand: rerunning this script overwrites them.
"""

from __future__ import annotations

import os
from pathlib import Path

import casadi as ca
import numpy as np
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver


MODEL_NAME = "px4ctrl_nmpc"
HORIZON = 12
DT = 0.04
GRAVITY = 9.805


def export_model() -> AcadosModel:
    """Build the continuous 10-state, 4-input rigid-body prediction model.

    [DYN-1] p_dot=v, v_dot=R(q)e3*a_T-g*e3.
    [DYN-3] q_dot=0.5*q tensor [0,omega], with body-frame omega.
    acados' ERK integrator later discretizes these continuous equations.
    """
    p = ca.SX.sym("p", 3)
    v = ca.SX.sym("v", 3)
    q = ca.SX.sym("q", 4)
    thrust_acceleration = ca.SX.sym("a_T")
    omega = ca.SX.sym("omega", 3)
    x = ca.vertcat(p, v, q)
    u = ca.vertcat(thrust_acceleration, omega)
    xdot = ca.SX.sym("xdot", 10)

    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    wx, wy, wz = omega[0], omega[1], omega[2]
    # [DYN-1] Third column of R(q), i.e. R(q)e3 for q=[qw,qx,qy,qz].
    body_z_world = ca.vertcat(
        2.0 * (qx * qz + qw * qy),
        2.0 * (qy * qz - qw * qx),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )
    # [DYN-2] Translational acceleration a=a_T*R(q)e3-g*e3.
    acceleration = thrust_acceleration * body_z_world + ca.vertcat(0.0, 0.0, -GRAVITY)
    # [DYN-3] Expanded Hamilton product 0.5*q tensor [0,omega].
    q_dot = 0.5 * ca.vertcat(
        -qx * wx - qy * wy - qz * wz,
        qw * wx + qy * wz - qz * wy,
        qw * wy + qz * wx - qx * wz,
        qw * wz + qx * wy - qy * wx,
    )

    model = AcadosModel()
    model.name = MODEL_NAME
    model.x = x
    model.xdot = xdot
    model.u = u
    model.f_expl_expr = ca.vertcat(v, acceleration, q_dot)
    model.f_impl_expr = xdot - model.f_expl_expr
    return model


def build_ocp(output_directory: Path) -> AcadosOcp:
    """Configure the multiple-shooting OCP, costs, bounds, and SQP method."""
    model = export_model()
    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = HORIZON
    ocp.solver_options.tf = HORIZON * DT

    # [ACADOS-COST-1] acados NONLINEAR_LS evaluates 0.5*e.T@W@e.
    # Therefore W=2*diag(weights) reproduces sum_i weights_i*e_i^2 used by
    # the other backends.  y=[p,v,q,a_T,omega] and y_e=[p,v,q].
    #
    # Note: acados uses component-wise quaternion residual here.  The runtime
    # wrapper makes the reference quaternion signs continuous before solving.
    # This generated OCP currently has no input-difference (Delta-u) residual;
    # that is an explicit implementation difference from the three shooting
    # backends, not an omitted line hidden elsewhere in the wrapper.
    stage_weights = np.array(
        [18.0] * 3 + [3.0] * 3 + [5.0] * 4 + [0.10, 0.12, 0.12, 0.12]
    )
    terminal_weights = np.array([35.0] * 3 + [6.0] * 3 + [10.0] * 4)
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.model.cost_y_expr = ca.vertcat(model.x, model.u)
    ocp.model.cost_y_expr_e = model.x
    ocp.cost.W = 2.0 * np.diag(stage_weights)
    ocp.cost.W_e = 2.0 * np.diag(terminal_weights)
    ocp.cost.yref = np.zeros(14)
    ocp.cost.yref[6] = 1.0
    ocp.cost.yref[10] = GRAVITY
    ocp.cost.yref_e = np.zeros(10)
    ocp.cost.yref_e[6] = 1.0

    # [ACADOS-CONSTRAINT-2] Box bounds on every u_k=[a_T,omega_x,omega_y,omega_z].
    # These generation-time values establish dimensions; the C++ wrapper
    # overwrites their numeric limits from SolverNmpcOptions before each solve.
    ocp.constraints.idxbu = np.arange(4)
    ocp.constraints.lbu = np.array([0.1, -14.0, -14.0, -14.0])
    ocp.constraints.ubu = np.array([50.0, 14.0, 14.0, 14.0])
    # [ACADOS-CONSTRAINT-1] x_0 equality-constraint dimensions.  The C++
    # wrapper replaces this nominal value with the latest measured state.
    ocp.constraints.x0 = np.array(
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0]
    )

    # [ACADOS-SOLVE-1] Full SQP repeatedly linearizes the nonlinear dynamics.
    # Gauss-Newton approximates the least-squares Hessian; HPIPM solves each
    # partially condensed QP. ERK is classical 4-stage Runge-Kutta, one
    # integration step per shooting interval; merit backtracking globalizes SQP.
    ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps = 1
    ocp.solver_options.nlp_solver_type = "SQP"
    ocp.solver_options.nlp_solver_max_iter = 30
    ocp.solver_options.globalization = "MERIT_BACKTRACKING"
    ocp.solver_options.print_level = 0
    ocp.solver_options.tol = 1.0e-5
    ocp.code_gen_options.code_export_directory = str(output_directory)
    return ocp


def main() -> None:
    """Regenerate the project-owned solver against an installed acados."""
    repository = Path(__file__).resolve().parents[1]
    output_directory = (
        repository
        / "src"
        / "realflight_modules"
        / "px4ctrl"
        / "generated"
        / "acados"
        / MODEL_NAME
    )
    output_directory.mkdir(parents=True, exist_ok=True)
    acados_source = Path(
        os.environ.get("ACADOS_SOURCE_DIR", repository / "3rdpart" / "src" / "acados")
    )
    install_prefix = Path(os.environ.get("ACADOS_INSTALL_PREFIX", "/usr/local"))
    acados_library = install_prefix / "lib" / "libacados.so"
    if not acados_library.exists():
        raise FileNotFoundError(f"missing {acados_library}; run 3rdpart/build_all.sh first")
    if not (acados_source / "interfaces" / "acados_template").is_dir():
        raise FileNotFoundError(f"missing acados source tree: {acados_source}")
    os.environ["ACADOS_SOURCE_DIR"] = str(acados_source)
    json_file = output_directory / f"{MODEL_NAME}_ocp.json"
    ocp = build_ocp(output_directory)
    ocp.code_gen_options.acados_include_path = str(install_prefix / "include")
    ocp.code_gen_options.acados_lib_path = str(install_prefix / "lib")
    AcadosOcpSolver.generate(ocp, json_file=str(json_file), verbose=True)
    AcadosOcpSolver.build(str(output_directory), with_cython=False, verbose=True)


if __name__ == "__main__":
    main()
