#!/usr/bin/env python3
"""Generate the structural acados model for AcadosNmpcSolver.

Runtime C++ configuration owns horizon, dt, gravity, bounds, weights and SQP
settings. Regenerate only when changing equations, cost dimensions, constraint
structure or solver/integrator type. Never edit the generated C files manually.
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
    """Build the continuous 13-state, 4-input rigid-body prediction model.

    [DYN-1] p_dot=v, v_dot=R(q)(e3*a_T+body_aero/mass)-g*e3.
    [DYN-3] q_dot uses measured/predicted body rate, not its command.
    [DYN-4] omega_dot=(omega_command-omega)/tau models the rate loop.
    acados' ERK integrator later discretizes these continuous equations.
    """
    p = ca.SX.sym("p", 3)
    v = ca.SX.sym("v", 3)
    q = ca.SX.sym("q", 4)
    thrust_acceleration = ca.SX.sym("a_T")
    omega = ca.SX.sym("omega", 3)
    actual_omega = ca.SX.sym("actual_omega", 3)
    x = ca.vertcat(p, v, q, actual_omega)
    u = ca.vertcat(thrust_acceleration, omega)
    xdot = ca.SX.sym("xdot", 13)
    parameters = ca.SX.sym("parameters", 12)  # g, previous u(4), tau(3), body drag/mass(3), kh/mass

    qw, qx, qy, qz = q[0], q[1], q[2], q[3]
    wx, wy, wz = actual_omega[0], actual_omega[1], actual_omega[2]
    # [DYN-1] Third column of R(q), i.e. R(q)e3 for q=[qw,qx,qy,qz].
    body_z_world = ca.vertcat(
        2.0 * (qx * qz + qw * qy),
        2.0 * (qy * qz - qw * qx),
        1.0 - 2.0 * (qx * qx + qy * qy),
    )
    # [DYN-2] Translational acceleration a=a_T*R(q)e3-g*e3.
    acceleration = thrust_acceleration * body_z_world + ca.vertcat(0.0, 0.0, -parameters[0])
    rotation = ca.horzcat(
        ca.vertcat(1 - 2*(qy*qy + qz*qz), 2*(qx*qy + qw*qz), 2*(qx*qz - qw*qy)),
        ca.vertcat(2*(qx*qy - qw*qz), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz + qw*qx)),
        body_z_world,
    )
    body_velocity = rotation.T @ v
    aero = -parameters[8:11] * body_velocity
    aero[2] += parameters[11] * (body_velocity[0]**2 + body_velocity[1]**2)
    acceleration += rotation @ aero
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
    model.p = parameters
    model.f_expl_expr = ca.vertcat(v, acceleration, q_dot, (omega - actual_omega) / parameters[5:8])
    model.f_impl_expr = xdot - model.f_expl_expr
    return model


def build_ocp(output_directory: Path) -> AcadosOcp:
    """Configure the multiple-shooting OCP, costs, bounds, and SQP method."""
    model = export_model()
    ocp = AcadosOcp()
    ocp.model = model
    ocp.solver_options.N_horizon = HORIZON
    ocp.solver_options.tf = HORIZON * DT

    # Numeric placeholders only; AcadosNmpcSolver supplies every W/yref at runtime.
    # Discrete sum: J=sum ||x-xref||_Q^2+||u-uref||_R^2 + terminal.
    # Stage zero additionally penalizes u0 - previous actually applied command;
    # this is inter-cycle smoothing, not an all-horizon Delta-u penalty.
    ocp.parameter_values = np.array([GRAVITY, GRAVITY, 0.0, 0.0, 0.0, 0.10, 0.083, 0.25, 0.0, 0.0, 0.0, 0.0])
    ocp.cost.cost_type = "NONLINEAR_LS"
    ocp.cost.cost_type_0 = "NONLINEAR_LS"
    ocp.cost.cost_type_e = "NONLINEAR_LS"
    ocp.model.cost_y_expr = ca.vertcat(model.x[:10], model.u)
    ocp.model.cost_y_expr_0 = ca.vertcat(model.x[:10], model.u, model.u - model.p[1:5])
    ocp.model.cost_y_expr_e = model.x[:10]
    ocp.cost.W = np.eye(14)
    ocp.cost.W_0 = np.eye(18)
    ocp.cost.W_e = np.eye(10)
    ocp.cost.yref = np.zeros(14)
    ocp.cost.yref_0 = np.zeros(18)
    ocp.cost.yref_e = np.zeros(10)
    for ref in (ocp.cost.yref, ocp.cost.yref_0, ocp.cost.yref_e):
        ref[6] = 1.0
    ocp.cost.yref[10] = GRAVITY
    ocp.cost.yref_0[10] = GRAVITY
    # Explicit unit scaling; C++ also restores this after runtime dt updates,
    # whose generated helper otherwise sets stage scaling to dt.
    ocp.solver_options.cost_scaling = np.ones(HORIZON + 1)

    # [ACADOS-CONSTRAINT-2] Box bounds on every u_k=[a_T,omega_x,omega_y,omega_z].
    # These generation-time values establish dimensions; the C++ wrapper
    # overwrites their numeric limits from AcadosNmpcOptions at configuration.
    ocp.constraints.idxbu = np.arange(4)
    ocp.constraints.lbu = np.array([0.1, -14.0, -14.0, -14.0])
    ocp.constraints.ubu = np.array([50.0, 14.0, 14.0, 14.0])
    # [ACADOS-CONSTRAINT-1] x_0 equality-constraint dimensions.  The C++
    # wrapper replaces this nominal value with the latest measured state.
    ocp.constraints.x0 = np.array(
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
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
    ocp.solver_options.nlp_solver_max_iter = 60
    ocp.solver_options.eval_residual_at_max_iter = True
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
    if not (install_prefix / "lib" / "link_libs.json").is_file():
        raise FileNotFoundError(
            f"missing {install_prefix / 'lib' / 'link_libs.json'}; "
            "rerun the updated 3rdpart/build_all.sh or use a complete ACADOS_INSTALL_PREFIX"
        )
    if not (acados_source / "interfaces" / "acados_template").is_dir():
        raise FileNotFoundError(f"missing acados source tree: {acados_source}")
    os.environ["ACADOS_SOURCE_DIR"] = str(acados_source)
    json_file = output_directory / f"{MODEL_NAME}_ocp.json"
    ocp = build_ocp(output_directory)
    ocp.code_gen_options.acados_include_path = str(install_prefix / "include")
    ocp.code_gen_options.acados_lib_path = str(install_prefix / "lib")
    AcadosOcpSolver.generate(ocp, json_file=str(json_file), verbose=True)
    # Normalize upstream template whitespace as part of generation, never by
    # editing individual generated files. This keeps git diff --check useful.
    for source in output_directory.rglob("*"):
        if source.suffix in {".c", ".h"}:
            source.write_text("\n".join(line.rstrip() for line in source.read_text().splitlines()) + "\n")
    # Normal CMake builds compile these sources; regeneration needs no compiler.


if __name__ == "__main__":
    main()
