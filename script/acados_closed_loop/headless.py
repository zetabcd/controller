"""Headless experiment adapter: original QuadSimNode/QUAD/MuJoCo and ROS controllers.

No plant/controller equations changed. Fixed deadline pacing replaces viewer sync
and the original relative sleep. Motor ODE retains measured wall dt. Raw states,
source-stamped debug messages and actuator messages are saved independently.
"""
import argparse
import time
from pathlib import Path
import numpy as np
import rclpy
import mujoco
from scipy.spatial.transform import Rotation as R
from ament_index_python.packages import get_package_share_directory
from rosgraph_msgs.msg import Clock
from quadsim_mujoco.quadsim_node import QuadSimNode, QUAD, init_mujoco, quat_mujoco2scipy


class RecordedSim(QuadSimNode):
    def __init__(self):
        self.debug_rows = []
        self.motor_rows = []
        self.solver_rows = []
        super().__init__('quadsim_node')

    def px4ctrldebug_callback(self, msg):
        super().px4ctrldebug_callback(msg)
        self.debug_rows.append([msg.timestamp * 1e-6, msg.state,
            msg.ref_p_x, -msg.ref_p_y, -msg.ref_p_z,
            msg.ref_v_x, -msg.ref_v_y, -msg.ref_v_z,
            msg.des_rate_x, -msg.des_rate_y, -msg.des_rate_z, msg.des_thrust])

        if getattr(msg, 'nmpc_active', False):
            self.solver_rows.append([msg.timestamp * 1e-6, msg.nmpc_status,
                msg.nmpc_iterations, msg.nmpc_solve_time_ms, msg.nmpc_residual,
                msg.nmpc_fallback])

    def actuator_motors_callback(self, msg):
        super().actuator_motors_callback(msg)
        self.motor_rows.append([msg.timestamp * 1e-6, *msg.control[:4]])


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--output', required=True)
    parser.add_argument('--duration', type=float, default=43)
    parser.add_argument('--seed', type=int, default=42)
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = RecordedSim()
    node.noise = np.random.default_rng(args.seed)
    quad = QUAD()
    node.parameter_set(quad)
    quad.wind_model.reset(U0=quad.param.noise.wind_mean,
        height=quad.param.noise.wind_height, dt=1/quad.param.sim_freq_max,
        duration=200.0, loop=True)
    quad.actuator_state.motor_vel_rad = np.full(4,
        np.sqrt(quad.param.uav.mass * quad.param.gra / (4*quad.param.motor.ct0)))
    quad.update_actuator_internal_state()
    model = mujoco.MjModel.from_xml_path(str(Path(
        get_package_share_directory('quadsim_mujoco')) / 'mjcf/quad.xml'))
    init_mujoco(model, quad, node)
    data = mujoco.MjData(model)
    # Match the FSM's hard-coded AUTO_HOVER altitude, avoiding a 4.5 m drop.
    data.qpos[2] = 0.5
    mujoco.mj_forward(model, data)
    clock_pub = node.create_publisher(Clock, '/clock', 10)
    body = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, 'quad')
    step = float(model.opt.timestep)
    start = time.perf_counter()
    quad.start_time = 1000.0
    last = start - step
    rows = []
    i = 0
    try:
        while rclpy.ok() and time.perf_counter() - start < args.duration:
            tick = time.perf_counter()
            dt = tick-last
            last = tick
            # Original simulator dispatches one ready callback per physics step.
            rclpy.spin_once(node, timeout_sec=0)
            quad.now_time = 1000.0 + i*step
            clock = Clock()
            ns = round(quad.now_time*1e9)
            clock.clock.sec, clock.clock.nanosec = divmod(ns, 10**9)
            clock_pub.publish(clock)
            quad.set_input(node.control)
            force, torque = quad.get_body_force_moment()
            data.xfrc_applied[body, :3] = force
            data.xfrc_applied[body, 3:] = torque
            quad.step(dt)
            mujoco.mj_step(model, data)
            Rbi = R.from_quat(quat_mujoco2scipy(quad.state.quat)).as_matrix()
            quad.state.Rbi = Rbi
            quad.state.omega = data.sensor('body_angvel').data.copy()
            quad.state.quat = data.sensor('body_quat').data.copy()
            quad.state.acc_B = data.sensor('body_linacc').data.copy()
            quad.state.acc = Rbi @ quad.state.acc_B
            quad.state.vel = data.sensor('body_vel').data.copy()
            quad.state.vel_B = Rbi.T @ quad.state.vel
            quad.state.pos = data.sensor('body_pos').data.copy()
            node.sens_topic_pub(quad)
            node.att_topic_pub(quad)
            node.local_pose_topic_pub(quad)
            rows.append([quad.now_time, data.time, dt, *quad.state.pos,
                *quad.state.vel, *quad.state.omega, *quad.state.quat,
                *quad.actuator_state.motor_vel_rad, tick-start, time.time()])
            if not np.isfinite(rows[-1]).all() or np.linalg.norm(quad.state.pos)>100:
                raise RuntimeError('Experiment diverged')
            i += 1
            remaining = start + i*step - time.perf_counter()
            if remaining > 0:
                time.sleep(remaining)
    finally:
        np.savez_compressed(args.output, state=np.array(rows),
            debug=np.array(node.debug_rows), motors=np.array(node.motor_rows),
            solver=np.array(node.solver_rows), seed=args.seed)
        print(f'Saved {len(rows)} states, {len(node.debug_rows)} MPC samples', flush=True)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
