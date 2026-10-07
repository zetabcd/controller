#!/usr/bin/env python3
"""Isolated MuJoCo check of CMD waiting, scene selection and repeated legs."""
import argparse
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--count', type=int, default=1)
    parser.add_argument('--return-count', type=int, default=0, help='0: outbound only; 1..3: return leg')
    parser.add_argument('--preparation-only', action='store_true', help='Check hover, CMD preparation and service gating only')
    parser.add_argument('--domain', type=int, default=78)
    parser.add_argument('--log', default='/tmp/gap_integration.log')
    parser.add_argument('--gap-params', default='src/realflight_modules/gap_planner/config/gaps.yaml')
    args = parser.parse_args()
    if not 1 <= args.count <= 3 or not 0 <= args.return_count <= 3 or not 70 <= args.domain <= 90:
        raise ValueError('Use counts 1..3 and isolated test domain 70..90')
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['ROS_LOG_DIR'] = '/tmp/gap_ros_logs'
    import rclpy
    from rclpy.node import Node
    from gap_msgs.msg import ExecutionStatus
    from px4debug_msgs.msg import Px4ctrlDebug
    from gap_msgs.srv import PlanGaps, UploadTrajectory, ConfigureGates
    from std_srvs.srv import Trigger
    from std_msgs.msg import String, UInt32
    import yaml
    import numpy as np
    from scipy.spatial.transform import Rotation
    with open(args.gap_params, encoding='utf-8') as config:
        cfg = yaml.safe_load(config)['gap_planner']['ros__parameters']

    stream = open(args.log, 'w', encoding='utf-8')
    # Production launch uses editable constants. Override them only in a
    # temporary test wrapper, keeping the user's launch file untouched.
    from ament_index_python.packages import get_package_share_directory
    launch_file = Path(get_package_share_directory('gap_planner')) / 'launch' / 'gap_flight.launch.py'
    import runpy
    settings = runpy.run_path(str(launch_file))
    control_file = Path(get_package_share_directory('px4ctrl')) / 'config' / settings['CONTROLLER_PARAMS']
    with control_file.open(encoding='utf-8') as stream_config:
        takeoff_height = yaml.safe_load(stream_config)['px4ctrl_node']['ros__parameters']['trajectory']['takeoff_height']
    test_directory = tempfile.TemporaryDirectory(prefix='gap_flow_')
    test_launch = Path(test_directory.name) / 'test.launch.py'
    test_launch.write_text(
        'import runpy\n'
        f'settings = runpy.run_path({str(launch_file)!r})\n'
        'generate_launch_description = settings["generate_launch_description"]\n'
        'generate_launch_description.__globals__.update('
        f'SIMULATION=True, HEADLESS=True, RVIZ=False, GAP_PARAMS={str(Path(args.gap_params).resolve())!r})\n',
        encoding='utf-8')
    process = subprocess.Popen(['ros2', 'launch', str(test_launch)], stdout=stream,
        stderr=subprocess.STDOUT, start_new_session=True)
    rclpy.init()
    node = Node('gap_flow_test')
    latest, planner_text, selected_count = [None], [''], [None]
    history, crossing_samples, references = [], {}, []
    current_count = [args.count]
    returning, previous_position = [False], [None]

    def status(message):
        latest[0] = message
        if message.state == 'EXECUTING':
            p, q = message.pose.position, message.pose.orientation
            names = cfg['gate_order'][:current_count[0]]
            if returning[0]:
                names = names[::-1]
            position = np.array([p.x, p.y, p.z])
            # A nearby infinite plane is not evidence of passing an opening.
            # Detect ordered, directed crossings and interpolate the actual
            # position at the plane, including the opening's finite Y/Z bounds.
            next_index = len(crossing_samples)
            if next_index < len(names) and previous_position[0] is not None:
                name = names[next_index]
                g = cfg['gates'][name]
                rotation = Rotation.from_euler('xyz', g['sim_rpy_deg'], degrees=True)
                center = np.array(g['sim_position']) + rotation.apply(g.get('opening_offset', [0, 0, 0]))
                w, x, y, z = g.get('opening_quaternion_wxyz', [1, 0, 0, 0])
                rotation *= Rotation.from_quat([x, y, z, w])
                before = rotation.inv().apply(previous_position[0]-center)
                after = rotation.inv().apply(position-center)
                direction = -1 if returning[0] else 1
                if direction*before[0] <= 0 < direction*after[0]:
                    point = before + (after-before)*(-before[0]/(after[0]-before[0]))
                    roll = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler('xyz', degrees=True)[0]
                    if abs(point[1]) < g['width']/2 and abs(point[2]) < g['height']/2:
                        crossing_samples[name] = (abs(point[0]), roll)
            previous_position[0] = position
        if not history or history[-1] != message.state:
            history.append(message.state)
            print(message.state, message.reason, flush=True)

    node.create_subscription(ExecutionStatus, '/gap/execution', status, 10)
    node.create_subscription(Px4ctrlDebug, '/debugPx4/ctrl',
        lambda m: references.append((m.state, -m.ref_p_z)), 10)
    node.create_subscription(String, '/gap/planner_status', lambda m: planner_text.__setitem__(0, m.data), 10)
    node.create_subscription(UInt32, '/gap/sim/selected_count', lambda m: selected_count.__setitem__(0, m.data), 10)
    planner = node.create_client(PlanGaps, '/gap/plan')
    start = node.create_client(Trigger, '/gap/start')
    upload = node.create_client(UploadTrajectory, '/gap/upload')
    configure = node.create_client(ConfigureGates, '/gap/sim/configure')

    def until(predicate, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if predicate():
                return
            if process.poll() is not None:
                raise RuntimeError('Launch exited; inspect ' + args.log)
        raise TimeoutError(f'{args.log}: state={latest[0]}, planner={planner_text[0]}')

    def call(client, request):
        until(client.service_is_ready, 20)
        future = client.call_async(request)
        until(future.done, 10)
        return future.result()

    def settle(seconds):
        deadline = time.monotonic()+seconds
        until(lambda: time.monotonic() >= deadline, seconds+2)

    try:
        # Early requests during CMD preparation must be rejected, not cached.
        until(lambda: latest[0] is not None and latest[0].state == 'PREPARING', 40)
        assert latest[0].command_mode and not latest[0].hovering
        assert not call(start, Trigger.Request()).success
        assert not call(upload, UploadTrajectory.Request()).accepted
        assert not call(planner, PlanGaps.Request(count=1)).accepted
        until(lambda: latest[0] is not None and latest[0].command_mode
              and latest[0].hovering and latest[0].feedback_valid, 40)
        hover_z = [z for state, z in references if state == 2]
        assert hover_z and abs(hover_z[0] - latest[0].takeoff_position.z - 0.2) < 1e-4
        cmd_z = [z for state, z in references if state == 3]
        assert cmd_z and abs(cmd_z[0] - hover_z[-1]) < 0.1, 'CMD height must start continuously'
        target_z = cmd_z[0] + takeoff_height
        assert abs(latest[0].pose.position.z - target_z) < 0.1
        assert abs(cmd_z[-1] - target_z) < 0.01
        settle(3)
        assert selected_count[0] == 0, 'Simulator must start with no active gates'
        assert latest[0].state == 'WAITING', 'Entering CMD must not execute an old trajectory'
        service_names = {name for name, _ in node.get_service_names_and_types()}
        assert '/gap/enter_cmd' not in service_names and '/gap/finish' not in service_names
        # A console opened after the switch receives the current periodic
        # status; it does not need the original AUX2 edge or first status frame.
        late_states = []
        late_subscription = node.create_subscription(ExecutionStatus, '/gap/execution', late_states.append, 10)
        until(lambda: bool(late_states) and late_states[-1].command_mode, 3)
        assert late_states[-1].state == 'WAITING'
        node.destroy_subscription(late_subscription)
        print('PASS AUX2 entered CMD; no console mode services; late subscriber sees WAITING', flush=True)
        assert not call(start, Trigger.Request()).success, 'Empty CMD must hold without starting'
        if args.preparation_only:
            print(f'PASS 0.2 m hover -> CMD preparation -> stable WAITING; states={history}', flush=True)
            return
        home = far = None
        counts = [args.count] + ([args.return_count] if args.return_count else [])
        for leg, count in enumerate(counts):
            settle(1)
            # COMPLETED means the reference ended; it does not imply that the
            # measured vehicle has already settled below the replanning limit.
            def stable_hold():
                state = latest[0]
                v = state.velocity
                return (state.command_mode and state.hovering and state.feedback_valid
                        and np.linalg.norm([v.x, v.y, v.z]) < 0.08)
            until(stable_hold, 15)
            current_count[0] = count
            returning[0] = leg > 0
            previous_position[0] = None
            crossing_samples.clear()
            planner_text[0] = ''
            assert not call(planner, PlanGaps.Request(count=0)).accepted
            if home is None:
                p = latest[0].pose.position
                home = np.array([p.x, p.y, p.z])
                far = home + np.array([cfg['mission']['goal_offset_x'], 0, 0])
            response = call(planner, PlanGaps.Request(count=count))
            assert response.accepted, response.message
            until(lambda: latest[0].state in ('READY', 'REJECTED') or planner_text[0].startswith('FAILED'), 130)
            assert latest[0].state == 'READY', (latest[0].reason, planner_text[0])
            assert selected_count[0] == count
            print(planner_text[0], flush=True)
            assert not call(upload, UploadTrajectory.Request()).accepted
            settle(0.5)
            assert latest[0].command_mode and latest[0].hovering and latest[0].state == 'READY'
            assert call(start, Trigger.Request()).success
            until(lambda: latest[0].state == 'EXECUTING', 5)
            assert not call(planner, PlanGaps.Request(count=1)).accepted
            assert not call(configure, ConfigureGates.Request(count=0)).accepted
            until(lambda: latest[0].state == 'COMPLETED' and latest[0].hovering, 60)
            assert latest[0].command_mode, 'Completed leg must stay in CMD'
            settle(0.5)
            p = latest[0].pose.position
            goal = home if leg else far
            error = np.linalg.norm(np.array([p.x, p.y, p.z])-goal)
            assert error < 0.2, f'Terminal tracking error {error:.3f} m'
            assert all(distance < 0.2 for distance, _ in crossing_samples.values())
            assert len(crossing_samples) == count
            print(f'PASS leg={leg} count={count} endpoint_error={error:.3f} m crossings={crossing_samples}', flush=True)
        settle(1)
        assert latest[0].command_mode and latest[0].hovering and latest[0].state == 'COMPLETED'
        assert history.count('PREPARING') == 1, 'Repeated uploaded legs must not repeat preparation'
        assert 'GAP COLLISION' not in Path(args.log).read_text(), 'MuJoCo detected frame contact'
        print(f'PASS mission states={history}', flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        os.killpg(process.pid, signal.SIGINT)
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(process.pid, signal.SIGTERM)
            process.wait(timeout=5)
        stream.close()
        test_directory.cleanup()


if __name__ == '__main__':
    main()
