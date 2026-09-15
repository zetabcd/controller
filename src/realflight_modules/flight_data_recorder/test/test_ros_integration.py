"""Opt-in DDS integration: run in an isolated ROS_DOMAIN_ID, without controllers."""

import os
import json
from pathlib import Path
import signal
import subprocess
import time

import numpy as np
import pytest
import yaml
from ament_index_python.packages import get_package_prefix, get_package_share_directory
from pyulog import ULog
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Float64, String
from rcl_interfaces.msg import Log, Parameter, ParameterEvent, ParameterValue
from px4_msgs.msg import ActuatorMotors, VehicleStatus
from px4debug_msgs.msg import Px4ctrlDebug

from flight_data_recorder.node import EXCLUDED_TOPICS, FlightDataRecorder


@pytest.mark.skipif(os.environ.get('FLIGHT_RECORDER_ROS_TEST') != '1',
                    reason='requires opt-in and an isolated ROS_DOMAIN_ID')
def test_live_discovery_full_rate_exclusions_and_two_flights(tmp_path):
    assert 200 <= int(os.environ['ROS_DOMAIN_ID']) <= 232
    rclpy.init(args=['--ros-args', '-p', f'output_directory:={tmp_path}',
                     '-p', 'discovery_interval_s:=0.05', '--log-level', 'warn'])
    recorder = FlightDataRecorder()
    publisher = Node('recorder_integration_publisher')
    executor = SingleThreadedExecutor()
    executor.add_node(recorder)
    executor.add_node(publisher)
    status = publisher.create_publisher(VehicleStatus, recorder.status_topic, qos_profile_sensor_data)
    debug = publisher.create_publisher(Px4ctrlDebug, '/debugPx4/ctrl', qos_profile_sensor_data)
    motors = publisher.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', qos_profile_sensor_data)

    def until(condition, timeout=10.0):
        deadline = time.monotonic() + timeout
        while not condition():
            assert time.monotonic() < deadline, 'Timed out waiting for DDS/recorder'
            executor.spin_once(timeout_sec=0.002)

    def drain(duration=0.1):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.002)

    try:
        excluded = []
        for topic in EXCLUDED_TOPICS:
            if topic == '/parameter_events':
                message = ParameterEvent(node=publisher.get_fully_qualified_name(), changed_parameters=[
                    Parameter(name='gain', value=ParameterValue(type=3, double_value=2.5)),
                ])
            elif topic == '/rosout':
                message = Log(msg='must not be recorded')
            else:
                message = String(data='must not be recorded')
            excluded.append((publisher.create_publisher(type(message), topic, 10), message))
        until(lambda: all(p.get_subscription_count() for p in (status, debug, motors)))
        status.publish(VehicleStatus(arming_state=1))
        debug.publish(Px4ctrlDebug(thr2acc=99.0))
        drain()
        assert not list(tmp_path.glob('*.ulg*'))
        status.publish(VehicleStatus(arming_state=2))
        until(lambda: recorder.capture.active)
        # Topic first appears while already recording, with a reliable publisher.
        late = publisher.create_publisher(Float64, '/recorder_test/late', 10)
        until(lambda: late.get_subscription_count() > 0)
        late.publish(Float64(data=3.25))
        for pub, message in excluded:
            pub.publish(message)
        samples = 400
        start = time.monotonic()
        for i in range(samples):
            debug.publish(Px4ctrlDebug(timestamp=i, thr2acc=1.0 + i / 1024.0))
            motors.publish(ActuatorMotors(timestamp=i, control=[i / 1024.0 + j / 32.0 for j in range(12)]))
            deadline = start + (i + 1) / 400.0
            while time.monotonic() < deadline:
                executor.spin_once(timeout_sec=0.0005)
        until(lambda: recorder.capture.received['ros/debugPx4/ctrl'] == samples and
              recorder.capture.received['ros/fmu/in/actuator_motors'] == samples)
        status.publish(VehicleStatus(arming_state=1))
        until(lambda: not recorder.capture.active)
        until(lambda: len(list(tmp_path.glob('*.ulg'))) == 1)
        first, = tmp_path.glob('*.ulg')
        log = ULog(str(first))
        assert not log.file_corruption and not first.name.endswith('.incomplete.ulg')
        data = log.get_dataset('ros/debugPx4/ctrl').data
        np.testing.assert_array_equal(data['msg_thr2acc'], [1.0 + i / 1024.0 for i in range(samples)])
        motor_data = log.get_dataset('ros/fmu/in/actuator_motors').data
        for j in range(12):
            np.testing.assert_array_equal(motor_data[f'msg_control[{j}]'],
                                          [i / 1024.0 + j / 32.0 for i in range(samples)])
        assert log.get_dataset('ros/recorder_test/late').data['msg_data'].tolist() == [3.25]
        assert not {entry.name for entry in log.data_list} & {'ros' + t for t in EXCLUDED_TOPICS}
        assert not any('.fields/' in entry.name or '.chunks/' in entry.name for entry in log.data_list)
        assert 'ros/parameter_events' not in recorder.capture.received
        assert not {topic for topic, _ in recorder.sources} & set(EXCLUDED_TOPICS)
        debug.publish(Px4ctrlDebug(thr2acc=99.0))
        drain()
        status.publish(VehicleStatus(arming_state=2))
        until(lambda: recorder.capture.active)
        debug.publish(Px4ctrlDebug(thr2acc=2.0))
        until(lambda: recorder.capture.received['ros/debugPx4/ctrl'] == 1)
        status.publish(VehicleStatus(arming_state=1))
        until(lambda: not recorder.capture.active)
        until(lambda: len(list(tmp_path.glob('*.ulg'))) == 2)
        second, = set(tmp_path.glob('*.ulg')) - {first}
        assert ULog(str(second)).get_dataset('ros/debugPx4/ctrl').data['msg_thr2acc'].tolist() == [2.0]
        until(lambda: not list(tmp_path.glob('*.pending')))
        print(f'PLOTJUGGLER_FIXTURE={first}')
    finally:
        recorder.destroy_node()
        publisher.destroy_node()
        executor.shutdown()
        rclpy.shutdown()


def simulation_yaml(tmp_path):
    share = Path(get_package_share_directory('flight_data_recorder'))
    config = yaml.safe_load((share / 'config' / 'recorder.yaml').read_text())
    config['flight_data_recorder']['ros__parameters'].update({
        'simulation_mode': True, 'output_directory': str(tmp_path / 'logs'),
        'use_sim_time': True, 'discovery_interval_s': 0.05, 'sync_interval_s': 0.05,
    })
    config_path = tmp_path / 'simulation.yaml'
    config_path.write_text(yaml.safe_dump(config))
    return config_path


@pytest.mark.skipif(os.environ.get('FLIGHT_RECORDER_ROS_TEST') != '1',
                    reason='requires opt-in and an isolated ROS_DOMAIN_ID')
def test_simulation_yaml_records_without_status_or_clock(tmp_path):
    assert 200 <= int(os.environ['ROS_DOMAIN_ID']) <= 232
    config_path = simulation_yaml(tmp_path)
    rclpy.init(args=['--ros-args', '--params-file', str(config_path), '--log-level', 'warn'])
    recorder = FlightDataRecorder()
    publisher = Node('recorder_simulation_test')
    executor = SingleThreadedExecutor()
    executor.add_node(recorder)
    executor.add_node(publisher)

    def until(condition, timeout=10.0):
        deadline = time.monotonic() + timeout
        while not condition():
            assert time.monotonic() < deadline, 'Timed out waiting for simulation data'
            executor.spin_once(timeout_sec=0.005)

    try:
        assert recorder.capture.active
        assert '/parameter_events' in recorder.excluded
        assert not publisher.get_publishers_info_by_topic('/fmu/out/vehicle_status_v1')
        assert not publisher.get_publishers_info_by_topic('/clock')
        # A paused ROS clock must not prevent discovering new simulation data.
        source = publisher.create_publisher(Float64, '/recorder_test/simulation', 10)
        until(lambda: source.get_subscription_count() > 0)
        for i in range(10):
            source.publish(Float64(data=i / 4.0))
            until(lambda: recorder.capture.received['ros/recorder_test/simulation'] == i + 1)
        assert not list((tmp_path / 'logs').glob('*.ulg'))
    finally:
        recorder.destroy_node()
        publisher.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
    path, = (tmp_path / 'logs').glob('*.ulg')
    assert not path.name.endswith('.incomplete.ulg')
    log = ULog(str(path))
    assert not log.file_corruption
    assert 'ros/parameter_events' not in {entry.name for entry in log.data_list}
    data = log.get_dataset('ros/recorder_test/simulation').data
    assert data['msg_data'].tolist() == [i / 4.0 for i in range(10)]
    assert data['recorder_ros_time_ns'].tolist() == [0] * 10
    assert json.loads(log.msg_info_dict['recorder_summary'])['reason'] == 'simulation_shutdown'
    assert not list((tmp_path / 'logs').glob('*.pending'))


@pytest.mark.skipif(os.environ.get('FLIGHT_RECORDER_ROS_TEST') != '1',
                    reason='requires opt-in and an isolated ROS_DOMAIN_ID')
def test_simulation_process_finishes_on_sigint(tmp_path):
    assert 200 <= int(os.environ['ROS_DOMAIN_ID']) <= 232
    config_path = simulation_yaml(tmp_path)
    executable = (Path(get_package_prefix('flight_data_recorder')) / 'lib' /
                  'flight_data_recorder' / 'flight_data_recorder_node')
    process = None
    log_path = tmp_path / 'node.log'

    def until(condition, timeout=10.0):
        deadline = time.monotonic() + timeout
        while not condition():
            assert process.poll() is None, log_path.read_text()
            assert time.monotonic() < deadline, log_path.read_text()
            time.sleep(0.01)

    try:
        with log_path.open('w') as stdout:
            process = subprocess.Popen(
                [str(executable), '--ros-args', '--params-file', str(config_path)],
                stdout=stdout, stderr=subprocess.STDOUT, start_new_session=True)
            until(lambda: 'simulation mode: recording from startup' in log_path.read_text())
            until(lambda: list((tmp_path / 'logs').glob('*.pending')))
            assert not list((tmp_path / 'logs').glob('*.ulg'))
            os.killpg(process.pid, signal.SIGINT)
            assert process.wait(timeout=10) == 0, log_path.read_text()
        assert 'No recent VehicleStatus' not in log_path.read_text()
        path, = (tmp_path / 'logs').glob('*.ulg')
        assert not path.name.endswith('.incomplete.ulg')
        log = ULog(str(path))
        assert not log.file_corruption
        assert json.loads(log.msg_info_dict['recorder_summary'])['reason'] == 'simulation_shutdown'
        assert not list((tmp_path / 'logs').glob('*.pending'))
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait()
