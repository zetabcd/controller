"""ROS 2 entry point: ARM gating by default, continuous capture for simulation."""

import hashlib
import json
import math
import queue
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rclpy.qos_event import SubscriptionEventCallbacks, UnsupportedEventTypeError
from rclpy.serialization import deserialize_message
from rcl_interfaces.msg import ParameterDescriptor
from rosidl_runtime_py.utilities import get_message
from uav_utils.project_paths import project_path

from .capture import Capture
from .codec import RosCodec


EXCLUDED_TOPICS = [
    '/joyStick', '/realflight/trajectory_markers', '/omtraj/markers',
    '/minco/markers', '/minco/trajectory', '/tf', '/tf_static', '/clock', '/rosout',
    '/parameter_events',
]
CORE_TOPICS = {
    '/fmu/out/vehicle_status_v1': 'px4_msgs/msg/VehicleStatus',
    '/fmu/out/vehicle_local_position': 'px4_msgs/msg/VehicleLocalPosition',
    '/fmu/out/vehicle_attitude': 'px4_msgs/msg/VehicleAttitude',
    '/fmu/out/sensor_combined': 'px4_msgs/msg/SensorCombined',
    '/fmu/out/battery_status': 'px4_msgs/msg/BatteryStatus',
    '/fmu/out/manual_control_setpoint': 'px4_msgs/msg/ManualControlSetpoint',
    '/fmu/out/vehicle_command_ack': 'px4_msgs/msg/VehicleCommandAck',
    '/fmu/in/vehicle_command': 'px4_msgs/msg/VehicleCommand',
    '/fmu/in/actuator_motors': 'px4_msgs/msg/ActuatorMotors',
    '/fmu/in/offboard_control_mode': 'px4_msgs/msg/OffboardControlMode',
    '/fmu/in/trajectory_setpoint': 'px4_msgs/msg/TrajectorySetpoint',
    '/rates_thrust_setpoint': 'ratectrl_msgs/msg/RatesThrustSetpoint',
    '/debugPx4/ctrl': 'px4debug_msgs/msg/Px4ctrlDebug',
    '/debugPx4/ratectrl': 'px4debug_msgs/msg/Px4ratectrlDebug',
}


class FlightDataRecorder(Node):
    def __init__(self):
        super().__init__('flight_data_recorder')
        output = self.declare_parameter('output_directory', 'datalog/flightlog').value
        self.output_directory = project_path(
            output or 'datalog/flightlog', get_package_share_directory('flight_data_recorder'))
        self.simulation_mode = self.declare_parameter(
            'simulation_mode', False, ParameterDescriptor(
                read_only=True,
                description='Record from startup until shutdown, independent of arming state.')).value
        self.status_topic = self.declare_parameter('status_topic', '/fmu/out/vehicle_status_v1').value
        self.excluded = set(self.declare_parameter('excluded_topics', EXCLUDED_TOPICS).value)
        if not self.simulation_mode and self.status_topic in self.excluded:
            raise ValueError('status_topic cannot be excluded: it controls recording')
        self.depth = int(self.declare_parameter('subscription_depth', 4096).value)
        interval = float(self.declare_parameter('discovery_interval_s', 0.5).value)
        sync_interval = float(self.declare_parameter('sync_interval_s', 1.0).value)
        max_bytes = int(self.declare_parameter('max_queue_bytes', 67108864).value)
        if self.depth < 1 or max_bytes < 1 or not all(
                math.isfinite(x) and x > 0 for x in (interval, sync_interval)):
            raise ValueError('depth, queue size and timer intervals must be positive and finite')
        self.auto_discover = bool(self.declare_parameter('auto_discover', True).value)
        self.sources = {}
        self.unavailable = {}
        self._reported = set()
        self.last_status_ns = None
        self._last_wait_warning = time.monotonic()
        self.capture = Capture(
            self.output_directory, deserialize_message,
            metadata={'excluded_topics': sorted(self.excluded), 'status_topic': self.status_topic,
                      'time_axis': 'monotonic microseconds; original source fields unchanged',
                      'coordinates': 'source message coordinates, no conversion',
                      'field_policy': 'primitives and fixed primitive arrays up to 128 elements; '
                                      'nested messages flattened; no child datasets',
                      'subscription_depth': self.depth,
                      'max_queue_bytes': max_bytes, 'sync_interval_s': sync_interval},
            max_queue_bytes=max_bytes, sync_interval=sync_interval,
            simulation_mode=self.simulation_mode)
        try:
            topics = dict(CORE_TOPICS)
            topics[self.status_topic] = 'px4_msgs/msg/VehicleStatus'
            for topic, type_name in topics.items():
                if topic not in self.excluded:
                    self._subscribe(topic, type_name)
            if not self.simulation_mode and (
                    self.status_topic, 'px4_msgs/msg/VehicleStatus') not in self.sources:
                raise RuntimeError('Unable to subscribe to the required VehicleStatus arming source')
            self._discover()
            # Topic discovery and disk-error reporting must also run when the
            # simulator's ROS clock has not started or is paused.
            self._housekeeping_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.timer = self.create_timer(interval, self._poll, clock=self._housekeeping_clock)
            self.capture.metadata['unavailable_topics'] = dict(self.unavailable)
            if self.simulation_mode:
                self.capture.start_simulation(time.monotonic_ns())
        except BaseException:
            self.capture.close()
            raise
        mode = ('ULog recorder simulation mode: recording from startup until shutdown'
                if self.simulation_mode else f'ULog recorder waiting for ARMED on {self.status_topic}')
        self.get_logger().info(
            f'{mode}; output={self.output_directory}; '
            f'excluding {len(self.excluded)} topics. Recording numeric fields at received rates; '
            'strings, sequences and fields requiring child datasets are omitted.')

    def _subscribe(self, topic, type_name):
        key = (topic, type_name)
        if key in self.sources or topic in self.excluded:
            return
        try:
            cls = get_message(type_name)
            dataset = 'ros' + topic
            if any(name == topic for name, _ in self.sources):
                dataset += '.type_' + hashlib.sha256(type_name.encode()).hexdigest()[:12]
            codec = RosCodec(topic, type_name, cls, dataset)
            # Volatile requests collect live samples without replaying retained
            # pre-arm values. BEST_EFFORT matches both reliable and BE publishers.
            # Use reliable when all currently known publishers offer it.
            endpoints = [p for p in self.get_publishers_info_by_topic(topic) if p.topic_type == type_name]
            reliable = bool(endpoints) and all(
                p.qos_profile.reliability == ReliabilityPolicy.RELIABLE for p in endpoints)
            qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=self.depth,
                             reliability=ReliabilityPolicy.RELIABLE if reliable else ReliabilityPolicy.BEST_EFFORT,
                             durability=DurabilityPolicy.VOLATILE)
            def callback(raw):
                self._receive(codec, raw)
            events = SubscriptionEventCallbacks(
                incompatible_qos=lambda event, t=topic: self._problem(
                    f'Incompatible QoS on {t}: policy={event.last_policy_kind}'))
            try:
                subscription = self.create_subscription(cls, topic, callback, qos, raw=True,
                                                        event_callbacks=events)
            except UnsupportedEventTypeError:
                self._problem(f'RMW lacks subscription loss events for {topic}; loss reporting unavailable')
                subscription = self.create_subscription(cls, topic, callback, qos, raw=True,
                                                        event_callbacks=SubscriptionEventCallbacks(use_default_callbacks=False))
            self.sources[key] = (subscription, codec, reliable)
            self.unavailable.pop(topic + ' ' + type_name, None)
            self.get_logger().info(f'Recording source registered: {topic} [{type_name}]')
        except Exception as error:
            description = f'{type(error).__name__}: {error}'
            self.unavailable[topic + ' ' + type_name] = description
            self._problem(f'Cannot record {topic} [{type_name}]: {description}')

    def _problem(self, text):
        self.capture.problem(text)
        if text not in self._reported:
            self.get_logger().error(text)
            self._reported.add(text)

    def _receive(self, codec, raw):
        steady = time.monotonic_ns()
        ros_ns = self.get_clock().now().nanoseconds
        state = None
        if (not self.simulation_mode and codec.topic == self.status_topic and
                codec.type_name == 'px4_msgs/msg/VehicleStatus'):
            try:
                message = deserialize_message(raw, codec.message_class)
                state = int(message.arming_state)
                self.last_status_ns = steady
            except Exception as error:
                self._problem(f'Cannot decode arming state: {error}')
                return
        was_active = self.capture.active
        self.capture.metadata['unavailable_topics'] = dict(self.unavailable)
        self.capture.receive(codec, raw, steady, ros_ns, state)
        if not was_active and self.capture.active:
            self.get_logger().info('ARMED: started a new ULog session')
        elif was_active and not self.capture.active:
            self.get_logger().info('DISARMED: finalizing ULog and syncing to disk')

    def _discover(self):
        if not self.auto_discover:
            return
        for topic, types in self.get_topic_names_and_types():
            if topic in self.excluded:
                continue
            # The graph also contains this recorder's pre-created subscriptions;
            # only publishers determine additional sources worth subscribing to.
            publishers = self.get_publishers_info_by_topic(topic)
            for type_name in types:
                endpoints = [p for p in publishers if p.topic_type == type_name]
                if not endpoints:
                    continue
                key = (topic, type_name)
                old = self.sources.get(key)
                if old and old[2] and any(
                        p.qos_profile.reliability != ReliabilityPolicy.RELIABLE for p in endpoints):
                    self._problem(f'Publisher QoS changed on {topic}; resubscribing with BEST_EFFORT')
                    self.destroy_subscription(old[0])
                    del self.sources[key]
                self._subscribe(topic, type_name)

    def _poll_results(self):
        while True:
            try:
                result = self.capture.results.get_nowait()
            except queue.Empty:
                break
            if 'error' in result:
                # The worker already assigns errors to the correct session.
                # Polling may happen after rearming, so do not mark a new flight.
                self.get_logger().error(
                    f'ULog writer failed: {result["error"]}; pending={result["pending"]}')
            elif result['path'] and not result['incomplete']:
                self.get_logger().info(f'ULog saved and synced: {result["path"]}')
            else:
                self.get_logger().error('Incomplete ULog session: ' + json.dumps(result, ensure_ascii=False))

    def _poll(self):
        self._discover()
        self._poll_results()
        now = time.monotonic()
        if not self.simulation_mode and now - self._last_wait_warning > 10.0:
            self._last_wait_warning = now
            if self.last_status_ns is None or now - self.last_status_ns / 1e9 > 10:
                self.get_logger().warning(
                    f'No recent VehicleStatus on {self.status_topic}; cannot infer arm/disarm from '
                    'FSM or throttle. For simulation without VehicleStatus, enable simulation_mode.')
        if self.capture.active and self.capture.dropped:
            self._problem('Recorder queue overflow; this session will be marked incomplete')

    def destroy_node(self):
        if hasattr(self, 'capture'):
            self.capture.close()
            self._poll_results()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = FlightDataRecorder()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
