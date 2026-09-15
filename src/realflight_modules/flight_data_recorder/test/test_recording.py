"""Round-trip actual generated ROS messages through the independent pyulog reader."""

import json
import struct

import numpy as np
import pytest
from pyulog import ULog
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.serialization import deserialize_message, serialize_message
from rosidl_runtime_py.utilities import get_message
from std_msgs.msg import Byte, Float64MultiArray, String, UInt64
from px4_msgs.msg import ActuatorMotors, UlogStream, VehicleStatus
from px4debug_msgs.msg import Px4ctrlDebug

from flight_data_recorder.capture import Capture
from flight_data_recorder.codec import RosCodec, Stamp
from flight_data_recorder.node import CORE_TOPICS
from flight_data_recorder.ulog import ULogWriter, assemble, packet, read_packets


def codec(topic, cls):
    return RosCodec(topic, cls.__module__.split('.')[0] + '/msg/' + cls.__name__, cls)


def read(path):
    log = ULog(str(path))
    assert not log.file_corruption
    return log


def send(capture, source, message, n, state=None):
    capture.receive(source, serialize_message(message), 1000000000 + n * 1000000,
                    1800000000000000000 - n * 1000000, state)


def saved(capture):
    capture.queue.join()
    results = []
    while not capture.results.empty():
        results.append(capture.results.get_nowait())
    return results


def test_all_core_fields_and_fixed_arrays(tmp_path):
    writer = ULogWriter(tmp_path, 1000000000, {})
    expected = {}
    for topic, name in CORE_TOPICS.items():
        cls = get_message(name)
        message = cls()
        # Non-default values expose array omissions, signs and integer narrowing.
        for field, type_name in cls.get_fields_and_field_types().items():
            if type_name in ('float', 'double'):
                setattr(message, field, -1.25)
            elif type_name.startswith('float['):
                setattr(message, field, [float(i) - 2.5 for i in range(len(getattr(message, field)))])
            elif type_name == 'uint64':
                setattr(message, field, 2**63 + 123)
            elif type_name == 'boolean':
                setattr(message, field, True)
        message = deserialize_message(serialize_message(message), cls)
        source = RosCodec(topic, name, cls)
        source.encode(message, Stamp(1000000000, 1800000000000000000, 1), writer)
        expected[source.dataset] = message
    log = read(writer.finish({}))
    for name, message in expected.items():
        data = log.get_dataset(name).data
        for field, type_name in message.get_fields_and_field_types().items():
            value = getattr(message, field)
            if type_name in ('float', 'double', 'boolean') or type_name.startswith(('int', 'uint')):
                assert data['msg_' + field][0] == value
            elif type_name.startswith('float['):
                for i, item in enumerate(value):
                    assert data[f'msg_{field}[{i}]'][0] == item
    assert log.get_dataset('ros/debugPx4/ctrl').data['msg_thr2acc'][0] == -1.25
    assert 'msg_control[11]' in log.get_dataset('ros/fmu/in/actuator_motors').data


def test_arm_disarm_rearm_and_source_clock_jump(tmp_path):
    status = codec('/fmu/out/vehicle_status_v1', VehicleStatus)
    debug = codec('/debugPx4/ctrl', Px4ctrlDebug)
    capture = Capture(tmp_path, deserialize_message)
    try:
        send(capture, debug, Px4ctrlDebug(thr2acc=99.0), 0)
        send(capture, status, VehicleStatus(arming_state=1), 1, 1)
        assert not list(tmp_path.iterdir())
        for base, values in ((10, [1.0, 1.25, 1.5]), (20, [2.0, 2.5])):
            send(capture, status, VehicleStatus(arming_state=2), base, 2)
            for i, value in enumerate(values):
                send(capture, debug, Px4ctrlDebug(thr2acc=value), base + 1 + i)
            send(capture, status, VehicleStatus(arming_state=1), base + 5, 1)
            send(capture, debug, Px4ctrlDebug(thr2acc=99.0), base + 6)
        results = saved(capture)
        assert len(results) == 2
        for result, expected in zip(results, ([1.0, 1.25, 1.5], [2.0, 2.5])):
            assert not result['incomplete']
            assert result['summary']['received'] == result['summary']['written']
            log = read(result['path'])
            data = log.get_dataset(debug.dataset).data
            assert data['msg_thr2acc'].tolist() == expected
            assert np.all(np.diff(data['timestamp']) > 0)
            assert np.all(np.diff(data['recorder_ros_time_ns']) < 0)
            assert log.get_dataset(status.dataset).data['msg_arming_state'].tolist() == [2, 1]
        assert not list(tmp_path.glob('*.pending'))
    finally:
        capture.close()


def test_dynamic_topics_omit_child_data_and_keep_nested_numeric_fields(tmp_path):
    writer = ULogWriter(tmp_path, 1000000000, {})
    ctrl = codec('/debugPx4/ctrl', Px4ctrlDebug)
    ctrl.encode(Px4ctrlDebug(thr2acc=1.5), Stamp(1000000000, 1, 1), writer)
    # These formats appear only after data already exists in the spool.
    strings = codec('/late/string', String)
    text = '推力\x00加速度😀' * 10000
    strings.encode(String(data=text), Stamp(1001000000, 2, 1), writer)
    arrays = codec('/late/array', Float64MultiArray)
    numbers = [i * -0.125 for i in range(263)]
    arrays.encode(Float64MultiArray(data=numbers), Stamp(1002000000, 3, 1), writer)
    arrays.encode(Float64MultiArray(data=[]), Stamp(1003000000, 4, 2), writer)
    poses = codec('/late/poses', PoseArray)
    poses.encode(PoseArray(poses=[Pose(), Pose()]), Stamp(1004000000, 5, 1), writer)
    poses.encode(PoseArray(), Stamp(1005000000, 6, 2), writer)
    pose = PoseStamped()
    pose.header.frame_id = 'world'
    pose.header.stamp.sec = 42
    pose.pose.position.x = 1.25
    pose.pose.orientation.w = 1.0
    codec('/late/pose', PoseStamped).encode(pose, Stamp(1006000000, 7, 1), writer)
    stream = UlogStream(timestamp=123, length=249, data=list(range(249)))
    codec('/late/stream', UlogStream).encode(stream, Stamp(1007000000, 8, 1), writer)
    path = writer.finish({})
    log = read(path)
    assert {entry.name for entry in log.data_list} == {
        ctrl.dataset, strings.dataset, arrays.dataset, poses.dataset,
        'ros/late/pose', 'ros/late/stream',
    }
    assert log.get_dataset(ctrl.dataset).data['msg_thr2acc'].tolist() == [1.5]
    assert not any(key.startswith('msg_') for key in log.get_dataset(strings.dataset).data)
    array_data = log.get_dataset(arrays.dataset).data
    assert array_data['msg_layout__data_offset'].tolist() == [0, 0]
    assert not any(key.startswith('msg_data') for key in array_data)
    pose_data = log.get_dataset('ros/late/pose').data
    assert pose_data['msg_header__stamp__sec'].tolist() == [42]
    assert pose_data['msg_pose__position__x'].tolist() == [1.25]
    assert pose_data['msg_pose__orientation__w'].tolist() == [1.0]
    assert 'msg_header__frame_id' not in pose_data
    assert 'msg_poses_length' not in log.get_dataset(poses.dataset).data
    stream_data = log.get_dataset('ros/late/stream').data
    assert stream_data['msg_timestamp'].tolist() == [123]
    assert stream_data['msg_length'].tolist() == [249]
    assert not any(key.startswith('msg_data') for key in stream_data)
    mappings = [json.loads(value) for key, value in log.msg_info_dict.items()
                if key.startswith('dataset_')]
    omitted = {entry['topic']: [field['path'] for field in entry['omitted_fields']]
               for entry in mappings}
    assert omitted['/late/string'] == [['data']]
    assert omitted['/late/poses'] == [['header', 'frame_id'], ['poses']]
    assert omitted['/late/stream'] == [['data']]
    assert log.msg_info_dict['recorder_schema_version'] == '2'
    with path.open('rb') as stream:
        stream.read(16)
        kinds = [kind for kind, _, _ in read_packets(stream)]
    assert max(i for i, kind in enumerate(kinds) if kind == 'F') < kinds.index('A')


def test_octet_and_uint64_remain_exact(tmp_path):
    writer = ULogWriter(tmp_path, 1_000_000, {})
    codec('/byte', Byte).encode(Byte(data=b'\xff'), Stamp(1_000_000, 1, 1), writer)
    codec('/counter', UInt64).encode(UInt64(data=2**64 - 1), Stamp(1_000_000, 1, 1), writer)
    log = read(writer.finish({}))
    assert log.get_dataset('ros/byte').data['msg_data'][0] == 255
    assert int(log.get_dataset('ros/counter').data['msg_data'][0]) == 2**64 - 1


def test_queue_overflow_is_reported_and_disarm_is_kept(tmp_path):
    capture = Capture(tmp_path, deserialize_message, max_queue_bytes=1)
    status = codec('/status', VehicleStatus)
    try:
        send(capture, status, VehicleStatus(arming_state=2), 1, 2)
        send(capture, codec('/debug', Px4ctrlDebug), Px4ctrlDebug(), 2)
        send(capture, status, VehicleStatus(arming_state=1), 3, 1)
        result, = saved(capture)
        assert result['incomplete']
        assert result['path'].endswith('.incomplete.ulg')
        assert result['summary']['queue_dropped'] == {'ros/debug': 1}
        assert read(result['path']).get_dataset('ros/status').data['msg_arming_state'].tolist() == [2, 1]
    finally:
        capture.close()


def test_shutdown_while_armed_is_incomplete(tmp_path):
    capture = Capture(tmp_path, deserialize_message)
    send(capture, codec('/status', VehicleStatus), VehicleStatus(arming_state=2), 1, 2)
    capture.close()
    result, = saved(capture)
    assert result['incomplete']
    assert result['summary']['reason'] == 'node_shutdown_while_armed'
    read(result['path'])


def test_simulation_records_without_arming_and_ignores_state_transitions(tmp_path):
    capture = Capture(tmp_path, deserialize_message, simulation_mode=True)
    debug = codec('/debug', Px4ctrlDebug)
    status = codec('/status', VehicleStatus)
    try:
        capture.start_simulation(1000000000)
        assert capture.active
        send(capture, debug, Px4ctrlDebug(thr2acc=1.0), 1)
        for n, state in enumerate((1, 2, 1), start=2):
            send(capture, status, VehicleStatus(arming_state=state), 2 * n, state)
            send(capture, debug, Px4ctrlDebug(thr2acc=float(n)), 2 * n + 1)
            assert capture.active
        capture.close()
        result, = saved(capture)
        assert not result['incomplete']
        assert result['summary']['reason'] == 'simulation_shutdown'
        log = read(result['path'])
        assert log.get_dataset('ros/debug').data['msg_thr2acc'].tolist() == [1.0, 2.0, 3.0, 4.0]
        assert log.get_dataset('ros/status').data['msg_arming_state'].tolist() == [1, 2, 1]
        assert json.loads(log.msg_info_dict['recorder_metadata'])['simulation_mode'] is True
        assert len(list(tmp_path.glob('*.ulg'))) == 1
        assert not list(tmp_path.glob('*.pending'))
    finally:
        capture.close()


@pytest.mark.parametrize('overflow', [False, True])
def test_simulation_empty_session_and_overflow_are_distinguished(tmp_path, overflow):
    capture = Capture(tmp_path, deserialize_message, simulation_mode=True, max_queue_bytes=1)
    try:
        capture.start_simulation(1000000000)
        if overflow:
            send(capture, codec('/debug', Px4ctrlDebug), Px4ctrlDebug(), 1)
        capture.close()
        result, = saved(capture)
        assert result['incomplete'] is overflow
        assert bool(result['summary']['queue_dropped']) is overflow
        read(result['path'])
    finally:
        capture.close()


def test_decode_failure_is_visible_and_worker_survives(tmp_path):
    capture = Capture(tmp_path, deserialize_message)
    status = codec('/status', VehicleStatus)
    try:
        send(capture, status, VehicleStatus(arming_state=2), 1, 2)
        capture.receive(codec('/bad', String), b'bad cdr', 1000001, 2)
        send(capture, status, VehicleStatus(arming_state=1), 3, 1)
        results = saved(capture)
        assert 'error' in results[0]
        assert results[-1]['incomplete']
        assert results[-1]['summary']['writer_errors']
        read(results[-1]['path'])
    finally:
        capture.close()


def test_disk_creation_failure_is_visible(tmp_path):
    blocked = tmp_path / 'file'
    blocked.write_text('not a directory')
    capture = Capture(blocked, deserialize_message)
    status = codec('/status', VehicleStatus)
    try:
        send(capture, status, VehicleStatus(arming_state=2), 1, 2)
        send(capture, status, VehicleStatus(arming_state=1), 2, 1)
        results = saved(capture)
        assert 'error' in results[0]
        assert results[-1]['incomplete'] and results[-1]['path'] is None
    finally:
        capture.close()


def test_recovery_rejects_live_writer_and_handles_partial_packets(tmp_path):
    writer = ULogWriter(tmp_path, 1_000_000, {})
    codec('/debug', Px4ctrlDebug).encode(Px4ctrlDebug(thr2acc=1.25), Stamp(1_000_000, 1, 1), writer)
    writer.sync()
    with pytest.raises(BlockingIOError):
        assemble(writer.pending, recovered=True)
    writer.abandon()
    with (writer.pending / 'definitions.bin').open('ab') as stream:
        stream.write(packet('F', b'unfinished:uint64_t timestamp;')[:-4])
    with (writer.pending / 'data.bin').open('ab') as stream:
        stream.write(packet('A', struct.pack('<BH', 0, 44) + b'unfinished'))
        stream.write(packet('D', struct.pack('<HQ', 44, 1001)))
        stream.write(b'\x20\x00Dpartial')
    path = assemble(writer.pending, recovered=True)
    log = read(path)
    assert path.name.endswith('.recovered.ulg')
    assert log.get_dataset('ros/debug').data['msg_thr2acc'][0] == 1.25
    assert 'recorder_recovered' in log.msg_info_dict


def test_long_metadata_and_original_ros_definitions(tmp_path):
    writer = ULogWriter(tmp_path, 1000000, {})
    text = '长字符串' * 20000
    writer.info('large_metadata', text)
    codec('/motors', ActuatorMotors).encode(ActuatorMotors(), Stamp(1000000, 1, 1), writer)
    log = read(writer.finish({}))
    assert ''.join(log.msg_info_multiple_dict['large_metadata'][0]) == text
    descriptions = [json.loads(value) for key, value in log.msg_info_dict.items() if key.startswith('ros_type_')]
    assert descriptions[0]['definitions']['px4_msgs/msg/ActuatorMotors']['control'] == 'float[12]'


def test_topic_names_containing_chunks_or_fields_are_preserved(tmp_path):
    writer = ULogWriter(tmp_path, 1000000, {})
    codec('/text', String).encode(String(data='complete'), Stamp(1000000, 1, 1), writer)
    codec('/text/data/chunks', UInt64).encode(UInt64(data=123), Stamp(1000000, 1, 1), writer)
    codec('/text/fields', UInt64).encode(UInt64(data=456), Stamp(1000000, 1, 1), writer)
    log = read(writer.finish({}))
    assert {entry.name for entry in log.data_list} == {
        'ros/text', 'ros/text/data/chunks', 'ros/text/fields',
    }
    assert log.get_dataset('ros/text/data/chunks').data['msg_data'][0] == 123
    assert log.get_dataset('ros/text/fields').data['msg_data'][0] == 456
