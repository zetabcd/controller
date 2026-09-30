"""Check the published IMU frame against a tilted MuJoCo accelerometer."""
from types import SimpleNamespace

import mujoco
import numpy as np
import pytest

from quadsim_mujoco.quadsim_node import QuadSimNode


@pytest.mark.parametrize('pitch', [0.0, 0.7, 2.4])
def test_accelerometer_is_body_specific_force(pitch):
    model = mujoco.MjModel.from_xml_string('''
        <mujoco><option gravity="0 0 -9.805"/>
        <worldbody><body pos="0 0 2"><freejoint/>
        <geom type="sphere" size="0.1" mass="1"/>
        <site name="imu"/></body></worldbody>
        <sensor><accelerometer name="acc" site="imu"/></sensor></mujoco>
    ''')
    data = mujoco.MjData(model)
    data.qpos[3:7] = [np.cos(pitch/2), 0, np.sin(pitch/2), 0]
    # Balance gravity in world coordinates, including for an inverted body.
    data.xfrc_applied[1, 2] = 9.805
    mujoco.mj_forward(model, data)
    rotation = data.xmat[1].reshape(3, 3)
    body_force = data.sensor('acc').data.copy()
    np.testing.assert_allclose(rotation @ body_force, [0, 0, 9.805], atol=1e-10)
    messages = []
    node = SimpleNamespace(sensor_combined_publisher=SimpleNamespace(publish=messages.append))
    quad = SimpleNamespace(
        now_time=1.0,
        state=SimpleNamespace(omega=np.zeros(3), acc_B=body_force, acc=rotation @ body_force),
        param=SimpleNamespace(noise=SimpleNamespace(gyro_is_valid=False, accel_is_valid=False)))
    QuadSimNode.sens_topic_pub(node, quad)
    # SensorCombined uses FRD; transform back to FLU as the real controller does.
    received = np.array(messages[0].accelerometer_m_s2) * [1, -1, -1]
    np.testing.assert_allclose(rotation @ received + [0, 0, -9.805], 0, atol=1e-6)
