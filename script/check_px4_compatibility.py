#!/usr/bin/env python3
"""Offline PX4 1.16 interface checks; source this workspace's install first."""

import ast
import inspect
from pathlib import Path
import re
import unittest

from ament_index_python.packages import get_package_prefix
from px4_msgs import msg as messages
from px4_msgs.srv import VehicleCommand
from rclpy.serialization import deserialize_message, serialize_message


ROOT = Path(__file__).resolve().parents[1]
# None means the official definition has no MESSAGE_VERSION constant.
VERSIONS = {
    "VehicleStatus": 1,
    "VehicleAttitude": 0,
    "VehicleLocalPosition": 0,
    "VehicleControlMode": 0,
    "BatteryStatus": 0,
    "ManualControlSetpoint": 0,
    "ActuatorMotors": 0,
    "TrajectorySetpoint": 0,
    "VehicleCommand": 0,
    "VehicleCommandAck": 0,
    "VehicleAngularVelocity": 0,
    "VehicleAcceleration": None,
    "EscStatus": None,
    "SensorCombined": None,
    "OffboardControlMode": None,
}


class Px4CompatibilityTest(unittest.TestCase):
    def test_workspace_and_versions(self):
        prefix = Path(get_package_prefix("px4_msgs")).resolve()
        self.assertIn(ROOT / "install", prefix.parents,
                        f"Loaded another workspace: {prefix}; source install/local_setup.bash")
        for name, version in VERSIONS.items():
            with self.subTest(message=name):
                cls = getattr(messages, name)
                module_path = Path(inspect.getfile(cls)).resolve()
                self.assertIn(ROOT, module_path.parents,
                                f"Python loaded another workspace: {module_path}")
                self.assertEqual(getattr(cls, "MESSAGE_VERSION", None), version)
                definition = (ROOT / "src/utils/px4_msgs/msg" / (name + ".msg")).read_text()
                match = re.search(r"^uint32 MESSAGE_VERSION\s*=\s*(\d+)", definition, re.M)
                self.assertEqual(int(match[1]) if match else None, version)
        fields = messages.VehicleStatus.get_fields_and_field_types()
        self.assertNotIn("avoidance_system_required", fields)
        self.assertNotIn("avoidance_system_valid", fields)

    def test_topic_literals(self):
        expected = {}
        for name, version in VERSIONS.items():
            base = re.sub(r"(?<!^)(?=[A-Z])", "_", name).lower()
            expected[base] = base + (f"_v{version}" if version else "")
        count = 0
        for directory in ("src/realflight_modules", "src/uav_simulator"):
            for path in (ROOT / directory).rglob("*"):
                if path.suffix not in (".cpp", ".h", ".py"):
                    continue
                for topic in re.findall(r"""["']/fmu/(?:in|out)/([^"']+)["']""",
                                        path.read_text()):
                    base = re.sub(r"_v\d+$", "", topic)
                    with self.subTest(file=str(path.relative_to(ROOT)), topic=topic):
                        self.assertIn(base, expected, "Add this message to VERSIONS")
                        self.assertEqual(topic, expected[base])
                    count += 1
        self.assertGreater(count, 0)

    def test_serialization(self):
        classes = [getattr(messages, name) for name in VERSIONS]
        classes += [VehicleCommand.Request, VehicleCommand.Response]
        for cls in classes:
            with self.subTest(message=cls.__name__):
                instance = cls()
                encoded = serialize_message(instance)
                decoded = deserialize_message(encoded, cls)
                self.assertEqual(serialize_message(decoded), encoded)

    def test_simulator_message_fields(self):
        path = ROOT / "src/uav_simulator/quadsim_mujoco/quadsim_mujoco/quadsim_node.py"
        tree = ast.parse(path.read_text())
        checked = set()
        for function in ast.walk(tree):
            if not isinstance(function, ast.FunctionDef):
                continue
            for statement in function.body:
                if not (isinstance(statement, ast.Assign)
                        and isinstance(statement.value, ast.Call)
                        and isinstance(statement.value.func, ast.Name)
                        and statement.value.func.id in VERSIONS):
                    continue
                name = statement.value.func.id
                variable = statement.targets[0]
                if not isinstance(variable, ast.Name):
                    continue
                fields = getattr(messages, name).get_fields_and_field_types()
                for node in ast.walk(function):
                    if (isinstance(node, ast.Attribute)
                            and isinstance(node.value, ast.Name)
                            and node.value.id == variable.id):
                        self.assertIn(node.attr, fields, f"{name}.{node.attr}")
                checked.add(name)
        self.assertEqual(checked, {"SensorCombined", "VehicleAttitude", "VehicleLocalPosition"})


if __name__ == "__main__":
    unittest.main(verbosity=2)
