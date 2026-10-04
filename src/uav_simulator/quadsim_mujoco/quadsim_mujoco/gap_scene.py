"""Construct the same finite-thickness rigid frames used by gap_planner."""
from pathlib import Path
import xml.etree.ElementTree as ET
import numpy as np
from scipy.spatial.transform import Rotation
import yaml
import mujoco


def scene_xml(model_path, configuration):
    path = Path(model_path)
    tree = ET.parse(path)
    root = tree.getroot()
    compiler = root.find('compiler')
    if compiler is not None and compiler.get('meshdir'):
        compiler.set('meshdir', str((path.parent / compiler.get('meshdir')).resolve()))
    with open(configuration, encoding='utf-8') as stream:
        cfg = yaml.safe_load(stream)['gap_planner']['ros__parameters']
    world = root.find('worldbody')
    for name in cfg['gate_order']:
        gate = cfg['gates'][name]
        rotation = Rotation.from_euler('xyz', gate['sim_rpy_deg'], degrees=True)
        center = np.array(gate['sim_position'], dtype=float)
        center += rotation.apply(gate.get('opening_offset', [0, 0, 0]))
        w, x, y, z = gate.get('opening_quaternion_wxyz', [1, 0, 0, 0])
        rotation = rotation * Rotation.from_quat([x, y, z, w])
        x, y, z, w = rotation.as_quat()
        body = ET.SubElement(world, 'body', name='gap_' + name,
                             pos=' '.join(map(str, center)), quat=f'{w} {x} {y} {z}')
        width, height = gate['width'], gate['height']
        depth, bar = gate['thickness'], gate.get('frame_width', 0.05)
        if min(width, height, depth, bar) <= 0:
            raise ValueError(f'Invalid gate dimensions: {name}')
        for index, sign in enumerate((-1, 1)):
            ET.SubElement(body, 'geom', name=f'gap_{name}_side_{index}', type='box',
                pos=f'0 {sign * (width + bar) / 2} 0',
                size=f'{depth / 2} {bar / 2} {(height + 2 * bar) / 2}',
                rgba='0.2 0.7 1 0.8', contype='1', conaffinity='1')
            ET.SubElement(body, 'geom', name=f'gap_{name}_cap_{index}', type='box',
                pos=f'0 0 {sign * (height + bar) / 2}',
                size=f'{depth / 2} {width / 2} {bar / 2}',
                rgba='0.2 0.7 1 0.8', contype='1', conaffinity='1')
    return ET.tostring(root, encoding='unicode')


class GateSelection:
    """Preallocated model slots are invisible and non-colliding until selected.

    Changing a selection never reloads the physics model or resets the aircraft.
    Call only on the simulator's executor/physics thread.
    """
    def __init__(self, model, configuration):
        with open(configuration, encoding='utf-8') as stream:
            cfg = yaml.safe_load(stream)['gap_planner']['ros__parameters']
        self.model = model
        self.names = cfg['gate_order']
        self.groups = []
        self.executing = False
        self.command_holding = False
        self.status_received = 0.0
        self.count = 0
        for name in self.names:
            ids = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f'gap_{name}_{kind}_{i}')
                   for kind in ('side', 'cap') for i in range(2)]
            self.groups.append(ids)
        self.select(0)

    def select(self, count):
        if not 0 <= count <= len(self.groups):
            raise ValueError('Gate count outside configured range')
        for index, ids in enumerate(self.groups):
            enabled = index < count
            self.model.geom_contype[ids] = int(enabled)
            self.model.geom_conaffinity[ids] = int(enabled)
            self.model.geom_rgba[ids, 3] = 0.8 if enabled else 0.0
        self.count = count
