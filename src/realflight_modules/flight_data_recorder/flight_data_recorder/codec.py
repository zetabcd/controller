"""Map ROS numeric fields to one fixed-layout ULog dataset per source.

Keep primitives, short fixed primitive arrays and flattened nested messages.
Omit fields that previously required .fields/ or .chunks/ child datasets.
"""

from dataclasses import dataclass
import hashlib
import json

from rosidl_parser.definition import (
    AbstractSequence, AbstractString, AbstractWString, Array, BasicType, NamespacedType,
)
from rosidl_runtime_py.utilities import get_message


BASIC = {
    'float': 'float', 'double': 'double', 'boolean': 'bool',
    'octet': 'uint8_t', 'char': 'uint8_t', 'wchar': 'uint16_t',
    **{f'{prefix}{bits}': f'{prefix}{bits}_t'
       for prefix in ('int', 'uint') for bits in (8, 16, 32, 64)},
}
MAX_FIXED_ARRAY = 128
COMMON = [
    ('uint64_t', 'timestamp', 1), ('uint64_t', 'recorder_sequence', 1),
    ('uint64_t', 'recorder_monotonic_ns', 1), ('int64_t', 'recorder_ros_time_ns', 1),
]


@dataclass(frozen=True)
class Stamp:
    monotonic_ns: int
    ros_ns: int
    sequence: int


class RosCodec:
    def __init__(self, topic, type_name, message_class, dataset=None):
        self.topic = topic
        self.type_name = type_name
        self.message_class = message_class
        self.dataset = dataset or 'ros' + topic
        self.type_definitions = {}
        self._describe(type_name, message_class)

    def _describe(self, type_name, cls):
        if type_name in self.type_definitions:
            return
        self.type_definitions[type_name] = cls.get_fields_and_field_types()
        for spec in cls.SLOT_TYPES:
            while isinstance(spec, (Array, AbstractSequence)):
                spec = spec.value_type
            if isinstance(spec, NamespacedType):
                name = '/'.join(spec.namespaced_name())
                self._describe(name, get_message(name))

    def encode(self, message, stamp, writer):
        if self.dataset not in writer.schemas:
            key = 'ros_type_' + hashlib.sha256(self.dataset.encode()).hexdigest()[:16]
            writer.info(key, json.dumps({'topic': self.topic, 'ros_type': self.type_name,
                                        'definitions': self.type_definitions}))
        fields = list(COMMON)
        values = [stamp.monotonic_ns // 1000, stamp.sequence, stamp.monotonic_ns, stamp.ros_ns]
        field_map = {}
        omitted_fields = []

        def add(kind, relative, data, count=1):
            name = 'msg_' + '__'.join(relative)
            original = list(relative)
            if name in field_map:
                name += '_' + hashlib.sha256(repr(original).encode()).hexdigest()[:12]
            if name in field_map:
                raise ValueError('Colliding flattened ROS field names')
            field_map[name] = original
            fields.append((kind, name, count))
            if count == 1:
                values.append(data)
            else:
                values.extend(data)

        def visit(item, item_spec, relative):
            if item_spec is None or isinstance(item_spec, NamespacedType):
                for field_name, field_spec in zip(item.get_fields_and_field_types(), item.SLOT_TYPES):
                    visit(getattr(item, field_name), field_spec, relative + (field_name,))
            elif isinstance(item_spec, BasicType):
                if item_spec.typename not in BASIC:
                    raise ValueError(f'Unsupported ROS primitive {item_spec.typename}')
                # ROS octet scalars are represented as one-byte bytes objects.
                if isinstance(item, (bytes, bytearray)) and len(item) == 1:
                    item = item[0]
                elif isinstance(item, str) and item_spec.typename in ('char', 'wchar'):
                    item = ord(item)
                add(BASIC[item_spec.typename], relative, item)
            elif isinstance(item_spec, (AbstractString, AbstractWString)):
                omitted_fields.append({'path': list(relative), 'reason': 'string'})
            elif isinstance(item_spec, (Array, AbstractSequence)):
                element = item_spec.value_type
                if (isinstance(item_spec, Array) and isinstance(element, BasicType)
                        and 0 < item_spec.size <= MAX_FIXED_ARRAY):
                    count = item_spec.size
                    add(BASIC[element.typename], relative, item[0] if count == 1 else item, count)
                else:
                    omitted_fields.append({'path': list(relative), 'reason': 'array_requires_child_dataset'})
            else:
                raise ValueError(f'Unsupported ROS field type {item_spec}')

        visit(message, None, ())
        mapping = {'topic': self.topic, 'ros_type': self.type_name, 'path': [],
                   'indices': 0, 'fields': field_map, 'omitted_fields': omitted_fields}
        writer.write(self.dataset, fields, values, mapping)
