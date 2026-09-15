"""Small ULog v1 writer; no PX4 struct layout or ROS CDR bytes enter ULog data.

Both pyulog and PlotJuggler require all F definitions before the first A record.
Spool definitions and data separately so topics discovered during flight remain
readable by unmodified loaders. Finalization is streamed, fsynced and atomic.
"""

import argparse
from datetime import datetime
import fcntl
import json
import os
from pathlib import Path
import struct
import uuid


MAGIC = b'ULog\x01\x12\x35\x01'
PRIMITIVES = {
    'int8_t': 'b', 'uint8_t': 'B', 'int16_t': 'h', 'uint16_t': 'H',
    'int32_t': 'i', 'uint32_t': 'I', 'int64_t': 'q', 'uint64_t': 'Q',
    'float': 'f', 'double': 'd', 'bool': '?', 'char': 'B',
}


def packet(kind, payload):
    if len(payload) > 65535:
        raise ValueError(f'ULog {kind} record exceeds 65535 bytes')
    return struct.pack('<HB', len(payload), ord(kind)) + payload


def info_packets(key, value):
    """M records preserve arbitrarily long UTF-8 metadata without truncation."""
    raw = str(value).encode('utf-8')
    typed_key = f'char[{len(raw)}] {key}'.encode('ascii')
    if len(typed_key) > 255:
        raise ValueError('ULog metadata key too long')
    if len(raw) + len(typed_key) + 1 <= 65535:
        yield packet('I', bytes([len(typed_key)]) + typed_key + raw)
    else:
        for offset in range(0, len(raw), 60000):
            yield packet('M', bytes([int(offset > 0), len(typed_key)]) +
                         typed_key + raw[offset:offset + 60000])


def sync_directory(path):
    fd = os.open(str(path), os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(fd)
    finally:
        os.close(fd)


def read_packets(stream):
    while header := stream.read(3):
        if len(header) != 3:
            raise EOFError('Partial ULog packet header')
        size, kind = struct.unpack('<HB', header)
        body = stream.read(size)
        if len(body) != size:
            raise EOFError('Partial ULog packet body')
        yield chr(kind), body, header + body


def assemble(pending, recovered=False):
    """Recover only inactive sessions; never consume a running writer's spools."""
    pending = Path(pending)
    with (pending / 'writer.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        return _assemble_locked(pending, recovered)


def _assemble_locked(pending, recovered=False):
    """Copy complete, defined packets only. Keep spools if finalization fails."""
    pending = Path(pending)
    manifest = json.loads((pending / 'manifest.json').read_text())
    suffix = '.recovered.ulg' if recovered else (
        '.incomplete.ulg' if manifest.get('incomplete') else '.ulg')
    target = pending.parent / (manifest['basename'] + suffix)
    # Unique temporary name, then link without replacing any existing log.
    temporary = pending / 'assembled.tmp'
    names = set()
    registered = set()
    with temporary.open('wb') as out:
        out.write(MAGIC + struct.pack('<Q', manifest['start_timestamp_us']))
        out.write(packet('B', bytes(40)))
        with (pending / 'definitions.bin').open('rb') as definitions:
            try:
                for kind, body, record in read_packets(definitions):
                    if kind == 'F':
                        names.add(body.split(b':', 1)[0])
                    out.write(record)
            except EOFError:
                if not recovered:
                    raise
        if recovered:
            for record in info_packets('recorder_recovered', 'true; session was not finalized'):
                out.write(record)
        with (pending / 'data.bin').open('rb') as data:
            try:
                for kind, body, record in read_packets(data):
                    if kind == 'A':
                        msg_id = struct.unpack_from('<H', body, 1)[0]
                        if body[3:] not in names:
                            if recovered:
                                continue
                            raise ValueError('Missing ULog format definition')
                        registered.add(msg_id)
                    elif kind == 'D' and struct.unpack_from('<H', body)[0] not in registered:
                        if recovered:
                            continue
                        raise ValueError('Missing ULog dataset registration')
                    out.write(record)
            except EOFError:
                if not recovered:
                    raise
        out.flush()
        os.fsync(out.fileno())
    os.link(temporary, target)
    sync_directory(target.parent)
    temporary.unlink()
    # Only remove this writer's own files, after the durable final log exists.
    for name in ('definitions.bin', 'data.bin', 'manifest.json', 'writer.lock', 'manifest.tmp'):
        (pending / name).unlink(missing_ok=True)
    pending.rmdir()
    return target


class ULogWriter:
    def __init__(self, directory, start_ns, metadata):
        directory = Path(directory)
        directory.mkdir(parents=True, exist_ok=True)
        basename = 'flight_' + datetime.now().strftime('%Y%m%d_%H%M%S_%f') + '_' + uuid.uuid4().hex[:8]
        self.pending = directory / (basename + '.ulg.pending')
        self.pending.mkdir()
        self._lock = (self.pending / 'writer.lock').open('a')
        fcntl.flock(self._lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        self.manifest = {'basename': basename, 'start_timestamp_us': start_ns // 1000}
        self.definitions = None
        self.data = None
        self.schemas = {}
        self.closed = False
        try:
            self._save_manifest()
            self.definitions = (self.pending / 'definitions.bin').open('wb')
            self.data = (self.pending / 'data.bin').open('wb')
            self.info('recorder_schema_version', '2')
            self.info('recorder_metadata', json.dumps(metadata, ensure_ascii=False))
            self.sync()
            sync_directory(self.pending)
            sync_directory(directory)
        except BaseException:
            self.abandon()
            raise

    def _save_manifest(self):
        with (self.pending / 'manifest.tmp').open('w') as stream:
            json.dump(self.manifest, stream)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(self.pending / 'manifest.tmp', self.pending / 'manifest.json')
        sync_directory(self.pending)

    def info(self, key, value):
        for record in info_packets(key, value):
            self.definitions.write(record)

    def write(self, name, fields, values, mapping=None):
        """fields: [(ULog primitive, field name, fixed array length), ...]."""
        schema = self.schemas.get(name)
        signature = tuple(fields)
        if schema is None:
            if len(self.schemas) >= 65536:
                raise ValueError('ULog dataset ID space exhausted')
            msg_id = len(self.schemas)
            declaration = name + ':' + ''.join(
                f'{kind}{"[" + str(count) + "]" if count != 1 else ""} {field};'
                for kind, field, count in fields)
            packing = struct.Struct('<' + ''.join(
                str(count) + PRIMITIVES[kind] for kind, _, count in fields))
            if packing.size + 2 > 65535:
                raise ValueError(f'ULog dataset {name} exceeds maximum data size')
            self.definitions.write(packet('F', declaration.encode('ascii')))
            if mapping is not None:
                self.info(f'dataset_{msg_id}', json.dumps(mapping, ensure_ascii=False))
            self.data.write(packet('A', struct.pack('<BH', 0, msg_id) + name.encode('ascii')))
            schema = [signature, msg_id, packing, -1]
            self.schemas[name] = schema
        elif schema[0] != signature:
            raise ValueError(f'Changing schema for {name}')
        values = list(values)
        # Closely spaced messages may share the same microsecond timestamp.
        # Keep the ULog timestamp strictly increasing; exact reception ns remains
        # in recorder_monotonic_ns alongside the original recorder sequence.
        values[0] = max(values[0], schema[3] + 1)
        schema[3] = values[0]
        self.data.write(packet('D', struct.pack('<H', schema[1]) + schema[2].pack(*values)))

    def sync(self):
        for stream in (self.definitions, self.data):
            stream.flush()
            os.fsync(stream.fileno())

    def finish(self, summary, incomplete=False):
        self.info('recorder_summary', json.dumps(summary, ensure_ascii=False))
        self.manifest['incomplete'] = incomplete
        self.sync()
        self._save_manifest()
        self.definitions.close()
        self.data.close()
        self.closed = True
        try:
            return _assemble_locked(self.pending)
        finally:
            self._lock.close()

    def abandon(self):
        """Leave recoverable spools; never advertise a failed log as complete."""
        for stream in (self.definitions, self.data):
            try:
                if stream is not None:
                    stream.close()
            except OSError:
                pass
        self._lock.close()
        self.closed = True


def recover_main():
    parser = argparse.ArgumentParser(description='Recover a recorder-owned .ulg.pending directory')
    parser.add_argument('pending', type=Path)
    args = parser.parse_args()
    print(assemble(args.pending, recovered=True))
