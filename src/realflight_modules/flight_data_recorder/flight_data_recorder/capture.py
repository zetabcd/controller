"""Single ordered receive queue; disk IO and ROS decoding run on one worker."""

from collections import Counter
import queue
import threading
import time

from .codec import Stamp
from .ulog import ULogWriter


class Capture:
    def __init__(self, directory, deserialize, metadata=None, max_queue_bytes=64 * 1024 * 1024,
                 sync_interval=1.0, simulation_mode=False):
        self.directory = directory
        self.deserialize = deserialize
        self.simulation_mode = simulation_mode
        self.metadata = dict(metadata or {}, simulation_mode=simulation_mode)
        self.max_queue_bytes = max_queue_bytes
        self.sync_interval = sync_interval
        self.queue = queue.Queue()
        self.results = queue.Queue()
        self.lock = threading.Lock()
        self.pending_bytes = 0
        self.active = False
        self.closed = False
        self.received = Counter()
        self.dropped = Counter()
        self.problems = []
        self.sequences = Counter()
        self.worker = threading.Thread(target=self._run, name='ulog-writer', daemon=False)
        self.worker.start()

    def _start(self, monotonic_ns):
        self.active = True
        self.received.clear()
        self.dropped.clear()
        self.problems.clear()
        self.queue.put(('start', monotonic_ns, dict(self.metadata)))

    def start_simulation(self, monotonic_ns):
        """Start after subscriptions are ready, without fabricating an ARM event."""
        if self.simulation_mode and not self.active and not self.closed:
            self._start(monotonic_ns)

    def receive(self, codec, raw, monotonic_ns, ros_ns, arming_state=None):
        """Called serially; VehicleStatus gates capture only outside simulation."""
        if self.closed:
            return
        if not self.simulation_mode and arming_state == 2 and not self.active:
            self._start(monotonic_ns)
        if not self.active:
            return
        key = codec.dataset
        self.sequences[key] += 1
        stamp = Stamp(monotonic_ns, ros_ns, self.sequences[key])
        self.received[key] += 1
        # The arming/disarming boundary is retained even if ordinary input has
        # overflowed. Its small record also closes the correct queue segment.
        with self.lock:
            boundary = not self.simulation_mode and arming_state in (1, 2)
            accepted = self.pending_bytes + len(raw) <= self.max_queue_bytes or boundary
            if accepted:
                self.pending_bytes += len(raw)
        if accepted:
            self.queue.put(('record', codec, raw, stamp))
        else:
            self.dropped[key] += 1
        if not self.simulation_mode and arming_state == 1:
            self._end('disarmed', monotonic_ns)

    def problem(self, text):
        if self.active and text not in self.problems:
            self.problems.append(text)

    def _end(self, reason, monotonic_ns):
        summary = {'reason': reason, 'end_monotonic_ns': monotonic_ns,
                   'received': dict(self.received), 'queue_dropped': dict(self.dropped),
                   'problems': list(self.problems),
                   'subscription_events': dict(self.metadata.get('subscription_events', {}))}
        self.queue.put(('end', summary))
        self.active = False

    def close(self):
        if self.closed:
            return
        if self.active:
            reason = 'simulation_shutdown' if self.simulation_mode else 'node_shutdown_while_armed'
            self._end(reason, time.monotonic_ns())
        self.closed = True
        self.queue.put(('stop',))
        self.worker.join()

    def _run(self):
        writer = None
        errors = []
        written = Counter()
        pending_path = None
        last_sync = time.monotonic()
        while True:
            try:
                work = self.queue.get(timeout=self.sync_interval)
            except queue.Empty:
                work = None
            try:
                if work is not None:
                    kind = work[0]
                    if kind == 'stop':
                        break
                    if kind == 'start':
                        errors = []
                        written.clear()
                        pending_path = None
                        writer = ULogWriter(self.directory, work[1], work[2])
                        pending_path = str(writer.pending)
                        initial_problems = work[2].get('unavailable_topics', {})
                        errors.extend(f'{name}: {error}' for name, error in initial_problems.items())
                    elif kind == 'record' and writer is not None:
                        _, codec, raw, stamp = work
                        message = self.deserialize(raw, codec.message_class)
                        codec.encode(message, stamp, writer)
                        written[codec.dataset] += 1
                    elif kind == 'end':
                        summary = dict(work[1], written=dict(written), writer_errors=list(errors))
                        incomplete = bool(errors or summary['queue_dropped'] or summary['problems'] or
                                          summary['reason'] not in ('disarmed', 'simulation_shutdown') or
                                          summary['received'] != summary['written'])
                        path = None
                        if writer is not None:
                            path = str(writer.finish(summary, incomplete=incomplete))
                            writer = None
                        self.results.put({'path': path, 'pending': pending_path,
                                          'incomplete': incomplete, 'summary': summary})
                if writer is not None and time.monotonic() - last_sync >= self.sync_interval:
                    writer.sync()
                    last_sync = time.monotonic()
            except Exception as error:  # preserve remaining messages after an encoding failure
                description = f'{type(error).__name__}: {error}'
                if description not in errors:
                    errors.append(description)
                    self.results.put({'error': description, 'pending': pending_path})
                if isinstance(error, OSError) and writer is not None:
                    writer.abandon()
                    writer = None
                if work is not None and work[0] == 'end':
                    if writer is not None:
                        writer.abandon()
                        writer = None
                    self.results.put({'path': None, 'pending': pending_path, 'incomplete': True,
                                      'summary': dict(work[1], writer_errors=list(errors))})
            finally:
                if work is not None:
                    if work[0] == 'record':
                        with self.lock:
                            self.pending_bytes -= len(work[2])
                    self.queue.task_done()
        if writer is not None:
            writer.abandon()
