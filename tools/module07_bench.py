"""Interactive isolated-power wire check; never arms RobotClient or retries MOVE.

Only heartbeat is automatic. pause/resume keeps the same TCP connection. The
explicit probe-estop command sends one raw 1 ms MOVE after checking the latch;
use exclusively with actuator power physically isolated. No import-time I/O.
"""
import argparse
import select
import socket
import sys
import time

from robot_comm.logging import WireLogger
from robot_comm.protocol import CAPS, LineFramer, expected_reply, parse_safety


class BenchError(RuntimeError):
    pass


class BenchSession:
    def __init__(self, sock, *, isolated=False, logger=None, timeout=.8):
        self.sock = sock
        self.isolated = isolated
        self.logger = logger or WireLogger()
        self.timeout = timeout
        self.framer = LineFramer()
        self.paused = False
        self.last_heartbeat = None
        self.heartbeat_due = time.monotonic()

    def receive(self, timeout=0):
        ready, _, _ = select.select([self.sock], [], [], timeout)
        if not ready:
            return []
        chunk = self.sock.recv(4096)
        if not chunk:
            raise BenchError('TCP closed; no automatic reconnect')
        def log(line):
            self.logger.emit('RX', line)
        return self.framer.feed(chunk, log)

    def _query(self, command):
        # Serial requests, no heartbeat transmission while waiting for this ACK.
        # The live reader still logs asynchronous events. No motion is retried.
        self.sock.sendall((command+'\n').encode('ascii'))
        self.logger.emit('TX', command)
        deadline = time.monotonic()+self.timeout
        while time.monotonic() < deadline:
            lines = self.receive(min(.05, max(0, deadline-time.monotonic())))
            for line in lines:
                if expected_reply(command, line):
                    return line
        raise BenchError('Reply timeout; not retried: '+command)

    def query(self, command):
        if command not in {'PING','STATUS','CAPS','SAFETY','HEARTBEAT','STOP','ABORT','ESTOP','RESET_ESTOP'}:
            raise BenchError('No normal MOVE/TARGET/ARRIVED in this communication-only tool')
        return self._query(command)

    def heartbeat(self):
        reply = self.query('HEARTBEAT')
        if reply != 'OK,HEARTBEAT':
            raise BenchError('Heartbeat rejected: '+reply)
        self.last_heartbeat = time.monotonic()
        self.heartbeat_due = self.last_heartbeat+.2

    def tick(self):
        if not self.paused and time.monotonic() >= self.heartbeat_due:
            self.heartbeat()

    def pause(self):
        self.paused = True
        self.logger.emit('INFO', 'HEARTBEAT PAUSED; TCP remains open; no automatic reset')

    def resume(self):
        self.heartbeat()  # Fresh heartbeat only; never RESET_ESTOP or task resume.
        self.paused = False
        self.logger.emit('INFO', 'HEARTBEAT RESUMED; fault latch unchanged')

    def probe_estop(self):
        """Bypass ONLY the host motion gate, exclusively for an isolated bench."""
        if not self.isolated:
            raise BenchError('Requires explicit --actuators-isolated confirmation')
        if self.query('CAPS') != CAPS:
            raise BenchError('Exact shared CAPS required; refusing wire MOVE probe')
        if self.query('ESTOP') != 'OK,ESTOP':
            raise BenchError('ESTOP not acknowledged; refusing wire MOVE probe')
        safety = parse_safety(self.query('SAFETY'))
        if not safety['estop']:
            raise BenchError('Controller does not report ESTOP=1; refusing wire MOVE probe')
        # Drain earlier replies before the single explicit test. This is not
        # RobotClient authorization, and success cannot unlock any powered move.
        if self.query('PING') != 'PONG':
            raise BenchError('Reply barrier failed; refusing wire MOVE probe')
        try:
            reply = self._query('MOVE,F,1')
        except (OSError, BenchError):
            self._stop_after_probe_failure()
            raise
        if reply != 'ERR,ESTOP_LATCHED':
            self._stop_after_probe_failure()
            raise BenchError('FAIL: expected ERR,ESTOP_LATCHED, received '+reply)
        self.logger.emit('INFO', 'PASS: firmware actually returned ERR,ESTOP_LATCHED to one wire MOVE,F,1')
        return reply

    def _stop_after_probe_failure(self):
        # Safety commands only. Do not resend the probe on any failure.
        for command in ('ESTOP','STOP'):
            try:
                self.sock.sendall((command+'\n').encode('ascii'))
                self.logger.emit('TX', command)
            except OSError:
                break


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=['simulation','physical'], default='simulation')
    parser.add_argument('--host')
    parser.add_argument('--port', type=int, default=3333)
    parser.add_argument('--actuators-isolated', action='store_true',
                        help='Operator confirms motor/pump/servo power physically isolated')
    parser.add_argument('--log-file')
    args = parser.parse_args(argv)
    if args.mode == 'physical' and (not args.host or not args.actuators_isolated):
        parser.error('Physical bench requires an explicit host and --actuators-isolated')
    host = args.host or '127.0.0.1'
    if args.mode == 'simulation' and host not in {'localhost','127.0.0.1','::1'}:
        parser.error('Simulation is loopback-only')
    logger = WireLogger(args.log_file)
    try:
        with socket.create_connection((host,args.port), timeout=.8) as sock:
            sock.settimeout(.8)
            session = BenchSession(sock, isolated=args.actuators_isolated, logger=logger)
            logger.emit('INFO', 'Communication bench; no arming or automatic reset. Only HEARTBEAT is automatic.')
            print('Commands: pause, resume, ping, status, caps, safety, estop, stop, reset, probe-estop, quit', flush=True)
            while True:
                session.tick()
                ready, _, _ = select.select([sys.stdin,sock], [], [], .05)
                if sock in ready:
                    session.receive()
                if sys.stdin not in ready:
                    continue
                text = sys.stdin.readline()
                if not text:
                    break
                command = text.strip().lower()
                if command == 'quit':
                    break
                if command == 'pause':
                    session.pause()
                elif command == 'resume':
                    session.resume()
                elif command == 'probe-estop':
                    session.probe_estop()
                elif command in {'ping','status','caps','safety','estop','stop','reset'}:
                    session.query('RESET_ESTOP' if command=='reset' else command.upper())
                else:
                    print('Unsupported command; normal MOVE/TARGET/ARRIVED are disabled', flush=True)
        return 0
    except (OSError, ValueError, BenchError) as exc:
        print('Bench stopped: '+str(exc), flush=True)
        return 1
    except KeyboardInterrupt:
        return 0
    finally:
        logger.close()


if __name__ == '__main__':
    raise SystemExit(main())
