"""Read-only TCP/IP discovery for the UR PolyScope 5 Dashboard (port 29999).

Uses only Python's standard library; no ROS2, motion, power or unlock commands.
PolyScope X uses another API; a failed Dashboard query does not identify its version.
"""
import argparse
import ipaddress
import json
import math
from pathlib import Path
import platform
import re
import socket
import time


QUERIES = (
    ('software', 'PolyscopeVersion'),
    ('model', 'get robot model'),
    ('robot_mode', 'robotmode'),
    ('safety_mode', 'safetymode'),
    ('program_running', 'running'),
    ('remote_control', 'is in remote control'),
)


class LineReader:
    def __init__(self, connection, timeout):
        self.connection = connection
        self.timeout = timeout
        self.buffer = bytearray()

    def read(self):
        deadline = time.monotonic() + self.timeout
        while b'\n' not in self.buffer:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise TimeoutError('Dashboard response timed out')
            self.connection.settimeout(remaining)
            chunk = self.connection.recv(1024)
            if not chunk:
                raise ConnectionError('Dashboard closed before a complete response')
            self.buffer.extend(chunk)
            if len(self.buffer) > 4096:
                raise ValueError('Dashboard response exceeds 4096 bytes')
        line, _, rest = self.buffer.partition(b'\n')
        self.buffer = bytearray(rest)
        return line.decode('utf-8', errors='replace').rstrip('\r')


def probe(host, port=29999, timeout=3.0):
    ipaddress.ip_address(host)  # Explicit single address; never scan a subnet or guess.
    if type(port) is not int or not 1 <= port <= 65535:
        raise ValueError('port must be between 1 and 65535')
    if not math.isfinite(timeout) or timeout <= 0:
        raise ValueError('timeout must be positive and finite')
    report = dict(schema='omindos.ur.dashboard.observation.v0.1', status='incomplete',
        host=host, port=port, read_only=True, ready_for_motion=False, physical_robot_verified=False,
        host_os=platform.platform(), python=platform.python_version(), responses={},
        software_version=None, software_family='unknown', model_response=None, error=None)
    try:
        with socket.create_connection((host, port), timeout=timeout) as connection:
            reader = LineReader(connection, timeout)
            greeting = reader.read()
            report['greeting'] = greeting
            if 'dashboard server' not in greeting.lower():
                raise ValueError('Unexpected service greeting; no queries sent')
            for key, command in QUERIES:
                connection.settimeout(timeout)
                connection.sendall((command + '\n').encode('ascii'))
                report['responses'][key] = reader.read()
        response = report['responses'].get('software', '')
        version = re.fullmatch(r'URSoftware\s+(\d+(?:\.\d+)+)(?:\s+.*)?', response.strip())
        if not version:
            raise ValueError('Software version was not recognized; inspect the teach pendant')
        report['software_version'] = version.group(1)
        major = int(version.group(1).split('.')[0])
        report['software_family'] = 'PolyScope 5' if major == 5 else 'other; verify manually'
        report['model_response'] = report['responses'].get('model')
        report['status'] = 'observed'
    except (OSError, ValueError) as exc:
        report['error'] = str(exc)
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', required=True, help='Explicit robot IP address')
    parser.add_argument('--port', type=int, default=29999)
    parser.add_argument('--timeout', type=float, default=3, help='Per-response deadline, in seconds')
    parser.add_argument('--output', type=Path, default=Path('community-results/hardware/tcp-report.json'))
    args = parser.parse_args()
    try:
        report = probe(args.host, args.port, args.timeout)
    except ValueError as exc:
        parser.error(str(exc))
    payload = json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False)
    print(payload)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(payload + '\n', encoding='utf-8')
    return 0 if report['status'] == 'observed' else 1


if __name__ == '__main__':
    raise SystemExit(main())
