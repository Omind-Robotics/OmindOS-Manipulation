"""Use a loopback fixture to verify bounded, strictly read-only TCP requests."""
import socket
import threading
import unittest

from community.hardware.ur_dashboard_probe import probe


class DashboardTests(unittest.TestCase):
    def run_fixture(self, replies, greeting=b'Connected: Universal Robots Dashboard Server\n'):
        listener = socket.socket()
        listener.bind(('127.0.0.1', 0))
        listener.listen(1)
        listener.settimeout(3)
        port = listener.getsockname()[1]
        commands = []

        def serve():
            try:
                with listener.accept()[0] as connection:
                    connection.settimeout(2)
                    connection.sendall(greeting)
                    with connection.makefile('rb') as incoming:
                        for reply in replies:
                            line = incoming.readline(4096)
                            if not line:
                                return
                            commands.append(line.decode().strip())
                            if reply is None:
                                return
                            connection.sendall(reply)
            finally:
                listener.close()

        thread = threading.Thread(target=serve)
        thread.start()
        try:
            result = probe('127.0.0.1', port, timeout=1)
        finally:
            thread.join(timeout=4)
        self.assertFalse(thread.is_alive())
        return result, commands

    def test_version_and_fixed_read_only_commands(self):
        report, commands = self.run_fixture([
            b'URSoftware 5.12.0.1101319 (Mar 22 2022)\n', b'UR3\n', b'Robotmode: IDLE\n',
            b'Safetymode: NORMAL\n', b'Program running: false\n', b'false\n'])
        self.assertEqual(commands, ['PolyscopeVersion', 'get robot model', 'robotmode',
                                   'safetymode', 'running', 'is in remote control'])
        self.assertEqual(report['software_version'], '5.12.0.1101319')
        self.assertEqual(report['status'], 'observed')
        self.assertFalse(report['ready_for_motion'])
        self.assertFalse(report['physical_robot_verified'])

    def test_disconnect_reports_incomplete_without_retrying_other_services(self):
        report, commands = self.run_fixture([None])
        self.assertEqual(commands, ['PolyscopeVersion'])
        self.assertEqual(report['status'], 'incomplete')
        self.assertIsNone(report['software_version'])

    def test_wrong_service_sends_no_command(self):
        report, commands = self.run_fixture([], b'HTTP/1.1 200 OK\n')
        self.assertEqual(commands, [])
        self.assertEqual(report['status'], 'incomplete')

    def test_unknown_version_is_not_assumed_polyscope_five(self):
        report, _ = self.run_fixture([b'Unknown command\n'] * 6)
        self.assertEqual(report['software_family'], 'unknown')
        self.assertEqual(report['status'], 'incomplete')

    def test_missing_newline_fails(self):
        report, commands = self.run_fixture([b'URSoftware 5.12.0'])
        self.assertEqual(report['status'], 'incomplete')
        self.assertEqual(commands, ['PolyscopeVersion'])

    def test_requires_explicit_ip_and_valid_limits(self):
        for host, port, timeout in [('ROBOT_IP', 29999, 3), ('127.0.0.1', 0, 3),
                                    ('127.0.0.1', 29999, float('nan'))]:
            with self.assertRaises(ValueError):
                probe(host, port, timeout)


if __name__ == '__main__':
    unittest.main()
