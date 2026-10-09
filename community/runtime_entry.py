"""Entry point frozen into the customer runtime; no source-directory dependency."""
import importlib
import sys

DEMOS = ('quick_start', 'ur3e_single_arm', 'grasping', 'motion_planning',
         'dual_arm', 'mobile_dual_arm', 'integrated_task', 'mobile_dual_pitch', 'rm75_navigation', 'dual_task_motion')


def main():
    if sys.argv[1:] == ['--version']:
        print('OmindOS Manipulation 0.2 runtime.2')
        return 0
    if len(sys.argv) >= 2 and sys.argv[1] == 'worker':
        if len(sys.argv) != 4:
            raise SystemExit('Usage: omindos-runtime worker REQUEST.json RESULT.json')
        from community.api_worker import main as worker
        sys.argv = [sys.argv[0]] + sys.argv[2:]
        worker()
        return 0
    if len(sys.argv) >= 3 and sys.argv[1] == 'demo' and sys.argv[2] in DEMOS:
        name = sys.argv[2]
        sys.argv = [sys.argv[0]] + sys.argv[3:]
        importlib.import_module('community.' + name).main()
        return 0
    raise SystemExit('Usage: omindos-runtime --version | worker REQUEST RESULT | demo NAME [OPTIONS]')


if __name__ == '__main__':
    raise SystemExit(main())
