"""Public, simulation-only planning API. Uses isolated local worker processes.

No hardware driver or ROS2 endpoint is contacted. Angles are radians; lengths
are metres. Use a context manager to stop unfinished work and release resources.
"""
import argparse
import copy
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time
import uuid

DEMOS = ('quick_start', 'ur3e_single_arm', 'grasping', 'motion_planning',
         'dual_arm', 'mobile_dual_arm', 'integrated_task', 'mobile_dual_pitch', 'rm75_navigation')


def _vector(values, length, name, positive=False):
    try:
        raw = list(values)
        if any(isinstance(v, (bool, str, bytes)) for v in raw):
            raise ValueError
        result = [float(v) for v in raw]
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError(f'{name} must contain {length} finite numbers') from exc
    if len(result) != length or not all(math.isfinite(v) for v in result):
        raise ValueError(f'{name} must contain {length} finite numbers')
    if positive and not all(v > 0 for v in result):
        raise ValueError(f'{name} must be positive')
    return result


class ManipulationClient:
    """Single-threaded local task client; task IDs live for this instance only.

    Every submit method returns an ID immediately. get_status, wait and stop
    return JSON-serializable snapshots. A successful plan is not robot execution.
    """

    def __init__(self, runtime=None):
        self._root = Path(__file__).resolve().parents[1]
        candidate = runtime or os.environ.get('OMINDOS_MANIPULATION_RUNTIME')
        bundled = self._root / 'runtime' / 'omindos-runtime'
        if candidate is None and bundled.is_file():
            candidate = bundled
        self._runtime = Path(candidate).resolve(strict=True) if candidate is not None else None
        if self._runtime is None and not (self._root / 'community/api_worker.py').is_file():
            raise FileNotFoundError('Set OMINDOS_MANIPULATION_RUNTIME to the release runtime executable')
        self._workspace = tempfile.TemporaryDirectory(prefix='omindos-manipulation-')
        self._tasks = {}
        self._closed = False

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()

    def _submit(self, operation, payload, seed):
        if self._closed:
            raise RuntimeError('Client is closed')
        if type(seed) is not int or not 0 <= seed < 2**32:
            raise ValueError('seed must be an integer in [0, 2**32)')
        task_id = uuid.uuid4().hex
        directory = Path(self._workspace.name) / task_id
        directory.mkdir()
        output = directory / 'result.json'
        if operation == 'demo':
            command = ([str(self._runtime), 'demo', payload['name']] if self._runtime else
                       [sys.executable, '-m', 'community.' + payload['name']]) + [
                       '--headless', '--seed', str(seed), '--output', str(output)]
        else:
            request = directory / 'request.json'
            request.write_text(json.dumps(dict(operation=operation, seed=seed, **payload),
                                          allow_nan=False), encoding='utf-8')
            command = ([str(self._runtime), 'worker'] if self._runtime else
                       [sys.executable, '-m', 'community.api_worker']) + [str(request), str(output)]
        log_path = directory / 'worker.log'
        with log_path.open('w', encoding='utf-8') as log:
            process = subprocess.Popen(command, cwd=self._root, stdout=log, stderr=log,
                                       start_new_session=os.name == 'posix')
        self._tasks[task_id] = dict(process=process, output=output, log=log_path,
            snapshot=dict(task_id=task_id, operation=operation, status='running',
                          simulation_only=True, result=None, error=None))
        return task_id

    def plan_joint_motion(self, start, goal, *, obstacles=(), seed=7):
        """Plan a bare UR3e's six-joint path, checking self and box collisions.

        obstacles: [{"size_m": [x,y,z], "position_m": [x,y,z]}], axis-aligned
        in the UR3e base frame. No tool, payload, timing or controller is modeled.
        """
        boxes = []
        for box in obstacles:
            if not isinstance(box, dict) or set(box) != {'size_m', 'position_m'}:
                raise ValueError('Each obstacle needs exactly size_m and position_m')
            boxes.append(dict(size_m=_vector(box['size_m'], 3, 'size_m', positive=True),
                              position_m=_vector(box['position_m'], 3, 'position_m')))
        return self._submit('joint_motion', dict(start=_vector(start, 6, 'start'),
                            goal=_vector(goal, 6, 'goal'), obstacles=boxes), seed)

    def plan_grasps(self, size_m, *, jaw_width_m=.065, seed=7):
        """Generate Robotiq85 geometric candidates for a centered, aligned box.

        Does not verify contact forces, force closure, arm reach or approach.
        """
        size = _vector(size_m, 3, 'size_m', positive=True)
        width = _vector([jaw_width_m], 1, 'jaw_width_m', positive=True)[0]
        if width > .085:
            raise ValueError('Robotiq85 jaw_width_m must be at most 0.085')
        return self._submit('grasps', dict(size_m=size, jaw_width_m=width), seed)

    @staticmethod
    def _config(path, arms=False):
        if path is None:
            return None
        path = Path(path).resolve(strict=True)
        data = json.loads(path.read_text(encoding='utf-8'))
        if not isinstance(data, dict):
            raise ValueError('Configuration must be a JSON object')
        if arms:
            for side in ('left', 'right'):
                entry = data[side]
                if entry.get('urdf_path') is not None:
                    entry['urdf_path'] = str((path.parent / entry['urdf_path']).resolve(strict=True))
        return data

    @staticmethod
    def _boxes(obstacles):
        boxes = []
        for box in obstacles:
            if not isinstance(box, dict) or set(box) != {'size_m', 'position_m'}:
                raise ValueError('Each obstacle needs exactly size_m and position_m')
            boxes.append(dict(size_m=_vector(box['size_m'], 3, 'size_m', positive=True),
                              position_m=_vector(box['position_m'], 3, 'position_m')))
        return boxes

    def get_mobile_model(self, *, arm_config=None, waist_config=None):
        """Return home positions, limits and URDF joint order for the 15-DoF model."""
        return self._submit('mobile_model', dict(arm_config=self._config(arm_config, True),
                            waist_config=self._config(waist_config)), 7)

    def plan_mobile_joint_motion(self, start, goal, *, base_pose=(0., 0., 0.),
                                 obstacles=(), arm_config=None, waist_config=None, seed=7):
        """Plan waist + left 7 + right 7; base stays fixed at map x,y,yaw.

        Boxes are in map coordinates. Output is an untimed, simulation-only
        sampled geometric path, not a controller trajectory or hardware command.
        """
        return self._submit('mobile_joint_motion', dict(
            start=_vector(start, 15, 'start'), goal=_vector(goal, 15, 'goal'),
            base_pose=_vector(base_pose, 3, 'base_pose x,y,yaw'),
            obstacles=self._boxes(obstacles), arm_config=self._config(arm_config, True),
            waist_config=self._config(waist_config)), seed)

    def plan_mobile_task(self, *, scene=None, navigation_trace=None, target_positions=None,
                         arm_config=None, waist_config=None, seed=7):
        """Offline navigation + verified arrival/stop + full-body joint planning.

        Supply exactly one navigation scene or already collected simulation trace.
        This does not publish ROS goals or execute a physical robot.
        """
        if (scene is None) == (navigation_trace is None):
            raise ValueError('Supply exactly one scene or navigation_trace')
        data = scene if scene is not None else navigation_trace
        if not isinstance(data, dict):
            raise ValueError('Navigation input must be a JSON object')
        data = json.loads(json.dumps(data, allow_nan=False))
        return self._submit('mobile_task', dict(
            scene=data if scene is not None else None,
            navigation_trace=data if navigation_trace is not None else None,
            target_positions=None if target_positions is None else _vector(target_positions, 15, 'target_positions'),
            arm_config=self._config(arm_config, True), waist_config=self._config(waist_config)), seed)

    def run_demo(self, name, *, seed=7):
        """Run a documented demo without a graphical window."""
        if name not in DEMOS:
            raise ValueError('Unknown demo; choose from ' + ', '.join(DEMOS))
        return self._submit('demo', dict(name=name), seed)

    def get_status(self, task_id):
        """Return running/succeeded/failed/canceled; does not query a robot."""
        task = self._tasks[task_id]
        snapshot = task['snapshot']
        code = task['process'].poll()
        if snapshot['status'] == 'running' and code is not None:
            try:
                result = json.loads(task['output'].read_text(encoding='utf-8'))
                if code != 0 or not isinstance(result, dict) or result.get('status') != 'passed':
                    raise ValueError('Worker did not report success')
                snapshot.update(status='succeeded', result=result)
            except (OSError, ValueError) as exc:
                details = task['log'].read_text(encoding='utf-8', errors='replace')[-2000:]
                snapshot.update(status='failed', error=dict(code='worker_failed',
                    message=f'{exc}; exit={code}', details=details))
        return copy.deepcopy(snapshot)

    @staticmethod
    def _terminate(process):
        if process.poll() is not None:
            return
        try:
            if os.name == 'posix':
                os.killpg(process.pid, signal.SIGTERM)
            else:
                process.terminate()
        except ProcessLookupError:
            pass
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            try:
                if os.name == 'posix':
                    os.killpg(process.pid, signal.SIGKILL)
                else:
                    process.kill()
            except ProcessLookupError:
                pass
            process.wait(timeout=3)

    def stop(self, task_id):
        """Cancel and reap this local calculation. This is not a hardware stop."""
        snapshot = self.get_status(task_id)
        if snapshot['status'] == 'running':
            task = self._tasks[task_id]
            self._terminate(task['process'])
            task['snapshot'].update(status='canceled', result=None,
                                    error=dict(code='canceled', message='Local calculation canceled'))
        return self.get_status(task_id)

    def wait(self, task_id, timeout=120):
        """Wait at most timeout seconds, then terminate and report failed/timeout.

        Timeout is measured from this call, not submission. No background watchdog
        runs when the caller neither polls nor waits. close() also stops workers.
        """
        seconds = _vector([timeout], 1, 'timeout', positive=True)[0]
        deadline = time.monotonic() + seconds
        while True:
            snapshot = self.get_status(task_id)
            if snapshot['status'] != 'running':
                return snapshot
            if time.monotonic() >= deadline:
                snapshot = self.stop(task_id)
                if snapshot['status'] == 'canceled':
                    self._tasks[task_id]['snapshot'].update(status='failed',
                        error=dict(code='timeout', message='Local calculation exceeded wait timeout'))
                return self.get_status(task_id)
            time.sleep(min(.05, max(0., deadline - time.monotonic())))

    def close(self):
        """Stop unfinished workers; cached results remain readable on this object."""
        if not self._closed:
            for task_id in self._tasks:
                self.stop(task_id)
            self._workspace.cleanup()
            self._closed = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('demo', choices=DEMOS)
    parser.add_argument('--seed', type=int, default=7)
    parser.add_argument('--timeout', type=float, default=120)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    with ManipulationClient() as client:
        try:
            task_id = client.run_demo(args.demo, seed=args.seed)
            result = client.wait(task_id, timeout=args.timeout)
        except ValueError as exc:
            parser.error(str(exc))
        except KeyboardInterrupt:
            result = client.stop(task_id)
        payload = json.dumps(result, indent=2, allow_nan=False)
        print(payload)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(payload + '\n', encoding='utf-8')
        return 0 if result['status'] == 'succeeded' else 1


if __name__ == '__main__':
    raise SystemExit(main())
