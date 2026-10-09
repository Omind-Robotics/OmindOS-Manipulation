"""Verify the actual extracted customer kit with a system Python and frozen core."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
RUNTIME = 'OmindOS-Manipulation-0.2-runtime-linux-amd64.tar.gz'
CLIENT = 'OmindOS-Manipulation-0.2-client.tar.gz'

CHECK = '''import copy,json,xml.etree.ElementTree as ET
from pathlib import Path
from sdk.omindos_client import ManipulationClient

def finish(client, task, expected="succeeded"):
    result=client.wait(task,timeout=120)
    assert result["status"] == expected, result
    return result["result"]

arms=Path("robot/arms.json")
waist=Path("robot/waist.json")
with ManipulationClient() as client:
    info=finish(client,client.get_mobile_model(arm_config=arms,waist_config=waist))
    assert info["dof"] == 15
    start=info["home_positions_rad"]
    identity=[[1.,0.,0.,0.],[0.,1.,0.,0.],[0.,0.,1.,0.],[0.,0.,0.,1.]]
    finish(client,client.plan_mobile_object_motion(start,[identity],info["home_tcp_poses"],
        obstacles=[{"size_m":[3,3,3],"position_m":[0,0,1]}],arm_config=arms,waist_config=waist),"failed")
    try:
        client.plan_mobile_object_motion(start,[identity],[])
    except ValueError:
        pass
    else:
        raise AssertionError("invalid grasp shape accepted")
    goal=start.copy();goal[0],goal[7],goal[14]=.1,.2,-.2
    path=finish(client,client.plan_mobile_joint_motion(start,goal,arm_config=arms,waist_config=waist))
    assert path["positions_rad"][0] == start and path["positions_rad"][-1] == goal
    finish(client,client.plan_mobile_joint_motion(start,goal,obstacles=[
        {"size_m":[3,3,3],"position_m":[0,0,1]}],arm_config=arms,waist_config=waist),"failed")
    tree=ET.parse("robot/arm-model/RM75-B.urdf")
    tree.find("joint[@name='joint_1']/limit").set("upper","1.60")
    changed=Path("robot/arm-model/changed.urdf");tree.write(changed)
    config=json.loads(arms.read_text());config["left"]["urdf_path"]="arm-model/changed.urdf"
    config["left"]["model"]="URDF-7R"
    changed_config=Path("robot/changed-arms.json");changed_config.write_text(json.dumps(config))
    info=finish(client,client.get_mobile_model(arm_config=changed_config,waist_config=waist))
    assert info["limits_rad"][1][1] == 1.60
    goal[1]=1.7
    finish(client,client.plan_mobile_joint_motion(start,goal,arm_config=changed_config,waist_config=waist),"failed")
    task=client.run_demo("integrated_task")
    assert client.stop(task)["status"] == "canceled"
    assert client.get_status(task)["status"] == "canceled"
    task=client.run_demo("integrated_task")
    result=client.wait(task,timeout=.000001)
    assert result["status"] == "failed" and result["error"]["code"] == "timeout"
    trace=json.loads(Path("results/customer-api.json").read_text())["mobile_task"]["result"]["navigation"]
    trace["samples"][-1]["twist"][0]=.1
    finish(client,client.plan_mobile_task(navigation_trace=trace),"failed")
print(json.dumps({"status":"passed","checks":["requested-15-joint-endpoints","collision-failure",
    "object-motion-collision-failure","invalid-grasp-shape","external-URDF-limits",
    "URDF-out-of-range-failure","cancel","timeout","stop-gate-failure"]}))
'''


def main():
    cli = argparse.ArgumentParser(description=__doc__)
    cli.add_argument('--candidate', type=Path, required=True)
    cli.add_argument('--python', default='/usr/bin/python3')
    args = cli.parse_args()
    candidate = args.candidate.resolve(strict=True)
    for line in (candidate/'SHA256SUMS').read_text().splitlines():
        digest, name = line.split()
        if name not in (RUNTIME, CLIENT):
            raise RuntimeError('Unexpected asset in checksums')
        with (candidate/name).open('rb') as f:
            if hashlib.file_digest(f, 'sha256').hexdigest() != digest:
                raise RuntimeError('Asset checksum mismatch: '+name)
    env = {k:v for k,v in os.environ.items() if k not in
           ('PYTHONPATH', 'PYTHONHOME', 'OMINDOS_MANIPULATION_RUNTIME')}
    started = time.monotonic()
    results = []
    with tempfile.TemporaryDirectory(prefix='omindos-customer-acceptance-') as temp:
        destination = Path(temp)
        with tarfile.open(candidate/CLIENT) as bundle:
            bundle.extractall(destination, filter='data')
        with tarfile.open(candidate/RUNTIME) as bundle:
            bundle.extractall(destination/'OmindOS-Manipulation-0.2', filter='data')
        customer = destination/'OmindOS-Manipulation-0.2'
        # Source is not available from the SDK's working directory. No checkout
        # path is put in sys.path or passed to the algorithm process.
        if (customer/'community').exists():
            raise RuntimeError('Development directory leaked into customer kit')
        for name, command in [('example', [args.python, 'examples/mobile_dual_api.py']),
                              ('dual-task-example', [args.python, 'examples/dual_task_api.py']),
                              ('negative-and-config-checks', [args.python, '-c', CHECK])]:
            run = subprocess.run(command, cwd=customer, env=env, capture_output=True, text=True, timeout=240)
            (candidate/(name+'.log')).write_text(run.stdout+run.stderr)
            if run.returncode:
                raise RuntimeError(f'{name}: exit={run.returncode}; see {candidate/name}.log')
            results.append(dict(name=name, status='passed'))
        core = ('community', 'motion', 'basis', 'modeling', 'robot_sim', 'visualization')
        sources = [str(p.relative_to(customer/'runtime')) for p in (customer/'runtime').rglob('*.py')
                   if any(x in core for x in p.relative_to(customer/'runtime').parts)]
        if sources:
            raise RuntimeError('Plaintext core sources found: '+str(sources[:10]))
        report = json.loads((customer/'results/customer-api.json').read_text())
        plan = report['mobile_task']['result']
        dual = json.loads((customer/'results/dual-task-api.json').read_text())
        assert dual['status'] == 'passed' and dual['dof'] == 15
        summary = dict(status='passed', version='0.2', package_revision='runtime.2',
                       independent_source_directory=True, client_python=args.python,
                       validation_platform=__import__('platform').platform(),
                       checks=results, core_plaintext_files=0, dof=15,
                       base_final_pose=plan['navigation']['samples'][-1]['pose'],
                       phase_history=plan['phase_history'],
                       dual_task=dict(samples=len(dual['positions_rad']),
                                      metrics={k: v for k, v in dual.items() if k.startswith('max_')},
                                      timed=dual['timed'], hardware_executed=dual['hardware_executed']),
                       elapsed_s=round(time.monotonic()-started, 2), hardware='not tested')
    (candidate/'VALIDATION.json').write_text(json.dumps(summary, indent=2)+'\n')
    print(json.dumps(summary))


if __name__ == '__main__':
    main()
