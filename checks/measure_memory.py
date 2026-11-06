"""Linux application memory for the robot container and its native viewer.

PSS apportions shared pages; do not add process RSS to estimate total usage.
Windows GPU allocations and the WSL VM's retained filesystem cache are separate.
"""
import argparse
import json
from pathlib import Path
import subprocess

CONTAINER_PROBE = r'''
import json, os
from pathlib import Path
rows=[]
for p in Path('/proc').iterdir():
    if not p.name.isdigit() or int(p.name)==os.getpid(): continue
    try:
        stat={l.split(':')[0]:int(l.split()[1]) for l in (p/'smaps_rollup').read_text().splitlines() if ':' in l and l.split()[1].isdigit()}
        if not stat.get('Rss'): continue
        name=(p/'comm').read_text().strip()
        argv=(p/'cmdline').read_bytes().split(b'\0')
        role=('simulation' if name in ('sim','camera') or b'xlerobot_sim.prepare_model' in argv
              else 'runtime' if name in ('docker-init','sleep','bash','sh') else 'ros')
        rows.append(dict(pid=int(p.name),name=name,role=role,rss_kib=stat['Rss'],pss_kib=stat['Pss'],private_kib=stat.get('Private_Clean',0)+stat.get('Private_Dirty',0)))
    except (OSError, ValueError): continue
stat={l.split()[0]:int(l.split()[1]) for l in Path('/sys/fs/cgroup/memory.stat').read_text().splitlines()}
current=int(Path('/sys/fs/cgroup/memory.current').read_text())
print(json.dumps(dict(processes=sorted(rows,key=lambda r:r['pss_kib'],reverse=True),cgroup_current_bytes=current,inactive_file_bytes=stat.get('inactive_file',0),docker_working_bytes=current-stat.get('inactive_file',0))))
'''


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--container',default='xlerobot-ros-dev-1')
    parser.add_argument('--runtime',type=Path,default=Path('.runtime'))
    parser.add_argument('--output',type=Path)
    args=parser.parse_args()
    if not Path('/proc/self/smaps_rollup').is_file():
        parser.error('Memory measurement requires a Linux/WSL shell with smaps_rollup')
    result=json.loads(subprocess.check_output(['docker','exec',args.container,'python3','-c',CONTAINER_PROBE],text=True))
    viewers=[]
    for p in Path('/proc').iterdir():
        if not p.name.isdigit(): continue
        try:
            command=(p/'cmdline').read_bytes().decode().split('\0')
            if 'multiros.viewer' not in command or '--runtime' not in command: continue
            runtime=Path(command[command.index('--runtime')+1])
            if not runtime.is_absolute():runtime=(p/'cwd').resolve()/runtime
            if runtime.resolve()!=args.runtime.resolve():continue
            stat={l.split(':')[0]:int(l.split()[1]) for l in (p/'smaps_rollup').read_text().splitlines() if ':' in l and l.split()[1].isdigit()}
            viewers.append(dict(pid=int(p.name),rss_kib=stat['Rss'],pss_kib=stat['Pss'],private_kib=stat.get('Private_Clean',0)+stat.get('Private_Dirty',0)))
        except (OSError,ValueError):continue
    result['native_viewers']=viewers
    simulation = [p for p in result['processes'] if p.get('role') == 'simulation']
    overhead = [p for p in result['processes'] if p.get('role') == 'runtime']
    ros = [p for p in result['processes'] if p not in simulation and p not in overhead]
    result['groups_pss_kib'] = {
        'simulation_physics_and_camera': sum(p['pss_kib'] for p in simulation),
        'ros_mapping_navigation_controllers_dashboard': sum(p['pss_kib'] for p in ros),
        'native_viewer': sum(p['pss_kib'] for p in viewers),
        'container_runtime_overhead': sum(p['pss_kib'] for p in overhead)}
    result['application_pss_kib']=sum(p['pss_kib'] for p in result['processes']+viewers)
    result['definitions']='PSS is proportionally shared Linux application memory; GPU/Windows VM allocations are not included. Docker working bytes subtract inactive file cache. Groups are process-based; ROS bindings inside the sim/camera process are included with simulation.'
    text=json.dumps(result,indent=2)+'\n'
    if args.output:args.output.write_text(text)
    print(text,end='')


if __name__=='__main__':main()
