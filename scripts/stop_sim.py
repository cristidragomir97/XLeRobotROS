#!/usr/bin/env python3
"""Gracefully stop this container's XLeRobot launch so RTAB-Map saves its database."""
from pathlib import Path
import os
import signal
import time


def launches():
    matches = []
    for entry in Path('/proc').iterdir():
        if not entry.name.isdecimal():
            continue
        try:
            args = (entry / 'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
        if any(args[i:i+2] == [b'launch', b'xlerobot_sim'] and
               args[i+2] in (b'sim.launch.py', b'sim_standard.launch.py', b'sim_mod101.launch.py')
               for i in range(len(args)-2)):
            matches.append(int(entry.name))
    return matches


def main():
    processes = launches()
    for pid in processes:
        try:
            os.kill(pid, signal.SIGINT)
        except ProcessLookupError:
            pass
    deadline = time.monotonic()+15.
    while set(launches()) & set(processes):
        if time.monotonic() >= deadline:
            raise RuntimeError('Simulation is still saving/shutting down; container was left running')
        time.sleep(.1)
    if processes:
        print('Simulation stopped; RTAB-Map shutdown completed.')


if __name__ == '__main__':
    main()
