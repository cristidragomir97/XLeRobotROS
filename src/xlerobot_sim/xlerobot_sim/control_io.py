"""Joint I/O only: controller_manager owns commands, Python owns MuJoCo physics."""
import mmap
from pathlib import Path
import struct
import time
import numpy as np

MAGIC = 0x584C45524F424F54
CAPACITY = 64
SIZE = 1592
from .configuration import WHEEL_JOINTS

class ControlExchange:
    def __init__(self, path, robot):
        self.names = robot.control_names
        self.robot = robot
        self.file = Path(path).open('w+b')
        self.file.truncate(SIZE)
        self.memory = mmap.mmap(self.file.fileno(), SIZE)
        self.memory[:] = bytes(SIZE)
        self.position = np.ndarray(CAPACITY, dtype='<f8', buffer=self.memory, offset=56)
        self.velocity = np.ndarray(CAPACITY, dtype='<f8', buffer=self.memory, offset=568)
        self.was_active = False
        self.sequence = 0
        self.update()
        struct.pack_into('<QI', self.memory, 0, MAGIC, len(self.names))

    def update(self):
        self.sequence += 1
        struct.pack_into('<Q', self.memory, 16, self.sequence)
        model, data = self.robot.model, self.robot.data
        for index, name in enumerate(self.names):
            joint = model.joint(name)
            self.position[index] = data.qpos[joint.qposadr[0]]
            self.velocity[index] = data.qvel[joint.dofadr[0]]
        struct.pack_into('<d', self.memory, 32, time.monotonic())
        self.sequence += 1
        struct.pack_into('<Q', self.memory, 16, self.sequence)

    def apply(self):
        active = False
        for _ in range(10):
            seq = struct.unpack_from('<Q', self.memory, 24)[0]
            if seq & 1:
                continue
            age = time.monotonic() - struct.unpack_from('<d', self.memory, 40)[0]
            enabled = struct.unpack_from('<Q', self.memory, 48)[0]
            command = np.frombuffer(self.memory, dtype='<f8', count=len(self.names), offset=1080).copy()
            if seq != struct.unpack_from('<Q', self.memory, 24)[0]:
                continue
            active = bool(enabled and seq and age < .5 and np.isfinite(command).all())
            break
        if active:
            self.robot.set_positions(self.names[3:], command[3:])
            rates = self.robot.base_control.limit_wheels(command[:3])
            body = np.linalg.solve(self.robot.base_control.kinematics, rates)
        else:
            body = np.zeros(3)
            if self.was_active:
                # Soft physics limits can deflect slightly beyond the command range.
                # Hold the nearest admissible position when the controller stops.
                held = []
                for name in self.names[3:]:
                    actuator, address = self.robot.positions[name]
                    limits = self.robot.model.actuator_ctrlrange[actuator]
                    held.append(np.clip(self.robot.data.qpos[address], *limits))
                self.robot.set_positions(self.names[3:], held)
        self.was_active = active
        return body

    def close(self):
        del self.position, self.velocity
        self.memory.close()
        self.file.close()
