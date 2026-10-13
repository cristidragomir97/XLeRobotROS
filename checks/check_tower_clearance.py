#!/usr/bin/env python3
"""Triangle-mesh tower clearance; optional sampled range/rail-length studies.

FCL checks the actual displayed triangles, not MuJoCo's convex mesh hulls.
The sweep is a finite sample, not continuous collision checking or planning.
"""
import argparse
from collections import Counter
import io
import json
from pathlib import Path
import tempfile

import fcl
import mujoco
import numpy as np
import xacro
from ament_index_python.packages import get_package_share_directory
from xlerobot_sim.configuration import description_path, arm_joints, HEAD_JOINTS
from xlerobot_sim.model import Robot, prepare_scene

ROOT = Path(__file__).resolve().parents[1]


class Clearance:
    def __init__(self, robot):
        self.robot = robot
        self.model, self.data = robot.model, robot.data
        self.initial_qpos = self.data.qpos.copy()
        self.objects = {}
        self.meshes = {}
        self.arms = {side: [] for side in ('L', 'R')}
        self.tower = []
        for geom in range(self.model.ngeom):
            if self.model.geom_group[geom] != 2:
                continue
            name = self.model.body(self.model.geom_bodyid[geom]).name
            if name in ('top_base_link', 'head_pan_link', 'head_tilt_link'):
                self.tower.append(geom)
            for side in self.arms:
                if name.startswith(f'mod101_{side}_'):
                    self.arms[side].append(geom)

    def object(self, geom):
        model = self.model
        if geom not in self.objects:
            if model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_MESH:
                mesh = int(model.geom_dataid[geom])
                if mesh not in self.meshes:
                    va, vn = model.mesh_vertadr[mesh], model.mesh_vertnum[mesh]
                    fa, fn = model.mesh_faceadr[mesh], model.mesh_facenum[mesh]
                    shape = fcl.BVHModel()
                    shape.beginModel(vn, fn)
                    shape.addSubModel(np.asarray(model.mesh_vert[va:va+vn], dtype=np.float64),
                                      np.asarray(model.mesh_face[fa:fa+fn], dtype=np.int32))
                    shape.endModel()
                    self.meshes[mesh] = shape
                shape = self.meshes[mesh]
            elif model.geom_type[geom] == mujoco.mjtGeom.mjGEOM_BOX:
                shape = fcl.Box(*(2*model.geom_size[geom]))
            else:
                raise ValueError(f'Unsupported visible arm/tower primitive: {model.geom_type[geom]}')
            self.objects[geom] = fcl.CollisionObject(shape)
        obj = self.objects[geom]
        obj.setTransform(fcl.Transform(self.data.geom_xmat[geom].reshape(3, 3), self.data.geom_xpos[geom]))
        return obj

    def pairs(self, side, distances=False):
        found = []
        for arm in self.arms[side]:
            for tower in self.tower:
                # Convex hull distance is a conservative broad phase only.
                threshold = .1 if distances else .001
                if mujoco.mj_geomDistance(self.model, self.data, arm, tower, threshold, None) >= threshold:
                    continue
                a, b = self.object(arm), self.object(tower)
                collided = bool(fcl.collide(a, b, fcl.CollisionRequest(), fcl.CollisionResult()))
                if collided or distances:
                    distance = 0. if collided else float(fcl.distance(a, b, fcl.DistanceRequest(), fcl.DistanceResult()))
                    found.append({'arm_link': self.model.body(self.model.geom_bodyid[arm]).name,
                                  'tower_mesh': self.model.mesh(self.model.geom_dataid[tower]).name,
                                  'collision': collided, 'clearance_m': distance})
        return sorted(found, key=lambda pair: pair['clearance_m'])

    def sweep(self, side):
        from itertools import product
        self.data.qpos[:] = self.initial_qpos
        names = arm_joints('mod101')[side] + HEAD_JOINTS
        grid_names = names[:3]
        grid = [dict(zip(grid_names, map(float, angles))) for angles in
                product(np.linspace(-1.57, 1.57, 7), np.linspace(0., np.pi, 7), np.linspace(0., np.pi, 5))]
        rng = np.random.default_rng(23)
        random = [{name: float(rng.uniform(*self.model.jnt_range[self.model.joint(name).id]))
                   for name in names} for _ in range(300)]
        counts, first = Counter(), {}
        collisions = Counter()
        for phase, configurations in [('base_shoulder_elbow_grid', grid), ('whole_arm_and_head_random', random)]:
            for configuration in configurations:
                for name, angle in configuration.items():
                    self.data.qpos[self.model.joint(name).qposadr[0]] = angle
                # mj_forward does not solve equality constraints. Place rack sliders
                # at the URDF-equivalent motor-to-slider relation for this study.
                for eq in range(self.model.neq):
                    if self.model.eq_type[eq] == mujoco.mjtEq.mjEQ_JOINT:
                        slave = self.model.jnt_qposadr[self.model.eq_obj1id[eq]]
                        master = self.model.jnt_qposadr[self.model.eq_obj2id[eq]]
                        self.data.qpos[slave] = self.model.eq_data[eq, 0] + self.model.eq_data[eq, 1]*self.data.qpos[master]
                mujoco.mj_forward(self.model, self.data)
                hits = self.pairs(side)
                collisions[phase] += bool(hits)
                for pair in hits:
                    link = pair['arm_link']
                    counts[link] += 1
                    first.setdefault(link, configuration)
        return {'sampled_poses': len(grid)+len(random), 'colliding_poses': sum(collisions.values()),
                'phases': {phase: {'sampled_poses': len(configurations), 'colliding_poses': collisions[phase]}
                           for phase, configurations in [('base_shoulder_elbow_grid', grid), ('whole_arm_and_head_random', random)]},
                'collisions_per_link': dict(counts), 'first_collision_joint_positions_rad': first}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--sweep', action='store_true')
    parser.add_argument('--shoulder-length', type=float, help='Study another rail length in metres without changing the authored description')
    parser.add_argument('--mount-rise', type=float, help='Study a mount Z offset in metres without changing the authored description')
    parser.add_argument('--output', type=Path)
    args = parser.parse_args()
    with tempfile.TemporaryDirectory(prefix='xlerobot-clearance-') as directory:
        description = description_path('mod101', directory)
        if args.shoulder_length is not None or args.mount_rise is not None:
            source = Path(get_package_share_directory('xlerobot_description')) / 'urdf/xlerobot_mod101.urdf.xacro'
            text = source.read_text()
            if args.shoulder_length is not None:
                import re
                text = re.sub(r'shoulder_ext_length="[^"]+"', f'shoulder_ext_length="{args.shoulder_length}"', text)
            if args.mount_rise is not None:
                import re
                text = re.sub(r'(<xacro:property name="arm_mount_rise" value=")[^"]+("/>)',
                              lambda match: match.group(1)+str(args.mount_rise)+match.group(2), text)
            document = xacro.parse(io.StringIO(text))
            xacro.process_doc(document)
            description.write_text(document.toprettyxml(indent='  '))
        robot = Robot(prepare_scene(ROOT / 'upstream/simulation/mujoco', directory, 'empty',
                                    configuration='mod101', description=description))
        check = Clearance(robot)
        report = {'configuration': 'mod101', 'geometry': 'compiled visual triangles and rail boxes',
                  'joint_pose': 'zero arm/head joints', 'study_shoulder_length_m': args.shoulder_length,
                  'study_mount_rise_m': args.mount_rise, 'neutral': {}, 'sweep': {}}
        for side in check.arms:
            pairs = check.pairs(side, distances=True)
            report['neutral'][side] = pairs
            hits = [p for p in pairs if p['collision']]
            print(f'{side}: neutral {len(hits)} overlapping arm/tower pairs; closest clearance {pairs[0]["clearance_m"]*1000:.3f} mm', flush=True)
        if args.sweep:
            for side in check.arms:
                report['sweep'][side] = check.sweep(side)
                result = report['sweep'][side]
                print(f'{side}: {result["colliding_poses"]}/{result["sampled_poses"]} sampled poses intersect the tower', flush=True)
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(json.dumps(report, indent=2)+'\n')
        assert not any(p['collision'] for pairs in report['neutral'].values() for p in pairs), 'Neutral arm/tower overlap; see report'
        print('PASS: neutral arm/tower meshes do not intersect; sampled sweeps do not certify arbitrary trajectories')


if __name__ == '__main__':
    main()
