"""Compare original and reduced models: collision/inertia/control and dynamics."""
import argparse
import json
import mujoco
import numpy as np


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--reference',required=True)
    parser.add_argument('--candidate',required=True)
    parser.add_argument('--snapshot')
    args=parser.parse_args()
    def load(path):
        return (mujoco.MjModel.from_binary_path(path) if path.endswith('.mjb')
                else mujoco.MjModel.from_xml_path(path))
    original=load(args.reference)
    reduced=load(args.candidate)
    assert (original.nq,original.nv,original.nu,original.ngeom)==(reduced.nq,reduced.nv,reduced.nu,reduced.ngeom)
    for field in ('body_mass','body_inertia','body_ipos','body_iquat','body_pos','body_quat',
                  'jnt_pos','jnt_axis','jnt_range','dof_damping','dof_frictionloss','dof_armature',
                  'actuator_gainprm','actuator_biasprm','actuator_ctrlrange','actuator_forcerange',
                  'geom_contype','geom_conaffinity','cam_pos','cam_quat','cam_fovy'):
        np.testing.assert_array_equal(getattr(original,field),getattr(reduced,field),err_msg=field)
    collision=(original.geom_contype!=0)|(original.geom_conaffinity!=0)
    for field in ('geom_type','geom_size','geom_pos','geom_quat','geom_friction','geom_solref','geom_solimp'):
        np.testing.assert_array_equal(getattr(original,field)[collision],getattr(reduced,field)[collision],err_msg=field)
    for g in np.flatnonzero(collision & (original.geom_type==mujoco.mjtGeom.mjGEOM_MESH)):
        i,j=int(original.geom_dataid[g]),int(reduced.geom_dataid[g])
        assert original.mesh(i).name==reduced.mesh(j).name
        for array, adr, num in [('mesh_vert','mesh_vertadr','mesh_vertnum'),
                                ('mesh_face','mesh_faceadr','mesh_facenum')]:
            slices=[]
            for model,index in [(original,i),(reduced,j)]:
                start=int(getattr(model,adr)[index]);count=int(getattr(model,num)[index])
                slices.append(getattr(model,array)[start:start+count])
            np.testing.assert_array_equal(*slices,err_msg=original.mesh(i).name)
    a,b=mujoco.MjData(original),mujoco.MjData(reduced)
    if args.snapshot:
        with np.load(args.snapshot) as state:
            for data in (a,b):
                for field in ('qpos','qvel','ctrl'):getattr(data,field)[:]=state[field]
    for _ in range(300):
        mujoco.mj_step(original,a);mujoco.mj_step(reduced,b)
    np.testing.assert_array_equal(a.qpos,b.qpos)
    np.testing.assert_array_equal(a.qvel,b.qvel)
    def triangles(m):
        return sum(int(m.mesh_facenum[int(m.geom_dataid[g])]) for g in range(m.ngeom)
                   if m.geom_type[g]==mujoco.mjtGeom.mjGEOM_MESH and m.geom_group[g] in (0,1,2) and m.geom_rgba[g,3]>0)
    print(json.dumps(dict(physics='identical after 300 steps',original_visible_triangles=triangles(original),
                         simplified_visible_triangles=triangles(reduced)),indent=2))


if __name__=='__main__':main()
