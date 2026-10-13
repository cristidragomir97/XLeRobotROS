#!/usr/bin/env python3
"""Prepare attributed RoboCasa/Lightwheel assets for the composed ROS scene.

Run with /opt/xlerobot-python/bin/python in the container. MuJoCo computes
inertials before the robot scene disables automatic geometry-derived mass.
Source models remain unchanged. Prepared XML keeps relative mesh/texture paths.
"""
from pathlib import Path
import copy,json,hashlib,urllib.request,zipfile
import xml.etree.ElementTree as ET
import mujoco
import numpy as np
ROOT=Path(__file__).resolve().parents[1]
ASSETS=ROOT/'assets/robocasa'
DATASET='nvidia/PhysicalAI-Robotics-Manipulation-Objects-Kitchen-MJCF'
SELECTION=[('fixtures_lightwheel','fridges','Refrigerator040',1.),('fixtures_lightwheel','stoves','Stove066',1.),('objects_lightwheel','glass_cup','GlassCup008',.45),('objects_lightwheel','honey_bottle','HoneyBottle002',.45)]
existing=ASSETS/'manifest.json'
revision=json.loads(existing.read_text())['revision'] if existing.exists() else json.load(urllib.request.urlopen('https://huggingface.co/api/datasets/'+DATASET))['sha']
manifest={'dataset':DATASET,'revision':revision,'license':'CC-BY-4.0','source_url':'https://huggingface.co/datasets/'+DATASET,'assets':[]}
for section,category,item,scale in SELECTION:
 folder=ASSETS/category/item;archive=ASSETS/'.downloads'/(category+'.zip');archive.parent.mkdir(parents=True,exist_ok=True)
 url=f'https://huggingface.co/datasets/{DATASET}/resolve/{revision}/{section}/{category}.zip'
 if not archive.exists():urllib.request.urlretrieve(url,archive)
 if not (folder/'model.xml').exists():
  with zipfile.ZipFile(archive) as z:
   for name in z.namelist():
    if name.startswith(category+'/'+item+'/') and '..' not in Path(name).parts:z.extract(name,ASSETS)
 source=folder/'model.xml';r=ET.parse(source).getroot()
 for element in r.iter():
  if element.tag=='mesh':
   old=[float(x) for x in element.get('scale','1 1 1').split()];element.set('scale',' '.join(str(x*scale) for x in old))
  if 'pos' in element.attrib:element.set('pos',' '.join(str(float(x)*scale) for x in element.get('pos').split()))
  if element.tag in ('geom','site') and 'size' in element.attrib:element.set('size',' '.join(str(float(x)*scale) for x in element.get('size').split()))
  if element.tag=='joint' and element.get('type')=='slide' and 'range' in element.attrib:element.set('range',' '.join(str(float(x)*scale) for x in element.get('range').split()))
 # Resolve inherited defaults with the native compiler, then save mass tensors.
 temp=folder/'scaled.tmp.xml';ET.ElementTree(r).write(temp)
 model=mujoco.MjModel.from_xml_path(str(temp));data=mujoco.MjData(model);mujoco.mj_forward(model,data)
 for body in r.find('worldbody').iter('body'):
  name=body.get('name')
  if not name:continue
  i=mujoco.mj_name2id(model,mujoco.mjtObj.mjOBJ_BODY,name)
  if i<0 or model.body_mass[i]<=0:continue
  existing=body.find('inertial')
  if existing is not None:body.remove(existing)
  ET.SubElement(body,'inertial',pos=' '.join(map(str,model.body_ipos[i])),quat=' '.join(map(str,model.body_iquat[i])),mass=str(model.body_mass[i]),diaginertia=' '.join(map(str,model.body_inertia[i])))
 prepared=folder/'prepared.xml';ET.indent(r);ET.ElementTree(r).write(prepared,encoding='unicode');temp.unlink()
 # Source region bbox is only metadata, never collision geometry.
 bbox=next(g for g in r.iter('geom') if g.get('name') in ('reg_bbox','reg_main')) if item.startswith(('Glass','Honey')) else None
 bounds = []
 for g in range(model.ngeom):
  if model.geom_type[g] != mujoco.mjtGeom.mjGEOM_MESH or model.geom_contype[g] or model.geom_conaffinity[g]:continue
  mesh=model.geom_dataid[g];start=model.mesh_vertadr[mesh];count=model.mesh_vertnum[mesh]
  vertices=model.mesh_vert[start:start+count] @ data.geom_xmat[g].reshape(3,3).T + data.geom_xpos[g]
  bounds.append(vertices)
 vertices=np.concatenate(bounds);bbox_min=vertices.min(axis=0);bbox_max=vertices.max(axis=0)
 manifest['assets'].append({'id':item,'category':category,'scale':scale,'xml':str(prepared.relative_to(ASSETS)), 'archive_sha256':hashlib.sha256(archive.read_bytes()).hexdigest(),'model_sha256':hashlib.sha256(source.read_bytes()).hexdigest(), 'bounds_m':[bbox_min.tolist(),bbox_max.tolist()], 'bbox_half_size':list(map(float,bbox.get('size').split())) if bbox is not None else None})
 print('Prepared',item,'scale',scale,flush=True)
(ASSETS/'manifest.json').write_text(json.dumps(manifest,indent=2)+'\n')
