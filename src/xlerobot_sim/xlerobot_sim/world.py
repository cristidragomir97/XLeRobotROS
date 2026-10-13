"""Attach a custom three-room home with attributed RoboCasa/Lightwheel assets."""
import copy,json,os,math
from pathlib import Path
import xml.etree.ElementTree as ET


def default_assets():
    local=Path(__file__).resolve().parents[3]/'assets/robocasa'
    return str(local if local.is_dir() else Path(os.environ.get('XLEROBOT_WORLD_ASSETS','/opt/xlerobot-assets/robocasa')))


def add_home(destination, assets_dir):
    destination=Path(destination);assets_dir=Path(assets_dir)
    manifest=assets_dir/'manifest.json'
    if not manifest.is_file():raise RuntimeError('RoboCasa assets are missing. Run ./scripts/manage.sh world-assets first.')
    metadata=json.loads(manifest.read_text());records={a['id']:a for a in metadata['assets']}
    tree=ET.parse(destination);scene=tree.getroot();world=scene.find('worldbody');assets=scene.find('asset')
    # MuJoCo 3.6 fixed-body fusion misbinds named sensor sites when additional
    # static appliance sites precede the robot sites in the compiled model.
    robot_path = destination.parent/'robot.xml'
    robot_tree = ET.parse(robot_path)
    robot_tree.getroot().find('compiler').set('fusestatic', 'false')
    robot_tree.write(robot_path)
    world.find("geom[@name='floor']").set('pos','0 0 -0.08')
    # Keep range measurements active without drawing 360 debug rays in the GUI.
    rgba = scene.find('visual/rgba')
    if rgba is None:rgba = ET.SubElement(scene.find('visual'), 'rgba')
    rgba.set('rangefinder','0 0 0 0')
    local = Path(__file__).resolve().parents[1]/'worlds/home.xml'
    if not local.is_file():
        from ament_index_python.packages import get_package_share_directory
        local = Path(get_package_share_directory('xlerobot_sim'))/'worlds/home.xml'
    source=ET.parse(local).getroot()
    for element in source.find('asset'):assets.append(copy.deepcopy(element))
    scene.append(copy.deepcopy(source.find('default')))
    for element in source.find('worldbody'):world.append(copy.deepcopy(element))

    def instance(asset_id, name, xy, support_height, *, loose=False):
        record=records[asset_id];xml=assets_dir/record['xml'];model=ET.parse(xml).getroot();prefix=name+'_'
        # Namespace model-local definitions and every reference. File paths remain
        # attached to the owning downloaded asset; the robot assets are untouched.
        refs=('name','class','childclass','mesh','material','texture','site','joint','body','joint1','joint2','body1','body2')
        for element in model.iter():
            for key in refs:
                if key in element.attrib:element.set(key,prefix+element.get(key))
            if 'file' in element.attrib:element.set('file',str((xml.parent/element.get('file')).resolve()))
            if element.tag=='geom':
                category=element.get('class','')
                if category.endswith('collision'):
                    element.set('contype','2');element.set('conaffinity','3');element.set('rgba','0 0 0 0');element.set('group','3')
                elif category.endswith(('visual','region','spawn')):
                    element.set('contype','0');element.set('conaffinity','0');element.set('group','2')
            if element.tag=='site':element.set('rgba','0 0 0 0')
        for element in model.find('asset'):assets.append(element)
        defaults=model.find('default')
        if defaults is not None:scene.append(defaults)
        body=model.find('.//body[@name="'+prefix+'object"]')
        body.set('name',name);body.set('pos',f'{xy[0]} {xy[1]} {support_height-record["bounds_m"][0][2]+0.002}')
        if loose:ET.SubElement(body,'freejoint',name=name+'_free')
        world.append(body)
        return {'name':name,'asset':asset_id,'support_height':support_height,'position':body.get('pos'),'loose':loose}

    instances=[instance('Refrigerator040','kitchen_fridge',[9.15,4.45],0),
               instance('Stove066','kitchen_stove',[8.1,4.45],0)]
    # Dining chairs, bookshelf books and a rug are simple authored furniture.
    def box(body,name,pos,size,rgba,solid=True):
        return ET.SubElement(body,'geom',name=name,type='box',pos=' '.join(map(str,pos)),size=' '.join(map(str,size)),rgba=rgba,**{'class':'home_solid' if solid else 'home_visual'})
    for number,(x,y,yaw) in enumerate([(7.5,-1.65,0),(8.5,-1.65,0),(7.5,.05,math.pi),(8.5,.05,math.pi)]):
        chair=ET.SubElement(world,'body',name=f'home_chair_{number}',pos=f'{x} {y} 0',quat=f'{math.cos(yaw/2)} 0 0 {math.sin(yaw/2)}')
        box(chair,f'chair_{number}_seat',[0,0,.44],[.23,.23,.035],'.53 .34 .19 1')
        box(chair,f'chair_{number}_back',[0,-.21,.7],[.23,.035,.28],'.53 .34 .19 1')
        for j,(dx,dy) in enumerate([(-.18,-.18),(.18,-.18),(-.18,.18),(.18,.18)]):box(chair,f'chair_{number}_leg_{j}',[dx,dy,.20],[.025,.025,.20],'.16 .18 .2 1')
    box(world,'home_rug',[1.5,8,.003],[1.5,1.3,.003],'.62 .42 .26 1',False)
    for i in range(12):box(world,'home_book_'+str(i),[6.1+i*.055,9.3,.71],[.02,.12,.13],('.65 .20 .16 1' if i%2 else '.20 .32 .62 1'))
    props=[('GlassCup008','pickup_cup_kitchen',[8.12,2.40],.67),
           ('HoneyBottle002','pickup_honey_kitchen',[8.4,2.5],.67),
           ('GlassCup008','pickup_cup_dining',[7.6,-.8],.72),
           ('HoneyBottle002','pickup_honey_dining',[8.3,-.65],.72),
           ('GlassCup008','pickup_cup_living',[1.3,7.85],.47),
           ('HoneyBottle002','pickup_honey_living',[1.7,7.9],.47)]
    for asset,name,xy,height in props:instances.append(instance(asset,name,xy,height,loose=True))
    for i,(x,y,z) in enumerate([(8.3,2.30,.691),(7.9,-.8,.741),(1.5,7.7,.491)]):
        body=ET.SubElement(world,'body',name=f'pickup_block_{i}',pos=f'{x} {y} {z}')
        ET.SubElement(body,'freejoint',name=f'pickup_block_{i}_free')
        ET.SubElement(body,'inertial',pos='0 0 0',mass='.035',diaginertia=' '.join(str(.035*(a*a+b*b)/12) for a,b in ((.03,.04),(.03,.04),(.03,.03))))
        box(body,f'pickup_block_{i}_geom',[0,0,0],[.015,.015,.02],['.8 .16 .12 1','.15 .5 .8 1','.18 .65 .25 1'][i])
    # The upstream robot contributes extra lights. Bound their combined exposure
    # so the shared home's pale checker tiles retain RGB-D tracking features.
    headlight = scene.find('visual/headlight')
    headlight.set('ambient','.15 .15 .15')
    headlight.set('diffuse','.3 .3 .3')
    for light in scene.findall('worldbody/light'):
        light.set('diffuse','.25 .25 .25')
        light.set('ambient','.02 .02 .02')
    for light in robot_tree.getroot().findall('worldbody/light'):
        light.set('diffuse','.25 .25 .25')
        light.set('ambient','.02 .02 .02')
    robot_tree.write(robot_path)
    ET.indent(scene);tree.write(destination,encoding='unicode')
    (destination.parent/'world_manifest.json').write_text(json.dumps({'world':'robocasa_home','rooms':['test','kitchen_dining','living'],'test_room':{'bounds_m':[[-6,-6],[6,6]],'size_m':[12,12],'spawn_xy':[0,0]},'assets':metadata,'instances':instances},indent=2)+'\n')
    return destination
