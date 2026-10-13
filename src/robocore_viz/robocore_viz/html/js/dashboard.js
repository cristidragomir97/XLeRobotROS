/* Configurable robot dashboard, using the vendored ROSboard websocket bridge. */
'use strict';
(() => {
  const $ = id => document.getElementById(id);
  const config = window.ROBOCORE_CONFIG;
  const bindings = config.topics;
  const defaults = {camera:config.cameras[0].topic, map:config.maps[0].topic};
  document.title = config.name + ' · Robocore Viz';
  $('robot-name').textContent = config.name;
  $('footer-name').textContent = config.name.toUpperCase();
  $('robot-tagline').textContent = config.tagline;
  document.querySelector('.brand').setAttribute('aria-label', config.name + ' home');
  $('drive-topic').textContent = config.drive.topic;
  $('strafe-mode').hidden = !config.drive.omnidirectional;
  for (const [id, items] of [['camera-select',config.cameras],['map-select',config.maps]])
    for (const item of items) $(id).add(new Option(item.label,item.topic));
  const storageKey = 'robocore-dashboard-' + config.id + '-v1';
  let preferences = {}; try { preferences = JSON.parse(localStorage.getItem(storageKey) || '{}'); } catch (_) {}
  let socket, connected = false, topics = {}, subscribed = new Set(), groups = [], jointState = new Map();
  let inspectionTopics = new Set();
  window.addEventListener('robocore-subscriptions',event=>{inspectionTopics=new Set(event.detail);reconcile();});
  const messages = new Map(), received = new Map(), histories = new Map();
  let rosTime=null, mapDescription="";
  const fmt = (value, digits = 2) => Number.isFinite(value) ? value.toFixed(digits) : '—';
  const yaw = q => q ? Math.atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z)) : NaN;
  const send = (op, payload) => { if (connected && socket.readyState === WebSocket.OPEN) socket.send(JSON.stringify([op,payload])); };
  const publish = (topicName, topicType, msg) => send('P',{topicName,topicType,msg});
  const save = () => { try { localStorage.setItem(storageKey,JSON.stringify({camera:$('camera-select').value,map:$('map-select').value})); } catch (_) {} };
  let toastTimer; function toast(text) { $('toast').textContent=text; $('toast').classList.add('visible'); clearTimeout(toastTimer); toastTimer=setTimeout(()=>$('toast').classList.remove('visible'),2500); }
  function scanRanges(m) {
    if (m._ranges_uint16) { const packed=m._ranges_uint16, raw=atob(packed.points), [lo,hi]=packed.bounds, result=[]; for(let i=0;i<raw.length;i+=2) { const n=raw.charCodeAt(i)|(raw.charCodeAt(i+1)<<8); result.push(n===65535?NaN:lo+n*(hi-lo)/65534); } return result; }
    return m.ranges || [];
  }
  const validRanges = m => scanRanges(m).filter(r=>Number.isFinite(r)&&r>=m.range_min&&r<=m.range_max);
  const catalogue = [
    {labels:[['X','m'],['Y','m'],['Heading','°']],values:m=>[m.pose.pose.position.x,m.pose.pose.position.y,yaw(m.pose.pose.orientation)*180/Math.PI],sample:m=>m.twist.twist.linear.x},
    {labels:[['Goal',''],['History','']],values:m=>{const s=m.status_list||[];return [s.length?['Unknown','Accepted','Executing','Canceling','Succeeded','Canceled','Aborted'][s.at(-1).status]||'Unknown':'Idle',s.length]},sample:m=>m.status_list?.at(-1)?.status||0},
    {labels:[['Distance','m'],['Waypoints','']],values:m=>{let d=0;const p=m.poses||[];for(let i=1;i<p.length;i++) d+=Math.hypot(p[i].pose.position.x-p[i-1].pose.position.x,p[i].pose.position.y-p[i-1].pose.position.y);return[d,p.length]},sample:m=>(m.poses||[]).length},
    {labels:[['Linear','m/s'],['Angular','rad/s']],values:m=>[m.twist.twist.linear.x,m.twist.twist.angular.z],sample:m=>m.twist.twist.angular.z},
    {labels:[['Yaw rate','rad/s'],['Acceleration','m/s²']],values:m=>[m.angular_velocity.z,Math.hypot(m.linear_acceleration.x,m.linear_acceleration.y,m.linear_acceleration.z)],sample:m=>m.angular_velocity.z},
    {labels:[['Nearest','m'],['Returns','']],values:m=>{const r=validRanges(m);return[r.length?Math.min(...r):NaN,r.length]},sample:m=>{const r=validRanges(m);return r.length?Math.min(...r):NaN}},
    {labels:[['Axes',''],['Peak velocity','rad/s']],values:m=>[(m.name||[]).length,Math.max(0,...(m.velocity||[]).filter(Number.isFinite).map(Math.abs))],sample:m=>Math.max(0,...(m.velocity||[]).filter(Number.isFinite).map(Math.abs))},
    {labels:[['CPU','%'],['Memory','%'],['Disk','%']],values:m=>[Array.isArray(m.cpu_percent)?m.cpu_percent.reduce((a,b)=>a+b,0)/m.cpu_percent.length:m.cpu_percent,m.virtual_memory_percent,m.disk_usage_percent],sample:m=>m.virtual_memory_percent}
  ];
  const kinds = ['localization','navigation','plan','odometry','imu','lidar','joints','compute'];
  const specs = config.telemetry.map(item => ({...catalogue[kinds.indexOf(item.kind)],...item}));
  for (const spec of specs) {
    const card=document.createElement('article');card.className='panel telemetry-panel';
    card.innerHTML='<div class="telemetry-heading"><h3></h3><span class="topic-status"><i></i><span>Waiting</span></span></div><div class="metrics"></div><svg class="sparkline" viewBox="0 0 240 29" preserveAspectRatio="none" aria-hidden="true"><path fill="none" stroke="#1a9e8e" stroke-width="1.3"/></svg><div class="telemetry-topic"></div>';
    card.querySelector('h3').textContent=spec.title;card.querySelector('.telemetry-topic').textContent=spec.topic;
    for(const [label,unit] of spec.labels) { const metric=document.createElement('div');metric.className='metric';metric.innerHTML='<strong>—</strong><small></small><span></span>';metric.querySelector('small').textContent=unit;metric.querySelector('span').textContent=label;card.querySelector('.metrics').append(metric); }
    $(spec.section+'-grid').append(card);spec.card=card;
  }
  function validPose(m) {
    const p=m?.pose?.pose||m?.pose,q=p?.orientation;
    if(!q||!Number.isFinite(p.position?.x)||!Number.isFinite(p.position?.y))return false;
    const norm=Math.hypot(q.x,q.y,q.z,q.w),cov=m.pose.covariance;
    return Number.isFinite(norm)&&norm>.5&&norm<1.5&&(!cov||[0,7,35].every(i=>cov[i]<1000));
  }
  function sourceAge(topic,m) {
    const receipt=(Date.now()-(received.get(topic)||0))/1000;
    const stamp=m?.header?.stamp;
    return Math.max(receipt,stamp&&Number.isFinite(rosTime)?Math.max(0,rosTime-stamp.sec-stamp.nanosec*1e-9):0);
  }
  function trackingLost() {return messages.get(bindings.tracking)?.lost===true||
    (messages.has(bindings.odom)&&!validPose(messages.get(bindings.odom)));}
  function poseForFrame(frame) {
    for(const topic of ['_robot_pose',bindings.odom,bindings.pose]) {
      const m=messages.get(topic);
      if(m?.header?.frame_id===frame&&validPose(m)&&sourceAge(topic,m)<2)return m;
    }
    return null;
  }
  function telemetry(topic,m) {
    const spec=specs.find(s=>s.topic===topic);
    if(spec&&['localization','odometry'].includes(spec.kind)&&!validPose(m))return;if(!spec)return;
    const values=spec.values(m);spec.card.querySelectorAll('.metric strong').forEach((el,i)=>{el.textContent=typeof values[i]==='string'?values[i]:fmt(values[i],['Axes','Returns','Waypoints','History'].includes(spec.labels[i][0])?0:2);});
    const value=spec.sample(m), history=histories.get(topic)||[];if(Number.isFinite(value)) history.push(value);if(history.length>60)history.shift();histories.set(topic,history);
    if(history.length>1) { const lo=Math.min(...history),hi=Math.max(...history),span=Math.max(hi-lo,.01);spec.card.querySelector('path').setAttribute('d',history.map((v,i)=>(i?'L':'M')+(i*240/(history.length-1)).toFixed(1)+','+(26-(v-lo)*22/span).toFixed(1)).join(' ')); }
  }
  function reconcile() {
    const desired=new Set([...specs.map(s=>s.topic),...inspectionTopics,'_robot_pose',bindings.tracking,bindings.joints,bindings.pose,bindings.odom,bindings.plan,$('camera-select').value,$('map-select').value]);
    desired.delete('');
    if(!connected)return;
    for(const topic of subscribed)if(!desired.has(topic)){send('u',{topicName:topic});subscribed.delete(topic);}
    for(const topic of desired)if(!subscribed.has(topic)){send('s',{topicName:topic,maxUpdateRate:topic===$('camera-select').value?12:topic.includes('costmap')||topic==='/map'?2:8});subscribed.add(topic);}
  }
  const selectors = [['camera-select',/\/Image$|\/CompressedImage$/],['map-select',/\/OccupancyGrid$|\/LaserScan$/]];
  function updateChoices() {
    for(const [id,pattern] of selectors) { const select=$(id);for(const [topic,type] of Object.entries(topics))if(pattern.test(type)&&![...select.options].some(o=>o.value===topic)){const o=new Option(topic,topic);select.add(o);} }
  }
  let cameraLast=0, mapImage=null, mapMessage=null, mapEpoch=0, zoom=1;
  function selectCamera() { cameraLast=0;$('camera-image').hidden=true;$('camera-image').removeAttribute('src');$('camera-empty').hidden=false;$('camera-panel').classList.remove('has-data');$('camera-topic').textContent=$('camera-select').value;$('camera-source').textContent=$('camera-select').selectedOptions[0].textContent.toUpperCase();$('camera-meta').textContent='Awaiting frames';reconcile();save(); }
  function selectMap() { mapEpoch++;mapImage=null;mapMessage=null;zoom=1;$('map-empty').hidden=false;$('map-topic').textContent=$('map-select').value;$('map-meta').textContent='Awaiting '+($('map-select').value.includes('scan')?'scan':'map');drawMap();reconcile();save(); }
  for(const id of ['camera-select','map-select']) { const key=id==='camera-select'?'camera':'map',select=$(id);if(preferences[key]){if(![...select.options].some(o=>o.value===preferences[key]))select.add(new Option(preferences[key],preferences[key]));select.value=preferences[key];}select.addEventListener('change',id==='camera-select'?selectCamera:selectMap); }
  $('reset-views').onclick=()=>{$('camera-select').value=defaults.camera;$('map-select').value=defaults.map;selectCamera();selectMap();toast('Default views restored');};
  function camera(m) { if(!m._data_jpeg)return;const now=performance.now(),shape=m._data_shape||[m.height,m.width];$('camera-image').src='data:image/jpeg;base64,'+m._data_jpeg;$('camera-image').hidden=false;$('camera-empty').hidden=true;$('camera-panel').classList.add('has-data');$('camera-meta').textContent=(shape[1]||'?')+' × '+(shape[0]||'?')+(cameraLast?' · '+fmt(1000/(now-cameraLast),1)+' Hz':'')+(m._depth_range?' · '+m._depth_range.map(v=>fmt(v,1)).join('–')+' m':'');cameraLast=now; }
  const canvas=$('map-canvas'),context=canvas.getContext('2d');
  function drawMap() {
    const rect=canvas.getBoundingClientRect(),dpr=devicePixelRatio||1;canvas.width=Math.round(rect.width*dpr);canvas.height=Math.round(rect.height*dpr);context.setTransform(dpr,0,0,dpr,0,0);context.clearRect(0,0,rect.width,rect.height);if(!mapMessage)return;
    const m=mapMessage,cx=rect.width/2,cy=rect.height/2;
    if(m.info&&mapImage) {
      const w=m.info.width*m.info.resolution,h=m.info.height*m.info.resolution,scale=Math.min((rect.width-36)/w,(rect.height-36)/h)*zoom;
      context.save();context.translate(cx,cy);context.scale(scale,-scale);context.imageSmoothingEnabled=false;context.save();context.scale(1,-1);context.drawImage(mapImage,-w/2,-h/2,w,h);context.restore();
      const origin=m.info.origin,theta=yaw(origin.orientation)||0,toPixel=p=>{const x=p.x-origin.position.x,y=p.y-origin.position.y;return{x:Math.cos(theta)*x+Math.sin(theta)*y-w/2,y:-Math.sin(theta)*x+Math.cos(theta)*y-h/2};};
      const frame=m.header?.frame_id,plan=messages.get(bindings.plan);if(plan?.header?.frame_id===frame&&plan.poses.length){context.strokeStyle='#34d399';context.lineWidth=1.6/scale;context.beginPath();plan.poses.forEach((p,i)=>{const v=toPixel(p.pose.position);i?context.lineTo(v.x,v.y):context.moveTo(v.x,v.y);});context.stroke();}
      const pose=poseForFrame(frame);if(pose){const p=(pose.pose.pose||pose.pose),v=toPixel(p.position);context.translate(v.x,v.y);context.rotate(yaw(p.orientation)-theta);context.fillStyle=trackingLost()?'#f59e0b':'#34d399';canvas.dataset.poseX=p.position.x;canvas.dataset.poseY=p.position.y;context.beginPath();context.moveTo(9/scale,0);context.lineTo(-6/scale,5/scale);context.lineTo(-6/scale,-5/scale);context.closePath();context.fill();}context.restore();
    } else if(m.angle_increment!==undefined) {
      const scale=Math.min(rect.width,rect.height)/(2*Math.min(m.range_max,8))*zoom;context.save();context.translate(cx,cy);context.strokeStyle='rgba(26,158,142,.18)';for(let r=1;r<=3;r++){context.beginPath();context.arc(0,0,r*2*scale,0,2*Math.PI);context.stroke();}context.fillStyle='#34d399';scanRanges(m).forEach((r,i)=>{if(Number.isFinite(r)&&r>=m.range_min&&r<=m.range_max){const a=m.angle_min+i*m.angle_increment;context.fillRect(Math.cos(a)*r*scale-1,-Math.sin(a)*r*scale-1,2,2);}});context.fillStyle='#fff';context.beginPath();context.arc(0,0,3,0,Math.PI*2);context.fill();context.restore();
    }
  }
  function map(m) { const epoch=++mapEpoch;mapMessage=m;if(m._data_jpeg){const img=new Image();img.onload=()=>{if(epoch!==mapEpoch)return;mapImage=img;$('map-empty').hidden=true;drawMap();};img.src='data:image/jpeg;base64,'+m._data_jpeg;mapDescription=fmt(m.info.width*m.info.resolution,1)+' × '+fmt(m.info.height*m.info.resolution,1)+' m · '+fmt(m.info.resolution,2)+' m/cell';updateMapStatus();}else if(m.angle_increment!==undefined){$('map-empty').hidden=true;$('map-meta').textContent=validRanges(m).length+' returns · '+(m.header?.frame_id||'lidar');drawMap();} }
  function updateMapStatus() {
    if(!mapMessage?.info)return;
    const age=sourceAge($('map-select').value,mapMessage),pose=poseForFrame(mapMessage.header?.frame_id);
    const lost=trackingLost(),state=!connected?'offline':lost?'lost':pose?'live':'stale';
    canvas.dataset.poseState=state;
    $('map-meta').textContent=mapDescription+' · Map update '+fmt(age,1)+'s ago · '+
      (state==='lost'?'Tracking lost — mapping paused':state==='live'?'Pose live':state==='offline'?'Disconnected':'Pose stale');
  }
  new ResizeObserver(drawMap).observe(canvas);$('fit-map').onclick=()=>{zoom=1;drawMap();};canvas.addEventListener('wheel',event=>{event.preventDefault();zoom=Math.max(.5,Math.min(8,zoom*Math.exp(-event.deltaY*.001)));drawMap();},{passive:false});
  let drive={x:0,y:0}, driveTimer, pointer=null;const keys=new Set();
  function commandDrive(){
    const speed=Number($('drive-speed').value)/100;
    const v=drive.y*config.drive.linear_max*speed;
    const strafe=config.drive.omnidirectional&&$('strafe-toggle').checked;
    const lateral=strafe?drive.x*config.drive.linear_max*speed:0;
    const w=strafe?0:drive.x*config.drive.angular_max*speed;
    const twist={linear:{x:v,y:lateral,z:0},angular:{x:0,y:0,z:w}};
    publish(config.drive.topic,config.drive.type,config.drive.type.endsWith('TwistStamped')?{header:{frame_id:''},twist}:twist);
    $('linear-value').innerHTML=fmt(v)+(strafe?' / '+fmt(lateral):'')+' <small>m/s</small>';
    $('angular-value').innerHTML=fmt(w)+' <small>rad/s</small>';
  }
  function moveDrive(x,y){drive={x,y};$('joystick-handle').style.transform=`translate(calc(-50% + ${-x*43}px),calc(-50% + ${-y*43}px))`;$('joystick').classList.add('active');commandDrive();if(!driveTimer)driveTimer=setInterval(commandDrive,50);}
  function stopDrive(){clearInterval(driveTimer);driveTimer=null;pointer=null;keys.clear();drive={x:0,y:0};commandDrive();$('joystick').classList.remove('active');$('joystick-handle').style.transform='translate(-50%,-50%)';}
  function pointerMove(e){if(pointer!==e.pointerId)return;const r=$('joystick').getBoundingClientRect(),radius=Math.min(r.width,r.height)*.35;let x=(r.width/2-(e.clientX-r.left))/radius,y=(r.height/2-(e.clientY-r.top))/radius;const norm=Math.max(1,Math.hypot(x,y));moveDrive(x/norm,y/norm);}
  $('joystick').onpointerdown=e=>{if(!connected||e.button!==0)return;pointer=e.pointerId;$('joystick').setPointerCapture(pointer);$('joystick').focus();pointerMove(e);};$('joystick').onpointermove=pointerMove;
  for(const event of ['pointerup','pointercancel','lostpointercapture'])$('joystick').addEventListener(event,stopDrive);
  $('joystick').onkeydown=e=>{if(!['ArrowUp','ArrowDown','ArrowLeft','ArrowRight'].includes(e.key)||!connected)return;e.preventDefault();keys.add(e.key);moveDrive((keys.has('ArrowLeft')?1:0)-(keys.has('ArrowRight')?1:0),(keys.has('ArrowUp')?1:0)-(keys.has('ArrowDown')?1:0));};
  $('joystick').onkeyup=e=>{if(keys.has(e.key)){e.preventDefault();keys.delete(e.key);if(!keys.size)stopDrive();else moveDrive((keys.has('ArrowLeft')?1:0)-(keys.has('ArrowRight')?1:0),(keys.has('ArrowUp')?1:0)-(keys.has('ArrowDown')?1:0));}};
  $('joystick').onblur=stopDrive;window.addEventListener('blur',stopDrive);document.addEventListener('visibilitychange',()=>{if(document.hidden)stopDrive();});window.addEventListener('pagehide',stopDrive);$('stop-drive').onclick=stopDrive;$('drive-speed').oninput=()=>{$('drive-speed-value').textContent=$('drive-speed').value+'%';if(driveTimer)commandDrive();};
  function jointFeedback(m){(m.name||[]).forEach((name,i)=>{if(Number.isFinite(m.position?.[i]))jointState.set(name,m.position[i]);});for(const group of groups)for(const joint of group.joints){const value=jointState.get(joint.name);if(value===undefined)continue;joint.feedback.textContent='now '+fmt(value,joint.unit==='m'?3:2);if(!joint.initialized){joint.input.value=Math.max(joint.min,Math.min(joint.max,value));joint.output.textContent=fmt(Number(joint.input.value),joint.unit==='m'?3:2)+' '+joint.unit;joint.initialized=true;}}updateControls();}
  function updateControls(){const fresh=connected&&Date.now()-(received.get(bindings.joints)||0)<3000;for(const group of groups)for(const joint of group.joints)joint.input.disabled=!fresh||!group.joints.every(j=>j.initialized);$('sync-joints').disabled=!fresh;$('drive-speed').disabled=!connected;$('joystick').setAttribute('aria-disabled',String(!connected));}
  function commandJoints(group){if(group.busyUntil&&performance.now()<group.busyUntil){clearTimeout(group.timer);group.timer=setTimeout(()=>commandJoints(group),group.busyUntil-performance.now());return;}if(!connected||Date.now()-(received.get(bindings.joints)||0)>3000)return;const positions=group.joints.map(j=>Number(j.input.value));if(group.type==='std_msgs/msg/Float64MultiArray')publish(group.topic,group.type,{data:positions});else{if(group.joints.some(j=>!Number.isFinite(j.velocity)||j.velocity<=0))return toast("Joint speed limit unavailable");const duration=Math.max(.2,...group.joints.map((j,i)=>Math.abs(positions[i]-jointState.get(j.name))/j.velocity))+.1;if(group.type.includes('/action/'))group.busyUntil=performance.now()+(duration+.5)*1000;publish(group.topic,group.type,{joint_names:group.joints.map(j=>j.name),points:[{positions,time_from_start:{sec:Math.floor(duration),nanosec:Math.round((duration%1)*1e9)}}]});}}
  async function loadGroups(){try{const response=await fetch('/robocore_viz/joint-controls',{cache:'no-store'});if(!response.ok)throw Error(response.status);const config=await response.json();if(!config.groups?.length){setTimeout(loadGroups,3000);return;}for(const group of groups)clearTimeout(group.timer);groups=config.groups;$('manipulation-grid').replaceChildren();const aux=document.createElement('div');aux.className='aux-controls';
    for(const group of groups){const arm=/^(left|right) arm$/i.test(group.title),panel=document.createElement('article');panel.className='panel '+(arm?'arm-panel':'aux-panel');panel.innerHTML='<header class="panel-header"><div><span class="panel-label">MANIPULATOR</span><h3></h3></div><span class="small-tag"></span></header><div class="joint-body"></div>';panel.querySelector('h3').textContent=group.title.replace(/\b\w/g,c=>c.toUpperCase());panel.querySelector('.small-tag').textContent=group.joints.length+' AXES';
      for(const joint of group.joints){const row=document.createElement('div');row.className='joint-row';const label=document.createElement('label');label.className='joint-label';const name=document.createElement('span');name.textContent=joint.name.replace(/^(left|right)_(arm|lift|gripper)_?/,'').replace(/_/g,' ').replace(/^joint\s*/,'Joint ').replace(/\b\w/g,c=>c.toUpperCase())||'Position';if(/gripper/i.test(group.title))name.textContent='Grip';else if(/lift/i.test(group.title))name.textContent='Height';else if(joint.label)name.textContent=joint.label;const values=document.createElement('span');values.className='joint-values';joint.output=document.createElement('output');joint.output.textContent='— '+joint.unit;joint.feedback=document.createElement('small');joint.feedback.textContent='now —';values.append(joint.output,joint.feedback);label.append(name,values);joint.input=document.createElement('input');joint.input.type='range';joint.input.min=joint.min;joint.input.max=joint.max;joint.input.step=joint.unit==='m'?.001:.01;joint.input.value=Math.max(joint.min,Math.min(joint.max,0));joint.input.disabled=true;joint.input.id='joint-'+joint.name;joint.input.dataset.joint=joint.name;joint.input.setAttribute('aria-label',joint.name);label.htmlFor=joint.input.id;joint.input.oninput=()=>{joint.output.textContent=fmt(Number(joint.input.value),joint.unit==='m'?3:2)+' '+joint.unit;clearTimeout(group.timer);group.timer=setTimeout(()=>commandJoints(group),120);};row.append(label,joint.input);panel.querySelector('.joint-body').append(row);}
      if(arm)$('manipulation-grid').append(panel);else aux.append(panel);
    }$('manipulation-grid').append(aux);$('joint-summary').textContent=groups.reduce((n,g)=>n+g.joints.length,0)+' controlled axes';const previous=messages.get(bindings.joints);if(previous)jointFeedback(previous);updateControls();
  }catch(_){setTimeout(loadGroups,3000);}}
  $('sync-joints').onclick=()=>{for(const g of groups){clearTimeout(g.timer);for(const j of g.joints)j.initialized=false;}const m=messages.get(bindings.joints);if(m)jointFeedback(m);toast('Sliders synced to robot feedback');};
  function connect(){socket=new WebSocket((location.protocol==='https:'?'wss://':'ws://')+location.host+'/robocore_viz/v1');socket.onopen=()=>{connected=true;window.dispatchEvent(new CustomEvent('robocore-connection',{detail:true}));subscribed.clear();$('connection').classList.add('online');$('connection').querySelector('span').textContent='Robot online';reconcile();loadGroups();updateControls();};socket.onmessage=e=>{try{const [op,data]=JSON.parse(e.data);if(op==='p')send('q',{s:data.s,t:Date.now()});else if(op==='t'){topics=data;updateChoices();window.dispatchEvent(new CustomEvent('robocore-topics',{detail:data}));}else if(op==='m'){window.dispatchEvent(new CustomEvent('robocore-message',{detail:data}));const topic=data._topic_name;messages.set(topic,data);received.set(topic,Date.now());if(['_robot_pose',bindings.tracking,bindings.odom].includes(topic)&&Number.isFinite(data._ros_time))rosTime=data._ros_time;telemetry(topic,data);if(topic===$('camera-select').value)camera(data);if(topic===$('map-select').value)map(data);if(topic===bindings.joints)jointFeedback(data);if([bindings.plan,bindings.pose,bindings.odom,'_robot_pose',bindings.tracking].includes(topic)){drawMap();updateMapStatus();}}}catch(error){console.warn('Dashboard message could not be rendered',error);}};socket.onclose=()=>{connected=false;window.dispatchEvent(new CustomEvent('robocore-connection',{detail:false}));stopDrive();subscribed.clear();received.clear();messages.clear();rosTime=null;mapEpoch++;drawMap();updateMapStatus();for(const g of groups){clearTimeout(g.timer);for(const j of g.joints)j.initialized=false;}$('connection').classList.remove('online');$('connection').querySelector('span').textContent='Reconnecting';updateControls();setTimeout(connect,2000);};socket.onerror=()=>socket.close();}
  setInterval(()=>{$('session-clock').textContent=new Date().toLocaleTimeString('en-GB');for(const spec of specs){const last=received.get(spec.topic),live=connected&&last&&Date.now()-last<5000,status=spec.card.querySelector('.topic-status');status.classList.toggle('live',Boolean(live));const lost=['localization','odometry'].includes(spec.kind)&&trackingLost();status.classList.toggle('live',Boolean(live&&!lost));status.querySelector('span').textContent=lost?'Tracking lost':live?'Live':last?'Stale':'Waiting';}if(cameraLast&&Date.now()-(received.get($('camera-select').value)||0)>3000){$('camera-panel').classList.remove('has-data');$('camera-meta').textContent='Feed paused';}drawMap();updateMapStatus();updateControls();},1000);
  selectCamera();selectMap();updateControls();connect();
})();
