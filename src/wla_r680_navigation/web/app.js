import {toLngLat, toRos, mapCorners, headingDegrees} from './coordinates.js';
const $ = id => document.getElementById(id), sdk = window.bicMap;
let map, state, cloud, cloudKey='', dimension=2, mapKey='', userMap=false, fitted=false;
let robot, selectedMarker, semanticMarkers=[], semanticKey='', mode=null, selection=null, lastReceived=0;
let velocityHistory=[], cameraUrl=null, toastTimer;
const empty = () => ({type:'FeatureCollection',features:[]});
function toast(message){$('toast').textContent=message;$('toast').style.display='block';clearTimeout(toastTimer);toastTimer=setTimeout(()=>$('toast').style.display='none',5000);}
function fresh(key, seconds=2){return state && (state.ages?.[key]??Infinity)<seconds;}
function setGeo(id, data, color, width=3){
  if(map.getSource(id))map.getSource(id).setData(data);
  else{map.addSource(id,{type:'geojson',data});map.addLayer({id,type:'line',source:id,paint:{'line-color':color,'line-width':width},layout:{'line-join':'round','line-cap':'round'}});}
}
function lineData(points){return points?.length>1?{type:'Feature',properties:{},geometry:{type:'LineString',coordinates:points.map(p=>toLngLat(...p))}}:empty();}
function fit(){
  let points=state?.map?mapCorners(state.map):getCloudPoints().map(p=>p.slice(0,2));
  if(!points.length && state?.robot)points=[toLngLat(state.robot.x-5,state.robot.y-5),toLngLat(state.robot.x+5,state.robot.y+5)];
  if(!points.length)return;
  const b=new sdk.maplibregl.LngLatBounds();points.forEach(p=>b.extend(p));map.fitBounds(b,{padding:60,maxZoom:22,pitch:dimension===3?55:0,duration:500});fitted=true;
}
function getCloudPoints(){
  const pts=[];
  for(const k of ['confirmed_obstacles','near_obstacles'])if(fresh(k,2))for(const p of state[k]||[]){if(p.length>=3&&p.every(Number.isFinite))pts.push([...toLngLat(p[0],p[1]),p[2]]);}
  return pts;
}
function renderCloud(force=false){
  const key=JSON.stringify([state?.cloud_versions,dimension,$('pointSize').value,$('cloudLayer').checked,fresh('near_obstacles'),fresh('confirmed_obstacles')]);
  if(key===cloudKey&&!force)return;cloudKey=key;
  const pts=$('cloudLayer').checked?getCloudPoints():[];
  const options={pointSize:Number($('pointSize').value),heightScale:1,pointOpacity:.85,is3D:false,useColorMap:true,zRange:[0,2],pointColor:'#ef9b3b'};
  if(!cloud)cloud=dimension===3?sdk.createPointCloud3D(map,pts,options):sdk.createPointCloud(map,pts,options);
  else cloud.update(pts,options);
  $('cloudInfo').textContent=`${dimension}D · ${pts.length.toLocaleString()} 点${pts.length?'':' · 等待新鲜点云'}`;
  if(!fitted&&pts.length)fit();
}
function switchDimension(n){dimension=n;cloud?.remove();cloud=null;map.easeTo({pitch:n===3?55:0,bearing:0,duration:500});$('view2d').classList.toggle('active',n===2);$('view3d').classList.toggle('active',n===3);renderCloud(true);}
function renderMap(){
  if(!state.map){$('mapInfo').textContent='等待地图数据';if(map.getLayer('occupancy'))map.removeLayer('occupancy');if(map.getSource('occupancy'))map.removeSource('occupancy');mapKey='';return;}
  const m=state.map,key=JSON.stringify([m.version,m.map_id,m.origin_x,m.origin_y,m.origin_yaw,m.width,m.height,m.resolution,userMap]);
  if(key===mapKey)return;mapKey=key;
  const url=`/api/${userMap&&m.user_map?'user_map':'map'}.png?v=${m.version}`,coordinates=mapCorners(m);
  if(map.getSource('occupancy'))map.getSource('occupancy').updateImage({url,coordinates});
  else{map.addSource('occupancy',{type:'image',url,coordinates});const before=map.getStyle().layers.find(l=>l.id!=='background')?.id;map.addLayer({id:'occupancy',type:'raster',source:'occupancy',paint:{'raster-opacity':.92,'raster-resampling':'nearest','raster-fade-duration':0}},before);}
  $('mapInfo').textContent=`${m.map_id||'ROS 地图'} · ${(m.width*m.resolution).toFixed(1)} × ${(m.height*m.resolution).toFixed(1)} m`;
  if(!fitted)fit();
}
function marker(element, xy){return new sdk.maplibregl.Marker({element}).setLngLat(toLngLat(...xy)).addTo(map);}
function renderRobot(){
  if(!state.robot)return;
  if(!robot){const e=document.createElement('div');e.className='robot';e.textContent='↑';robot=marker(e,[state.robot.x,state.robot.y]);robot.setRotationAlignment('map');}
  robot.setLngLat(toLngLat(state.robot.x,state.robot.y)).setRotation(headingDegrees(state.robot.yaw));robot.getElement().classList.toggle('stale',!state.connected.robot);
  $('position').textContent=`map · x ${state.robot.x.toFixed(2)}　y ${state.robot.y.toFixed(2)}　${state.connected.robot?'实时位姿':'最后位置 · 定位已过期'}`;
}
function renderSemantics(){
  const sem=state.semantic||{},key=JSON.stringify([sem,$('semanticLayer').checked]);if(key===semanticKey)return;semanticKey=key;
  semanticMarkers.forEach(m=>m.remove());semanticMarkers=[];if(!$('semanticLayer').checked)return;
  for(const p of [...(sem.home?[{...sem.home,label:'出生点'}]:[]),...(sem.furniture||[])]){const e=document.createElement('div');e.className='tag';e.textContent=p.label||p.id;semanticMarkers.push(marker(e,[p.x,p.y]).setOffset([0,-30]));}
}
function updateControls(){
  const online=Date.now()-lastReceived<2500,c=state?.controls||{};
  $('submit').disabled=!online||!selection||selection.yaw==null||!(mode==='goal'?c.goal:c.initial_pose);
  $('goalMode').disabled=!online||!c.goal;$('poseMode').disabled=!online||!c.initial_pose;
  $('cancel').disabled=!online||!c.cancel;$('clear').disabled=!online||!c.clear_costmaps;
  $('home').disabled=!online||!c.goal||!state?.semantic?.home||!state?.connected?.robot;
  $('controlHint').textContent=!online?'网关连接中断，控制已禁用':state?.read_only?'此端口为只读预览，服务器已禁用控制。':c.goal?'目标通过原有导航网关提交；执行状态以 ROS 回执为准。':'导航目标入口尚无订阅者；Agent 控制模式需使用 Agent 任务入口。';
}
function updateUi(){
  $('connection').textContent=state.read_only?'只读预览在线':'网关在线';$('connection').className='badge ok';
  $('mapHealth').textContent=`地图 ${state.connected.map?'已加载':'未收到'}`;
  $('poseHealth').textContent=`位姿 ${state.connected.robot?'新鲜':'丢失 / 未就绪'}`;$('poseHealth').className=state.connected.robot?'ok':'warn';
  $('motionHealth').textContent=`导航权限 ${fresh('motion_allowed')?(state.motion_allowed?'允许':'禁止'):'未知'}`;
  $('goalStatus').textContent=state.goal_status||'未知';
  $('odom').textContent=fresh('odom')?`${state.odom.vx.toFixed(2)} m/s · ${state.odom.wz.toFixed(2)} rad/s`:'速度数据过期';
  $('mapMessage').textContent=!state.map?'等待 ROS 地图；点云在收到后显示':!state.connected.robot?'定位未就绪；保留最后位置，仅供参考':'';$('mapMessage').hidden=!$('mapMessage').textContent;
  const objects=fresh('detections')&&Array.isArray(state.detections?.objects)?state.detections.objects:[];
  $('detections').replaceChildren();if(!objects.length)$('detections').textContent='暂无新鲜识别结果';
  for(const o of objects.slice(0,30)){const e=document.createElement('div');e.className='object';const l=document.createElement('span'),r=document.createElement('span');l.textContent=`${o.label||'目标'} #${o.track_id??'—'}`;r.textContent=Number.isFinite(o.confidence)?`${Math.round(o.confidence*100)}%`:'';e.append(l,r);$('detections').append(e);}
  const fs=state.object_follow;$('followStatus').textContent=fresh('object_follow')?`物体跟随：${typeof fs==='object'?(fs.state||fs.status||fs.reason||JSON.stringify(fs).slice(0,120)):fs}`:'物体跟随：状态过期 / 未收到';
  updateControls();
}
function updateVelocity(){
  const t=state.server_time;velocityHistory.push({t,v:state.velocity_commands,valid:fresh('velocity_cmd',1)&&fresh('velocity_smoothed',1)});velocityHistory=velocityHistory.filter(p=>p.t>t-30);
  const c=$('velocity'),ctx=c.getContext('2d'),r=c.getBoundingClientRect();c.width=Math.max(300,r.width*devicePixelRatio);c.height=100*devicePixelRatio;
  const axis=$('velocityAxis').value,limit=Math.max(.2,...velocityHistory.flatMap(p=>[Math.abs(p.v?.cmd?.[axis]||0),Math.abs(p.v?.smoothed?.[axis]||0)]));
  ctx.clearRect(0,0,c.width,c.height);ctx.strokeStyle='#dce5ed';ctx.beginPath();ctx.moveTo(0,c.height/2);ctx.lineTo(c.width,c.height/2);ctx.stroke();
  for(const [key,color]of [['cmd','#168ac0'],['smoothed','#ed9c36']]){ctx.strokeStyle=color;ctx.lineWidth=2*devicePixelRatio;ctx.beginPath();let pen=false;for(const p of velocityHistory){if(!p.valid||!p.v?.[key]){pen=false;continue;}const x=(p.t-t+30)/30*c.width,y=c.height/2-p.v[key][axis]/limit*c.height*.42;pen?ctx.lineTo(x,y):ctx.moveTo(x,y);pen=true;}ctx.stroke();}
}
function render(){renderMap();renderRobot();renderSemantics();renderCloud();for(const [key,color]of [['global_path','#168ad5'],['local_path','#e35a76']])setGeo(key,lineData($('pathLayer').checked&&fresh(key,3)?state[key]:[]),color);if(state.goal)setGeo('goal-direction',lineData([[state.goal.x,state.goal.y],[state.goal.x+.5*Math.cos(state.goal.yaw),state.goal.y+.5*Math.sin(state.goal.yaw)]]),'#f5a135',4);updateUi();updateVelocity();}
async function poll(){try{const r=await fetch('/api/state',{cache:'no-store',signal:AbortSignal.timeout(2500)});if(!r.ok)throw Error(r.status);state=await r.json();lastReceived=Date.now();render();}catch(e){$('connection').textContent='连接中断';$('connection').className='badge bad';updateControls();}finally{setTimeout(poll,document.hidden?2000:500);}}
async function cameraPoll(){try{
  if(document.hidden)return;
  const r=await fetch('/api/camera.jpg',{cache:'no-store',signal:AbortSignal.timeout(2500)});if(!r.ok)throw Error('no image');
  const blob=await r.blob(),url=URL.createObjectURL(blob),old=cameraUrl;cameraUrl=url;$('camera').src=url;$('camera').style.display='block';$('cameraEmpty').style.display='none';if(old)URL.revokeObjectURL(old);
  $('cameraStatus').textContent=r.headers.get('X-Detection-Matched')==='true'?'图像 + 同帧识别框':'图像 · 暂无同帧检测';
}catch(e){$('camera').style.display='none';$('cameraEmpty').style.display='grid';$('cameraStatus').textContent='图像未收到 / 已过期';}finally{setTimeout(cameraPoll,document.hidden?2000:350);}}
async function post(path,body={}){const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body),signal:AbortSignal.timeout(3000)});if(!r.ok)throw Error(await r.text());return r.json();}
function selectMode(value){mode=value;selection=null;selectedMarker?.remove();selectedMarker=null;setGeo('selection-direction',empty(),'#f5a135');$('selection').textContent=value==='goal'?'点击地图选择目标位置，再点击确定朝向。':'点击地图选择初始位置，再点击确定朝向。';$('goalMode').classList.toggle('active',value==='goal');$('poseMode').classList.toggle('active',value==='initial_pose');updateControls();}
async function init(){
  await sdk.init();map=sdk.createMap({container:'map',center:[0,0],zoom:19,maxZoom:24,pitch:0,backgroundColor:'#e8eff5',attributionControl:false,renderWorldCopies:false,style:{version:8,sources:{},layers:[{id:'background',type:'background',paint:{'background-color':'#e8eff5'}}]}});
  sdk.addZoomControl(map,'bottom-right');new ResizeObserver(()=>map.resize()).observe($('map'));
  map.on('error',e=>{if(e.error)console.error(e.error);});
  map.on('load',()=>{poll();cameraPoll();});
  map.on('mousemove',e=>{const [x,y]=toRos(e.lngLat.lng,e.lngLat.lat);$('cursor').textContent=`x ${x.toFixed(2)}　y ${y.toFixed(2)}`;});
  map.on('click',e=>{if(!mode)return;const [x,y]=toRos(e.lngLat.lng,e.lngLat.lat);if(!selection||selection.yaw!=null){selection={x,y,yaw:null};selectedMarker?.remove();const el=document.createElement('div');el.className='selected';selectedMarker=marker(el,[x,y]);$('selection').textContent=`位置 ${x.toFixed(2)}, ${y.toFixed(2)}；再点击确定朝向。`;}else{if(Math.hypot(x-selection.x,y-selection.y)<.05){toast('请在离目标稍远的位置点击以确定朝向');return;}selection.yaw=Math.atan2(y-selection.y,x-selection.x);setGeo('selection-direction',lineData([[selection.x,selection.y],[x,y]]),'#f5a135',4);$('selection').textContent=`${mode==='goal'?'导航目标':'初始位姿'} (${selection.x.toFixed(2)}, ${selection.y.toFixed(2)}) · ${(selection.yaw*180/Math.PI).toFixed(0)}°`;}updateControls();});
  $('view2d').onclick=()=>switchDimension(2);$('view3d').onclick=()=>switchDimension(3);$('fit').onclick=fit;
  $('userMap').onclick=()=>{userMap=!userMap;$('userMap').classList.toggle('active',userMap);if(state)renderMap();};
  $('goalMode').onclick=()=>selectMode('goal');$('poseMode').onclick=()=>selectMode('initial_pose');
  $('submit').onclick=async()=>{const target={...selection},kind=mode;$('submit').disabled=true;try{await post(`/api/${kind}`,target);toast('网关已接收，等待 ROS 执行反馈');mode=null;selection=null;selectedMarker?.remove();selectedMarker=null;setGeo('selection-direction',empty(),'#f5a135');$('selection').textContent='已提交。可重新选择目标。';}catch(e){toast(`提交未确认：${e.message}。请先核对任务状态。`);}finally{updateControls();}};
  for(const [id,path]of [['cancel','cancel'],['home','return_home'],['clear','clear_costmaps']])$(id).onclick=async()=>{try{await post(`/api/${path}`);toast('请求已提交，等待 ROS 回执');}catch(e){toast(`请求未确认：${e.message}`);}};
  for(const id of ['pointSize','cloudLayer','pathLayer','semanticLayer'])$(id).onchange=()=>{if(state)render();};
  setInterval(()=>{if(Date.now()-lastReceived>2500){$('connection').textContent='连接中断';$('connection').className='badge bad';$('poseHealth').textContent='定位数据过期';robot?.getElement().classList.add('stale');if(cloud){cloud.update([]);cloudKey='';}updateControls();}},500);
}
init().catch(e=>{$('mapMessage').hidden=false;$('mapMessage').textContent=`地图引擎加载失败：${e.message}`;console.error(e);});
