'use strict';
const canvas=document.getElementById('mapCanvas'),ctx=canvas.getContext('2d');
const wrap=document.getElementById('mapWrap'),mapImage=new Image();
let state=null,mapVersion=-1,scale=1,panX=0,panY=0,fitted=false,mode='goal';
let drag=null,panning=null,dpr=window.devicePixelRatio||1;
const enabled={global_path:true,local_path:true,near_obstacles:true,confirmed_obstacles:true,home:true,semantic_furniture:true};
const $=id=>document.getElementById(id);
const velocityCanvas=$('velocityCanvas'),velocityCtx=velocityCanvas.getContext('2d');
const velocityHistory=[];
let velocityMode='vx';
function velocitySample(){
  if(!state)return;
  const age=state.ages||{},commands=state.velocity_commands||{},t=state.server_time;
  const valid=key=>age['velocity_'+key]<1&&commands[key]?
    {vx:commands[key].vx,wz:commands[key].wz}:null;
  velocityHistory.push({t,cmd:valid('cmd'),smoothed:valid('smoothed')});
  while(velocityHistory.length&&velocityHistory[0].t<t-30)velocityHistory.shift();
  const unit=velocityMode==='vx'?'m/s':'rad/s';
  $('rawVelocity').textContent=velocityHistory.at(-1).cmd?
    '原始：'+velocityHistory.at(-1).cmd[velocityMode].toFixed(2)+' '+unit:'原始：无新数据';
  $('smoothVelocity').textContent=velocityHistory.at(-1).smoothed?
    '平滑：'+velocityHistory.at(-1).smoothed[velocityMode].toFixed(2)+' '+unit:'平滑：无新数据';
  drawVelocity();
}
function drawVelocity(){
  const box=velocityCanvas.parentElement.getBoundingClientRect(),w=box.width,h=box.height;
  if(w<1||h<1)return;
  const ratio=window.devicePixelRatio||1;
  velocityCanvas.width=Math.floor(w*ratio);velocityCanvas.height=Math.floor(h*ratio);
  velocityCtx.setTransform(ratio,0,0,ratio,0,0);
  velocityCtx.clearRect(0,0,w,h);
  const left=38,right=w-8,top=10,bottom=h-19,now=velocityHistory.at(-1)?.t||Date.now()/1000;
  const values=velocityHistory.flatMap(p=>[p.cmd?.[velocityMode],p.smoothed?.[velocityMode]]).filter(Number.isFinite);
  const limit=Math.max(velocityMode==='vx'?0.5:0.5,...values.map(Math.abs))*1.15;
  velocityCtx.font='10px ui-monospace,monospace';
  velocityCtx.fillStyle='#8292a0';
  velocityCtx.fillText('+'+limit.toFixed(1),3,top+5);
  velocityCtx.fillText('0',20,(top+bottom)/2+3);
  velocityCtx.fillText('-'+limit.toFixed(1),3,bottom+3);
  velocityCtx.fillText('-30s',left,bottom+14);
  velocityCtx.fillText('now',right-22,bottom+14);
  velocityCtx.strokeStyle='#2b3947';velocityCtx.lineWidth=1;
  for(const y of [top,(top+bottom)/2,bottom]){
    velocityCtx.beginPath();velocityCtx.moveTo(left,y);velocityCtx.lineTo(right,y);velocityCtx.stroke();
  }
  const trace=(key,color)=>{
    velocityCtx.strokeStyle=color;velocityCtx.lineWidth=2;velocityCtx.beginPath();
    let drawing=false;
    for(const sample of velocityHistory){
      const v=sample[key]?.[velocityMode];
      if(!Number.isFinite(v)){drawing=false;continue}
      const x=left+(sample.t-(now-30))/30*(right-left);
      const y=(top+bottom)/2-v/limit*(bottom-top)/2;
      if(!drawing)velocityCtx.moveTo(x,y);else velocityCtx.lineTo(x,y);
      drawing=true;
    }
    velocityCtx.stroke();
  };
  trace('cmd','#27a7ff');trace('smoothed','#ffad32');
}
new ResizeObserver(drawVelocity).observe(velocityCanvas.parentElement);
$('linearVelocity').onclick=()=>{
  velocityMode='vx';$('linearVelocity').classList.add('active');
  $('angularVelocity').classList.remove('active');drawVelocity();
};
$('angularVelocity').onclick=()=>{
  velocityMode='wz';$('angularVelocity').classList.add('active');
  $('linearVelocity').classList.remove('active');drawVelocity();
};
function resize(){const r=wrap.getBoundingClientRect();dpr=window.devicePixelRatio||1;canvas.width=Math.max(1,Math.floor(r.width*dpr));canvas.height=Math.max(1,Math.floor(r.height*dpr));canvas.style.width=r.width+'px';canvas.style.height=r.height+'px';draw();}
new ResizeObserver(resize).observe(wrap);
function fit(){if(!state?.map)return;const w=state.map.width,h=state.map.height,rect=wrap.getBoundingClientRect();scale=Math.min((rect.width-36)/w,(rect.height-36)/h);panX=(rect.width-w*scale)/2;panY=(rect.height-h*scale)/2;fitted=true;draw();}
function worldToScreen(x,y){const m=state.map;return[panX+(x-m.origin_x)/m.resolution*scale,panY+(m.height-(y-m.origin_y)/m.resolution)*scale];}
function screenToWorld(x,y){const m=state.map;return[m.origin_x+(x-panX)/scale*m.resolution,m.origin_y+(m.height-(y-panY)/scale)*m.resolution];}
function path(points,color,width){if(!points?.length||!state?.map)return;ctx.beginPath();points.forEach((p,i)=>{const s=worldToScreen(p[0],p[1]);i?ctx.lineTo(...s):ctx.moveTo(...s)});ctx.strokeStyle=color;ctx.lineWidth=width;ctx.lineJoin='round';ctx.lineCap='round';ctx.stroke();}
function points(items,color,size){if(!items?.length||!state?.map)return;ctx.fillStyle=color;for(const p of items){const s=worldToScreen(p[0],p[1]);ctx.fillRect(s[0]-size/2,s[1]-size/2,size,size)}}
function arrow(x,y,yaw,color,label){const p=worldToScreen(x,y),len=28;ctx.save();ctx.translate(...p);ctx.rotate(-yaw);ctx.strokeStyle=color;ctx.fillStyle=color;ctx.lineWidth=3;ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(len,0);ctx.lineTo(len-8,-6);ctx.moveTo(len,0);ctx.lineTo(len-8,6);ctx.stroke();ctx.beginPath();ctx.arc(0,0,5,0,Math.PI*2);ctx.fill();ctx.restore();if(label){ctx.fillStyle=color;ctx.font='600 11px system-ui';ctx.fillText(label,p[0]+9,p[1]-9)}}
function drawSemantic(){
  const semantic=state?.semantic;
  if(!semantic||!state?.map)return;
  if(enabled.semantic_furniture){
    for(const item of semantic.furniture||[]){
      if(item.status!=='confirmed')continue;
      const [x,y]=worldToScreen(item.x,item.y);
      ctx.save();ctx.beginPath();ctx.arc(x,y,7,0,Math.PI*2);
      ctx.fillStyle='#287dff';
      ctx.strokeStyle='#071421';ctx.lineWidth=2;ctx.fill();ctx.stroke();
      ctx.fillStyle='#fff';ctx.font='bold 10px system-ui';ctx.textAlign='center';
      ctx.fillText('✓',x,y+3.5);ctx.restore();
      if(scale>0.75&&item.label){ctx.save();ctx.font='600 11px system-ui';
        ctx.fillStyle='#fff';ctx.strokeStyle='#071421';ctx.lineWidth=3;
        ctx.strokeText(item.label,x+10,y-7);ctx.fillText(item.label,x+10,y-7);ctx.restore();}
    }
  }
  if(enabled.home&&semantic.home){const [x,y]=worldToScreen(semantic.home.x,semantic.home.y);
    ctx.save();ctx.translate(x,y);ctx.fillStyle='#30dc9b';ctx.strokeStyle='#06291c';ctx.lineWidth=2;
    ctx.beginPath();ctx.moveTo(0,-11);ctx.lineTo(11,-2);ctx.lineTo(8,-2);ctx.lineTo(8,8);
    ctx.lineTo(-8,8);ctx.lineTo(-8,-2);ctx.lineTo(-11,-2);ctx.closePath();ctx.fill();ctx.stroke();
    ctx.restore();ctx.save();ctx.font='600 11px system-ui';ctx.fillStyle='#30dc9b';
    ctx.strokeStyle='#071421';ctx.lineWidth=3;ctx.strokeText('出生点',x+13,y-9);
    ctx.fillText('出生点',x+13,y-9);ctx.restore();}
}
function robot(){if(!state?.robot||!state?.map)return;const r=state.robot,p=worldToScreen(r.x,r.y),metersToPx=scale/state.map.resolution;ctx.save();ctx.translate(...p);ctx.rotate(-r.yaw);ctx.fillStyle='#16d5b244';ctx.strokeStyle='#28f3cd';ctx.lineWidth=2;ctx.fillRect(-.2*metersToPx,-.2*metersToPx,.4*metersToPx,.4*metersToPx);ctx.strokeRect(-.2*metersToPx,-.2*metersToPx,.4*metersToPx,.4*metersToPx);ctx.beginPath();ctx.moveTo(0,0);ctx.lineTo(.32*metersToPx,0);ctx.stroke();ctx.restore()}
function draw(){const rect=wrap.getBoundingClientRect();ctx.setTransform(dpr,0,0,dpr,0,0);ctx.clearRect(0,0,rect.width,rect.height);ctx.fillStyle='#080b0f';ctx.fillRect(0,0,rect.width,rect.height);if(!state?.map||!mapImage.complete)return;ctx.imageSmoothingEnabled=false;ctx.drawImage(mapImage,panX,panY,state.map.width*scale,state.map.height*scale);if(enabled.confirmed_obstacles)points(state.confirmed_obstacles,'#d75cffcc',3);if(enabled.near_obstacles)points(state.near_obstacles,'#ffad32cc',3);drawSemantic();if(enabled.global_path)path(state.global_path,'#279fff',3);if(enabled.local_path)path(state.local_path,'#ff4057',4);if(state.goal)arrow(state.goal.x,state.goal.y,state.goal.yaw,'#69d0ff','目标');robot();if(drag&&drag.end){const yaw=Math.atan2(drag.end[1]-drag.start[1],drag.end[0]-drag.start[0]);arrow(drag.start[0],drag.start[1],yaw,mode==='goal'?'#69d0ff':'#ffcd5d',mode==='goal'?'新目标':'初始位姿')}}
function chip(id,ok){const e=$(id);e.classList.toggle('ok',ok);e.classList.toggle('bad',!ok)}
function updateUi(){if(!state)return;const semantic=state.semantic||{};const furniture=semantic.furniture||[];const confirmed=furniture.filter(item=>item.status==='confirmed').length;$('semanticInfo').textContent=`语义地图：${semantic.home?'出生点已标记':'无出生点'} · 已确认家具 ${confirmed}`;chip('mapChip',state.connected.map);chip('tfChip',state.connected.robot);chip('odomChip',state.connected.odom);$('goalChip').textContent=state.goal_status;$('goalStatus').textContent=state.goal_status;$('vx').textContent=(state.odom?.vx??0).toFixed(2);$('wz').textContent=(state.odom?.wz??0).toFixed(2);$('pose').textContent=state.robot?`${state.robot.x.toFixed(2)}, ${state.robot.y.toFixed(2)}`:'—';$('clock').textContent=`网关在线 · ${new Date(state.server_time*1000).toLocaleTimeString()}`;if(state.map){const id=state.map.map_id?`${state.map.map_id} · `:'';$('mapInfo').textContent=`${id}${(state.map.width*state.map.resolution).toFixed(1)} × ${(state.map.height*state.map.resolution).toFixed(1)} m · ${state.map.resolution.toFixed(3)} m/格`;}}
async function poll(){try{const response=await fetch('/api/state',{cache:'no-store'});if(!response.ok)throw Error(response.status);state=await response.json();if(state.map&&state.map.version!==mapVersion){mapVersion=state.map.version;mapImage.src=`/api/map.png?v=${mapVersion}`;mapImage.onload=()=>{if(!fitted)fit();draw()}}updateUi();velocitySample();draw()}catch(e){$('clock').textContent='网关连接中断'}setTimeout(poll,200)}
function position(event){const r=canvas.getBoundingClientRect();return[event.clientX-r.left,event.clientY-r.top]}
canvas.addEventListener('wheel',e=>{if(!state?.map)return;e.preventDefault();const p=position(e),before=screenToWorld(...p),factor=e.deltaY<0?1.15:1/1.15;scale=Math.max(.05,Math.min(20,scale*factor));const after=worldToScreen(...before);panX+=p[0]-after[0];panY+=p[1]-after[1];draw()},{passive:false});
canvas.addEventListener('contextmenu',e=>e.preventDefault());
canvas.addEventListener('pointerdown',e=>{if(!state?.map)return;canvas.setPointerCapture(e.pointerId);const p=position(e);if(e.button===2||e.button===1){panning={p,x:panX,y:panY}}else if(e.button===0){drag={start:screenToWorld(...p),end:null}}});
canvas.addEventListener('pointermove',e=>{if(!state?.map)return;const p=position(e),w=screenToWorld(...p);$('cursorPos').textContent=`x ${w[0].toFixed(2)}　y ${w[1].toFixed(2)}`;if(panning){panX=panning.x+p[0]-panning.p[0];panY=panning.y+p[1]-panning.p[1];draw()}else if(drag){drag.end=w;draw()}});
canvas.addEventListener('pointerup',async e=>{if(panning){panning=null;return}if(!drag)return;const end=drag.end||drag.start,yaw=Math.atan2(end[1]-drag.start[1],end[0]-drag.start[0]),payload={x:drag.start[0],y:drag.start[1],yaw};drag=null;draw();await post(mode==='goal'?'/api/goal':'/api/initial_pose',payload);toast(mode==='goal'?'导航目标已发送':'初始位姿已发送')});
async function post(url,body={}){const response=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)});if(!response.ok)throw Error(await response.text())}
function toast(message){const e=$('toast');e.textContent=message;e.classList.add('show');setTimeout(()=>e.classList.remove('show'),1800)}
function setMode(value){mode=value;$('goalMode').classList.toggle('active',value==='goal');$('poseMode').classList.toggle('active',value==='initial_pose')}
$('goalMode').onclick=()=>setMode('goal');$('poseMode').onclick=()=>setMode('initial_pose');$('fitMap').onclick=fit;
$('cancelGoal').onclick=async()=>{await post('/api/cancel');toast('已请求取消导航')};
$('clearMaps').onclick=async()=>{await post('/api/clear_costmaps');toast('已请求清理代价地图')};
document.querySelectorAll('[data-layer]').forEach(e=>e.onchange=()=>{enabled[e.dataset.layer]=e.checked;draw()});
mapImage.onload=()=>{if(!fitted)fit();draw()};
resize();poll();
