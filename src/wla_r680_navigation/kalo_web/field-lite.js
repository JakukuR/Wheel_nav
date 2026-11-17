(function(root){
'use strict';
function valid(j){
 if(!j||j.schema!=='stereo-snapshot-field/v1'||j.frame!=='sunplus_left_rect_optical_frame'||j.temporal_fusion!==false||j.navigation_connected!==false)return false;
 if(!j.available)return true;
 const finite=Number.isFinite;
 if(!finite(j.compute_ms)||j.compute_ms<0||!Number.isInteger(j.occupied_voxels)||j.occupied_voxels<0||j.occupied_voxels>144000)return false;
 if(!finite(j.stamp)||j.stamp<=0||!Array.isArray(j.shape)||j.shape.join(',')!=='60,40,60'||!finite(j.resolution_m)||Math.abs(j.resolution_m-.05)>1e-6||!Array.isArray(j.origin_m)||j.origin_m.length!==3||!j.origin_m.every(finite)||!finite(j.max_distance_m)||j.max_distance_m!==1)return false;
 if(!Array.isArray(j.surface_voxels)||j.surface_voxels.length>4000||!j.surface_voxels.every(p=>Array.isArray(p)&&p.length===3&&p.every((v,i)=>Number.isInteger(v)&&v>=0&&v<j.shape[i])))return false;
 return Array.isArray(j.slices)&&j.slices.length===3&&j.slices.every(s=>finite(s.y_m)&&Array.isArray(s.state)&&Array.isArray(s.distance_mm)&&s.state.length===3600&&s.distance_mm.length===3600&&s.state.every((v,i)=>Number.isInteger(v)&&v>=0&&v<=2&&(v===0?s.distance_mm[i]===null:Number.isInteger(s.distance_mm[i])&&Math.abs(s.distance_mm[i])<=1000&&(v===1?s.distance_mm[i]>=0:s.distance_mm[i]<=0))));
}
function color(state,mm){if(!state)return '#0b1018';if(state===2)return '#ff685e';const t=Math.max(0,Math.min(1,mm/1000));return `rgb(${Math.round(237-202*t)},${Math.round(181-54*t)},${Math.round(74+129*t)})`;}
const api={valid,color};if(typeof module!=='undefined')module.exports=api;if(!root.document)return;
const $=id=>document.getElementById(id),surface=$('surface'),sliceCanvas=$('sliceCanvas');let data=null,last=0,lag=0,times=[],ws,generation=0,sub=null,channel=null,az=-.4,el=.65,zoom=1,drag=null;
function empty(reason){data=null;surface.getContext('2d').clearRect(0,0,surface.width,surface.height);sliceCanvas.getContext('2d').clearRect(0,0,sliceCanvas.width,sliceCanvas.height);$('surfaceEmpty').hidden=false;$('sliceEmpty').hidden=false;$('surfaceEmpty').textContent=reason;$('sliceEmpty').textContent=reason;$('hover').textContent='当前距离数据不可用';}
function drawSurface(){if(!data)return;const c=surface.getContext('2d'),w=surface.width,h=surface.height;c.fillStyle='#0b1018';c.fillRect(0,0,w,h);const scale=105*zoom;
 const project=(x,y,z)=>{z-=1.5;const xx=Math.cos(az)*x+Math.sin(az)*z,zz=-Math.sin(az)*x+Math.cos(az)*z;return [w*.5+xx*scale,h*.53+(y*Math.cos(el)-zz*Math.sin(el))*scale,zz*Math.cos(el)+y*Math.sin(el)];};
 function line(a,b,col){const p=project(...a),q=project(...b);c.strokeStyle=col;c.beginPath();c.moveTo(p[0],p[1]);c.lineTo(q[0],q[1]);c.stroke();}
 for(let x=-1.5;x<=1.5;x+=.5)line([x,0,0],[x,0,3],'#273746');for(let z=0;z<=3;z+=.5)line([-1.5,0,z],[1.5,0,z],'#273746');
 const points=data.surface_voxels.map(v=>{const p=v.map((n,i)=>data.origin_m[i]+(n+.5)*data.resolution_m);return {p,q:project(...p)};}).sort((a,b)=>b.q[2]-a.q[2]);
 for(const {p,q} of points){const t=p[2]/3;c.fillStyle=`rgb(${Math.round(255-170*t)},${Math.round(143+60*t)},${Math.round(70+175*t)})`;c.fillRect(q[0]-2,q[1]-2,4,4);}
 line([0,0,0],[.3,0,0],'#ff685e');line([0,0,0],[0,.3,0],'#7cda99');line([0,0,0],[0,0,.4],'#6cb8ff');const o=project(0,0,0);c.fillStyle='white';c.font='13px system-ui';c.fillText('相机',o[0]+5,o[1]-7);
}
function drawSlice(){if(!data)return;const c=sliceCanvas.getContext('2d'),w=sliceCanvas.width,h=sliceCanvas.height,s=data.slices[Number($('slice').value)],size=330,x0=(w-size)/2,y0=22,cell=size/60;c.fillStyle='#101720';c.fillRect(0,0,w,h);
 for(let z=0;z<60;z++)for(let x=0;x<60;x++){const i=z*60+x;c.fillStyle=color(s.state[i],s.distance_mm[i]);c.fillRect(x0+x*cell,y0+(59-z)*cell,cell+.2,cell+.2);}
 c.strokeStyle='#c1d2e144';c.lineWidth=1;for(let i=0;i<=60;i+=10){c.beginPath();c.moveTo(x0+i*cell,y0);c.lineTo(x0+i*cell,y0+size);c.moveTo(x0,y0+i*cell);c.lineTo(x0+size,y0+i*cell);c.stroke();}
 c.fillStyle='#c8d5e2';c.font='13px system-ui';for(let z=0;z<=3;z+=.5)c.fillText(z.toFixed(1)+' m',x0-44,y0+size-z/3*size+4);c.fillText('左 1.5 m',x0,y0+size+22);c.fillText('相机 / 0',x0+size/2-25,y0+size+22);c.fillText('右 1.5 m',x0+size-54,y0+size+22);c.fillText(`光学 y = ${s.y_m.toFixed(3)} m（向下为正）`,x0,y0+size+47);
}
function render(){drawSurface();drawSlice();}
surface.onpointerdown=e=>{drag=[e.clientX,e.clientY];surface.setPointerCapture(e.pointerId);};surface.onpointermove=e=>{if(!drag)return;az+=(e.clientX-drag[0])*.01;el=Math.max(-1.4,Math.min(1.4,el+(e.clientY-drag[1])*.01));drag=[e.clientX,e.clientY];drawSurface();};surface.onpointerup=()=>{drag=null;};surface.onpointercancel=()=>{drag=null;};surface.addEventListener('wheel',e=>{e.preventDefault();zoom=Math.max(.5,Math.min(3,zoom*Math.exp(-e.deltaY*.001)));drawSurface();},{passive:false});$('reset').onclick=()=>{az=-.4;el=.65;zoom=1;drawSurface();};$('slice').onchange=drawSlice;
sliceCanvas.onpointermove=e=>{if(!data)return;const r=sliceCanvas.getBoundingClientRect(),px=(e.clientX-r.left)*700/r.width,py=(e.clientY-r.top)*420/r.height,x=Math.floor((px-185)/5.5),z=59-Math.floor((py-22)/5.5);if(x<0||x>=60||z<0||z>=60)return;const s=data.slices[Number($('slice').value)],i=z*60+x;$('hover').textContent=`左右 ${(-1.5+(x+.5)*.05).toFixed(2)} m · 前方 ${((z+.5)*.05).toFixed(2)} m · ${s.state[i]===0?'未知':s.state[i]===2?'表面体素':`距已观测表面 ${(s.distance_mm[i]/1000).toFixed(2)} m`}`;};
function connect(){const g=++generation;ws=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/ws`,['foxglove.sdk.v1']);ws.binaryType='arraybuffer';ws.onopen=()=>{$('link').textContent='已连接 · 只读';};ws.onclose=()=>{if(g!==generation)return;sub=null;channel=null;last=0;times=[];empty('连接断开');$('link').textContent='正在重连';setTimeout(connect,2000);};
 ws.onmessage=e=>{if(g!==generation)return;if(typeof e.data==='string'){const m=JSON.parse(e.data);if(m.op==='advertise')for(const ch of m.channels)if(ch.topic==='/r680/debug/stereo/field'&&ch.schemaName==='std_msgs/msg/String'&&sub===null){channel=ch.id;sub=1;ws.send(JSON.stringify({op:'subscribe',subscriptions:[{id:sub,channelId:channel}]}));}if(m.op==='unadvertise'&&m.channelIds.includes(channel)){ws.send(JSON.stringify({op:'unsubscribe',subscriptionIds:[sub]}));channel=null;sub=null;empty('距离场节点已离线');}return;}
  try{const b=new Uint8Array(e.data);if(b.length<21||b[0]!==1)return;const v=new DataView(e.data);if(v.getUint32(1,true)!==sub)return;const c=b.subarray(13),d=new DataView(c.buffer,c.byteOffset,c.byteLength),n=d.getUint32(4,c[1]===1);if(n<1||n>c.length-8)return;const j=JSON.parse(new TextDecoder().decode(c.subarray(8,8+n-1)));if(!valid(j)){empty('距离场格式无效');return;}last=performance.now();times.push(last);times=times.filter(t=>t>last-3000);lag=Number(v.getBigUint64(5,true))/1e6-(j.stamp||0)*1000;
   if(!j.available||lag< -20||lag>500){empty('等待新鲜深度 / 当前帧不可用');return;}if(!data)$('hover').textContent='横轴：相机左右；纵轴：相机前方距离';data=j;$('surfaceEmpty').hidden=true;$('sliceEmpty').hidden=true;render();
  }catch{empty('数据解析失败');}
 };
}
setInterval(()=>{const age=last?performance.now()-last:Infinity;if(age>750){empty('距离场已过期，等待新帧');$('metrics').textContent='暂无有效距离场';return;}if(!data){$('metrics').textContent='输入不可用；旧地图已清除';return;}const t=times.filter(x=>x>performance.now()-3000),hz=t.length>1?(t.length-1)*1000/(t[t.length-1]-t[0]):0;$('metrics').textContent=`${hz.toFixed(1)} Hz · ${data.compute_ms.toFixed(1)} ms 计算 · ${Math.max(0,lag+age).toFixed(0)} ms 源龄 · ${data.occupied_voxels} 个表面体素 · 5 cm 网格 · 未知区域不作自由空间`;},200);
connect();
})(typeof window==='undefined'?globalThis:window);
