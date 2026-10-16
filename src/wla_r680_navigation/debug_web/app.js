'use strict';
const planning = location.pathname === '/planning';
const view = planning ? 'planning' : 'detections';
const $ = id => document.getElementById(id);
const canvas = $('canvas'), ctx = canvas.getContext('2d');
let paused = false, data = null, picture = null, lastSuccess = 0;
const enabled = new Set(planning ? ['voxels','points','person_path','object_path','nav_path','local_path','person_grid','object_grid'] : ['person','objects']);
const messages = {ok:'数据已对齐', waiting_for_rgb:'等待 RGB 相机数据', rgb_stale:'RGB 已过期，等待新图像', waiting_for_exact_detection_frame:'等待与 RGB 同帧的检测结果', waiting_for_matching_camera_info:'等待匹配的相机内参 CameraInfo', camera_info_stale_or_time_mismatch:'CameraInfo 过期或时间不匹配', waiting_for_topic:'尚未收到数据', source_stale_or_zero_stamp:'数据过期或缺少有效时间戳', image_source_time_mismatch:'与图像时间差过大', oversized_message:'数据量超出显示上限'};
const explain = reason => messages[reason] || reason || '等待数据';
const labelMap = {person:'人物',chair:'椅子',bottle:'瓶子',cup:'杯子',couch:'沙发',dining_table:'餐桌',tv:'电视',laptop:'笔记本电脑',backpack:'背包',cell_phone:'手机'};
const labelText = name => labelMap[name] ? `${labelMap[name]} · ${name}` : name;
function toggle(id, label) {
  const el = document.createElement('label'), input = document.createElement('input');el.dataset.layer=id;
  input.type='checkbox'; input.checked=enabled.has(id); input.addEventListener('change',()=>{input.checked?enabled.add(id):enabled.delete(id);draw();});
  el.append(input, document.createTextNode(' '+label)); $('toggles').append(el);
}
if (planning) {
  $('title').textContent='体素与路径投影'; $('eyebrow').textContent='GEOMETRY / RGB PROJECTION';
  $('subtitle').textContent='将三维障碍与规划路径映射到相机视角。';
  $('viewLabel').textContent='RGB + 空间几何'; $('listTitle').textContent='投影图层';
  $('note').textContent='确认体素为障碍记忆快照；二维障碍栅格仅画地面轮廓。叠加不做遮挡消除，可能显示在前景物体后方的几何。路径需规划器已产生结果。';
  $('footerNote').textContent='按图像时刻转换 TF · 缺少变换时停止叠加';
  toggle('voxels','体素');toggle('points','障碍点');
  for (const [id,label] of [['person_path','人物路径'],['object_path','物品路径'],['nav_path','全局路径'],['local_path','局部轨迹'],['person_grid','人物栅格'],['object_grid','物品栅格']]) toggle(id,label);
} else { toggle('person','人物');toggle('objects','物品'); }
$(planning?'planningLink':'detectionLink').classList.add('active');
$('other').href=planning?'/detections':'/planning';
document.title=`R680 · ${planning?'体素与路径':'检测调试'}`;
$('pause').addEventListener('click',()=>{paused=!paused;$('pause').textContent=paused?'恢复实时':'暂停画面';$('paused').hidden=!paused;if(!paused){data=null;picture=null;clear('恢复连接中');}});
function clear(reason) { ctx.clearRect(0,0,canvas.width,canvas.height);$('empty').hidden=false;$('empty').querySelector('h2').textContent=reason; }
function draw() {
  if (!data || !picture) return;
  canvas.width=data.width;canvas.height=data.height;ctx.drawImage(picture,0,0,canvas.width,canvas.height);
  ctx.lineWidth=Math.max(1.5,canvas.width/500);ctx.lineJoin='round';
  if (planning) {
    for (const layer of data.layers || []) {
      if (!enabled.has(layer.id) || layer.reason!=='ok') continue;
      ctx.strokeStyle=layer.color;ctx.fillStyle=layer.color;ctx.globalAlpha=layer.id.endsWith('grid')?.5:.85;
      ctx.lineWidth=layer.id.includes('path')?3:1;
      ctx.beginPath();for(const [x,y,u,v] of layer.segments || []){ctx.moveTo(x,y);ctx.lineTo(u,v);}ctx.stroke();
      for(const [x,y] of layer.points || [])ctx.fillRect(x-1,y-1,2,2);
    }
    ctx.globalAlpha=1;
  } else {
    for(const o of data.objects || []) {
      if(!enabled.has(o.label==='person'?'person':'objects'))continue;
      const [x,y,x2,y2]=o.bbox, color=o.label==='person'?'#54e0ce':'#ffbe69';ctx.strokeStyle=color;ctx.strokeRect(x,y,x2-x,y2-y);
      const caption=`${o.label} #${o.track_id ?? '—'}  ${Math.round(o.confidence*100)}%${o.distance===null?'':`  ${o.distance.toFixed(2)}m`}`;
      ctx.font=`${Math.max(12,canvas.width/64)}px sans-serif`;const w=ctx.measureText(caption).width+12,h=22;
      const bx=Math.max(0,Math.min(x,canvas.width-w)),by=Math.max(0,y-h);
      ctx.fillStyle=color;ctx.fillRect(bx,by,w,h);ctx.fillStyle='#092027';ctx.fillText(caption,bx+6,by+15);
    }
  }
}
function item(title, right, detail, color) {
  const el=document.createElement('div');el.className='item';const row=document.createElement('div');row.className='item-title';
  const left=document.createElement('span'),swatch=document.createElement('i');swatch.className='swatch';swatch.style.backgroundColor=color;left.append(swatch,document.createTextNode(title));
  const value=document.createElement('span');value.textContent=right;row.append(left,value);const small=document.createElement('small');small.textContent=detail;el.append(row,small);return el;
}
function update() {
  if(planning && data.mode){
    const local=data.mode==='local';
    for(const id of ['nav_path','local_path']){const el=document.querySelector(`[data-layer="${id}"]`);if(el)el.hidden=local;}
    $('note').textContent=local?'独立局部模式：当前深度帧体素；人物／物品跟随自己的局部路径。RGB 与几何时间差 ≤30 ms，期间运动未补偿；不累积地图、不做遮挡消除。':'确认体素为障碍记忆快照；二维障碍栅格仅画地面轮廓。不做遮挡消除，路径需规划器已产生结果。';
    $('footerNote').textContent=local?'相机局部几何 · 无 Nav2 / 地图 / 里程计依赖':'按图像时刻转换 TF · 缺少变换时停止叠加';
  }
  $('reason').textContent=explain(data.reason);$('items').replaceChildren();
  if(planning){
    for(const l of data.layers || [])$('items').append(item(l.label,l.reason==='ok'?(l.id.includes('path') && !l.count?'暂无路径':`${l.count} 个`):'—',`${explain(l.reason)} · ${l.topic}${l.skew_ms===undefined?'':` · Δt ${l.skew_ms} ms`}`,l.color));
    $('count').textContent=`${(data.layers || []).filter(x=>x.reason==='ok').length} / ${(data.layers || []).length}`;
  }else{
    for(const o of data.objects || [])$('items').append(item(labelText(o.label),`${Math.round(o.confidence*100)}%`,`ID ${o.track_id ?? '—'} · ${o.distance===null?'距离不可用':o.distance.toFixed(2)+' m'}${o.yaw===null?'':` · 方位 ${(o.yaw*180/Math.PI).toFixed(1)}°`}`,o.label==='person'?'#54e0ce':'#ffbe69'));
    $('count').textContent=(data.objects || []).length;
    if(!(data.objects || []).length){const p=document.createElement('p');p.className='muted';p.textContent=data.reason==='ok'?'当前帧没有检测到目标。':'尚无匹配的检测结果。';$('items').append(p);}
  }
  $('statuses').replaceChildren();
  for(const [name,s] of Object.entries(data.statuses || {})){const p=document.createElement('p');p.textContent=`${name}: ${s.state} ${s.reason} · ${s.age_s}s 前`; $('statuses').append(p);}
  $('connection').textContent=data.image?'画面已接入':'等待数据';$('dot').style.backgroundColor=data.image?'#54e0ce':'#d8a85d';
  $('meta').textContent=`ROS 域 ${data.domain ?? '—'} · ${data.render_ms ?? '—'} ms 处理`;
  $('frame').textContent=data.frame_id?`${data.frame_id} · ${data.width} × ${data.height}`:'尚无图像';
  $('frameAge').textContent=data.image?`图像年龄 ${data.age_ms} ms`:'—';
}
async function poll(){
  if(!paused && !document.hidden){
    try{
      const response=await fetch(`/api/${view}`,{cache:'no-store',signal:AbortSignal.timeout(2500)});
      if(!response.ok)throw new Error(`HTTP ${response.status}`);
      const next=await response.json();
      let img=null;
      if(next.image){img=new Image();img.src='data:image/jpeg;base64,'+next.image;await img.decode();}
      if(!paused){data=next;picture=img;lastSuccess=performance.now();update();if(img){$('empty').hidden=true;draw();}else{clear(explain(data.reason));}}
    }catch(e){if(!paused){data=null;picture=null;clear('连接中断');$('connection').textContent='连接中断';$('dot').style.backgroundColor='#e28b80';$('reason').textContent=e.message;$('items').replaceChildren();$('count').textContent='—';}}
  }
  setTimeout(poll,200);
}
setInterval(()=>{if(!paused && lastSuccess && performance.now()-lastSuccess>1500){picture=null;clear('画面已过期，等待更新');}},400);
poll();
