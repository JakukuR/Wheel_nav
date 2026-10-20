'use strict';
const planning = location.pathname === '/planning';
const view = planning ? 'planning' : 'detections';
const $ = id => document.getElementById(id);
const canvas = $('canvas'), ctx = canvas.getContext('2d');
let paused = false, data = null, picture = null, lastSuccess = 0;
let etag = null, pictureStamp = null, requestMs = null, delayed = false, expired = false;
let displayEpoch = 0;
const HOLD_MS = 1500;
const rows = new Map(), statusRows = new Map();
function text(el,value){value=String(value);if(el.textContent!==value)el.textContent=value;}
// Reconcile by identity; keep scroll position and existing DOM nodes.
function syncRows(parent, cache, entries, make, patch){
  const wanted=new Set(entries.map(e=>e.key));
  for(const [key,row] of cache)if(!wanted.has(key)){row.el.remove();cache.delete(key);}
  entries.forEach((entry,index)=>{
    let row=cache.get(entry.key);if(!row){row=make(entry);cache.set(entry.key,row);}
    patch(row,entry);
    if(parent.children[index]!==row.el)parent.insertBefore(row.el,parent.children[index]||null);
  });
}
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
  $('topdownPanel').hidden=false; $('title').textContent='体素与路径'; $('eyebrow').textContent='GEOMETRY / LIVE TOP VIEW';
  $('subtitle').textContent='俯视检查障碍、目标和局部路径，下方保留相机投影。';
  $('viewLabel').textContent='RGB + 空间几何'; $('listTitle').textContent='投影图层';
  $('note').textContent='确认体素为障碍记忆快照；二维障碍栅格仅画地面轮廓。叠加不做遮挡消除，可能显示在前景物体后方的几何。路径需规划器已产生结果。';
  $('footerNote').textContent='按图像时刻转换 TF · 缺少变换时停止叠加';
  toggle('voxels','体素');toggle('points','障碍点');
  for (const [id,label] of [['person_path','人物路径'],['object_path','物品路径'],['nav_path','全局路径'],['local_path','局部轨迹'],['person_grid','人物栅格'],['object_grid','物品栅格']]) toggle(id,label);
} else { toggle('person','人物');toggle('objects','物品'); }
$(planning?'planningLink':'detectionLink').classList.add('active');
$('other').href=planning?'/detections':'/planning';
document.title=`R680 · ${planning?'体素与路径':'检测调试'}`;
$('pause').addEventListener('click',()=>{
  paused=!paused;displayEpoch++;if(planning)TopdownView.setDelay(null);$('pause').textContent=paused?'恢复实时':'暂停画面';
  $('paused').hidden=!paused;$('paused').textContent='画面已暂停 · 非实时';
  if(planning)TopdownView.draw(data?.topdown,enabled,paused);
  if(!paused){etag=null;lastSuccess=0;data=null;picture=null;pictureStamp=null;clear('恢复连接中');}
});
function clearRgb(reason){
  ctx.clearRect(0,0,canvas.width,canvas.height);
  $('empty').hidden=false;text($('empty').querySelector('h2'),reason);
}
function clear(reason){if(planning)TopdownView.clear(reason);clearRgb(reason);}
function lag(reason){
  if(expired)return;
  delayed=true;
  const hold=lastSuccess && performance.now()-lastSuccess<=HOLD_MS;
  text($('connection'),hold?'更新暂缓':'等待更新');$('dot').style.backgroundColor='#d8a85d';
  text($('reason'),reason);
  if(hold){
    // Preserve pixels briefly, with an explicit historical-frame label.
    $('paused').hidden=false;text($('paused'),'更新暂缓 · 显示上一帧（非实时）');
    if(planning)TopdownView.setDelay('更新暂缓 · 显示上一帧（非实时）');
  }else{
    expired=true;picture=null;pictureStamp=null;etag=null;
    data=null;if(planning)TopdownView.setDelay(null);clear('画面已过期，等待更新');$('paused').hidden=true;
    text($('frameAge'),'已过期');
  }
}
function draw() {
  if(planning)TopdownView.draw(data?.topdown, enabled, paused);
  if (!data || !picture) return;
  if(canvas.width!==data.width)canvas.width=data.width;
  if(canvas.height!==data.height)canvas.height=data.height;
  ctx.drawImage(picture,0,0,canvas.width,canvas.height);
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
function makeItem(){
  const el=document.createElement('div');el.className='item';const row=document.createElement('div');row.className='item-title';
  const left=document.createElement('span'),swatch=document.createElement('i'),title=document.createElement('span');
  swatch.className='swatch';left.append(swatch,title);
  const right=document.createElement('span'),detail=document.createElement('small');row.append(left,right);el.append(row,detail);
  return {el,title,right,detail,swatch};
}
function patchItem(row,e){text(row.title,e.title);text(row.right,e.right);text(row.detail,e.detail);row.swatch.style.backgroundColor=e.color;}
function update() {
  if(planning && data.mode){
    const local=data.mode==='local';
    for(const id of ['nav_path','local_path']){const el=document.querySelector(`[data-layer="${id}"]`);if(el)el.hidden=local;}
    $('note').textContent=local?'局部模式：障碍点来自跟随算法的 C++ 过滤结果；栅格保留算法的局部历史。RGB 与几何时间差 ≤30 ms，期间运动未补偿；不做遮挡消除。':'确认体素为障碍记忆快照；二维障碍栅格仅画地面轮廓。不做遮挡消除，路径需规划器已产生结果。';
    $('footerNote').textContent=local?'算法同源障碍 · 需跟随节点、深度与轮速 · 无 Nav2 依赖':'按图像时刻转换 TF · 缺少变换时停止叠加';
  }
  text($('reason'),explain(data.reason));
  const entries=[];
  if(planning){
    for(const l of data.layers || [])entries.push({key:l.id,title:l.label,right:l.reason==='ok'?(l.id.includes('path') && !l.count?'暂无路径':`${l.count} 个`):'—',detail:`${explain(l.reason)} · ${l.topic}${l.skew_ms===undefined?'':` · Δt ${l.skew_ms} ms`}`,color:l.color});
    text($('count'),`${(data.layers || []).filter(x=>x.reason==='ok').length} / ${(data.layers || []).length}`);
  }else{
    for(const o of data.objects || [])entries.push({key:o.id??`${o.label}:${o.track_id}`,title:labelText(o.label),right:`${Math.round(o.confidence*100)}%`,detail:`ID ${o.track_id ?? '—'} · ${o.distance===null?'距离不可用':o.distance.toFixed(2)+' m'}${o.yaw===null?'':` · 方位 ${(o.yaw*180/Math.PI).toFixed(1)}°`}`,color:o.label==='person'?'#54e0ce':'#ffbe69'});
    text($('count'),(data.objects || []).length);
    if(!entries.length)entries.push({key:'empty',title:data.reason==='ok'?'当前帧没有检测到目标。':'尚无匹配的检测结果。',right:'',detail:'',color:'transparent'});
  }
  // The initial placeholder is removed once, not on every response.
  if(!rows.size)$('items').replaceChildren();
  syncRows($('items'),rows,entries,makeItem,patchItem);
  syncRows($('statuses'),statusRows,Object.entries(data.statuses || {}).map(([key,s])=>({key,value:`${key}: ${s.state} ${s.reason} · ${s.age_s}s 前`})),()=>({el:document.createElement('p')}),(row,e)=>text(row.el,e.value));
  $('connection').textContent=data.image?'画面已接入':'等待数据';$('dot').style.backgroundColor=data.image?'#54e0ce':'#d8a85d';
  $('meta').textContent=`ROS 域 ${data.domain ?? '—'} · 页面 ${data.render_ms ?? '—'} ms · 请求 ${requestMs ?? '—'} ms`;
  $('frame').textContent=data.frame_id?`${data.frame_id} · ${data.width} × ${data.height}`:'尚无图像';
  $('frameAge').textContent=data.image?`图像年龄 ${data.age_ms} ms`:'—';
}
async function poll(){
  if(!paused && !document.hidden){
    const epoch=displayEpoch;
    try{
      const started=performance.now();
      const response=await fetch(`/api/${view}`,{cache:'no-store',headers:etag?{'If-None-Match':etag}:{},signal:AbortSignal.timeout(2500)});
      if(response.status===304){
        if(!paused && epoch===displayEpoch && data){
          lastSuccess=performance.now();
          if(delayed){delayed=false;expired=false;$('paused').hidden=true;if(planning)TopdownView.setDelay(null);update();draw();}
        }
      }else{
        if(!response.ok)throw new Error(`HTTP ${response.status}`);
        const next=await response.json();requestMs=Math.round(performance.now()-started);
        let img=null;
        if(next.image){
          if(picture && next.stamp_ns && next.stamp_ns===pictureStamp)img=picture;
          else{img=new Image();img.src='data:image/jpeg;base64,'+next.image;await img.decode();}
        }
        if(!paused && epoch===displayEpoch){
          etag=response.headers.get('ETag');data=next;picture=img;pictureStamp=img?next.stamp_ns:null;
          lastSuccess=performance.now();delayed=false;expired=false;$('paused').hidden=true;if(planning)TopdownView.setDelay(null);
          update();
          if(img){$('empty').hidden=true;draw();}
          else{clearRgb(explain(data.reason));if(planning)TopdownView.draw(data.topdown,enabled,paused);}
        }
      }
    }catch(e){if(!paused && epoch===displayEpoch)lag(`更新暂缓：${e.message}`);}
  }
  setTimeout(poll,200);
}
setInterval(()=>{
  if(!paused && lastSuccess && performance.now()-lastSuccess>HOLD_MS)lag('更新超时');
},200);
poll();
