'use strict';
// Read-only Canvas renderer. XY are robot-relative meters, never pixel detections.
window.TopdownView = (() => {
  const el = id => document.getElementById(id);
  let last = null, selectedLayers = new Set(), frozen = false;
  const stateRows=new Map();
  let delayNotice=null;
  function setDelay(reason){delayNotice=reason;if(reason)el('topBadge').textContent=reason;}
  const validPoint = p => Array.isArray(p) && p.length >= 2 && p.slice(0,2).every(Number.isFinite);
  function draw(top, enabled = selectedLayers, paused = frozen) {
    last=top;selectedLayers=enabled;frozen=paused;
    const canvas=el('topCanvas');if(!canvas)return;
    const c=canvas.getContext('2d'), w=canvas.width, h=canvas.height;
    const span=Number(el('topRange').value), scale=(h-80)/span, ox=w/2, oy=h*.70;
    const p=(x,y)=>[ox-y*scale,oy-x*scale];
    c.clearRect(0,0,w,h);c.fillStyle='#101923';c.fillRect(0,0,w,h);
    const poly=(points,fill,stroke)=>{
      if(!points.length)return;
      c.beginPath();points.forEach((q,i)=>{const xy=p(...q);i?c.lineTo(...xy):c.moveTo(...xy);});c.closePath();
      if(fill){c.fillStyle=fill;c.fill();}if(stroke){c.strokeStyle=stroke;c.stroke();}
    };
    const circle=(xy,r,fill,stroke)=>{c.beginPath();c.arc(...p(...xy),r*scale,0,Math.PI*2);if(fill){c.fillStyle=fill;c.fill();}if(stroke){c.strokeStyle=stroke;c.stroke();}};
    const label=(xy,text,color='#b4c5d6')=>{c.fillStyle=color;c.font='13px sans-serif';const a=p(...xy);c.fillText(text,a[0]+10,a[1]-10);};
    const layers=(top?.layers||[]).filter(l=>l.reason==='ok');
    // Render known free cells first, then occupied cells, independent of RGB toggles.
    for(const occupied of [false,true])for(const l of layers){
      if(l.kind!=='grid'||!enabled.has(l.id)||!l.width||(!occupied&&!el('topFree').checked))continue;
      const b=l.basis,o=l.origin;if(!b||!validPoint(o))continue;
      c.beginPath();
      for(let i=0;i<l.cells.length;i++){
        const value=l.cells[i];if(occupied?value<50:value!==0)continue;
        const col=i%l.width,row=Math.floor(i/l.width),x=o[0]+b[0][0]*col+b[0][1]*row,y=o[1]+b[1][0]*col+b[1][1]*row;
        const a=p(x,y);if(a[0]<-30||a[0]>w+30||a[1]<-30||a[1]>h+30)continue;
        c.moveTo(...a);c.lineTo(...p(x+b[0][0],y+b[1][0]));c.lineTo(...p(x+b[0][0]+b[0][1],y+b[1][0]+b[1][1]));c.lineTo(...p(x+b[0][1],y+b[1][1]));c.closePath();
      }
      c.fillStyle=occupied?l.color:'#244344';c.globalAlpha=occupied?.85:.65;c.fill();c.globalAlpha=1;
    }
    c.lineWidth=1;c.strokeStyle='#49607444';c.font='11px sans-serif';c.fillStyle='#8296ab';
    for(let v=-14;v<=14;v++){
      const [a,b]=p(v,0),[d,e]=p(0,v);
      if(b>=35&&b<h-20){c.beginPath();c.moveTo(0,b);c.lineTo(w,b);c.stroke();c.fillText(`${v}m`,8,b-4);}
      if(d>=30&&d<w-30){c.beginPath();c.moveTo(d,0);c.lineTo(d,h);c.stroke();c.fillText(`${v}m`,d+4,h-10);}
    }
    if(el('topPoints').checked)for(const l of layers){
      if(l.kind!=='points'||!enabled.has(l.id))continue;c.fillStyle='#ef927c';c.globalAlpha=.75;
      for(const q of l.points||[])if(validPoint(q)){const a=p(...q);c.fillRect(a[0]-1.5,a[1]-1.5,3,3);}c.globalAlpha=1;
    }
    for(const l of layers){
      if(l.kind!=='path'||!enabled.has(l.id))continue;c.strokeStyle=l.color;c.lineWidth=3;c.beginPath();
      (l.points||[]).filter(validPoint).forEach((q,i)=>i?c.lineTo(...p(...q)):c.moveTo(...p(...q)));c.stroke();
      if(l.points?.length){circle(l.points.at(-1),.06,l.color);label(l.points.at(-1),l.label,l.color);}
    }
    for(const obj of top?.objects||[]){
      if(!validPoint(obj.xy))continue;const color=obj.selected?'#ffc675':'#b6a1ff';
      circle(obj.xy,.10,color);if(obj.selected){c.lineWidth=2;circle(obj.xy,.18,null,color);}
      const short=String(obj.id).split(':').at(-1);label(obj.xy,`${obj.selected?'锁定 ':''}${obj.label==='person'?'人物':obj.label} #${short}`,color);
    }
    const prior=top?.follow?.local_free_prior;
    const priorActive=prior?.state==='continuous'&&Number.isFinite(prior.radius_m)&&prior.radius_m>0&&prior.radius_m<=.5&&enabled.has('person_grid')&&layers.some(l=>l.id==='person_grid');
    if(priorActive){
      c.lineWidth=1.5;c.setLineDash([3,5]);circle([0,0],prior.radius_m,null,'#e6bf69');c.setLineDash([]);
      label([-prior.radius_m-.35,0],`默认空闲 ${prior.radius_m.toFixed(2)} m · 障碍优先`,'#e6bf69');
    }
    c.lineWidth=1.5;c.setLineDash([5,5]);circle([0,0],(top?.robot_radius||.19)+(top?.obstacle_margin||.10),null,'#f2f6ff88');c.setLineDash([]);
    circle([0,0],top?.robot_radius||.19,'#dde7f01c','#f2f6ff');poly([[.15,0],[-.10,.10],[-.05,0],[-.10,-.10]],'#f2f6ff');
    label([-.36,0],'车辆原点','#e9eef5');c.fillStyle='#b4c5d6';c.font='13px sans-serif';c.fillText('↑ 前方 +X',w/2-35,25);c.fillText('← 左侧 +Y',18,26);
    c.strokeStyle='#d5e3ee';c.lineWidth=2;c.beginPath();c.moveTo(w-35-scale,h-37);c.lineTo(w-35,h-37);c.stroke();c.fillText('1 m',w-35-scale/2-10,h-45);
    const active=top?.reason==='ok';el('topBadge').textContent=paused?'已暂停 · 非实时':delayNotice?delayNotice:active?'实时局部快照':'等待新鲜局部数据';
    el('topMeta').textContent=`${top?.frame_id||'车体坐标'} · ${layers.length} 个有效图层 · ${priorActive?'黄色虚线内默认空闲（障碍优先，非传感器实测）':'未知区域不代表空闲'}`;
    const follow=top?.follow||{}, velocity=Number.isFinite(follow.linear_mps)?` · ${follow.linear_mps.toFixed(2)} m/s`:'';
    const identity=follow.reid, reidText=identity&&identity.state!=='disabled'?` · ReID ${identity.state}${Number.isFinite(identity.lost_s)?' · 未核验 '+identity.lost_s.toFixed(1)+' s':''}`:'';
    el('topFollow').textContent=follow.state?`${follow.state} · ${follow.safety_reason||'—'}${velocity}${reidText}`:'暂无新鲜跟随状态';
    const states=el('topLayers'),entries=(top?.layers||[]).map(l=>[l.id,`${l.label}：${l.reason==='ok'?`${l.count} 个 · ${l.age_ms} ms 前`:l.reason}`]);
    entries.push(['objects',`目标观测：${top?.objects_reason||'等待数据'}${top?.objects_age_ms===undefined?'':` · ${top.objects_age_ms} ms 前`}`]);
    const keys=new Set(entries.map(e=>e[0]));
    for(const [key,row] of stateRows)if(!keys.has(key)){row.remove();stateRows.delete(key);}
    entries.forEach(([key,value],i)=>{
      let row=stateRows.get(key);if(!row){row=document.createElement('p');stateRows.set(key,row);}
      if(row.textContent!==value)row.textContent=value;
      if(states.children[i]!==row)states.insertBefore(row,states.children[i]||null);
    });
  }
  function clear(reason){draw(null);el('topBadge').textContent=reason;}
  document.addEventListener('DOMContentLoaded',()=>{for(const id of ['topRange','topFree','topPoints'])el(id)?.addEventListener('change',()=>draw(last));});
  return {draw,clear,setDelay};
})();
