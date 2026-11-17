'use strict';
(() => {
  const decoder = new TextDecoder();
  class Cdr {
    constructor(bytes) { this.b=bytes;this.v=new DataView(bytes.buffer,bytes.byteOffset,bytes.byteLength);this.o=4;this.le=bytes[1]===1; }
    u32() { this.o+=(4-(this.o-4)%4)%4;const n=this.v.getUint32(this.o,this.le);this.o+=4;return n; }
    str() { const n=this.u32();if(n<1||this.o+n>this.b.length)throw Error('string');const s=decoder.decode(this.b.subarray(this.o,this.o+n-1));this.o+=n;return s; }
    image() { const sec=this.u32()|0,ns=this.u32();this.str();const format=this.str(),n=this.u32();if(n>this.b.length-this.o)throw Error('image');return {stamp:sec*1000+ns/1e6,format,data:this.b.subarray(this.o,this.o+n)}; }
  }
  const previewTopics=document.body.dataset.previewTopics?.split(',');
  const previewIds=(document.body.dataset.previewIds||'left,right,rgb').split(',');
  const cameras=previewIds.map((id,index)=>({id,topic:previewTopics?.[index]||`/r680/debug/calibration/${id}/compressed`,el:document.getElementById(id),times:[],last:0,lag:0,pending:false,url:null,token:0}));
  const byTopic=new Map(cameras.map(c=>[c.topic,c]));
  const statusTopic=document.body.dataset.statusTopic;
  if(statusTopic)byTopic.set(statusTopic,{type:'status',last:0});
  const modelStatus=JSON.parse(document.body.dataset.modelStatus||'{}');
  for(const [id,topic] of Object.entries(modelStatus))byTopic.set(topic,{type:'model_status',id,last:0});
  let ws,subscriptions=new Map(),channels=new Map(),nextSub=1,generation=0;
  for(const c of cameras)c.el.querySelector('button').onclick=()=>c.el.requestFullscreen().catch(()=>{});
  document.getElementById('guides').onchange=e=>document.body.classList.toggle('no-guides',!e.target.checked);
  function subscribe() {
    const add=[];
    for(const [id,c] of channels)if(![...subscriptions.values()].some(s=>s.channel===id)) {
      const sub=nextSub++;subscriptions.set(sub,{channel:id,c});add.push({id:sub,channelId:id});
    }
    if(add.length)ws.send(JSON.stringify({op:'subscribe',subscriptions:add}));
  }
  function connect() {
    const current=++generation;
    ws=new WebSocket(`${location.protocol==='https:'?'wss':'ws'}://${location.host}/ws`,['foxglove.sdk.v1']);ws.binaryType='arraybuffer';
    ws.onopen=()=>{document.getElementById('link').textContent='已连接 · 只读预览';};
    ws.onclose=()=>{subscriptions.clear();channels.clear();for(const c of cameras){c.times=[];c.last=0;}document.getElementById('link').textContent='连接断开，正在重连';setTimeout(connect,2000);};
    ws.onmessage=e=>{
      if(typeof e.data==='string') {
        const m=JSON.parse(e.data);
        if(m.op==='advertise'){for(const ch of m.channels){const c=byTopic.get(ch.topic);if(c&&(ch.schemaName==='sensor_msgs/msg/CompressedImage'||(c.type&&ch.schemaName==='std_msgs/msg/String')))channels.set(ch.id,c);}subscribe();}
        if(m.op==='unadvertise'){const drop=[];for(const id of m.channelIds){channels.delete(id);for(const [sub,s] of subscriptions)if(s.channel===id){drop.push(sub);subscriptions.delete(sub);s.c.last=0;}}if(drop.length)ws.send(JSON.stringify({op:'unsubscribe',subscriptionIds:drop}));}
        return;
      }
      const b=new Uint8Array(e.data);if(b.length<13||b[0]!==1)return;
      const v=new DataView(e.data),s=subscriptions.get(v.getUint32(1,true));if(!s)return;
      const c=s.c;
      if(c.type==='model_status'){try{const j=JSON.parse(new Cdr(b.subarray(13)).str());c.last=performance.now();const m=j.metrics||{};document.getElementById(c.id).querySelector('.prediction-status').textContent=`${j.fresh?'预测更新中':'预测不可用'} · 计算 ${(m.compute_ms||0).toFixed(1)} ms · 源龄 ${(m.source_age_ms||0).toFixed(0)} ms · 均值 ${(m.mean_predicted_m||0).toFixed(2)} m · ${j.reason==='prediction_only_not_distance_validated'?'未做实测距离验收':j.reason}`;}catch{}return;}
      if(c.type==='status'){try{const j=JSON.parse(new Cdr(b.subarray(13)).str());c.last=performance.now();const metrics=document.getElementById('metrics');if(metrics){const x=j.metrics||{};const stereo=cameras.find(x=>x.id==='rgb');if(stereo)stereo.disabled=!!j.rgb_only;metrics.textContent=j.schema==='wla-stereo-network/v1'?`${j.model} · ${j.depth_fresh?'双目深度更新中':'深度不可用'} · 有效 ${((x.valid_depth_fraction||0)*100).toFixed(1)}% · GPU ${(x.gpu_ms||0).toFixed(1)} ms · 处理 ${(x.pipeline_ms||0).toFixed(1)} ms · ${x.rgb_color_available?'RGB 可染色':'RGB 未配对，深度独立输出'} · CPU ${(j.cpu_c||0).toFixed(1)}°C / GPU ${(j.gpu_c||0).toFixed(1)}°C · ${j.thermal_paused?'过温保护已暂停，降温后重启':j.reason}`:j.rgb_only?`${j.backend} · 完整视野标定 RGB · SGBM/BM 已关闭 · ${j.color_fresh?'RGB 更新中':'RGB 不可用'}`:`${j.backend} · ${j.algorithm?.toUpperCase()} · ${j.calibrated?'已加载标定':'未标定，仅二维'} · ${j.depth_fresh?'深度更新中':'深度不可用'} · 有效深度 ${((x.valid_depth_fraction||0)*100).toFixed(1)}% · 计算 ${(x.compute_ms||0).toFixed(1)} ms · RGB/双目时间差 ${(x.host_skew_ms||0).toFixed(1)} ms · ${j.reason}`;}}catch{}return;}
      let m;try{m=new Cdr(b.subarray(13)).image();}catch{return;}
      c.last=performance.now();c.times.push(c.last);c.times=c.times.filter(t=>t>c.last-3000);c.lag=Number(v.getBigUint64(5,true))/1e6-m.stamp;c.stamp=m.stamp;
      if(c.pending)return;c.pending=true;
      const url=URL.createObjectURL(new Blob([m.data],{type:'image/jpeg'})),img=c.el.querySelector('img'),token=++c.token;
      img.onload=()=>{c.pending=false;c.width=img.naturalWidth;c.height=img.naturalHeight;if(c.url)URL.revokeObjectURL(c.url);c.url=url;if(current!==generation||token!==c.token)return;c.el.querySelector('.empty').hidden=true;};
      img.onerror=()=>{c.pending=false;URL.revokeObjectURL(url);};img.src=url;
    };
  }
  setInterval(()=>{
    for(const [id,topic] of Object.entries(modelStatus)){const c=byTopic.get(topic);if(!c.last||performance.now()-c.last>2000)document.getElementById(id).querySelector('.prediction-status').textContent='等待模型推理状态，当前预测不可用';}
    if(statusTopic){const c=byTopic.get(statusTopic);if(!c.last||performance.now()-c.last>2000)document.getElementById('metrics').textContent='等待处理节点状态，当前数值不可用';}
    for(const c of cameras){const now=performance.now(),age=c.last?now-c.last:Infinity,t=c.times.filter(x=>x>now-3000),fps=t.length>1?(t.length-1)*1000/(t[t.length-1]-t[0]):0,img=c.el.querySelector('img'),empty=c.el.querySelector('.empty');
      if(c.disabled){empty.hidden=false;empty.textContent='SGBM/BM 已关闭';}else if(age>1500){empty.hidden=false;empty.textContent=c.last?'画面已停止更新':'等待图像';}
      c.el.querySelector('.meta').textContent=c.last?`${c.width||"—"}×${c.height||"—"} · ${age>1500?'0.0':fps.toFixed(1)} fps · 最近接收 ${Math.round(age)} ms 前 · 采集→桥接 ${Math.max(0,c.lag).toFixed(0)} ms`:'尚未收到图像';
    }
  },300);
  connect();
})();
