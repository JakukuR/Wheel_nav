/* Read-only telemetry view. No ROS publications, filtering, position or TF estimates. */
(function (root) {
  'use strict';
  const TOPICS = ['/robot_state/attitude', '/robot_state/rpy', '/joint_states', '/robot_state/status', '/robot_state/geometry'];
  const JOINTS = [['abad_L_Joint', '左外展'], ['abad_R_Joint', '右外展'], ['hip_L_Joint', '左髋'], ['hip_R_Joint', '右髋'],
    ['knee_L_Joint', '左膝'], ['knee_R_Joint', '右膝'], ['wheel_L_Joint', '左轮'], ['wheel_R_Joint', '右轮']];
  const DATA_TTL = 500, STATUS_TTL = 1500;
  const finite = a => Array.isArray(a) && a.every(Number.isFinite);
  // User-confirmed debug display convention: roll/yaw reversed, pitch unchanged.
  // Conjugation by Ry(pi): q_display = C*q*C^-1 = [-qx, qy, -qz, qw].
  // This stays a proper rotation and keeps the diagram/quaternion/RPY consistent.
  // It is not an IMU mounting calibration and is never published back to ROS.
  function DisplayAttitude(q, rpy) {
    return { q:q?[-q[0],q[1],-q[2],q[3]]:null, rpy:rpy?[-rpy[0],rpy[1],-rpy[2]]:null };
  }
  class StateModel {
    constructor() { this.connected = false; this.entries = new Map(); this.rates = new Map(); }
    reset() { this.entries.clear(); this.rates.clear(); this.connected = false; }
    receive(topic, msg, recv, now) {
      if (!TOPICS.includes(topic)) return;
      let valid = false;
      if (topic === TOPICS[3] || topic === TOPICS[4]) {
        try { if (typeof msg === 'string') msg = JSON.parse(msg); valid = msg && typeof msg === 'object' && !Array.isArray(msg); } catch (_) { valid = false; }
      } else if (msg && msg.header && Number.isFinite(msg.header.t) && msg.header.t > 0) {
        if (topic === TOPICS[0]) valid = finite(msg.q) && msg.q.length === 4 && Math.abs(Math.hypot(...msg.q) - 1) < .05;
        if (topic === TOPICS[1]) valid = finite(msg.v) && msg.v.length === 3;
        if (topic === TOPICS[2]) valid = Array.isArray(msg.names) && msg.names.length <= 64 &&
          msg.names.every(n => typeof n === 'string') && new Set(msg.names).size === msg.names.length &&
          finite(msg.positions) && msg.names.length === msg.positions.length;
      }
      this.entries.set(topic, { msg, recv, arrived: now, valid });
      const times = this.rates.get(topic) || [];
      times.push(now); while (times.length > 800 || (times.length > 1 && now - times[0] > 2000)) times.shift();
      this.rates.set(topic, times);
    }
    snapshot(now) {
      const entry = topic => this.entries.get(topic);
      const statusEntry = entry(TOPICS[3]);
      const status = this.connected && statusEntry && statusEntry.valid && now - statusEntry.arrived <= STATUS_TTL ? statusEntry.msg : null;
      const age = record => record && record.valid ? record.recv - record.msg.header.t + now - record.arrived : Infinity;
      const current = record => this.connected && record && record.valid && now - record.arrived <= DATA_TTL && age(record) >= -20 && age(record) <= DATA_TTL;
      const geometryEntry=entry(TOPICS[4]);
      const geometry=this.connected && geometryEntry && geometryEntry.valid && now-geometryEntry.arrived<=DATA_TTL && geometryEntry.msg.available && Number.isFinite(geometryEntry.msg.stamp) && Math.abs(geometryEntry.recv-geometryEntry.msg.stamp*1000)+now-geometryEntry.arrived<=DATA_TTL ? geometryEntry.msg : null;
      const q = entry(TOPICS[0]), rpy = entry(TOPICS[1]), joints = entry(TOPICS[2]);
      const imuReady = !!(current(q) && (!status || status.attitude_available !== false));
      const rpyReady = !!(imuReady && current(rpy) && Math.abs(q.msg.header.t - rpy.msg.header.t) <= 30);
      const jointsReady = !!(current(joints) && (!status || status.joint_state_available !== false));
      const rate = topic => {
        const times = (this.rates.get(topic) || []).filter(t => now - t <= 2000);
        return times.length > 1 && now - times[times.length - 1] <= DATA_TTL ? (times.length - 1) * 1000 / (times[times.length - 1] - times[0] || 1) : 0;
      };
      return { connected: this.connected, status, geometry, imuReady, rpyReady, jointsReady,
        q: imuReady ? q.msg.q : null, rpy: rpyReady ? rpy.msg.v : null, joints: jointsReady ? joints.msg : null,
        imuAge: imuReady ? Math.max(0, age(q)) : null, jointAge: jointsReady ? Math.max(0, age(joints)) : null,
        imuHz: rate(TOPICS[0]), jointHz: rate(TOPICS[2]), hadImu: !!q, hadJoints: !!joints };
    }
  }
  function Rotate(q, p) {
    const [x,y,z,w] = q.map(v => v / Math.hypot(...q)), [a,b,c] = p;
    return [(1-2*(y*y+z*z))*a+2*(x*y-z*w)*b+2*(x*z+y*w)*c,
      2*(x*y+z*w)*a+(1-2*(x*x+z*z))*b+2*(y*z-x*w)*c,
      2*(x*z-y*w)*a+2*(y*z+x*w)*b+(1-2*(x*x+y*y))*c];
  }
  function DrawAttitude(canvas, q, geometry, camera) {
    const ctx=canvas.getContext('2d');if(!ctx) return;
    const W=canvas.width,H=canvas.height;ctx.clearRect(0,0,W,H);if(!q) return;
    const styles=getComputedStyle(document.documentElement), color=n=>styles.getPropertyValue(n).trim();
    const frames=geometry&&Array.isArray(geometry.frames)?geometry.frames:[];
    const world=p=>Rotate(q,p), ca=Math.cos(camera.az),sa=Math.sin(camera.az),ce=Math.cos(camera.el),se=Math.sin(camera.el);
    const view=p=>{const [x,y,z]=world(p);return [ca*x-sa*y,se*(sa*x+ca*y)+ce*z,ce*(sa*x+ca*y)-se*z];};
    const points=frames.map(f=>view(f.p));points.push(view([0,0,.16]),view([0,0,-.25]));
    const span=Math.max(.35,...points.map(p=>Math.hypot(p[0],p[1])*2));
    const scale=Math.min((W-250)/span,(H-85)/span)*camera.zoom;
    const P=p=>{const v=view(p);return [W*.5+v[0]*scale,H*.43-v[1]*scale];};
    const line=(a,b,c,width=1,dash=[])=>{ctx.setLineDash(dash);ctx.beginPath();ctx.moveTo(...P(a));ctx.lineTo(...P(b));ctx.strokeStyle=c;ctx.lineWidth=width;ctx.stroke();ctx.setLineDash([]);};
    const text=(label,p,c,size=11)=>{ctx.font=`${size}px system-ui`;ctx.fillStyle=c;ctx.fillText(label,...p);};
    const origin=[0,0,0], axes=[[[.18,0,0],'--sakura','X 前'],[[0,.15,0],'--mint','Y 左'],[[0,0,.16],'--azure','Z 上']];
    const ground=geometry&&geometry.ground;
    if(ground&&ground.available) {
      const n=ground.normal,h=ground.offset_m;
      if(n&&Math.abs(n[2])>.15) {
        const G=(x,y)=>[x,y,(-h-n[0]*x-n[1]*y)/n[2]];
        for(let a=-.3;a<=.301;a+=.05){line(G(a,-.3),G(a,.3),color('--line'));line(G(-.3,a),G(.3,a),color('--line'));}
      }
    }
    for(const [axis,c,label] of axes){line(origin,axis,color(c),2);const p=P(axis);text(label,[p[0]+3,p[1]-4],color(c));}
    const halfLength=.145,halfWidth=.11; // 29 cm long x 22 cm wide; original height/origin
    const corners=[[-halfLength,-halfWidth,-.025],[halfLength,-halfWidth,-.025],[halfLength,halfWidth,-.025],[-halfLength,halfWidth,-.025],[-halfLength,-halfWidth,.055],[halfLength,-halfWidth,.055],[halfLength,halfWidth,.055],[-halfLength,halfWidth,.055]];
    for(const [a,b] of [[0,1],[1,2],[2,3],[3,0],[4,5],[5,6],[6,7],[7,4],[0,4],[1,5],[2,6],[3,7]])line(corners[a],corners[b],color('--ink-soft'),1.2);
    const map=new Map(frames.map(f=>[f.name,f])),jointFrames=frames.filter(f=>f.kind==='joint');
    const jointNames=new Map(JOINTS);
    for(const f of jointFrames){const parent=map.get(f.parent);if(parent)line(parent.p,f.p,color('--azure'),4);}
    const plus=(a,b)=>a.map((v,i)=>v+b[i]);
    for(const f of frames.filter(f=>f.kind==='wheel')) {
      const circle=[];for(let i=0;i<=32;i++){const t=i*Math.PI/16;circle.push(plus(f.p,Rotate(f.q,[f.radius*Math.cos(t),f.radius*Math.sin(t),0])));}
      for(let i=1;i<circle.length;i++)line(circle[i-1],circle[i],color('--ink'),3);
      line(f.p,circle[0],color('--sun'),2);
    }
    const sensorColors={front_tof_R:'--sakura',front_tof_L:'--azure',back_tof_R:'--sun',back_tof_L:'--mint'};
    const sensors=frames.filter(f=>f.kind==='camera'||f.kind==='tof');
    for(const f of sensors){const end=plus(f.p,Rotate(f.q,[.065,0,0]));line(f.p,end,color(f.kind==='camera'?'--sun':sensorColors[f.name]||'--mint'),2);}
    const sensorLabel={upper_camera_R:'上右 RGB',bottom_camera_L:'下左 灰度',bottom_camera_R:'下右 灰度',front_tof_R:'ToF 前右',front_tof_L:'ToF 前左',back_tof_R:'ToF 后右',back_tof_L:'ToF 后左'};
    for(const f of [...frames].sort((a,b)=>view(a.p)[2]-view(b.p)[2])) {
      if(f.kind==='wheel')continue;
      const p=P(f.p),c=color(f.kind==='imu'?'--sakura':f.kind==='camera'?'--sun':f.kind==='tof'?(sensorColors[f.name]||'--mint'):'--azure');
      ctx.fillStyle=c;ctx.beginPath();ctx.arc(...p,f.kind==='imu'?6:4,0,Math.PI*2);ctx.fill();
    }
    const labelColumn=(items,x,start,label,c)=>items.forEach((f,i)=>{const a=P(f.p),y=start+i*23;ctx.strokeStyle=color('--line');ctx.lineWidth=.8;ctx.beginPath();ctx.moveTo(...a);ctx.lineTo(x< W/2?x+85:x-6,y-4);ctx.stroke();text(label(f),[x,y],color(c));});
    labelColumn(jointFrames,12,40,f=>jointNames.get(f.joint)||f.joint,'--azure');
    labelColumn(sensors,W-112,40,f=>sensorLabel[f.name]||f.name,'--ink');
    const p=P(origin);text('IMU / base_Link',[p[0]+7,p[1]+15],color('--sakura'));
    text(frames.length?'8 关节 · 4 ToF · 3 相机':'等待关节骨架',[12,H-34],color('--ink-soft'),12);
    text('拖动旋转 · 滚轮缩放 · 黄色轮辐表示轮轴角度',[12,H-14],color('--ink-soft'));
  }
  function Mount(doc) {
    const model = new StateModel(), $ = id => doc.getElementById(id);
    const camera={az:-.65,el:.55,zoom:1};const canvas=$('robotAttitudeCanvas');let drag=null;
    canvas.addEventListener('pointerdown',e=>{drag=[e.clientX,e.clientY];canvas.setPointerCapture(e.pointerId);});
    canvas.addEventListener('pointermove',e=>{if(!drag)return;camera.az+=(e.clientX-drag[0])*.01;camera.el=Math.max(-1.2,Math.min(1.2,camera.el+(e.clientY-drag[1])*.01));drag=[e.clientX,e.clientY];});
    canvas.addEventListener('pointerup',()=>{drag=null;});canvas.addEventListener('pointercancel',()=>{drag=null;});
    canvas.addEventListener('wheel',e=>{e.preventDefault();camera.zoom=Math.max(.6,Math.min(2,camera.zoom*Math.exp(-e.deltaY*.001)));},{passive:false});
    for (const [name,label] of JOINTS) {
      const row=doc.createElement('div'); row.className='joint-row'; row.title=name;
      const text=doc.createElement('span'); text.textContent=label;
      const value=doc.createElement('b'); value.id='robotJoint-'+name; value.textContent='—';
      row.append(text,value); $('robotJointGrid').append(row);
    }
    const degree = value => `${(value*180/Math.PI).toFixed(2)}°`;
    const render = () => {
      const s=model.snapshot(performance.now()), badge=$('robotStateLink');
      const display=DisplayAttitude(s.q,s.rpy);
      badge.textContent=!s.connected?'桥接未连接':s.imuReady&&s.jointsReady?'姿态与关节在线':s.imuReady?'姿态在线 · 等待关节':s.jointsReady?'关节在线 · 等待姿态':s.hadImu||s.hadJoints?'数据已过期':'等待电控数据';
      badge.className='pill '+(s.imuReady&&s.jointsReady?'live':s.connected?'quiet':'down');
      ['robotRoll','robotPitch','robotYaw'].forEach((id,i)=>{ $(id).textContent=display.rpy?degree(display.rpy[i]):'—'; });
      $('robotQuaternion').textContent=display.q?display.q.map(v=>v.toFixed(5)).join(' / '):'— / — / — / —';
      $('robotAttitudeEmpty').hidden=s.imuReady;
      $('robotAttitudeEmpty').textContent=s.hadImu?'姿态数据已过期':'等待姿态数据';
      $('robotAttitudeMeta').textContent=s.imuReady?`IMU ${s.imuHz.toFixed(0)} Hz · 帧龄 ${s.imuAge.toFixed(0)} ms`:'IMU — · 帧龄 —';
      $('robotJointMeta').textContent=s.jointsReady?`${s.jointHz.toFixed(1)} Hz · 接收帧龄 ${s.jointAge.toFixed(0)} ms`:s.hadJoints?'关节数据已过期':'等待关节数据';
      const angles=new Map(s.joints?s.joints.names.map((name,i)=>[name,s.joints.positions[i]]):[]);
      JOINTS.forEach(([name])=>{ $('robotJoint-'+name).textContent=angles.has(name)?degree(angles.get(name)):'—'; });
      const source=s.status&&s.status.attitude_source;
      $('robotSourceMeta').textContent=s.status?`姿态来源：${source==='host_mahony'?'电控 Mahony':source==='device_quaternion'?'IMU 设备四元数':source||'等待电控'} · ${s.status.body_frame||'—'}`:'尚未收到状态适配器信息';
      const errors=s.status?[s.status.imu_error,s.status.joint_error].filter(Boolean).join('；'):'';
      $('robotStateNote').textContent=errors||(!s.imuReady&&!s.jointsReady?'等待原电控 IMU 与实测关节反馈；缺数据时不显示零姿态。':'骨架随实测关节更新；传感器使用 CAD 安装位，尚未做光学校准。');
      const ground=s.geometry&&s.geometry.ground;
      $('robotGroundMeta').textContent=ground&&ground.available?`轮底支撑平面 · IMU 离平面 ${(ground.offset_m*100).toFixed(1)} cm · 双轮接地假设`:`地面不可用：${ground&&ground.reason||'等待新鲜 IMU 与关节状态'}`;
      DrawAttitude($('robotAttitudeCanvas'),display.q,s.geometry,camera);
    };
    render(); setInterval(render,100);
    return { topics:TOPICS, receive:(topic,msg,recv)=>model.receive(topic,msg,recv,performance.now()),
      reset:()=>{ model.reset(); render(); }, setConnected:up=>{ model.connected=up; render(); } };
  }
  const api={ StateModel, Rotate, DisplayAttitude, topics:TOPICS };
  if (typeof module!=='undefined' && module.exports) module.exports=api;
  if (root && root.document) root.KaloRobotState=Mount(root.document);
})(typeof window!=='undefined'?window:null);
