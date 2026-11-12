/* Read-only telemetry view. No ROS publications, filtering, position or TF estimates. */
(function (root) {
  'use strict';
  const TOPICS = ['/robot_state/attitude', '/robot_state/rpy', '/joint_states', '/robot_state/status'];
  const JOINTS = [['abad_L_Joint', '左外展'], ['abad_R_Joint', '右外展'], ['hip_L_Joint', '左髋'], ['hip_R_Joint', '右髋'],
    ['knee_L_Joint', '左膝'], ['knee_R_Joint', '右膝'], ['wheel_L_Joint', '左轮'], ['wheel_R_Joint', '右轮']];
  const DATA_TTL = 500, STATUS_TTL = 1500;
  const finite = a => Array.isArray(a) && a.every(Number.isFinite);
  class StateModel {
    constructor() { this.connected = false; this.entries = new Map(); this.rates = new Map(); }
    reset() { this.entries.clear(); this.rates.clear(); this.connected = false; }
    receive(topic, msg, recv, now) {
      if (!TOPICS.includes(topic)) return;
      let valid = false;
      if (topic === TOPICS[3]) {
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
      const q = entry(TOPICS[0]), rpy = entry(TOPICS[1]), joints = entry(TOPICS[2]);
      const imuReady = !!(current(q) && (!status || status.attitude_available !== false));
      const rpyReady = !!(imuReady && current(rpy) && Math.abs(q.msg.header.t - rpy.msg.header.t) <= 30);
      const jointsReady = !!(current(joints) && (!status || status.joint_state_available !== false));
      const rate = topic => {
        const times = (this.rates.get(topic) || []).filter(t => now - t <= 2000);
        return times.length > 1 && now - times[times.length - 1] <= DATA_TTL ? (times.length - 1) * 1000 / (times[times.length - 1] - times[0] || 1) : 0;
      };
      return { connected: this.connected, status, imuReady, rpyReady, jointsReady,
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
  function DrawAttitude(canvas, q) {
    const ctx = canvas.getContext('2d'); if (!ctx) return;
    const W = canvas.width, H = canvas.height;
    ctx.clearRect(0,0,W,H); if (!q) return;
    const colors = getComputedStyle(document.documentElement);
    const color = name => colors.getPropertyValue(name).trim();
    const view = ([x,y,z]) => [.7071*x-.7071*y, -.36*x-.36*y+.86*z, .61*x+.61*y+.51*z];
    const P = p => { const v = view(p); return [W/2+v[0]*330, H*.57-v[1]*330]; };
    const line = (a,b,c,width=1) => { const A=P(a), B=P(b); ctx.strokeStyle=c; ctx.lineWidth=width; ctx.beginPath(); ctx.moveTo(...A); ctx.lineTo(...B); ctx.stroke(); };
    const origin = [0,0,0];
    for (const [axis, c, label] of [[[.35,0,0], '--sakura','X'],[[0,.35,0], '--mint','Y'],[[0,0,.26],'--azure','Z']]) {
      line(origin,axis,color(c),1.5); const p=P(axis); ctx.fillStyle=color(c); ctx.font='bold 12px system-ui'; ctx.fillText(label,p[0]+3,p[1]-4);
    }
    const vertices = [[-.18,-.10,-.045],[.18,-.10,-.045],[.18,.10,-.045],[-.18,.10,-.045],
      [-.18,-.10,.045],[.18,-.10,.045],[.18,.10,.045],[-.18,.10,.045]].map(p=>Rotate(q,p));
    const faces = [[0,1,2,3],[4,7,6,5],[0,4,5,1],[1,5,6,2],[2,6,7,3],[3,7,4,0]];
    faces.sort((a,b)=>a.reduce((s,i)=>s+view(vertices[i])[2],0)-b.reduce((s,i)=>s+view(vertices[i])[2],0));
    for (const face of faces) {
      const points=face.map(i=>P(vertices[i])); ctx.beginPath(); ctx.moveTo(...points[0]); points.slice(1).forEach(p=>ctx.lineTo(...p)); ctx.closePath();
      ctx.globalAlpha=.85; ctx.fillStyle=color('--sky-deep'); ctx.fill(); ctx.globalAlpha=1; ctx.strokeStyle=color('--azure'); ctx.lineWidth=1.6; ctx.stroke();
    }
    line(Rotate(q,[0,0,.055]),Rotate(q,[.24,0,.055]),color('--sun'),4);
    ctx.fillStyle=color('--ink-soft'); ctx.font='12px system-ui'; ctx.fillText('黄色方向为机身前方',12,H-12);
  }
  function Mount(doc) {
    const model = new StateModel(), $ = id => doc.getElementById(id);
    for (const [name,label] of JOINTS) {
      const row=doc.createElement('div'); row.className='joint-row'; row.title=name;
      const text=doc.createElement('span'); text.textContent=label;
      const value=doc.createElement('b'); value.id='robotJoint-'+name; value.textContent='—';
      row.append(text,value); $('robotJointGrid').append(row);
    }
    const degree = value => `${(value*180/Math.PI).toFixed(2)}°`;
    const render = () => {
      const s=model.snapshot(performance.now()), badge=$('robotStateLink');
      badge.textContent=!s.connected?'桥接未连接':s.imuReady&&s.jointsReady?'姿态与关节在线':s.imuReady?'姿态在线 · 等待关节':s.jointsReady?'关节在线 · 等待姿态':s.hadImu||s.hadJoints?'数据已过期':'等待电控数据';
      badge.className='pill '+(s.imuReady&&s.jointsReady?'live':s.connected?'quiet':'down');
      ['robotRoll','robotPitch','robotYaw'].forEach((id,i)=>{ $(id).textContent=s.rpy?degree(s.rpy[i]):'—'; });
      $('robotQuaternion').textContent=s.q?s.q.map(v=>v.toFixed(5)).join(' / '):'— / — / — / —';
      $('robotAttitudeEmpty').hidden=s.imuReady;
      $('robotAttitudeEmpty').textContent=s.hadImu?'姿态数据已过期':'等待姿态数据';
      $('robotAttitudeMeta').textContent=s.imuReady?`IMU ${s.imuHz.toFixed(0)} Hz · 帧龄 ${s.imuAge.toFixed(0)} ms`:'IMU — · 帧龄 —';
      $('robotJointMeta').textContent=s.jointsReady?`${s.jointHz.toFixed(1)} Hz · 接收帧龄 ${s.jointAge.toFixed(0)} ms`:s.hadJoints?'关节数据已过期':'等待关节数据';
      const angles=new Map(s.joints?s.joints.names.map((name,i)=>[name,s.joints.positions[i]]):[]);
      JOINTS.forEach(([name])=>{ $('robotJoint-'+name).textContent=angles.has(name)?degree(angles.get(name)):'—'; });
      const source=s.status&&s.status.attitude_source;
      $('robotSourceMeta').textContent=s.status?`姿态来源：${source==='host_mahony'?'电控 Mahony':source==='device_quaternion'?'IMU 设备四元数':source||'等待电控'} · ${s.status.body_frame||'—'}`:'尚未收到状态适配器信息';
      const errors=s.status?[s.status.imu_error,s.status.joint_error].filter(Boolean).join('；'):'';
      $('robotStateNote').textContent=errors||(!s.imuReady&&!s.jointsReady?'等待原电控 IMU 与实测关节反馈；缺数据时不显示零姿态。':'关节帧龄按状态接收时间计算；旋转示意不代表定位或离地高度。');
      DrawAttitude($('robotAttitudeCanvas'),s.q);
    };
    render(); setInterval(render,100);
    return { topics:TOPICS, receive:(topic,msg,recv)=>model.receive(topic,msg,recv,performance.now()),
      reset:()=>{ model.reset(); render(); }, setConnected:up=>{ model.connected=up; render(); } };
  }
  const api={ StateModel, Rotate, topics:TOPICS };
  if (typeof module!=='undefined' && module.exports) module.exports=api;
  if (root && root.document) root.KaloRobotState=Mount(root.document);
})(typeof window!=='undefined'?window:null);
