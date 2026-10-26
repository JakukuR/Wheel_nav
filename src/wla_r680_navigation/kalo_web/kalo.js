// Kalo 调试台: foxglove_bridge (foxglove.sdk.v1) -> CDR decode -> WebGL layers.
// The robot only forwards serialized messages; projection, pairing by stamp
// and drawing happen here. A hidden layer unsubscribes its topics.
'use strict';

const $ = id => document.getElementById(id);
const css = name => getComputedStyle(document.documentElement).getPropertyValue(name).trim();
const rgb = name => { const c = document.createElement('canvas').getContext('2d'); c.fillStyle = css(name); const h = c.fillStyle;
  return h.startsWith('#') ? [1, 3, 5].map(i => parseInt(h.slice(i, i + 2), 16) / 255) : [0.5, 0.5, 0.5]; };
const utf8 = new TextDecoder();

// ---------------------------------------------------------------- CDR ---
// Plain CDR after a 4-byte encapsulation header; alignment is relative to the payload.
class Cdr {
  constructor(bytes) { this.b = bytes; this.v = new DataView(bytes.buffer, bytes.byteOffset, bytes.byteLength); this.o = 4; this.le = bytes[1] === 1; }
  align(n) { const r = (this.o - 4) % n; if (r) this.o += n - r; }
  u8() { return this.v.getUint8(this.o++); }
  u32() { this.align(4); const x = this.v.getUint32(this.o, this.le); this.o += 4; return x; }
  i32() { this.align(4); const x = this.v.getInt32(this.o, this.le); this.o += 4; return x; }
  f32() { this.align(4); const x = this.v.getFloat32(this.o, this.le); this.o += 4; return x; }
  f64() { this.align(8); const x = this.v.getFloat64(this.o, this.le); this.o += 8; return x; }
  f64s(n) { const a = new Array(n); for (let i = 0; i < n; i++) a[i] = this.f64(); return a; }
  str() { const n = this.u32(); const s = utf8.decode(this.b.subarray(this.o, this.o + Math.max(0, n - 1))); this.o += n; return s; }
  bytes() { const n = this.u32(); const s = this.b.subarray(this.o, this.o + n); this.o += n; return s; }
  stampMs() { const sec = this.i32(), ns = this.u32(); return sec * 1e3 + ns / 1e6; }
  header() { const t = this.stampMs(); return { t, frame: this.str() }; }
  pose() { return { p: this.f64s(3), q: this.f64s(4) }; }
}

const decode = {
  'sensor_msgs/msg/CompressedImage': r => ({ header: r.header(), format: r.str(), data: r.bytes() }),
  'sensor_msgs/msg/CameraInfo': r => { const header = r.header(); const height = r.u32(), width = r.u32(); r.str();
    const nd = r.u32(); r.f64s(nd); return { header, height, width, k: r.f64s(9) }; },
  'sensor_msgs/msg/PointCloud2': r => {
    const header = r.header(); const height = r.u32(), width = r.u32(); const fields = {};
    for (let i = 0, n = r.u32(); i < n; i++) { const name = r.str(); fields[name] = { offset: r.u32(), type: r.u8(), count: r.u32() }; }
    const big = r.u8(); const step = r.u32(); r.u32(); const data = r.bytes();
    const count = height * width, xyz = new Float32Array(count * 3);
    if (!fields.x || !fields.y || !fields.z || big) return { header, xyz: new Float32Array(0) };
    const dv = new DataView(data.buffer, data.byteOffset, data.byteLength);
    for (let i = 0; i < count; i++) { const b = i * step;
      xyz[3 * i] = dv.getFloat32(b + fields.x.offset, true); xyz[3 * i + 1] = dv.getFloat32(b + fields.y.offset, true); xyz[3 * i + 2] = dv.getFloat32(b + fields.z.offset, true); }
    return { header, xyz };
  },
  'nav_msgs/msg/OccupancyGrid': r => { const header = r.header(); r.stampMs(); const resolution = r.f32(), width = r.u32(), height = r.u32();
    const origin = r.pose(); const n = r.u32(); const data = new Int8Array(r.b.slice(r.o, r.o + n).buffer); return { header, resolution, width, height, origin, data }; },
  'nav_msgs/msg/Path': r => { const header = r.header(); const poses = [];
    for (let i = 0, n = r.u32(); i < n; i++) { r.header(); poses.push(r.pose().p); } return { header, poses }; },
  'tf2_msgs/msg/TFMessage': r => { const out = [];
    for (let i = 0, n = r.u32(); i < n; i++) { const header = r.header(); const child = r.str(); out.push({ parent: header.frame, child, t: r.f64s(3), q: r.f64s(4) }); } return out; },
  'std_msgs/msg/String': r => r.str(),
};

// ---------------------------------------------------------------- TF ---
const mat = {
  fromTq([x, y, z], [qx, qy, qz, qw]) {
    return [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw), x,
            2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw), y,
            2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy), z, 0, 0, 0, 1];
  },
  mul(a, b) { const o = new Array(16).fill(0);
    for (let i = 0; i < 4; i++) for (let j = 0; j < 4; j++) for (let k = 0; k < 4; k++) o[4 * i + j] += a[4 * i + k] * b[4 * k + j]; return o; },
  inv(m) { // rigid transform
    const r = [m[0], m[4], m[8], m[1], m[5], m[9], m[2], m[6], m[10]], t = [m[3], m[7], m[11]];
    const ti = [0, 1, 2].map(i => -(r[3 * i] * t[0] + r[3 * i + 1] * t[1] + r[3 * i + 2] * t[2]));
    return [r[0], r[1], r[2], ti[0], r[3], r[4], r[5], ti[1], r[6], r[7], r[8], ti[2], 0, 0, 0, 1];
  },
  I: [1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1],
  colMajor(m) { return new Float32Array([m[0], m[4], m[8], m[12], m[1], m[5], m[9], m[13], m[2], m[6], m[10], m[14], m[3], m[7], m[11], m[15]]); },
};
const tf = {
  edges: new Map(), // child -> {parent, m}
  add(list) { for (const e of list) this.edges.set(e.child, { parent: e.parent, m: mat.fromTq(e.t, e.q) }); },
  toRoot(frame) { let m = mat.I, f = frame, guard = 0;
    while (this.edges.has(f) && guard++ < 32) { const e = this.edges.get(f); m = mat.mul(e.m, m); f = e.parent; } return { root: f, m }; },
  lookup(target, source) { // target_T_source, latest transforms
    if (target === source) return mat.I;
    const a = this.toRoot(source), b = this.toRoot(target);
    return a.root === b.root ? mat.mul(mat.inv(b.m), a.m) : null;
  },
};

// ------------------------------------------------------------- layers ---
// ttl: how far (ms) a layer's stamp may be from the camera frame it is projected on. The camera
// stream is 10 Hz, so geometry is paired within 60 ms; the top-down view always shows the newest data.
const LAYERS = [
  { id: 'camera', label: '相机画面', color: '--azure', topics: ['/r680/debug/color/compressed', '/r680/d455/color/camera_info'], on: true },
  { id: 'boxes', label: '检测框', color: '--sakura', topics: ['/r680/perception/detections', '/r680/perception/relative_objects'], ttl: 150, on: true },
  { id: 'mask', label: '目标掩膜', color: '--sakura', topics: ['/r680/perception/target_mask'], ttl: 150, on: true },
  { id: 'points', label: '深度障碍点', color: '--azure', topics: ['/r680/person_follow/depth_obstacles'], ttl: 60, on: true },
  { id: 'grid', label: '障碍栅格', color: '--sun', topics: ['/r680/person_follow/local_obstacles'], ttl: 1000, on: true },
  { id: 'path', label: '跟随路径', color: '--mint', topics: ['/r680/person_follow/local_path'], ttl: 1000, on: true },
  { id: 'navpoints', label: '导航点云', color: '--sun', topics: ['/r680_nav/d455/points'], ttl: 60, on: true },
  { id: 'navpath', label: '导航路径', color: '--sakura', topics: ['/plan', '/optimal_trajectory'], ttl: 5000, on: true },
];
const ALWAYS = ['/tf', '/tf_static', '/r680_nav/perception/status', '/r680_nav/perception/timing', '/r680/person_follow/status'];
try { const saved = JSON.parse(localStorage.getItem('kalo-layers') || '{}'); for (const l of LAYERS) if (l.id in saved) l.on = saved[l.id]; } catch (e) { /* storage blocked */ }

const store = new Map();   // topic -> [{t, recv, msg}] newest last, ~1.5 s
const rates = new Map();   // topic -> receive timestamps (ms, browser clock)
function remember(topic, t, recv, msg) {
  const list = store.get(topic) || []; list.push({ t, recv, msg });
  while (list.length > 1 && list[list.length - 1].recv - list[0].recv > 1500) list.shift();
  store.set(topic, list);
  const r = rates.get(topic) || []; r.push(performance.now()); while (r.length && performance.now() - r[0] > 2000) r.shift(); rates.set(topic, r);
}
const hz = topic => (rates.get(topic) || []).filter(t => performance.now() - t <= 2000).length / 2;
const latest = topic => { const l = store.get(topic); return l && l[l.length - 1]; };
function nearest(topic, t, ttl) {
  let best = null;
  for (const e of store.get(topic) || []) if (!best || Math.abs(e.t - t) < Math.abs(best.t - t)) best = e;
  return best && Math.abs(best.t - t) <= ttl ? best : null;
}

// -------------------------------------------------------------- bridge ---
const bridge = { ws: null, channels: new Map(), byTopic: new Map(), subs: new Map(), nextSub: 1, bytes: 0, up: false };
function wanted() {
  const set = new Set(ALWAYS);
  for (const l of LAYERS) if (l.on) l.topics.forEach(t => set.add(t));
  return set;
}
function syncSubscriptions() {
  if (!bridge.up) return;
  const want = wanted(), add = [], drop = [];
  for (const [topic, ch] of bridge.byTopic) if (want.has(topic) && ![...bridge.subs.values()].some(s => s.topic === topic)) {
    const id = bridge.nextSub++; bridge.subs.set(id, { topic, schema: ch.schemaName }); add.push({ id, channelId: ch.id }); }
  for (const [id, s] of bridge.subs) if (!want.has(s.topic) || !bridge.byTopic.has(s.topic)) { drop.push(id); bridge.subs.delete(id); store.delete(s.topic); }
  if (add.length) bridge.ws.send(JSON.stringify({ op: 'subscribe', subscriptions: add }));
  if (drop.length) bridge.ws.send(JSON.stringify({ op: 'unsubscribe', subscriptionIds: drop }));
  renderChips();
}
function connect() { // same origin: kalo_debug_web relays /ws to the local bridge
  const ws = new WebSocket(`${location.protocol === 'https:' ? 'wss' : 'ws'}://${location.host}/ws`, ['foxglove.sdk.v1']);
  ws.binaryType = 'arraybuffer'; bridge.ws = ws;
  ws.onopen = () => { bridge.up = true; setLink(true); };
  ws.onclose = () => { bridge.up = false; bridge.channels.clear(); bridge.byTopic.clear(); bridge.subs.clear(); setLink(false); setTimeout(connect, 2000); };
  ws.onmessage = ev => {
    if (typeof ev.data === 'string') {
      const m = JSON.parse(ev.data);
      if (m.op === 'advertise') { for (const ch of m.channels) { bridge.channels.set(ch.id, ch); bridge.byTopic.set(ch.topic, ch); } syncSubscriptions(); }
      else if (m.op === 'unadvertise') { for (const id of m.channelIds) { const ch = bridge.channels.get(id); if (ch) { bridge.byTopic.delete(ch.topic); bridge.channels.delete(id); } } syncSubscriptions(); }
      return;
    }
    bridge.bytes += ev.data.byteLength;
    const bytes = new Uint8Array(ev.data);
    if (bytes[0] !== 1) return;
    const dv = new DataView(ev.data); const sub = bridge.subs.get(dv.getUint32(1, true)); if (!sub) return;
    const recv = Number(dv.getBigUint64(5, true)) / 1e6;  // bridge receive time, robot clock, ms
    const fn = decode[sub.schema]; if (!fn) return;
    let msg; try { msg = fn(new Cdr(bytes.subarray(13))); } catch (e) { return; }
    onMessage(sub.topic, msg, recv);
  };
}

// ---------------------------------------------------------- messages ---
const state = { image: null, imageT: 0, imageRecv: 0, imageBitmapPending: false, info: null, dirtyCam: true, dirtyTop: true,
  timing: [], follow: null };
function onMessage(topic, msg, recv) {
  if (topic === '/tf' || topic === '/tf_static') { tf.add(msg); state.dirtyCam = state.dirtyTop = true; return; }
  if (topic === '/r680/d455/color/camera_info') { state.info = msg; return; }
  if (topic === '/r680/debug/color/compressed') {
    remember(topic, msg.header.t, recv, null);
    if (state.imageBitmapPending) return;  // decode at most one frame at a time
    state.imageBitmapPending = true;
    createImageBitmap(new Blob([msg.data], { type: 'image/jpeg' })).then(bmp => {
      if (state.image) state.image.close(); state.image = bmp; state.imageT = msg.header.t; state.imageRecv = recv; state.imageFrame = msg.header.frame;
      state.imageBitmapPending = false; state.dirtyCam = true; }, () => { state.imageBitmapPending = false; });
    return;
  }
  if (topic === '/r680_nav/perception/timing') { try { const j = JSON.parse(msg); if (j.outcome === 'published') { state.timing.push((j.projection_end_ns - j.stamp_ns) / 1e6); if (state.timing.length > 40) state.timing.shift(); } } catch (e) {} return; }
  if (topic === '/r680/person_follow/status') { try { state.follow = JSON.parse(msg); } catch (e) {} return; }
  if (typeof msg === 'string') { // detections, relative objects, target mask, detector status
    let j; try { j = JSON.parse(msg); } catch (e) { return; }
    const t = typeof j.stamp_ns === 'number' ? j.stamp_ns / 1e6 : recv;
    if (topic === '/r680/perception/target_mask' && j.bits) decodeMask(j).then(m => { j.decoded = m; state.dirtyCam = true; });
    remember(topic, t, recv, j); state.dirtyCam = state.dirtyTop = true; return;
  }
  remember(topic, msg.header.t, recv, msg); state.dirtyCam = state.dirtyTop = true;
}
async function decodeMask(j) { // base64(zlib(packbits(mask)))
  const raw = Uint8Array.from(atob(j.bits), c => c.charCodeAt(0));
  const stream = new Blob([raw]).stream().pipeThrough(new DecompressionStream('deflate'));
  const bits = new Uint8Array(await new Response(stream).arrayBuffer());
  const img = new ImageData(j.width, j.height), color = rgb('--sakura');
  for (let i = 0; i < j.width * j.height; i++) if (bits[i >> 3] & (128 >> (i & 7))) {
    img.data[4 * i] = color[0] * 255; img.data[4 * i + 1] = color[1] * 255; img.data[4 * i + 2] = color[2] * 255; img.data[4 * i + 3] = 90; }
  return createImageBitmap(img);
}

// ------------------------------------------------------------- WebGL ---
function program(gl, vs, fs) {
  const p = gl.createProgram();
  for (const [type, src] of [[gl.VERTEX_SHADER, vs], [gl.FRAGMENT_SHADER, fs]]) {
    const s = gl.createShader(type); gl.shaderSource(s, '#version 300 es\nprecision highp float;\n' + src); gl.compileShader(s);
    if (!gl.getShaderParameter(s, gl.COMPILE_STATUS)) throw new Error(gl.getShaderInfoLog(s)); gl.attachShader(p, s); }
  gl.linkProgram(p); if (!gl.getProgramParameter(p, gl.LINK_STATUS)) throw new Error(gl.getProgramInfoLog(p));
  const u = {}; for (let i = 0; i < gl.getProgramParameter(p, gl.ACTIVE_UNIFORMS); i++) { const n = gl.getActiveUniform(p, i).name; u[n] = gl.getUniformLocation(p, n); }
  return { p, u };
}
function upload(gl, buf, data) { gl.bindBuffer(gl.ARRAY_BUFFER, buf); gl.bufferData(gl.ARRAY_BUFFER, data, gl.STREAM_DRAW); }
function attrib(gl, buf, size) { gl.bindBuffer(gl.ARRAY_BUFFER, buf); gl.enableVertexAttribArray(0); gl.vertexAttribPointer(0, size, gl.FLOAT, false, 0, 0); }

// Camera: image quad + points projected with the pinhole model in the shader.
const cam = (() => {
  const gl = $('camGl').getContext('webgl2', { premultipliedAlpha: false }); if (!gl) return null;
  const quad = program(gl, 'layout(location=0) in vec2 a; out vec2 uv; void main(){ uv=vec2(a.x*.5+.5,.5-a.y*.5); gl_Position=vec4(a,0,1); }',
    'uniform sampler2D img; in vec2 uv; out vec4 o; void main(){ o=texture(img,uv); }');
  const pts = program(gl, `layout(location=0) in vec3 a; uniform mat4 T; uniform vec4 K; uniform vec2 S; out float d;
    void main(){ vec4 p=T*vec4(a,1.); d=p.z; if(p.z<.1){ gl_Position=vec4(2.,2.,2.,1.); return; }
      vec2 px=vec2(K.x*p.x/p.z+K.z, K.y*p.y/p.z+K.w); gl_Position=vec4(px.x/S.x*2.-1., 1.-px.y/S.y*2., 0., 1.); gl_PointSize=clamp(7./p.z,2.,6.); }`,
    'uniform vec3 near; uniform vec3 far; in float d; out vec4 o; void main(){ o=vec4(mix(near,far,clamp((d-.5)/4.,0.,1.)),.85); }');
  const qbuf = gl.createBuffer(); upload(gl, qbuf, new Float32Array([-1, -1, 1, -1, -1, 1, 1, 1]));
  const pbuf = gl.createBuffer(), tex = gl.createTexture();
  return { gl, quad, pts, qbuf, pbuf, tex };
})();

// Top-down: robot frame, x forward is up, y left is left.
const topView = (() => {
  const gl = $('topGl').getContext('webgl2', { premultipliedAlpha: false }); if (!gl) return null;
  const geo = program(gl, 'layout(location=0) in vec3 a; uniform mat4 T; uniform float s; uniform vec2 S; void main(){ vec4 p=T*vec4(a,1.); vec2 px=vec2(S.x*.5-p.y*s, S.y-28.-p.x*s); gl_Position=vec4(px.x/S.x*2.-1.,1.-px.y/S.y*2.,0.,1.); gl_PointSize=3.; }',
    'uniform vec4 c; out vec4 o; void main(){ o=c; }');
  const grid = program(gl, 'layout(location=0) in vec2 a; uniform mat4 T; uniform vec2 G; uniform float s; uniform vec2 S; out vec2 uv; void main(){ uv=a; vec4 p=T*vec4(a*G,0.,1.); vec2 px=vec2(S.x*.5-p.y*s, S.y-28.-p.x*s); gl_Position=vec4(px.x/S.x*2.-1.,1.-px.y/S.y*2.,0.,1.); }',
    'uniform sampler2D g; uniform vec3 free; uniform vec3 occ; in vec2 uv; out vec4 o; void main(){ float v=texture(g,uv).r; if(v<.5/255.) discard; o = v>.6 ? vec4(occ,.75) : vec4(free,.16); }');
  return { gl, geo, grid, buf: gl.createBuffer(), gbuf: gl.createBuffer(), gtex: gl.createTexture() };
})();

function drawCamera() {
  if (!cam || !state.image) return;
  const { gl } = cam, W = state.image.width, H = state.image.height, canvas = $('camGl');
  if (canvas.width !== W) { canvas.width = W; canvas.height = H; $('camOverlay').width = W; $('camOverlay').height = H; }
  gl.viewport(0, 0, W, H); gl.clearColor(0, 0, 0, 0); gl.clear(gl.COLOR_BUFFER_BIT);
  gl.useProgram(cam.quad.p); gl.bindTexture(gl.TEXTURE_2D, cam.tex);
  gl.texImage2D(gl.TEXTURE_2D, 0, gl.RGBA, gl.RGBA, gl.UNSIGNED_BYTE, state.image);
  gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.LINEAR); gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_S, gl.CLAMP_TO_EDGE); gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_WRAP_T, gl.CLAMP_TO_EDGE);
  attrib(gl, cam.qbuf, 2); gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4);
  const info = state.info, layerNotes = {};
  gl.enable(gl.BLEND); gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
  for (const [layer, topic] of [['points', '/r680/person_follow/depth_obstacles'], ['navpoints', '/r680_nav/d455/points']]) {
    const L = LAYERS.find(l => l.id === layer); if (!L.on || !info) continue;
    const e = nearest(topic, state.imageT, L.ttl); if (!e) { layerNotes[layer] = store.get(topic) ? '时间不匹配' : ''; continue; }
    const T = tf.lookup(state.imageFrame, e.msg.header.frame); if (!T) { layerNotes[layer] = '缺少 TF'; continue; }
    layerNotes[layer] = `${(e.t - state.imageT).toFixed(0)} ms`;
    gl.useProgram(cam.pts.p); upload(gl, cam.pbuf, e.msg.xyz); attrib(gl, cam.pbuf, 3);
    gl.uniformMatrix4fv(cam.pts.u.T, false, mat.colMajor(T)); gl.uniform4f(cam.pts.u.K, info.k[0], info.k[4], info.k[2], info.k[5]);
    gl.uniform2f(cam.pts.u.S, W, H); gl.uniform3fv(cam.pts.u.near, rgb('--sakura')); gl.uniform3fv(cam.pts.u.far, rgb('--azure'));
    gl.drawArrays(gl.POINTS, 0, e.msg.xyz.length / 3);
  }
  // 2D overlay: target mask and detection boxes on the frame they were computed for.
  const ctx = $('camOverlay').getContext('2d'); ctx.clearRect(0, 0, W, H);
  const mask = LAYERS.find(l => l.id === 'mask').on && nearest('/r680/perception/target_mask', state.imageT, 150);
  if (mask && mask.msg.decoded) ctx.drawImage(mask.msg.decoded, 0, 0, W, H);
  if (LAYERS.find(l => l.id === 'boxes').on) {
    const det = nearest('/r680/perception/detections', state.imageT, 150);
    const rel = det && nearest('/r680/perception/relative_objects', det.t, 1);
    const dist = new Map((rel ? rel.msg.objects : []).map(o => [o.id, o.distance]));
    if (det) {
      layerNotes.boxes = `${(det.t - state.imageT).toFixed(0)} ms`;
      const sx = W / det.msg.width, sy = H / det.msg.height; ctx.lineWidth = 3; ctx.font = '700 15px system-ui, sans-serif';
      for (const o of det.msg.objects || []) {
        const [x1, y1, x2, y2] = o.bbox_xyxy.map((v, i) => v * (i % 2 ? sy : sx)); const color = css(o.label === 'person' ? '--sakura' : '--sun');
        const d = dist.get(`${det.msg.session}:${det.msg.epoch}:${o.track_id}`);
        const text = `${o.label} #${o.track_id}${typeof d === 'number' ? ' · ' + d.toFixed(2) + ' m' : ''}`;
        ctx.strokeStyle = color; ctx.strokeRect(x1, y1, x2 - x1, y2 - y1);
        const w = ctx.measureText(text).width + 14, bx = Math.max(0, Math.min(x1, W - w)), by = Math.max(0, y1 - 24);
        ctx.fillStyle = color; ctx.beginPath(); ctx.roundRect(bx, by, w, 22, 8); ctx.fill(); ctx.fillStyle = '#22304e'; ctx.fillText(text, bx + 7, by + 16);
      }
    } else layerNotes.boxes = store.get('/r680/perception/detections') ? '时间不匹配' : '';
  }
  state.notes = Object.assign(state.notes || {}, layerNotes);
}

function drawTop() {
  if (!topView) return;
  const { gl } = topView, canvas = $('topGl'), dpr = Math.min(2, window.devicePixelRatio || 1);
  const size = Math.round(canvas.clientWidth * dpr) || 600; if (canvas.width !== size) { canvas.width = canvas.height = size; $('topOverlay').width = $('topOverlay').height = size; }
  const S = [size, size], range = Number($('range').value), s = (size - 40) / range;
  gl.viewport(0, 0, size, size); gl.clearColor(0, 0, 0, 0); gl.clear(gl.COLOR_BUFFER_BIT); gl.enable(gl.BLEND); gl.blendFunc(gl.SRC_ALPHA, gl.ONE_MINUS_SRC_ALPHA);
  const base = (window.KALO && KALO.base_frame) || 'r680_mapping_floor', on = id => LAYERS.find(l => l.id === id).on;
  const pick = topic => latest(topic);
  // metre rings
  gl.useProgram(topView.geo.p); gl.uniform1f(topView.geo.u.s, s); gl.uniform2fv(topView.geo.u.S, S); gl.uniformMatrix4fv(topView.geo.u.T, false, mat.colMajor(mat.I));
  const ring = []; for (let r = 1; r <= range; r++) for (let a = 0; a <= 64; a++) { const t = a / 64 * Math.PI * 2; ring.push(r * Math.cos(t), r * Math.sin(t), 0); }
  upload(gl, topView.buf, new Float32Array(ring)); attrib(gl, topView.buf, 3); gl.uniform4f(topView.geo.u.c, ...rgb('--ink-soft'), .18);
  for (let r = 0; r < range; r++) gl.drawArrays(gl.LINE_STRIP, r * 65, 65);
  if (on('grid')) { const e = pick('/r680/person_follow/local_obstacles'); const T = e && tf.lookup(base, e.msg.header.frame);
    if (e && T) { const g = e.msg, cells = new Uint8Array(g.width * g.height);
      for (let i = 0; i < cells.length; i++) cells[i] = g.data[i] < 0 ? 0 : 1 + Math.round(g.data[i] / 100 * 254);
      gl.useProgram(topView.grid.p); gl.bindTexture(gl.TEXTURE_2D, topView.gtex); gl.pixelStorei(gl.UNPACK_ALIGNMENT, 1);
      gl.texImage2D(gl.TEXTURE_2D, 0, gl.R8, g.width, g.height, 0, gl.RED, gl.UNSIGNED_BYTE, cells);
      gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MIN_FILTER, gl.NEAREST); gl.texParameteri(gl.TEXTURE_2D, gl.TEXTURE_MAG_FILTER, gl.NEAREST);
      const Tg = mat.mul(T, mat.fromTq(g.origin.p, g.origin.q));
      upload(gl, topView.gbuf, new Float32Array([0, 0, 1, 0, 0, 1, 1, 1])); gl.bindBuffer(gl.ARRAY_BUFFER, topView.gbuf); gl.enableVertexAttribArray(0); gl.vertexAttribPointer(0, 2, gl.FLOAT, false, 0, 0);
      gl.uniformMatrix4fv(topView.grid.u.T, false, mat.colMajor(Tg)); gl.uniform2f(topView.grid.u.G, g.width * g.resolution, g.height * g.resolution);
      gl.uniform1f(topView.grid.u.s, s); gl.uniform2fv(topView.grid.u.S, S); gl.uniform3fv(topView.grid.u.free, rgb('--mint')); gl.uniform3fv(topView.grid.u.occ, rgb('--sun'));
      gl.drawArrays(gl.TRIANGLE_STRIP, 0, 4); } }
  gl.useProgram(topView.geo.p); gl.uniform1f(topView.geo.u.s, s); gl.uniform2fv(topView.geo.u.S, S);
  for (const [layer, topic, color] of [['points', '/r680/person_follow/depth_obstacles', '--azure'], ['navpoints', '/r680_nav/d455/points', '--sun']]) {
    if (!on(layer)) continue; const e = pick(topic); const T = e && tf.lookup(base, e.msg.header.frame); if (!T) continue;
    upload(gl, topView.buf, e.msg.xyz); attrib(gl, topView.buf, 3); gl.uniformMatrix4fv(topView.geo.u.T, false, mat.colMajor(T)); gl.uniform4f(topView.geo.u.c, ...rgb(color), .85);
    gl.drawArrays(gl.POINTS, 0, e.msg.xyz.length / 3);
  }
  for (const [layer, topic, color] of [['path', '/r680/person_follow/local_path', '--mint'], ['navpath', '/plan', '--sakura'], ['navpath', '/optimal_trajectory', '--azure']]) {
    if (!on(layer)) continue; const e = latest(topic); const T = e && tf.lookup(base, e.msg.header.frame); if (!T || !e.msg.poses.length) continue;
    upload(gl, topView.buf, new Float32Array(e.msg.poses.flat())); attrib(gl, topView.buf, 3); gl.uniformMatrix4fv(topView.geo.u.T, false, mat.colMajor(T)); gl.uniform4f(topView.geo.u.c, ...rgb(color), 1);
    gl.lineWidth(2); gl.drawArrays(gl.LINE_STRIP, 0, e.msg.poses.length);
  }
  // overlay: robot and detected objects
  const ctx = $('topOverlay').getContext('2d'); ctx.clearRect(0, 0, size, size);
  const P = (x, y) => [size / 2 - y * s, size - 28 - x * s];
  ctx.font = `700 ${12 * dpr}px system-ui, sans-serif`; ctx.fillStyle = css('--ink-soft');
  for (let r = 1; r <= range; r++) { const [px, py] = P(r, 0); ctx.fillText(`${r} m`, px + 4 * dpr, py + 12 * dpr); }
  if (on('boxes')) { const e = latest('/r680/perception/relative_objects');
    for (const o of (e && e.msg.objects) || []) { if (typeof o.distance !== 'number' || typeof o.yaw !== 'number') continue;
      const [px, py] = P(o.distance * Math.cos(o.yaw), o.distance * Math.sin(o.yaw)); ctx.fillStyle = css(o.label === 'person' ? '--sakura' : '--sun');
      ctx.beginPath(); ctx.arc(px, py, 7 * dpr, 0, Math.PI * 2); ctx.fill(); ctx.fillStyle = css('--ink'); ctx.fillText(`${o.label} ${o.distance.toFixed(1)} m`, px + 10 * dpr, py + 4 * dpr); } }
  const [rx, ry] = P(0, 0); ctx.fillStyle = css('--azure'); ctx.strokeStyle = css('--ink'); ctx.lineWidth = 2 * dpr;
  ctx.beginPath(); ctx.roundRect(rx - 14 * dpr, ry - 16 * dpr, 28 * dpr, 32 * dpr, 9 * dpr); ctx.fill(); ctx.stroke();
  ctx.fillStyle = css('--sun'); ctx.beginPath(); ctx.moveTo(rx, ry - 28 * dpr); ctx.lineTo(rx - 7 * dpr, ry - 18 * dpr); ctx.lineTo(rx + 7 * dpr, ry - 18 * dpr); ctx.fill();
}

// ----------------------------------------------------------------- UI ---
function setLink(up) { const el = $('link'); el.textContent = up ? '已连接 · 推送' : '已断开，正在重连'; el.className = 'pill ' + (up ? 'live' : 'down'); }
function renderChips() {
  const box = $('layers'); if (!box.children.length) for (const l of LAYERS) {
    const b = document.createElement('button'); b.type = 'button'; b.className = 'layer'; b.id = 'layer-' + l.id; b.style.setProperty('--c', css(l.color));
    b.innerHTML = '<i></i><span></span><small></small>'; b.querySelector('span').textContent = l.label;
    b.addEventListener('click', () => { l.on = !l.on; try { localStorage.setItem('kalo-layers', JSON.stringify(Object.fromEntries(LAYERS.map(x => [x.id, x.on])))); } catch (e) {}
      syncSubscriptions(); state.dirtyCam = state.dirtyTop = true; }); box.appendChild(b); }
  for (const l of LAYERS) { const b = $('layer-' + l.id); b.setAttribute('aria-pressed', String(l.on));
    const present = l.topics.some(t => bridge.byTopic.has(t)); b.classList.toggle('absent', !present);
    const rate = Math.max(...l.topics.map(hz)); const note = (state.notes || {})[l.id];
    b.querySelector('small').textContent = !present ? '未发布' : !l.on ? '已退订' : `${rate.toFixed(rate < 10 ? 1 : 0)} Hz${note ? ' · ' + note : ''}`; }
}
function renderStats() {
  const det = hz('/r680/perception/relative_objects'); $('sDet').textContent = det ? `${det.toFixed(1)} Hz` : '—';
  const t = [...state.timing].sort((a, b) => a - b); $('sLat').textContent = t.length ? `${t[t.length >> 1].toFixed(0)} ms` : '—';
  const f = state.follow; $('sFollow').textContent = f ? ({ idle: '空闲', tracking: '跟随中', searching: '搜索中' }[f.state] || f.state || '—') : '—';
  $('sFollowWhy').textContent = f && f.reason ? `跟随 · ${f.reason}` : '跟随状态';
  const fps = hz('/r680/debug/color/compressed'); const age = state.imageRecv && state.imageT ? state.imageRecv - state.imageT : null;
  $('sCam').textContent = fps ? `${fps.toFixed(0)} fps · ${age === null ? '—' : age.toFixed(0) + ' ms'}` : '—';
  $('camEmpty').hidden = !!state.image && LAYERS[0].on;
  $('camMeta').textContent = state.image ? `${state.image.width}×${state.image.height} · NVJPG` : '';
}

let wireMark = performance.now();
setInterval(() => { const now = performance.now(); $('wire').textContent = `${(bridge.bytes / ((now - wireMark) / 1000) / 1e6).toFixed(2)} MB/s`;
  bridge.bytes = 0; wireMark = now; renderChips(); renderStats(); state.dirtyTop = true; }, 1000);
$('range').addEventListener('change', () => { state.dirtyTop = true; });
window.matchMedia('(prefers-color-scheme: dark)').addEventListener('change', () => { state.dirtyCam = state.dirtyTop = true; });
(function frame() {
  if (state.dirtyCam) { state.dirtyCam = false; if (LAYERS[0].on) drawCamera(); }
  if (state.dirtyTop) { state.dirtyTop = false; drawTop(); }
  requestAnimationFrame(frame);
})();

fetch('config.json', { cache: 'no-store' }).then(r => r.json()).catch(() => ({})).then(cfg => {
  window.KALO = Object.assign({ bridge_port: 8765, base_frame: 'r680_mapping_floor', domain: '?' }, cfg);
  $('domain').textContent = `ROS 域 ${KALO.domain}`; renderChips(); connect();
});
