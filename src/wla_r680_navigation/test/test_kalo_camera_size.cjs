// Run the real drawCamera setup with a fake WebGL viewport, no browser or ROS.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const source = fs.readFileSync(path.join(__dirname, '../kalo_web/kalo.js'), 'utf8');
const begin = source.indexOf('function drawCamera() {');
const end = source.indexOf('  gl.viewport(', begin);
assert.ok(begin >= 0 && end > begin);
const frame = {style: {}};
const elements = {
  camGl: {width: 640, height: 480, parentElement: frame},
  camOverlay: {width: 640, height: 480},
};
const state = {image: {width: 640, height: 360}};
const scope = vm.createContext({cam: {gl: {}}, state, $: id => elements[id]});
vm.runInContext(source.slice(begin, end) + '\n}', scope);
for (const [width, height] of [[640,360], [640,480], [1280,720], [640,360]]) {
  state.image = {width, height};
  vm.runInContext('drawCamera()', scope);
  assert.deepEqual([elements.camGl.width,elements.camGl.height], [width,height]);
  assert.deepEqual([elements.camOverlay.width,elements.camOverlay.height], [width,height]);
  assert.equal(frame.style.aspectRatio, `${width} / ${height}`);
}
console.log('Kalo camera resolution changes keep image, overlay and aspect ratio aligned');
