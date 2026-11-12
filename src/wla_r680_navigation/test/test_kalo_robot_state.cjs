// node test/test_kalo_robot_state.cjs [ROS-generated fixture.json]
const assert = require('node:assert/strict');
const fs = require('node:fs');
const path = require('node:path');
const vm = require('node:vm');
const {StateModel, Rotate} = require('../kalo_web/robot-state.js');
const data = new StateModel();
data.connected = true;
const header = {t:10000,frame:'imu_start_gravity'};
const put = (topic,msg,now=100) => data.receive(topic,msg,10003,now);
put('/robot_state/attitude',{header,q:[0,0,0,1]});
put('/robot_state/rpy',{header,v:[.1,-.2,.3]});
put('/joint_states',{header,names:['hip_L_Joint'],positions:[.5]});
put('/robot_state/status',JSON.stringify({attitude_available:true,joint_state_available:true}));
let state=data.snapshot(110);
assert.equal(state.imuReady,true);assert.equal(state.rpyReady,true);assert.equal(state.jointsReady,true);
assert.equal(state.imuAge,13);assert.equal(state.rpy[1],-.2);
state=data.snapshot(700);
assert.equal(state.imuReady,false);assert.equal(state.jointsReady,false);assert.equal(state.q,null);
put('/robot_state/status',JSON.stringify({attitude_available:false,joint_state_available:false}),120);
state=data.snapshot(130);assert.equal(state.imuReady,false);assert.equal(state.jointsReady,false);
data.reset();data.connected=true;
put('/robot_state/attitude',{header,q:[0,0,0,0]});
assert.equal(data.snapshot(110).imuReady,false);
put('/joint_states',{header,names:['same','same'],positions:[1,2]});
assert.equal(data.snapshot(110).jointsReady,false);
put('/robot_state/attitude',{header,q:[0,0,0,1]});
put('/robot_state/rpy',{header:{...header,t:9900},v:[0,0,0]});
assert.equal(data.snapshot(110).rpyReady,false);
put('/robot_state/rpy',{header,v:[NaN,0,0]});
assert.equal(data.snapshot(110).rpyReady,false);
put('/robot_state/attitude',{header:{...header,t:11000},q:[0,0,0,1]});
assert.equal(data.snapshot(110).imuReady,false);
data.reset();assert.equal(data.snapshot(110).connected,false);assert.equal(data.snapshot(110).hadImu,false);
const rotated=Rotate([0,0,Math.sin(Math.PI/4),Math.cos(Math.PI/4)],[1,0,0]);
assert.ok(Math.abs(rotated[0])<1e-12 && Math.abs(rotated[1]-1)<1e-12);
let fixtureCount=0;
if(process.argv[2]) {
  const source=fs.readFileSync(path.join(__dirname,'../kalo_web/kalo.js'),'utf8');
  const prefix=source.slice(0,source.indexOf('// ---------------------------------------------------------------- TF'));
  const scope=vm.createContext({TextDecoder});
  vm.runInContext(prefix+'\nthis.CODEC={Cdr,decode};',scope);
  const fixtures=JSON.parse(fs.readFileSync(process.argv[2],'utf8'));
  for(const fixture of fixtures) {
    const bytes=Uint8Array.from(Buffer.from(fixture.cdr,'base64'));
    const result=scope.CODEC.decode[fixture.schema](new scope.CODEC.Cdr(bytes));
    if(fixture.schema.includes('QuaternionStamped')) assert.equal(result.q[3],1);
    if(fixture.schema.includes('Vector3Stamped')) assert.equal(result.v[1],-.2);
    if(fixture.schema.includes('JointState')) {
      assert.equal(result.names.length,8);assert.equal(result.names[7],'wheel_R_Joint');assert.equal(result.positions[7],.8);
      assert.equal(result.velocities.length,0);assert.equal(result.efforts.length,0);
    }
    if(fixture.schema.includes('String')) assert.equal(JSON.parse(result).attitude_available,true);
    else assert.ok(Math.abs(result.header.t-1794476000123.4568)<.001);
    fixtureCount++;
  }
}
console.log(JSON.stringify({checks:'freshness, source rejection, invalid quaternion, names, time pairing, disconnect, rotation and ROS CDR',rosFixtures:fixtureCount}));
