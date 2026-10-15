import test from 'node:test';
import assert from 'node:assert/strict';
import {toRos,toLngLat,mapCorners,headingDegrees} from '../web/coordinates.js';
test('ROS metres round trip without scale drift',()=>{for(const p of [[0,0],[1,0],[0,1],[-58.7,92.6],[10000,-10000]]){const q=toRos(...toLngLat(...p));assert.ok(Math.hypot(p[0]-q[0],p[1]-q[1])<1e-8);}});
test('raster corners preserve origin yaw and top row orientation',()=>{const corners=mapCorners({width:20,height:10,resolution:.1,origin_x:3,origin_y:4,origin_yaw:Math.PI/2}).map(c=>toRos(...c));const expected=[[2,4],[2,6],[3,6],[3,4]];corners.forEach((p,i)=>assert.ok(Math.hypot(p[0]-expected[i][0],p[1]-expected[i][1])<1e-8));});
test('ROS east/north headings map to clockwise north marker convention',()=>{assert.equal(headingDegrees(0),90);assert.equal(headingDegrees(Math.PI/2),0);});
