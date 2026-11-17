const assert=require('node:assert/strict');
const {valid,color}=require('../kalo_web/field-lite.js');
const blank=()=>({schema:'stereo-snapshot-field/v1',frame:'sunplus_left_rect_optical_frame',temporal_fusion:false,navigation_connected:false,available:true,compute_ms:10,occupied_voxels:1,stamp:1,shape:[60,40,60],resolution_m:.05,origin_m:[-1.5,-1,0],max_distance_m:1,surface_voxels:[[30,20,20]],slices:Array.from({length:3},()=>({y_m:0,state:Array(3600).fill(0),distance_mm:Array(3600).fill(null)}))});
assert(valid(blank()));
assert(valid({...blank(),available:false}));
for(const mutate of [j=>j.slices[0].distance_mm[0]=0,j=>j.slices[0].state[0]=1,j=>j.slices[0].distance_mm.pop(),j=>j.surface_voxels[0][0]=60,j=>j.temporal_fusion=true,j=>j.navigation_connected=true,j=>j.frame='base_Link',j=>j.compute_ms=NaN]){const j=blank();mutate(j);assert(!valid(j));}
const signed=blank();signed.slices[0].state[0]=1;signed.slices[0].distance_mm[0]=100;signed.slices[0].state[1]=2;signed.slices[0].distance_mm[1]=-50;assert(valid(signed));signed.slices[0].distance_mm[0]=-100;assert(!valid(signed));
assert.notEqual(color(0,null),color(1,1000));assert.notEqual(color(1,0),color(2,-50));
console.log('Snapshot field unknown/sign/frame/bounds UI contracts passed');
