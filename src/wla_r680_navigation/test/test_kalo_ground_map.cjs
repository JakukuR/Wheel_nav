const assert=require('node:assert/strict');
const {valid}=require('../kalo_web/ground-lite.js');
const fixture=()=>({schema:'stereo-ground-map/v1',frame:'stereo_local_ground_preview',temporal_fusion:false,navigation_connected:false,available:true,stamp:1,compute_ms:1,width:80,height:80,resolution_m:.05,classes:Array(6400).fill(0),points_mm:[[500,0,0,1],[800,0,40,2]],plane:{angle_to_support_deg:2,offset_delta_m:.01,prior_residual_m:-.02,rms_m:.005,inlier_ratio:.5}});
assert(valid(fixture()));assert(valid({...fixture(),available:false}));
for(const change of [j=>j.classes[0]=-1,j=>j.classes.pop(),j=>j.points_mm[0][0]=Infinity,j=>j.points_mm[0][3]=9,j=>j.navigation_connected=true,j=>j.frame='map',j=>j.plane.rms_m=NaN]){const j=fixture();change(j);assert(!valid(j));}
console.log('Ground map schema, frame, classes and bounds passed');
