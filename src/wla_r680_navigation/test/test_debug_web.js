// Browser-independent state/DOM regression harness; no ROS or physical outputs.
'use strict';
const assert=require('node:assert/strict'),fs=require('node:fs'),vm=require('node:vm'),path=require('node:path');
class Element{
  constructor(tag='div'){this.tag=tag;this.children=[];this.style={};this.dataset={};this.listeners={};this.textContent='';this.hidden=false;this.resizes=0;this._width=960;this._height=600;this.classList={add(){}};}
  set width(v){this._width=v;this.resizes++;}get width(){return this._width;}
  set height(v){this._height=v;this.resizes++;}get height(){return this._height;}
  append(...els){for(const e of els)this.insertBefore(e,null);}
  insertBefore(el,before){el.remove?.();const at=before?this.children.indexOf(before):this.children.length;this.children.splice(at,0,el);el.parent=this;}
  remove(){if(this.parent){this.parent.children.splice(this.parent.children.indexOf(this),1);this.parent=null;}}
  replaceChildren(...els){for(const e of [...this.children])e.remove();this.append(...els);}
  addEventListener(k,f){this.listeners[k]=f;}
  querySelector(){return this.heading??=(new Element('h2'));}
  getContext(){return new Proxy({draws:0,clears:0,drawImage(){this.draws++;},clearRect(){this.clears++;},measureText(){return {width:10};}}, {get(o,k){return k in o?o[k]:()=>{};}});}
}
const flush=()=>new Promise(resolve=>setImmediate(resolve));
async function run(planning){
 let now=100,fetches=0,decodes=0,draws=0,fail=false,response=null,status=200;
 const elements=new Map(),timeouts=[],intervals=[],top={draw(){draws++;},clear(){draws++;},setDelay(){}};
 const get=id=>{if(!elements.has(id))elements.set(id,new Element());return elements.get(id);};
 const doc={hidden:false,getElementById:get,createElement:t=>new Element(t),createTextNode:t=>({textContent:t}),querySelector:()=>null};
 const snap={width:640,height:480,stamp_ns:'100',image:'jpeg',reason:'ok',mode:'local',topdown:{},statuses:{person:{state:'idle',reason:'',age_s:0}},layers:[{id:'points',label:'点',reason:'ok',count:3,color:'red',topic:'t',points:[]}],objects:[{label:'person',track_id:4,bbox:[1,2,3,4],confidence:.9,distance:1,yaw:0}]};
 response=snap;
 const context=vm.createContext({document:doc,location:{pathname:planning?'/planning':'/detections'},TopdownView:top,performance:{now:()=>now},AbortSignal:{timeout(){}},Image:class{async decode(){decodes++;}},fetch:async()=>{fetches++;if(fail)throw Error('network');return {ok:true,status,headers:{get:()=> 'W/"1"'},json:async()=>JSON.parse(JSON.stringify(response))};},setTimeout:f=>timeouts.push(f),setInterval:f=>intervals.push(f),console,Set,Map});
 vm.runInContext(fs.readFileSync(path.join(__dirname,'../debug_web/app.js'),'utf8'),context);
 await flush();
 const initial=get('items').children[0],resizes=get('canvas').resizes;
 assert.equal(decodes,1);if(planning)assert.equal(draws,1,'one topdown paint per response');
 now+=200;await vm.runInContext('poll()',context);await flush();
 assert.equal(get('items').children[0],initial,'keep DOM identity on routine frame update');
 assert.equal(get('canvas').resizes,resizes,'do not reset canvas dimensions every frame');
 assert.equal(decodes,1,'reuse decoded JPEG for unchanged camera frame');
 const before304=draws;status=304;now+=200;await vm.runInContext('poll()',context);
 assert.equal(draws,before304,'unchanged snapshot skips painting');status=200;
 fail=true;now+=100;await vm.runInContext('poll()',context);
 assert.equal(get('empty').hidden,true,'short transport loss retains pixels');
 assert.match(get('paused').textContent,/非实时/);assert.equal(get('paused').hidden,false);
 now+=1600;intervals[0]();assert.equal(get('empty').hidden,false,'expired historical image is cleared');
 assert.equal(vm.runInContext('data',context),null,'expiry cannot resurrect old data on a toggle');
 fail=false;now+=10;await vm.runInContext('poll()',context);await flush();
 assert.equal(get('empty').hidden,true);assert.equal(get('paused').hidden,true);
 // Explicit sensor unavailability is never held as live, despite healthy HTTP.
 response={...snap,image:null,reason:'rgb_stale'};now+=100;await vm.runInContext('poll()',context);
 assert.equal(get('empty').hidden,false);
 // Pause stops fetching; resume forces a new snapshot (no stale ETag reuse).
 get('pause').listeners.click();const n=fetches;await vm.runInContext('poll()',context);assert.equal(fetches,n);
 get('pause').listeners.click();assert.equal(vm.runInContext('etag',context),null);
 console.log('PASS',planning?'planning':'detections','stable DOM/canvas, decode reuse, 304, bounded non-live hold, recovery, invalid data, pause');
}
(async()=>{await run(true);await run(false);})().catch(e=>{console.error(e);process.exitCode=1;});
