const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict');
const app=fs.readFileSync('ui/app.js','utf8');const a=app.indexOf('  function buildCleaningRaster('),b=app.indexOf('\n  function ',a+1);
const c=vm.createContext({finite:Number.isFinite,document:{createElement:()=>{
 const canvas={};canvas.getContext=()=>({createImageData:(w,h)=>({data:new Uint8ClampedArray(w*h*4)}),putImageData:image=>{canvas.pixels=image.data;}});return canvas;
}}});vm.runInContext(app.slice(a,b),c);
const grid={width:2,height:2,resolution:.05,origin:{x:0,y:0,yaw:0},frame_id:'map',data_sha256:'same'};
const layer={...grid,eligible_runs:[[0,4]],covered_runs:[[0,1]],blocked_runs:[[2,1]],eligible_cells:4,covered_cells:1};
const r=c.buildCleaningRaster(layer,grid,true);
assert.deepEqual(Array.from(r.pixels.slice(0,4)),[211,62,54,200]);
assert.deepEqual(Array.from(r.pixels.slice(8,12)),[22,132,125,125]);
assert.deepEqual(Array.from(c.buildCleaningRaster(layer,grid,false).pixels.slice(0,4)),[242,190,63,130]);
assert.equal(c.buildCleaningRaster(layer,{...grid,data_sha256:'different'},true),null);
assert.equal(c.buildCleaningRaster({...layer,covered_runs:[[3,2]]},grid,true),null);
assert.equal(c.buildCleaningRaster({...layer,eligible_runs:[[0,1]],eligible_cells:1,blocked_runs:[[2,1]]},grid,true),null);
console.log('Measured coverage raster: orientation, counts, stale blocking, map mismatch, malformed runs passed');
