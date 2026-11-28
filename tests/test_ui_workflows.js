// Exercise the production renderers with a small DOM, including state resets.
const assert = require('node:assert/strict');
const fs = require('node:fs');
const vm = require('node:vm');
const app = fs.readFileSync('ui/app.js', 'utf8');
class Element {
  constructor() { this.children=[]; this.hidden=false; this.textContent=''; }
  replaceChildren() { this.children=[]; }
  append(node) { this.children.push(node); }
}
const boxes = Object.fromEntries(['memory-trace','task-sequence','cleaning-phases'].map(k=>[k,new Element()]));
const context = vm.createContext({document:{createElement:()=>new Element()},
  el:id=>boxes[id], finite:Number.isFinite, formatElapsed:n=>`${Math.floor(n)} 秒`,
  statusLabel:s=>s, state:{sequence:null}});
context.selectedTask=()=>context.state.task;
for (const name of ['renderCleaningPhases','renderSequence','renderMemoryTrace']) {
  const start=app.indexOf(`  function ${name}(`);
  const end=app.indexOf('\n  function ',start+1);
  vm.runInContext(app.slice(start,end),context);
}
const content=node=>[node.textContent,...node.children.map(content)].join('\n');
context.renderMemoryTrace({history:[{id:'abc',x:1,y:2,state:'needs_verification'}],
  live_confirmed_count:0, views:[{index:0,dwell_completed:true,valid_projections:0,
    historical_sources:[{task_id:'real-task',frames:31}]}]});
assert.match(content(boxes['memory-trace']), /real-task/);
assert.match(content(boxes['memory-trace']), /现场已确认 0 个/);
assert.match(content(boxes['memory-trace']), /未取得有效目标投影/);
assert.match(content(boxes['memory-trace']), /不表示物品已消失/);
context.renderMemoryTrace(null);
assert.equal(boxes['memory-trace'].hidden,true);
assert.equal(boxes['memory-trace'].children.length,0);
context.state.task={id:'first'};
context.state.sequence={state:'failed',tasks:[{id:'first',command:'去厨房找杯子',state:'completed'},
  {command:'去卧室找书',state:'skipped'}]};
context.renderSequence();
assert.match(content(boxes['task-sequence']),/未执行/);
context.state.task={id:'new_cleaning'};context.renderSequence();
assert.equal(boxes['task-sequence'].hidden,true);
context.renderCleaningPhases({cleaning_phases:[{phase:'local_supplement',measured_new_cells:696,elapsed_seconds:180.07}],blocked_lanes:3});
assert.match(content(boxes['cleaning-phases']),/实际新增 696 格/);
assert.match(content(boxes['cleaning-phases']),/不表示永久不可达/);
context.renderCleaningPhases(null);
assert.equal(boxes['cleaning-phases'].hidden,true);
console.log('UI renderers: history, zero observations, queue failure, supplement, reset passed');
