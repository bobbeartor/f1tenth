// Run with serve.py active. Optional first argument selects the server URL.
const assert = require('node:assert/strict');
const {execFileSync} = require('node:child_process');
const {randomUUID} = require('node:crypto');
const baseUrl = process.argv[2] || 'http://127.0.0.1:8772';

global.window = {VFHSim: require('./core.js')};
global.crypto = {randomUUID};
global.XMLHttpRequest = class {
  open() {}
  send(payload) {
    this.responseText = execFileSync('curl', [
      '-fsS', '--max-time', '3', '--data-binary', payload, baseUrl + '/plan'
    ], {encoding: 'utf8'});
    this.status = 200;
  }
};
require('./native.js');
const simulator = window.VFHSim;
assert.equal(simulator.defaults.lengthSlack, .40);
assert.equal(simulator.defaults.curveSamples, 17);
assert.equal(simulator.defaults.steeringWeight, .1);
assert.equal(simulator.defaults.laneWeight, 1.5);
const trial = new simulator.Simulation({}, []);
assert.equal(trial.result.laneTrust, 'current');
assert.equal(trial.result.usedVfh, 0);
trial.c.laneVisible = false;
trial.time += .05;
trial.senseAndPlan();
assert.equal(trial.result.laneTrust, 'direction_memory');
assert.equal(trial.result.memoryPath.length, 2);
assert(trial.result.usedVfh);
assert(trial.result.speed > 0);
trial.time += 60;
trial.pose.x += .5;
trial.pose.y += .65;
trial.senseAndPlan();
assert(trial.result.hasLaneMemory);
assert(trial.result.speed > 0);
assert(Math.abs(trial.result.memoryPath[0][0]-trial.pose.x)<1e-9);
assert(Math.abs(trial.result.memoryPath[0][1]-trial.pose.y)<1e-9);
trial.pose.y = 0;
trial.c.laneVisible = true;
trial.c.laneVisibleLength = .2;
trial.time += .05;
trial.senseAndPlan();
assert.equal(trial.result.laneTrust, 'current');
trial.pose.y=0;
trial.c.laneVisibleLength=3;
trial.time+=.05;
trial.senseAndPlan();
assert.equal(trial.result.laneTrust, 'current');

const blocked = new simulator.Simulation({laneVisible: false, hazardEnabled: 0}, [
  {id: 'wall', shape: 'box', x: .22, y: 0, w: .14, h: 3}
]);
assert.equal(blocked.result.speed, 0);
console.log('PASS defaults, immediate lane acceptance, direction-only memory, short fragments, reacquisition, blocked path');

for (const scene of ['empty', 'static', 'guidance']) {
  const run = new simulator.Simulation({courseLength: 16}, simulator.presets(scene));
  let stoppedFrames = 0;
  for (let step = 0; step < 2000 && !run.done && !run.collided; step++) {
    run.step(.02);
    stoppedFrames = run.result.speed === 0 ? stoppedFrames + 1 : 0;
    if (stoppedFrames > 60) break;
  }
  console.log(JSON.stringify({
    scene, x: run.pose.x, y: run.pose.y,
    collision: run.collided, done: run.done, status: run.result.status
  }));
  assert(!run.collided, scene + ' collided');
  assert(run.done, scene + ' failed to complete the course');
  assert(Math.abs(run.pose.y) < .1, scene + ' did not return to the centerline');
}
