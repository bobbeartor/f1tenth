// node --test src/auto_control/test/tunnel/remote_preview_geometry.test.mjs
import { test } from 'node:test';
import assert from 'node:assert/strict';
import { readFileSync } from 'node:fs';

const html = readFileSync(new URL('../../auto_control_tunnel/web/remote_preview.html', import.meta.url), 'utf8');
const end = html.lastIndexOf('/*', html.indexOf('@pure-end'));
const pure = html.slice(html.indexOf('@pure-begin'), end).replace(/^[^\n]*\n/, '');
const lib = await import('data:text/javascript,' + encodeURIComponent(pure));

test('laneEdges offsets by half width along the path normal', () => {
  const { left, right } = lib.laneEdges([[0, 0], [1, 0]], 0.6);
  assert.deepEqual(left, [[0, 0.3], [1, 0.3]]);
  assert.deepEqual(right, [[0, -0.3], [1, -0.3]]);
  assert.deepEqual(lib.laneEdges([[0, 0]], 0.6), { left: [], right: [] });
});

test('display hides missing or stale values', () => {
  assert.equal(lib.display(null, 2, 0), '--');
  assert.equal(lib.display(1.234, 2, 600), '--');
  assert.equal(lib.display(1.234, 2, 100), '1.23');
});

test('tunnel darkness target and 1s approach', () => {
  assert.equal(lib.tunnelTarget('ENTERING', 0), 1);
  assert.equal(lib.tunnelTarget('EXITING', 1), 0);
  assert.equal(lib.tunnelTarget(null, 0.4), 0.4);
  assert.ok(Math.abs(lib.approach(0, 1, 0.6) - 0.8343) < 1e-3);
  assert.ok(Math.abs(lib.approach(0, 1, 1.0) - 0.95) < 1e-3);
  assert.equal(lib.approach(0.5, 0.5, 1), 0.5);
});

const straight = (y=0, n=24) => Array.from({length:n}, (_,i) => [3*i/(n-1), y]);
const lane = (points=straight(), extra={}) => ({points, valid:true, age_ms:20,
  state:3, source:'RGB', lane_width_m:0.65, ...extra});
const separation = (a,b) => Math.hypot(a[0]-b[0], a[1]-b[1]);
function assertPointsClose(actual, expected){
  assert.equal(actual.length, expected.length);
  actual.forEach((p,i) => assert.ok(separation(p, expected[i]) < 1e-10));
}

test('spatial smoothing reduces ripples, preserves endpoints and bounds corner displacement', () => {
  const raw = straight().map(([x],i) => [x, i%2 ? 0.02 : -0.02]);
  const before = structuredClone(raw), smooth = lib.smoothLaneSpatial(raw);
  assert.deepEqual(raw, before);
  assert.deepEqual(smooth[0], raw[0]);
  assert.deepEqual(smooth.at(-1), raw.at(-1));
  assert.ok(smooth.slice(1,-1).every(p => Math.abs(p[1]) <= 0.005001));
  const corner = [[0,0],[0.5,0],[0.5,0.5]];
  assert.ok(separation(lib.smoothLaneSpatial(corner)[1], corner[1]) <= 0.015001);
  assertPointsClose(lib.smoothLaneSpatial(straight()), straight());
});

test('temporal smoothing attenuates alternating frame jitter by at least 60 percent', () => {
  const filter = new lib.PreviewLaneFilter();
  let outputEnergy = 0, inputEnergy = 0;
  for (let i=0; i<150; i++){
    const y = i%2 ? 0.018 : -0.018;
    const points = filter.update(lane(straight(y)), i*1000/30);
    if (i>30){ outputEnergy += points[0][1]**2; inputEnergy += y**2; }
  }
  assert.ok(Math.sqrt(outputEnergy/inputEnergy) < 0.4);
});

test('temporal response uses elapsed observation time, independent of frame rate', () => {
  const response = dt => {
    const filter = new lib.PreviewLaneFilter();
    filter.update(lane(), 0);
    for (let t=dt; t<=200; t+=dt) filter.update(lane(straight(0.02)), t);
    return filter.points[0][1];
  };
  assert.ok(Math.abs(response(20)-response(40)) < 1e-10);
});

test('temporal lag stays within 3cm and a large path change is immediate', () => {
  const filter = new lib.PreviewLaneFilter();
  filter.update(lane(), 0);
  const shifted = straight(0.09);
  filter.update(lane(shifted), 10).forEach((p,i) => assert.ok(separation(p, shifted[i]) <= 0.030001));
  const corner = straight(0.4);
  assertPointsClose(filter.update(lane(corner), 40), corner);
});

test('duplicate observations do not apply the filter again', () => {
  const filter = new lib.PreviewLaneFilter();
  filter.update(lane(), 0);
  const shifted = lane(straight(0.02)), first = filter.update(shifted, 33);
  assert.deepEqual(filter.update(shifted, 34), first);
});

test('loss, source/support changes, long gaps and clock resets discard history', () => {
  for (const extra of [{source:'IR'}, {state:1}, {lane_width_m:0.7}]){
    const filter = new lib.PreviewLaneFilter(); filter.update(lane(), 0);
    assertPointsClose(filter.update(lane(straight(0.02), extra), 33), straight(0.02));
  }
  for (const time of [300, -1]){
    const filter = new lib.PreviewLaneFilter(); filter.update(lane(), 0);
    assertPointsClose(filter.update(lane(straight(0.02)), time), straight(0.02));
  }
  for (const invalid of [{valid:false}, {age_ms:null}, {age_ms:501}, {points:[]},
      {points:[[0,0],[1,NaN]]}]){
    const filter = new lib.PreviewLaneFilter(); filter.update(lane(), 0);
    assert.deepEqual(filter.update(lane(straight(), invalid), 33), []);
    assertPointsClose(filter.update(lane(straight(0.02)), 66), straight(0.02));
  }
  const filter = new lib.PreviewLaneFilter(); filter.update(lane(), 0); filter.clear();
  assertPointsClose(filter.update(lane(straight(0.02)), 33), straight(0.02));
});

test('browser elapsed time expires frozen geometry even if SSE stops', () => {
  assert.equal(lib.laneFresh(lane(), 480), true);
  assert.equal(lib.laneFresh(lane(), 481), false);
  assert.equal(lib.laneFresh(lane(straight(), {age_ms:null})), false);
});

test('changed point counts and a path turning backwards remain finite without inventing new segments', () => {
  const filter = new lib.PreviewLaneFilter();
  const curve = Array.from({length:24}, (_,i) => [Math.sin(i/23*Math.PI), 1-Math.cos(i/23*Math.PI)]);
  filter.update(lane(curve), 0);
  const shorter = curve.filter((_,i) => i%2 === 0);
  const points = filter.update(lane(shorter), 33), target = lib.smoothLaneSpatial(shorter);
  assert.equal(points.length, shorter.length);
  points.forEach((p,i) => {
    assert.ok(p.every(Number.isFinite));
    assert.ok(separation(p, target[i]) <= 0.030001);
  });
  const extended = straight(0,12);
  filter.clear(); filter.update(lane(extended.slice(0,6)), 0);
  assert.deepEqual(filter.update(lane(extended), 33).at(-1), extended.at(-1));
});
