/* Course geometry is sensor ground truth and evaluation data, never an obstacle map input. */
(function () {
  'use strict';
  const points = [{x: 11.5, y: .5, s: 0}];
  function append(x, y) {
    const previous = points[points.length - 1];
    const length = Math.hypot(x - previous.x, y - previous.y);
    if (length > 1e-9) points.push({x, y, s: previous.s + length});
  }
  function line(x, y) {
    const previous = points[points.length - 1];
    const steps = Math.ceil(Math.hypot(x - previous.x, y - previous.y) / .025);
    for (let index = 1; index <= steps; index++)
      append(previous.x + (x - previous.x) * index / steps, previous.y + (y - previous.y) * index / steps);
  }
  function arc(x, y, from, to) {
    const steps = Math.ceil((to - from) * .5 / .025);
    for (let index = 1; index <= steps; index++) {
      const angle = from + (to - from) * index / steps;
      append(x + .5 * Math.cos(angle), y + .5 * Math.sin(angle));
    }
  }
  line(12, .5);
  arc(12, 1, -Math.PI / 2, 0);
  line(12.5, 3);
  arc(12, 3, 0, Math.PI / 2);
  line(1, 3.5);
  arc(1, 3, Math.PI / 2, Math.PI);
  line(.5, 1);
  arc(1, 1, Math.PI, 3 * Math.PI / 2);
  line(11.5, .5);
  const length = points[points.length - 1].s;

  function trackPoint(distance) {
    const s = ((distance % length) + length) % length;
    let low = 1, high = points.length - 1;
    while (low < high) {
      const middle = Math.floor((low + high) / 2);
      if (points[middle].s < s) low = middle + 1;
      else high = middle;
    }
    const a = points[low - 1], b = points[low];
    const t = (s - a.s) / (b.s - a.s);
    return {x: a.x + t * (b.x - a.x), y: a.y + t * (b.y - a.y),
      yaw: Math.atan2(b.y - a.y, b.x - a.x)};
  }
  function projectTrack(point) {
    let best = {distance: Infinity, s: 0};
    for (let index = 1; index < points.length; index++) {
      const a = points[index - 1], b = points[index];
      const dx = b.x - a.x, dy = b.y - a.y, span = b.s - a.s;
      const t = Math.max(0, Math.min(1, ((point.x - a.x) * dx + (point.y - a.y) * dy) / (span * span)));
      const distance = Math.hypot(point.x - a.x - t * dx, point.y - a.y - t * dy);
      if (distance < best.distance) best = {distance, s: a.s + t * span};
    }
    return best;
  }
  function trackBoundary(offset) {
    return points.map(point => {
      const heading = trackPoint(point.s).yaw;
      return {x: point.x - Math.sin(heading) * offset, y: point.y + Math.cos(heading) * offset};
    });
  }
  function trackObstacles() {
    const cones = [];
    function cone(x, y) {
      cones.push({id: 'track-cone-' + cones.length, shape: 'box', x, y, w: .13, h: .13});
    }
    // Left/right are relative to the top straight's right-to-left travel.
    // Tape centers are 68cm apart. Two 13cm bases touch, protruding 18cm
    // from the clear lane edge and leaving a 47cm clear gap.
    for (const [x, side] of [[10.75, -1], [10.25, 1], [9.75, -1], [9.25, 1]]) {
      cone(x, 3.5 + side * .34);
      cone(x, 3.5 + side * .21);
    }
    // Uniform arc-length stations give approximately 18.6cm between pairs.
    const samples = [{distance: 0, x: 0, y: 0}];
    for (let index = 1; index <= 1200; index++) {
      const x = 3 * index / 1200;
      const y = .34 * (1 - Math.cos(2 * Math.PI * x / 3));
      const previous = samples[samples.length - 1];
      samples.push({x, y, distance: previous.distance + Math.hypot(x - previous.x, y - previous.y)});
    }
    const guideLength = samples[samples.length - 1].distance;
    let sampleIndex = 1;
    for (let index = 0; index < 19; index++) {
      const station = guideLength * index / 18;
      while (sampleIndex < samples.length - 1 && samples[sampleIndex].distance < station) sampleIndex++;
      const a = samples[sampleIndex - 1], b = samples[sampleIndex];
      const t = (station - a.distance) / (b.distance - a.distance);
      const x = a.x + t * (b.x - a.x), y = .34 * (1 - Math.cos(2 * Math.PI * x / 3));
      const slope = .34 * 2 * Math.PI / 3 * Math.sin(2 * Math.PI * x / 3);
      const scale = Math.hypot(1, slope);
      for (const side of [-1, 1])
        cone(7 - x + side * .34 * slope / scale, 3.5 - y - side * .34 / scale);
    }
    return cones;
  }
  const geometry = {trackPoint, projectTrack, trackBoundary, trackObstacles, trackLength: length};
  if (typeof module !== 'undefined' && module.exports) module.exports = geometry;
  else Object.assign(window.VFHSim, geometry);
})();
