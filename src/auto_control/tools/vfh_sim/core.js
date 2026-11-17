/* Standalone reference model. No ROS node or full-world map is accessible to Planner. */
(function (scope) {
  'use strict';
  const DEG = Math.PI / 180;
  const clamp = (x, a, b) => Math.max(a, Math.min(b, x));
  const wrap = a => Math.atan2(Math.sin(a), Math.cos(a));
  const defaults = {
    trackLayout: 'straight', courseLength: 16, laneWidth: .65, departure: .70,
    bevWidth: 120, bevHeight: 300, resolution: .01,
    width: .30, length: .50, wheelbase: .33, frontOverhang: .08,
    maxSteer: 30, steerRate: 240, speed: 1, acceleration: .7, deceleration: 1.2,
    responseDelay: .15, margin: .04, range: 2, angularStep: 2, planHz: 20,
    horizon: 1.5, sampleStep: .04, memoryTime: 2, memoryDistance: .5,
    targetWeight: 5, previousWeight: .6, clearanceWeight: .14,
    lowThreshold: .25, highThreshold: .5, occlusion: true, laneVisible: true,
    entryReferenceWeight: 8, approachSpeed: .5, corridorLookahead: .65
  };
  function validate(input) {
    const c = {...defaults, ...input};
    for (const key of Object.keys(defaults)) {
      if (typeof defaults[key] === 'number' && !Number.isFinite(c[key])) throw Error(key + ': 유한한 숫자가 필요합니다.');
    }
    if (!['straight','loop'].includes(c.trackLayout)) throw Error('알 수 없는 트랙 형상입니다.');
    if(c.rearExtent!==undefined && (!Number.isFinite(c.rearExtent) || c.rearExtent<c.wheelbase ||
      Math.abs(c.rearExtent+c.frontOverhang-c.length)>1e-6))
      throw Error('앞차축 기준 앞뒤 길이의 합은 전장과 같아야 하고 뒤차축이 차체 안에 있어야 합니다.');
    if (c.width < .1 || c.width > .8 || c.length < .2 || c.length > 1.5 ||
      c.wheelbase < .1 || c.wheelbase > c.length || c.frontOverhang < 0 ||
      c.frontOverhang + c.wheelbase > c.length || c.maxSteer < 5 || c.maxSteer > 45 ||
      c.steerRate <= 0 || c.speed <= 0 || c.speed > 3 || c.acceleration <= 0 || c.deceleration <= 0 ||
      c.range < .4 || c.range > 3 || c.angularStep < .5 || c.angularStep > 10 ||
      c.planHz < 5 || c.planHz > 100 || c.horizon < .3 || c.horizon > 3 ||
      c.margin < 0 || c.margin > .3 || c.laneWidth < .3 || c.departure < 0 ||
      c.sampleStep < .01 || c.sampleStep > .10 || c.lowThreshold < 0 ||
      c.highThreshold <= c.lowThreshold || c.memoryTime <= 0 || c.memoryDistance <= 0 ||
      c.responseDelay < 0 || c.targetWeight < 0 || c.previousWeight < 0 || c.clearanceWeight < 0 ||
      c.entryReferenceWeight<0 || c.approachSpeed<=0 || c.approachSpeed>3 ||
      c.corridorLookahead<.3 || c.corridorLookahead>1.5 ||
      typeof c.laneVisible!=='boolean' || typeof c.occlusion!=='boolean' ||
      c.courseLength<4 || c.courseLength>32 || c.bevWidth!==120 || c.bevHeight!==300 || c.resolution!==.01) {
      throw Error('차량 치수·주행·VFH 설정 범위를 확인하세요. 전장 ≥ 축간거리 + 앞 돌출 길이가 필요합니다.');
    }
    return c;
  }
  function local(p, pose) {
    const dx = p.x - pose.x, dy = p.y - pose.y;
    return {x: dx * Math.cos(pose.yaw) + dy * Math.sin(pose.yaw),
      y: -dx * Math.sin(pose.yaw) + dy * Math.cos(pose.yaw)};
  }
  function world(p, pose) {
    return {x: pose.x + p.x * Math.cos(pose.yaw) - p.y * Math.sin(pose.yaw),
      y: pose.y + p.x * Math.sin(pose.yaw) + p.y * Math.cos(pose.yaw)};
  }
  function corners(p, c, extra = 0) {
    const front = c.frontOverhang + extra, back = (c.rearExtent ?? c.length - c.frontOverhang) + extra;
    return [[front, c.width / 2 + extra], [front, -c.width / 2 - extra],
      [-back, -c.width / 2 - extra], [-back, c.width / 2 + extra]].map(([x,y]) => world({x,y},p));
  }
  function advance(p, steer, distance, c) {
    const k = Math.tan(steer) / c.wheelbase;
    const rear = world({x: -c.wheelbase, y: 0}, p);
    const yaw = p.yaw + distance * k;
    if (Math.abs(k) > 1e-8) {
      rear.x += (Math.sin(yaw) - Math.sin(p.yaw)) / k;
      rear.y += (Math.cos(p.yaw) - Math.cos(yaw)) / k;
    } else {rear.x += distance * Math.cos(p.yaw); rear.y += distance * Math.sin(p.yaw);}
    return {x: rear.x + c.wheelbase * Math.cos(yaw), y: rear.y + c.wheelbase * Math.sin(yaw), yaw};
  }
  function rayShape(origin, direction, o) {
    if (o.shape === 'circle') {
      const x = origin.x - o.x, y = origin.y - o.y, b = x * direction.x + y * direction.y;
      const d = b*b - (x*x + y*y - o.w*o.w/4);
      if (d < 0) return Infinity;
      const t = -b - Math.sqrt(d); return t > 0 ? t : Infinity;
    }
    let lo = 0, hi = Infinity;
    for (const [pos, dir, ctr, size] of [[origin.x,direction.x,o.x,o.w],[origin.y,direction.y,o.y,o.h]]) {
      if (Math.abs(dir) < 1e-9) {if (Math.abs(pos-ctr) > size/2) return Infinity;}
      else {let a = (ctr-size/2-pos)/dir, b = (ctr+size/2-pos)/dir; if (a>b) [a,b]=[b,a]; lo=Math.max(lo,a); hi=Math.min(hi,b);}
    }
    return lo <= hi && hi > 0 ? lo : Infinity;
  }
  // Ground truth is used ONLY here to generate a local, first-return observation.
  function sense(obstacles, pose, c) {
    const rays = [], hits = [], half = (c.terrainWidth??c.bevWidth*c.resolution)/2, depth = c.terrainForward??c.bevHeight*c.resolution;
    for (let deg = -89; deg <= 89; deg += 1) {
      const angle=deg*DEG, co=Math.cos(angle), si=Math.sin(angle);
      const limit=Math.min(depth/co, Math.abs(si)>1e-9?half/Math.abs(si):Infinity);
      const dir={x:Math.cos(pose.yaw+angle),y:Math.sin(pose.yaw+angle)};
      let distance=limit, hit=false;
      const all=[];
      for (const o of obstacles) {
        const t=rayShape(pose,dir,o);
        if (t < limit) {
          if (t < distance) {distance=t; hit=true;}
          if (!c.occlusion) all.push(t);
        }
      }
      const ranges = c.occlusion ? (hit?[distance]:[]) : all;
      for (const r of ranges) {
        const px=clamp(Math.floor((half-r*si)/c.resolution),0,c.bevWidth-1);
        const py=clamp(Math.floor((depth-r*co)/c.resolution),0,c.bevHeight-1);
        const point=world(c.terrainWidth?{x:r*co,y:r*si}:{x:depth-(py+.5)*c.resolution,y:half-(px+.5)*c.resolution},pose);
        hits.push({...point,px,py});
      }
      rays.push({angle,distance: c.occlusion?distance:limit,hit:c.occlusion&&hit});
    }
    return {rays,hits}; // No object IDs, object dimensions, or out-of-view endpoints.
  }
  function bodyPointClearance(point, pose, c) {
    const q=local(point,pose), front=c.frontOverhang, back=c.rearExtent??c.length-c.frontOverhang;
    return Math.hypot(Math.max(-back-q.x,0,q.x-front),Math.max(Math.abs(q.y)-c.width/2,0));
  }
  function collision(obstacles, pose, c) {
    const body=corners(pose,c);
    for (const o of obstacles) {
      if (o.shape==='circle') {if(bodyPointClearance(o,pose,c)<=o.w/2) return true;}
      else {
        const box=[{x:o.x-o.w/2,y:o.y-o.h/2},{x:o.x+o.w/2,y:o.y-o.h/2},
          {x:o.x+o.w/2,y:o.y+o.h/2},{x:o.x-o.w/2,y:o.y+o.h/2}];
        let overlaps=true;
        for(const a of [{x:1,y:0},{x:0,y:1},{x:Math.cos(pose.yaw),y:Math.sin(pose.yaw)},
          {x:-Math.sin(pose.yaw),y:Math.cos(pose.yaw)}]) {
          const pa=body.map(p=>p.x*a.x+p.y*a.y), pb=box.map(p=>p.x*a.x+p.y*a.y);
          if(Math.max(...pa)<Math.min(...pb)||Math.max(...pb)<Math.min(...pa)) {overlaps=false;break;}
        }
        if(overlaps) return true;
      }
    }
    return false;
  }
  class Planner {
    constructor(c) {this.c=c;this.memory=new Map();this.free=new Set();this.active=false;this.side=0;
      this.lastDirection=0;this.lastSeen=-Infinity;this.lastSeenDistance=0;this.maxSeenX=-Infinity;
      this.clearSince=null;this.histogram=[];this.candidates=[];this.status='차선 추종';this.planCount=0;
      this.lastLane=null;this.corridor=null;this.approaching=false;this.entrySearching=false;this.referencePath=[];this.entryCount=0;}
    key(p, step=.04) {return Math.floor(p.x/step)+','+Math.floor(p.y/step);}
    observe(obs,p,time,distance) {
      const c=this.c;
      for(const ray of obs.rays) {
        for(let r=.02;r<ray.distance-.02;r+=.04) {
          this.free.add(this.key(world({x:r*Math.cos(ray.angle),y:r*Math.sin(ray.angle)},p)));
        }
      }
      for(const [key,pt] of this.memory) {
        const q=local(pt,p), d=Math.hypot(q.x,q.y), a=Math.atan2(q.y,q.x)/DEG;
        const idx=Math.round(a)+89;
        const inView=q.x>0&&q.x<3&&Math.abs(q.y)<.6&&idx>=0&&idx<obs.rays.length;
        if((inView&&d<obs.rays[idx].distance-.06)||q.x<-(c.length+.4)||time-pt.time>12) this.memory.delete(key);
      }
      for(const pt of obs.hits) {
        const q=local(pt,p);
        this.memory.set(this.key(pt,.025),{x:pt.x,y:pt.y,time});
        if(q.x<c.range&&Math.abs(q.y)<.6) {this.lastSeen=time;this.lastSeenDistance=distance;}
      }
      // The present footprint is known free only after the independent evaluator accepted it.
      for(let x=-(c.length-c.frontOverhang);x<c.frontOverhang;x+=.04)
        for(let y=-c.width/2;y<c.width/2;y+=.04) this.free.add(this.key(world({x,y},p)));
    }
    histogramFor(points,p) {
      const c=this.c, n=Math.ceil(180/c.angularStep), density=new Array(n).fill(0);
      for(const pt of points) {
        const q=local(pt,p), d=Math.hypot(q.x,q.y);
        if(q.x<0||d>c.range||d<.02) continue;
        const angle=Math.atan2(q.y,q.x), radius=c.width/2+c.margin+.018;
        const spread=Math.asin(Math.min(.999,radius/d));
        for(let i=Math.max(0,Math.floor((angle-spread)/DEG/c.angularStep+n/2));
          i<=Math.min(n-1,Math.ceil((angle+spread)/DEG/c.angularStep+n/2));i++) {
          density[i]=Math.max(density[i], .6+.4*(1-d/c.range));
        }
      }
      const old=this.histogram;
      this.histogram=density.map((d,i)=>({angle:(i+.5-n/2)*c.angularStep*DEG,density:d,
        blocked:d>=c.highThreshold?true:d<=c.lowThreshold?false:!!old[i]?.blocked}));
      return this.histogram;
    }
    known(point) {return this.free.has(this.key(point));}
    laneY(x) {
      return this.lastLane?this.lastLane.y+Math.tan(this.lastLane.heading)*(x-this.lastLane.x):0;
    }
    // Cross sections are built solely from previously measured surface points.
    // Follow connected free intervals instead of treating both corridor walls as one object.
    corridorReference(points,p,time) {
      const c=this.c, limit=c.laneWidth/2+c.departure-c.width/2-.02;
      const route=[], half=c.width/2+c.margin+.025;
      let previous=p.y, paired=0, firstPair=null, lastWall=-Infinity;
      for(let x=p.x+.16;x<=p.x+c.range+.001;x+=.12) {
        const samples=points.filter(pt=>pt.x>=x-(c.length-c.frontOverhang)-.025&&pt.x<=x+c.frontOverhang+.025);
        const spans=samples.map(pt=>[Math.max(-limit,pt.y-half),Math.min(limit,pt.y+half)])
          .filter(a=>a[0]<a[1]).sort((a,b)=>a[0]-b[0]);
        const merged=[];
        for(const span of spans){const last=merged[merged.length-1];if(last&&span[0]<=last[1]+.025)last[1]=Math.max(last[1],span[1]);else merged.push([...span]);}
        const free=[];let lo=-limit;
        for(const span of merged){if(span[0]>lo+.025)free.push([lo,span[0]]);lo=Math.max(lo,span[1]);}
        if(lo<limit-.025)free.push([lo,limit]);
        // This is a reference corridor, not a motion primitive: allow a step in
        // the observed wall so look-ahead can begin turning before that step.
        // Crossing an inflated wall remains disallowed; rollouts check curvature.
        const connection=Math.max(.16,2*half-.03);
        const reachable=free.filter(b=>previous>=b[0]-connection&&previous<=b[1]+connection);
        if(!reachable.length)break;
        let chosen=null,best=Infinity;
        for(const band of reachable) {
          const pad=Math.min(.03,(band[1]-band[0])/4), y=clamp(this.laneY(x),band[0]+pad,band[1]-pad);
          const score=Math.abs(y-previous)*2+Math.abs(y-this.laneY(x));
          if(score<best){chosen={x,y,lo:band[0],hi:band[1]};best=score;}
        }
        if(!chosen)break;
        const bounded=chosen.lo>-limit+.01&&chosen.hi<limit-.01;
        if(bounded&&chosen.hi-chosen.lo>.04){paired++;if(firstPair===null)firstPair=x;}
        for(const pt of samples)if(Math.abs(pt.y-(chosen.lo-half))<.08||Math.abs(pt.y-(chosen.hi+half))<.08)
          lastWall=Math.max(lastWall,pt.x);
        route.push({...chosen,bounded});previous=chosen.y;
      }
      this.approaching=paired>=2;
      if(this.approaching&&!this.corridor) {
        this.corridor={entryX:firstPair,lastWallX:lastWall,entered:false,lastPairTime:time};
      }
      if(this.corridor) {
        const cc=this.corridor;
        if(this.approaching){cc.lastPairTime=time;cc.lastWallX=Math.max(cc.lastWallX,lastWall);}
        else if(Number.isFinite(lastWall)&&lastWall<p.x+c.range-.2)cc.lastWallX=Math.max(cc.lastWallX,lastWall);
        if(!cc.entered&&p.x>=cc.entryX-.05){cc.entered=true;this.entryCount++;}
        const rear=Math.min(...corners(p,c).map(pt=>pt.x));
        if(cc.entered&&rear>cc.lastWallX+.06&&time-cc.lastPairTime>.3)this.corridor=null;
      }
      return route;
    }
    routeY(route,x) {
      if(!route.length)return this.laneY(x);
      if(x<=route[0].x)return route[0].y;
      for(let i=1;i<route.length;i++)if(x<=route[i].x){const a=route[i-1],b=route[i];return a.y+(b.y-a.y)*(x-a.x)/(b.x-a.x);}
      return route[route.length-1].y;
    }
    rollout(p,steer,theta,points,reference=null,targetTheta=0) {
      const c=this.c, path=[{...p}], sample=c.sampleStep, velocity=Math.max(.35,p.v||0),
        targetYaw=p.yaw+theta, target=world({x:Math.max(.65,c.horizon*.75)*Math.cos(theta),y:Math.max(.65,c.horizon*.75)*Math.sin(theta)},p);
      let q={...p},delta=steer,travel=0,minClear=1,firstSteer=steer,reason='可';
      for(let s=sample;s<=c.horizon+.001;s+=sample) {
        const follow=reference?.length?{x:q.x+c.corridorLookahead,y:this.routeY(reference,q.x+c.corridorLookahead)}:target;
        const diff=local(follow,q), alpha=Math.atan2(diff.y,diff.x)+(reference?theta-targetTheta:0);
        const desired=(reference||s<c.horizon*.75)?Math.atan2(2*c.wheelbase*Math.sin(alpha),Math.max(.35,Math.hypot(diff.x,diff.y))):
          clamp(wrap(targetYaw-q.yaw)*1.2,-c.maxSteer*DEG,c.maxSteer*DEG);
        delta+=clamp(clamp(desired,-c.maxSteer*DEG,c.maxSteer*DEG)-delta,-c.steerRate*DEG*sample/velocity,c.steerRate*DEG*sample/velocity);
        q=advance(q,delta,sample,c);
        if(s===sample) firstSteer=delta;
        if(Math.cos(q.yaw)<.15) {reason='방향 한계';break;}
        if(corners(q,c).some(pt=>Math.abs(pt.y)>c.laneWidth/2+c.departure)) {reason='이탈 한도';break;}
        if(reference&&this.corridor?.entered) {
          const band=reference.find(pt=>pt.x>=q.x);
          if(band&&q.x>=reference[0].x&&(q.y<band.lo||q.y>band.hi)){reason='선택한 통로 경계';break;}
        }
        let clearance=1;
        for(const pt of points) clearance=Math.min(clearance,bodyPointClearance(pt,q,c));
        if(clearance<c.margin+.018+sample*.55) {reason='장애물';break;}
        // The front and side of the moving body must be in current or previously observed free space.
        const probes=[...corners(q,c),world({x:c.frontOverhang,y:0},q)];
        if(s>.10&&probes.some(pt=>!this.known(pt))) {reason='미관측';break;}
        minClear=Math.min(minClear,clearance);travel=s;path.push({...q});
      }
      return {theta,path,travel,minClear,firstSteer,reason};
    }
    plan(obs,p,time,distance) {
      this.planCount++;this.observe(obs,p,time,distance);
      const c=this.c, points=[...this.memory.values()].filter(pt=>{
        const q=local(pt,p);return q.x>-c.length-.2&&q.x<c.range+.4&&Math.abs(q.y)<1.3;
      });
      if(obs.lane?.valid)this.lastLane={x:p.x,y:obs.lane.centerY,heading:obs.lane.heading};
      const corridorRoute=this.corridorReference(points,p,time);
      const guidance=!!this.corridor;
      this.referencePath=guidance?corridorRoute:[];
      let wallEvidence=false,groupStart=null,lastX=-Infinity,count=0;
      for(const pt of [...points].filter(pt=>pt.x>p.x).sort((a,b)=>a.x-b.x)) {
        if(pt.x-lastX>.25){groupStart=pt.x;count=0;}lastX=pt.x;count++;
        if(count>=8&&pt.x-groupStart>.45)wallEvidence=true;
      }
      const blocking=points.filter(pt=>pt.x>p.x-c.frontOverhang&&pt.x<p.x+c.range&&
        Math.abs(pt.y-this.laneY(pt.x))<c.width/2+c.margin+.025);
      if(!guidance&&!wallEvidence&&!this.active&&blocking.length) {
        this.active=true;
        const avg=blocking.reduce((s,o)=>s+o.y,0)/blocking.length;
        this.side=avg>=0?-1:1;
        this.maxSeenX=Math.min(...blocking.map(pt=>pt.x));
      }
      this.entrySearching=!guidance&&!this.active&&wallEvidence;
      // Extend only a connected, previously observed boundary. A distant independent
      // obstacle must not prevent completion of the current avoidance manoeuvre.
      if(this.active)for(const pt of [...points].sort((a,b)=>a.x-b.x)) {
        if(pt.x<=this.maxSeenX+.28&&pt.x>=this.maxSeenX-.5)this.maxSeenX=Math.max(this.maxSeenX,pt.x);
      }
      const rearX=Math.min(...corners(p,c).map(pt=>pt.x));
      if(this.active&&rearX>this.maxSeenX+.02&&time-this.lastSeen>.15) {
        if(this.clearSince===null)this.clearSince=time;
        if(time-this.clearSince>.30){this.active=false;this.side=0;this.maxSeenX=-Infinity;}
      } else this.clearSince=null;
      // A lane-parallel target is only a preference; obstacle clearance is hard checked.
      let goalY=this.laneY(p.x+.9);
      if(guidance)goalY=this.routeY(corridorRoute,p.x+c.corridorLookahead);
      else if(this.active) {
        const ahead=points.filter(pt=>pt.x>rearX-.1&&pt.x<=this.maxSeenX+.08);
        if(ahead.length) {
          goalY=this.side>0?Math.max(...ahead.map(pt=>pt.y))+c.width/2+c.margin+.09:
            Math.min(...ahead.map(pt=>pt.y))-c.width/2-c.margin-.09;
          goalY=clamp(goalY,-c.laneWidth/2-c.departure+c.width/2+.03,c.laneWidth/2+c.departure-c.width/2-.03);
        } else goalY=p.y;
      }
      const target=wrap(Math.atan2(goalY-p.y,guidance?c.corridorLookahead:.9)-p.yaw), histogram=this.histogramFor(points,p);
      const returning=!guidance&&!this.active&&Math.abs(p.y-this.laneY(p.x))>.04;
      const trackingReference=guidance?corridorRoute:returning?
        Array.from({length:18},(_,i)=>({x:p.x+.16+i*.12,y:this.laneY(p.x+.16+i*.12)})):null;
      if(trackingReference)this.referencePath=trackingReference;
      const directions=new Set([0,target,this.lastDirection-p.yaw]);
      let start=null;
      for(let i=0;i<=histogram.length;i++) {
        if(i<histogram.length&&!histogram[i].blocked) {if(start===null)start=i;}
        else if(start!==null) {
          const lo=histogram[start].angle,hi=histogram[i-1].angle;
          directions.add(clamp(target,lo,hi));directions.add((lo+hi)/2);
          directions.add(lo+Math.min(.06,(hi-lo)/2));directions.add(hi-Math.min(.06,(hi-lo)/2));start=null;
        }
      }
      // Curved approach candidates supplement straight-ray valleys: short steering arcs can
      // be collision-free even when their final heading bin points behind an obstacle.
      for(let a=-60;a<=60;a+=10)directions.add(a*DEG);
      let best=null;
      this.candidates=[];
      for(const theta of directions) {
        if(Math.abs(theta)>80*DEG)continue;
        const a=this.rollout(p,p.steer,theta,points,trackingReference,target), end=a.path[a.path.length-1];
        const bin=histogram[clamp(Math.floor((theta/DEG+90)/c.angularStep),0,histogram.length-1)];
        const referenceError=guidance?Math.abs(end.y-this.routeY(corridorRoute,end.x)):Math.abs(end.y-goalY);
        a.score=3.2*a.travel+1.1*(end.x-p.x)-(guidance?c.entryReferenceWeight:c.targetWeight*(this.active?1:1.6))*referenceError-
          (this.active?.5:1)*Math.abs(end.yaw)-c.previousWeight*Math.abs(wrap(p.yaw+theta-this.lastDirection))+
          c.clearanceWeight*Math.min(.7,a.minClear)+(bin.blocked?-.12:.12);
        if(!guidance&&this.active&&this.side*(end.y-p.y)<-.10) a.score-=.8;
        this.candidates.push(a);
        if(!best||a.score>best.score)best=a;
      }
      // If the chosen side is blocked, safely re-evaluate the other side on a later cycle.
      if(best&&best.travel<.16&&this.active&&!guidance&&p.v<.04) {
        this.blockedCycles=(this.blockedCycles||0)+1;
        if(this.blockedCycles>c.planHz){this.side*=-1;this.blockedCycles=0;}
      } else this.blockedCycles=0;
      let usable=best?Math.max(0,best.travel-.08):0;
      let budget=Infinity;
      // The short blind-travel budget applies when BOTH the lane and cone
      // observations are unavailable. A visible lane plus fresh measured free
      // space is not a blind-memory-only continuation.
      if((this.active||guidance||this.entrySearching)&&!obs.lane?.valid&&time-this.lastSeen>1/c.planHz+.01) {
        const remaining=c.memoryDistance-(distance-this.lastSeenDistance);
        budget=Math.max(0,remaining);usable=Math.min(usable,budget);
        if(time-this.lastSeen>=c.memoryTime)usable=0;
      }
      const speedLimit=Math.max(0,Math.sqrt((c.deceleration*c.responseDelay)**2+2*c.deceleration*usable)-c.deceleration*c.responseDelay);
      const curvature=best?Math.abs(Math.tan(best.firstSteer)/c.wheelbase):0;
      let speed=Math.min(c.speed,speedLimit,Math.sqrt(.8/Math.max(.05,curvature)));
      if(this.entrySearching||(this.approaching&&!this.corridor?.entered))speed=Math.min(speed,c.approachSpeed);
      const laneFallback=!obs.lane?.valid&&(guidance||this.entrySearching)&&!!this.lastLane;
      if(!obs.lane?.valid&&!laneFallback)speed=0;
      if(usable<.006)speed=0;
      this.status=!obs.lane?.valid&&!laneFallback?'차선 유실·정지':speed<.02?'정지·재계획':
        guidance?(this.corridor.entered?'유도 통로 유지':'유도 입구 접근')+(laneFallback?' · 마지막 차선 방향':''):
        this.entrySearching?'유도 입구 탐색'+(laneFallback?' · 마지막 차선 방향':''):
        Number.isFinite(budget)?'콘 관측 기억':this.active?'장애물 회피':Math.abs(p.y)>.07?'차선 복귀':'차선 추종';
      this.lastDirection=p.yaw+(best?.theta||0);
      return {steer:best?.firstSteer||0,speed,path:best?.path||[],safeDistance:usable,
        points,histogram,candidates:this.candidates,status:this.status,active:this.active||guidance,
        referencePath:this.referencePath,corridorEntered:!!this.corridor?.entered,laneFallback};
    }
  }
  function presets(name) {
    const circle=(x,y,w=.13)=>({id:'o'+Math.random().toString(36).slice(2,9),shape:'circle',x,y,w,h:w});
    if(name==='track')return api.trackObstacles();
    if(name==='empty')return [];
    if(name==='static')return [circle(2,.12,.18),circle(3.6,-.12,.20),circle(5.1,.13,.16)];
    if(name==='blocked')return [{id:'wall',shape:'box',x:2,y:0,w:.20,h:2.5}];
    const objects=[];
    // A close-spaced cone boundary detours left and then rejoins the original lane.
    // All geometry is in the editor; only first-return pixels enter Planner.
    for(let x=1.65;x<=5.0;x+=.12) {
      const t=clamp((x-2.75)/(5.0-2.75),0,1);
      const y=.08-.40*Math.sin(Math.PI*t)**2-.25*t;
      objects.push(circle(x,y,.13));
    }
    return objects;
  }
  function timingStats(values) {
    if(!values.length)return {count:0,meanMs:0,p95Ms:0,maxMs:0,totalMs:0};
    const ordered=[...values].sort((a,b)=>a-b),total=values.reduce((a,b)=>a+b,0);
    return {count:values.length,meanMs:total/values.length,p95Ms:ordered[Math.ceil(values.length*.95)-1],maxMs:ordered[ordered.length-1],totalMs:total};
  }
  function fingerprint(config,obstacles) {
    const c=validate(config),sorted=Object.fromEntries(Object.keys(c).sort().map(k=>[k,c[k]]));
    const data=JSON.stringify({config:sorted,obstacles:obstacles.map(o=>({shape:o.shape,x:o.x,y:o.y,w:o.w,h:o.h}))});
    let hash=2166136261;for(let i=0;i<data.length;i++){hash^=data.charCodeAt(i);hash=Math.imul(hash,16777619);}
    return (hash>>>0).toString(16);
  }
  class Simulation {
    constructor(config={},obstacles=presets('guidance'),plannerFactory=c=>new Planner(c),markers=[]) {this.markers=markers.map(m=>({...m}));this.plannerFactory=plannerFactory;this.c=validate(config);this.obstacles=JSON.parse(JSON.stringify(obstacles));this.reset();}
    reset() {
      const start=this.c.trackLayout==='loop'?api.trackPoint(0):{x:0,y:0,yaw:0};
      this.lapProgress=0;this.previousTrackPosition=0;
      this.pose={...start,v:0,steer:0};this.time=0;this.distance=0;this.trace=[{...this.pose,t:0}];
      this.planner=this.plannerFactory(this.c);this.observation={rays:[],hits:[]};this.result=null;
      this.done=false;this.collided=collision(this.obstacles,this.pose,this.c);this.departureViolation=false;this.nextPlan=0;
      this.timings={warmupCalls:20,calls:0,sensor:[],planner:[],activePlanner:[],hazard:[],pipeline:[],integration:[],loop:[]};
      this.minClearance=Infinity;this.maxDeparture=0;this.markerIndex=0;this.checkMarkers(this.pose);this.senseAndPlan();
    }
    recordTime(stage,value) {
      const values=this.timings[stage];if(values.length>=4096)values.shift();values.push(value);
    }
    checkMarkers(from) {
      const dx=this.pose.x-from.x,dy=this.pose.y-from.y,d2=dx*dx+dy*dy;
      let earliest=0;
      while(this.markerIndex<this.markers.length) {
        const m=this.markers[this.markerIndex],x=m.x-from.x,y=m.y-from.y,r=m.w/2;
        const t=d2?(x*dx+y*dy)/d2:0;
        const nearest=clamp(t,earliest,1);
        if(Math.hypot(x-nearest*dx,y-nearest*dy)>r)break;
        if(d2)earliest=Math.max(earliest,t-Math.sqrt(Math.max(0,r*r-((x-t*dx)**2+(y-t*dy)**2))/d2));
        this.markerIndex++;
      }
    }
    senseAndPlan() {
      const started=performance.now();
      this.observation=sense(this.obstacles,this.pose,this.c);
      this.observation.lane={valid:this.c.laneVisible,centerY:0,heading:0};
      const sensed=performance.now();
      this.result=this.planner.plan(this.observation,this.pose,this.time,this.distance);
      const finished=performance.now();this.timings.calls++;
      if(this.timings.calls>this.timings.warmupCalls) {
        this.recordTime('sensor',sensed-started);this.recordTime('planner',this.result.planMs??finished-sensed);
        if(this.result.filterMs!==undefined)this.recordTime('hazard',this.result.filterMs);
        this.recordTime('pipeline',finished-started);
        if(this.result.active||this.result.points.length)this.recordTime('activePlanner',this.result.planMs??finished-sensed);
      }
      this.nextPlan=this.time+1/this.c.planHz;
    }
    step(dt=.01) {
      if(this.done||this.collided)return;
      const stepStarted=performance.now();
      if(this.time+1e-8>=this.nextPlan)this.senseAndPlan();
      const integrated=performance.now();
      const p=this.pose,c=this.c,r=this.actuation||this.result;
      const steeringDelta=this.planner.separateActuation?0:c.terrainWidth?(r.speed>0?(1-Math.pow(.6,dt*100))*(r.steer-p.steer):0):r.steer-p.steer;
      p.steer+=clamp(steeringDelta,-c.steerRate*DEG*dt,c.steerRate*DEG*dt);
      p.v+=clamp(r.speed-p.v,-c.deceleration*dt,c.acceleration*dt);
      // Subcentimetre integration checks the swept motion, not only plan points.
      this.pose={...advance(p,p.steer,p.v*dt,c),v:Math.max(0,p.v),steer:p.steer};
      this.time+=dt;this.distance+=p.v*dt;
      this.collided=collision(this.obstacles,this.pose,c);
      if(!this.collided)this.checkMarkers(p);
      const departure=Math.max(0,...corners(this.pose,c).map(pt=>(c.trackLayout==='loop'?api.projectTrack(pt).distance:Math.abs(pt.y))-c.laneWidth/2));
      this.maxDeparture=Math.max(this.maxDeparture,departure);
      this.departureViolation=!this.planner.ownsDeparturePolicy && departure>c.departure+.002;
      if(this.trace.length===0||this.time-this.trace[this.trace.length-1].t>.05)this.trace.push({...this.pose,t:this.time});
      if(this.departureViolation && !r.laneMissing && !this.planner.ownsDeparturePolicy)this.done=true;
      if(c.trackLayout==='loop') {
        const position=api.projectTrack(this.pose);
        let delta=position.s-this.previousTrackPosition;
        if(delta>api.trackLength/2)delta-=api.trackLength;
        if(delta<-api.trackLength/2)delta+=api.trackLength;
        this.previousTrackPosition=position.s;
        if(position.distance<1.5)this.lapProgress+=delta;
        if(this.lapProgress>=api.trackLength-.08 && position.distance<c.laneWidth/2) {
          this.done=true;this.pose.v=0;
        }
      }else if(this.pose.x>=c.courseLength){this.done=true;this.pose.v=0;}
      // Independent evaluator only. These values never feed the local planner.
      for(const o of this.obstacles)if(o.shape==='circle')this.minClearance=Math.min(this.minClearance,bodyPointClearance(o,this.pose,c)-o.w/2);
      if(this.timings.calls>this.timings.warmupCalls){this.recordTime('integration',performance.now()-integrated);this.recordTime('loop',performance.now()-stepStarted);}
    }
    performanceStats() {
      return Object.fromEntries(['sensor','planner','activePlanner','hazard','pipeline','integration','loop'].map(k=>[k,timingStats(this.timings[k])]));
    }
    run(seconds=35) {for(let n=0;n<seconds/.01&&!this.done&&!this.collided;n++)this.step(.01);return this.summary();}
    summary() {
      const reachedFinish=this.done&&!this.collided&&(this.c.trackLayout==='loop'?this.lapProgress>=api.trackLength-.08:this.pose.x>=this.c.courseLength);
      const laneReturned=corners(this.pose,this.c).every(p=>(this.c.trackLayout==='loop'?api.projectTrack(p).distance:Math.abs(p.y))<=this.c.laneWidth/2);
      return {time:+this.time.toFixed(2),x:+this.pose.x.toFixed(3),y:+this.pose.y.toFixed(3),
      reachedFinish,laneReturned,markersPassed:this.markerIndex,markersTotal:this.markers.length,
      completed:reachedFinish&&laneReturned&&this.markerIndex===this.markers.length,
      collision:this.collided,departureViolation:this.departureViolation,maxDeparture:+this.maxDeparture.toFixed(3),
      minClearance:Number.isFinite(this.minClearance)?+this.minClearance.toFixed(3):null,status:this.result.status,
      corridorEntries:this.planner.entryCount,
      corridorActive:!!this.planner.corridor,
      plannerCalls:this.planner.planCount,observedMemoryPoints:this.planner.memory.size};}
  }
  const api={defaults,validate,local,world,corners,advance,sense,collision,Planner,Simulation,presets,clamp,DEG,timingStats,fingerprint};
  if(typeof module!=='undefined'&&module.exports){Object.assign(api,require('./track.js'));module.exports=api;}
  else scope.VFHSim=api;
})(typeof window!=='undefined'?window:globalThis);
