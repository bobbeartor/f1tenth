/* Ideal observations; the shared ROS C++ policy owns every path decision. */
(() => {
  const M=window.VFHSim, profile=window.VFHSimVehicleProfile;
  if(!profile)throw Error('실차 설정이 없습니다. serve.py로 시뮬레이터를 실행하세요.');
  Object.assign(M.defaults,profile.defaults,{laneVisibleLength:profile.defaults.laneVisibleLength});
  M.trackDefaults={};
  M.vehicleProfile=profile;
  const ReferenceSimulation=M.Simulation, referenceValidate=M.validate;
  M.validate=input=>{
    const c=referenceValidate(input),rt=profile.runtime;
    if(c.speed>rt.speed_ceiling || c.deceleration>rt.deceleration_ceiling)
      throw Error('실차 YAML의 속도·감속도 상한을 초과했습니다.');
    if(c.maxSteer>rt.steering_ceiling_deg || c.steerRate>rt.steering_rate_ceiling_deg_per_sec)
      throw Error('실차 YAML의 조향각·조향 변화율 상한을 초과했습니다.');
    if(![0,1].includes(c.laneMemoryEnabled))throw Error('차선 방향 기억은 0 또는 1입니다.');
    for(const key of ['rgbHz','depthHz','controlHz','imuHz','previewHz'])
      if(c[key]!==profile.defaults[key])throw Error(key+': 실차 기준 고정 설정입니다.');
    return c;
  };
  const clonePose=(p,time)=>({...p,t:time});
  class NativePlanner {
    constructor(config) {
      this.config=config;this.ownsDeparturePolicy=true;this.separateActuation=true;
      this.session=crypto.randomUUID();this.memory=new Map();this.planCount=0;this.entryCount=0;
      this.nextRgb=0;this.nextDepth=0;this.pendingDepth=[];this.sensorCounts={rgb:0,depth:0,control:0};
    }
    captureDepth(observation,pose,time) {
      const c=this.config,res=c.terrainCell,width=Math.ceil(c.terrainForward/res),height=Math.ceil(c.terrainWidth/res),y0=-c.terrainWidth/2;
      const hazards=new Set();
      for(const hit of observation.hits) {
        const p=M.local(hit,pose),u=Math.floor(p.x/res),v=Math.floor((p.y-y0)/res);
        if(u>=0 && u<width && v>=0 && v<height)hazards.add(v*width+u);
      }
      this.depth={pose:clonePose(pose,time),hazards:[...hazards],width,height,res,y0};
      this.pendingDepth.push(this.depth);
      if(this.pendingDepth.length>64)throw Error('계획 주기에 비해 관측이 너무 많이 쌓였습니다.');
      ++this.sensorCounts.depth;
    }
    captureLane(pose,time) {
      const c=this.config,points=[];
      if(c.laneVisible) {
        const position=c.trackLayout==='loop'?M.projectTrack(pose).s:pose.x;
        for(let distance=0;distance<=3.1;distance+=.02) {
          const source=c.trackLayout==='loop'?M.trackPoint(position+distance):{x:position+distance,y:0};
          const p=M.local(source,pose);
          if(p.x>=c.laneMinX && p.x<=Math.min(c.laneMaxX,c.laneVisibleLength) && Math.abs(p.y)<=c.laneBevHalfWidth)points.push(p);
        }
      }
      this.lane={pose:clonePose(pose,time),points};++this.sensorCounts.rgb;
    }
    plan(pose,time) {
      const c=this.config,d=this.depth,l=this.lane;
      if(!d || !l)throw Error('정상 센서 입력 초기화를 기다립니다.');
      const key=d.pose.t+'|'+(l.points.length>=2?l.pose.t:-1)+'|'+c.laneMemoryEnabled;
      if(this.cached && this.cachedKey===key)return {...this.cached,replanned:false,filterMs:0,planMs:0};
      const data=['V2',profile.fingerprint,this.session,d.pose.t,d.pose.x,d.pose.y,d.pose.yaw,pose.v,pose.steer,
        c.planHz,c.speed,c.horizon,c.sampleStep,c.margin,c.laneWidth,c.width,c.rearExtent,
        c.frontOverhang,c.wheelbase,c.maxSteer*M.DEG,c.steerRate*M.DEG,c.deceleration,c.responseDelay,
        c.laneLookahead,c.referenceDistance,c.lowThreshold,c.highThreshold,c.hazardEnabled,c.hazardFrames,
        c.hazardSmall,c.hazardRadius,c.hazardGap,c.extraCurves,c.curveSamples,c.lengthSlack,c.laneWeight,
        c.steeringWeight,d.width,d.height,d.res,d.y0,0,0,0,0,c.clearanceWeight,l.points.length,d.hazards.length,
        ...l.points.flatMap(p=>[p.x,p.y]),...d.hazards,
        c.laneMemoryEnabled,time,l.pose.t,l.pose.x,l.pose.y,l.pose.yaw,this.pendingDepth.length];
      for(const frame of this.pendingDepth)data.push(frame.pose.t,frame.pose.x,frame.pose.y,frame.pose.yaw,frame.hazards.length,...frame.hazards);
      data.push(c.uncertainty,c.hazardHold,c.obstacleMemoryEnabled,c.obstacleMemorySeconds,c.obstacleMemoryDistance,c.obstacleMemoryClearance);
      data.push(c.targetAcceleration,c.steeringCurrentWeight,c.stanleyGain,c.stanleySoftening,c.maxCurvature,c.cornerHeading*M.DEG,c.cornerOpposing);
      const req=new XMLHttpRequest();req.open('POST','/plan',false);req.send(data.join(' '));
      let result;
      try {result=JSON.parse(req.responseText);}catch {throw Error('로컬 C++ 계획기 응답 실패');}
      if(result.error)throw Error(result.error);
      if(req.status!==200)throw Error('로컬 C++ 계획기 요청 실패');
      this.pendingDepth=[];
      const path=result.path.map(([x,y])=>M.world({x,y},d.pose));
      this.lastDirection=d.pose.yaw+(result.path.length?Math.atan2(result.path.at(-1)[1],result.path.at(-1)[0]):0);
      if(result.replanned)++this.planCount;
      const peak=Math.max(1,...result.histogram.map(([value])=>value));
      this.cachedKey=key;
      this.cached={...result,path,referencePath:(result.referencePath||result.memoryPath||[]).map(([x,y])=>M.world({x,y},d.pose)),
        active:!!result.usedVfh,points:result.hazards,
        histogram:result.histogram.map(([value,blocked])=>({density:value/peak,blocked})),
        grid:{...result.grid,pose:{...d.pose}}};
      this.memory.clear();
      const g=this.cached.grid;
      for(const index of result.retainedHazards||[])this.memory.set(index,M.world({
        x:g.x0+(index%g.width+.5)*g.res,y:g.y0+(Math.floor(index/g.width)+.5)*g.res},g.pose));
      return this.cached;
    }
  }
  M.Simulation=class extends ReferenceSimulation {
    constructor(config,obstacles,markers=[]) {
      super(M.validate({...M.defaults,...config}),obstacles,c=>new NativePlanner(c),markers);
    }
    reset() {this.actuation=null;super.reset();this.nextControl=0;}
    observeSensors() {
      const p=this.planner,c=this.c,t=this.time,eps=1e-9;
      if(t+eps>=p.nextDepth) {
        const start=performance.now();
        this.observation=M.sense(this.obstacles,this.pose,c);
        p.captureDepth(this.observation,this.pose,t);
        do {p.nextDepth+=1/c.depthHz;}while(p.nextDepth<=t+eps);
        if(this.timings.calls>this.timings.warmupCalls)this.recordTime('sensor',performance.now()-start);
      }
      if(t+eps>=p.nextRgb) {
        p.captureLane(this.pose,t);
        do {p.nextRgb+=1/c.rgbHz;}while(p.nextRgb<=t+eps);
      }
    }
    senseAndPlan() {
      this.observeSensors();
      const started=performance.now();this.result=this.planner.plan(this.pose,this.time);
      const finished=performance.now();
      if(this.result.replanned) {
        ++this.timings.calls;
        if(this.timings.calls>this.timings.warmupCalls) {
          this.recordTime('planner',this.result.planMs);this.recordTime('hazard',this.result.filterMs);
          this.recordTime('pipeline',finished-started);
          if(this.result.active || this.result.points.length)this.recordTime('activePlanner',this.result.planMs);
        }
      }
      this.nextPlan=this.time+1/this.c.planHz;
    }
    control() {
      const c=this.c,r=this.result,p=this.pose,dt=1/c.controlHz;
      if(r.speed>0) {
        const desired=M.clamp(r.steer,-c.maxSteer*M.DEG,c.maxSteer*M.DEG);
        const weight=1-Math.pow(1-c.steeringCurrentWeight,dt*c.controlHz);
        p.steer+=M.clamp(weight*(desired-p.steer),-c.steerRate*M.DEG*dt,c.steerRate*M.DEG*dt);
      }
      // Normal-input test: zero target retains steering, as the real Stop path does.
      this.actuation={speed:r.speed,steer:p.steer};++this.planner.sensorCounts.control;
      do {this.nextControl+=dt;}while(this.nextControl<=this.time+1e-9);
    }
    step(dt=.01) {
      const end=this.time+dt;
      while(this.time<end-1e-10 && !this.done && !this.collided) {
        this.observeSensors();
        if(this.time+1e-9>=this.nextPlan)this.senseAndPlan();
        if(this.time+1e-9>=this.nextControl)this.control();
        const until=Math.min(end,this.time+.005,this.planner.nextDepth,this.planner.nextRgb,this.nextPlan,this.nextControl);
        ReferenceSimulation.prototype.step.call(this,until-this.time);
      }
    }
  };
  M.native=true;
})();
