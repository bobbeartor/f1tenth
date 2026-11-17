(function () {
  'use strict';
  const root=document.getElementById('vfh-course-sim'), M=window.VFHSim;
  const el=s=>root.querySelector(s), all=s=>[...root.querySelectorAll(s)];
  const saved=window.openai?.widgetState;
  const savedModel=saved?.modelContent||saved;
  const initial=window.VFHSimInitialScene;
  const baseline=window.VFHSimBenchmark;
  const planningProfiles=(M.native && window.simPlanner!=='frenet'?window.VFHSimNativeProfiles:window.VFHSimProfiles)?.profiles||[];
  let jetsonRatio=Number(saved?.privateContent?.jetsonRatio)||baseline?.jetson?.timeRatio||4;
  let lastPerfAt=-Infinity;const renderTimes=[];
  let config={...M.defaults,...M.trackDefaults,trackLayout:'loop',...initial?.config}, objects=initial?.obstacles?JSON.parse(JSON.stringify(initial.obstacles)):M.presets('track'), markers=[], selected=null, sim, running=false, drag=null, lastFrame=0, accumulator=0;
  let frameRequest=null,lastCanvasFrame=0;
  if(M.native) {
    const source=el('[data-out="policy-source"]'),p=M.vehicleProfile;
    const summary=`실차 YAML 기준 · RGB ${config.rgbHz} / Depth ${config.depthHz} / 계획 ${config.planHz} / 구동 ${config.controlHz} Hz · 정상 입력 · 정책 ${p.fingerprint}`;
    source.textContent=summary;
    setInterval(async()=>{
      try {
        const response=await fetch('/profile',{cache:'no-store'}),current=await response.json();
        if(current.restartRequired || current.fingerprint!==p.fingerprint) {
          stopFrames();
          source.textContent=current.restartRequired ?
            '실차 정책 파일이 바뀌었습니다. 배치를 JSON으로 보관한 뒤 시뮬레이터 서버를 재시작하세요.':
            '시뮬레이터가 갱신됐습니다. 배치를 JSON으로 보관한 뒤 페이지를 새로고침하세요.';
          source.style.color='var(--destructive)';source.dataset.stale='true';
        }
      }catch { /* A stopped server is reported by the next planning request. */ }
    },3000);
  }
  let worldMap={scale:1,left:38,top:20,height:240};
  const frenet=M.native && window.simPlanner==="frenet";
  el('[data-act="track-tuning"]').hidden=!M.native || frenet;
  const marksEnabled=M.native && !frenet;
  el('option[value="marker"]').hidden=!marksEnabled;
  const selectedItem=()=>markers.find(m=>m.id===selected)||objects.find(o=>o.id===selected);
  const updateMarkerTool=()=>{el('[data-field="marker-size"]').closest('label').hidden=
    !marksEnabled || el('[data-field="tool"]').value!=='marker';};
  el('[data-field="tool"]').addEventListener('change',updateMarkerTool);
  updateMarkerTool();
  const fields=M.native?[
    ['courseLength','코스 길이(m)',4,32,1],
    ['length','차체 전장(m)',.43,1.5,.01],['frontOverhang','앞차축→앞끝(m)',0,.4,.01],
    ['rearExtent','앞차축→뒤끝(m)',.33,1,.01],['width','차체 폭(m)',.1,.8,.01],
    ['wheelbase','축간거리(m)',.1,1,.01],['maxSteer','최대 조향각(°)',5,M.vehicleProfile.runtime.steering_ceiling_deg,1],
    ['steerRate','조향 변화율(°/s)',10,M.vehicleProfile.runtime.steering_rate_ceiling_deg_per_sec,10],['acceleration','차량 모델 가속도(m/s²)',.1,3,.1],
    ['speed','목표 속도(m/s)',.05,M.vehicleProfile.runtime.speed_ceiling,.05],
    ['planHz','계획 주기(Hz)',5,80,1],['horizon','계획 거리(m)',.3,2.5,.1],
    ['sampleStep','검사 간격(m)',.02,.08,.01],['margin','패딩(m)',0,.2,.01],
    ['laneLookahead','차선 lookahead(m)',.01,.8,.01],
    ['referenceDistance','회피 목표 거리(m)',.3,3,.1],['lowThreshold','히스토그램 하한',0,999,.1],
    ['highThreshold','히스토그램 상한',.001,1000,.1],['deceleration','계획 감속도(m/s²)',.1,M.vehicleProfile.runtime.deceleration_ceiling,.05],
    ['responseDelay','입력 반응 지연(s)',0,1,.01],['laneMemoryEnabled','차선 방향 기억(0/1)',0,1,1],['hazardEnabled','연속성 필터(0/1)',0,1,1],
    ['hazardFrames','확인 프레임',1,5,1],['hazardSmall','작은 덩어리 최대 셀',1,50,1],
    ['hazardRadius','대응 반경(셀)',0,2,1],['hazardGap','최대 관측 간격(s)',.02,.5,.01],
    ['extraCurves','추가 곡선(0/1)',0,1,1],['curveSamples','곡선 샘플(홀수)',3,17,2],
    ['lengthSlack','길이 비교 여유(m)',0,.5,.01],['laneWeight','복귀 가중치',0,5,.1],
    ['steeringWeight','조향 연속성 가중치',0,5,.1],
    ['laneVisibleLength','보이는 차선 길이(m)',.05,3,.05],
    ['clearanceWeight','장애물 여유 가중치',0,5,.1],
    ['terrainCell','terrain 셀 크기(m)',.025,.12,.005],
    ['uncertainty','장애물 위치 불확실성(m)',0,.5,.01],
    ['hazardHold','확정 장애물 깜빡임 유지(s)',0,1,.05],
    ['obstacleMemoryEnabled','측면·후미 장애물 기억(0/1)',0,1,1],
    ['obstacleMemorySeconds','장애물 최대 기억 시간(s)',0,5,.1],
    ['obstacleMemoryDistance','차체 주변 기억 거리(m)',.1,2,.05],
    ['obstacleMemoryClearance','후미 통과 추가 여유(m)',0,.5,.01],
    ['targetAcceleration','목표 속도 회복 가속도(m/s²)',.05,3,.05],
    ['steeringCurrentWeight','조향 현재값 가중치',.01,1,.01],
    ['stanleyGain','Stanley 횡오차 보정 계수',0,10,.1],
    ['stanleySoftening','Stanley 저속 완화 속도(m/s)',.01,3,.01],
    ['maxCurvature','최대 곡률(1/m)',.1,10,.1],
    ['cornerHeading','코너 헤딩 기준각(°)',0,180,1],
    ['cornerOpposing','코너 반대 조향 보정 비율',0,.99,.01]
  ].filter(([key])=>!frenet || !['lowThreshold','highThreshold','extraCurves','clearanceWeight'].includes(key)):[
    ['courseLength','코스 길이(m)',4,32,1],
    ['speed','목표 속도(m/s)',.1,3,.1],['range','VFH 전방 거리(m)',.4,3,.1],
    ['angularStep','VFH 각도 간격(°)',.5,10,.5],['planHz','계획 주기(Hz)',5,40,1],
    ['horizon','후보 경로 길이(m)',.3,3,.1],['sampleStep','경로 검사 간격(m)',.01,.1,.01],
    ['margin','안전 여유(m)',0,.3,.01],['laneWidth','차선 폭(m)',.3,1.5,.01],
    ['departure','차선 밖 이탈 한도(m)',0,1.5,.05],['width','차량 폭(m)',.1,.8,.01],
    ['length','차량 전장(m)',.2,1.5,.01],['wheelbase','축간거리(m)',.1,1,.01],
    ['frontOverhang','앞차축→앞범퍼(m)',0,.4,.01],['maxSteer','최대 조향각(°)',5,45,1],
    ['steerRate','조향 변화율(°/s)',10,500,10],['acceleration','가속도(m/s²)',.1,3,.1],
    ['deceleration','제동 감속도(m/s²)',.1,4,.1],['responseDelay','제동 계획 지연(s)',0,.5,.01],
    ['memoryTime','콘 미관측 한도(s)',.1,5,.1],['memoryDistance','콘 미관측 한도(m)',.1,2,.05],
    ['targetWeight','목표 방향 가중치',0,10,.1],['previousWeight','이전 방향 가중치',0,5,.1],
    ['clearanceWeight','통과 여유 가중치',0,3,.05],['lowThreshold','히스토그램 하한',0,.8,.05],
    ['highThreshold','히스토그램 상한',.1,1,.05],
    ['entryReferenceWeight','입구·통로 기준 가중치',0,20,.5],['approachSpeed','입구 탐색 속도 상한(m/s)',.1,2,.1],
    ['corridorLookahead','통로 추종 전방 주시(m)',.3,1.5,.05]
  ];
  function parseScene(state) {
    if(!state||!(M.native?[1,2,3,4,5]:[1,2]).includes(state.version))throw Error('지원하지 않는 장면 버전입니다.');
    const migrated={...state.config};
    if(M.native)for(const key of ['corridorEnabled','corridorWeight','corridorMinLength','corridorMaxWidth','departure','laneFadeSeconds','laneFadeDistance'])delete migrated[key];
    if(state.version===1&&migrated.courseLength===8)migrated.courseLength=16;
    // Legacy scenes preserve layout and sensor toggles. Their JS-only tuning
    // and old braking limits are not meaningful for the native planner.
    const input=M.native && state.version<3?Object.fromEntries(
      ['courseLength','laneVisible','occlusion','trackLayout'].filter(key=>key in migrated).map(key=>[key,migrated[key]])):migrated;
    if(M.native && state.version<4)
      input.rearExtent=(input.length??M.defaults.length)-(input.frontOverhang??M.defaults.frontOverhang);
    const c=M.validate(input);
    if(M.native)for(const [key,,min,max,step] of fields) {
      if(c[key]<min || c[key]>max || (step>=1 && Math.abs((c[key]-min)/step-Math.round((c[key]-min)/step))>1e-8))
        throw Error(key+': 설정 범위를 확인하세요.');
    }
    if(!Array.isArray(state.obstacles)||state.obstacles.length>180)throw Error('장애물은 최대 180개까지 가능합니다.');
    const obs=state.obstacles.map((o,i)=>{
      if(!['circle','box'].includes(o.shape)||![o.x,o.y,o.w,o.h].every(Number.isFinite)||
        o.x<0||o.x>(c.trackLayout==='loop'?13:c.courseLength)||o.y<(c.trackLayout==='loop'?0:-1.5)||o.y>(c.trackLayout==='loop'?4:1.5)||o.w<.04||o.w>2||o.h<.04||o.h>3)throw Error('장애물 '+(i+1)+'의 형상·좌표·크기를 확인하세요.');
      return {id:'o'+i,shape:o.shape,x:o.x,y:o.y,w:o.w,h:o.shape==='circle'?o.w:o.h};
    });
    const marks=state.markers??[];
    if(!Array.isArray(marks)||marks.length>180)throw Error('통과 영역은 최대 180개까지 가능합니다.');
    const markerList=marks.map((m,i)=>{
      if(!m || ![m.x,m.y,m.w].every(Number.isFinite) ||
          m.x<0 || m.x>(c.trackLayout==='loop'?13:c.courseLength) ||
          m.y<(c.trackLayout==='loop'?0:-1.5) || m.y>(c.trackLayout==='loop'?4:1.5) ||
          m.w<.04 || m.w>2)throw Error('통과 영역 '+(i+1)+'의 좌표·지름을 확인하세요.');
      return {id:'m'+i,shape:'marker',x:m.x,y:m.y,w:m.w};
    });
    return {config:c,obstacles:obs,markers:marksEnabled?markerList:[]};
  }
  function snapshot() {
    const settings={...config};
    if(M.native)for(const key of ['departure','range','angularStep','memoryTime','memoryDistance','targetWeight','previousWeight','entryReferenceWeight','approachSpeed','corridorLookahead','laneTrustSeconds','laneTrustLength','laneTrustOffset','laneTrustAngle'])delete settings[key];
    return {version:M.native?5:2,config:settings,obstacles:objects,...(marksEnabled?{markers}: {})};
  }
  function save() {
    const data=snapshot();
    const summary={scene:data,summary:sim.summary()};
    if(JSON.stringify(summary).length<15500)window.openai?.setWidgetState?.({modelContent:summary,privateContent:{selected,jetsonRatio}})?.catch(()=>{});
  }
  if(initial && marksEnabled && Array.isArray(initial.markers)) {
    markers=initial.markers.map((m,i)=>({...m,id:'m'+i,shape:'marker'}));
  }
  if(savedModel?.scene)try {const s=parseScene(savedModel.scene);config=s.config;objects=s.obstacles;markers=s.markers;selected=saved?.privateContent?.selected||null;}catch{}
  const colourProbe=document.createElement('span');colourProbe.hidden=true;colourProbe.setAttribute('aria-hidden','true');root.append(colourProbe);
  // Resolve nested theme tokens / light-dark() through CSS before using Canvas2D.
  function colour(name) {colourProbe.style.color=`var(${name})`;return getComputedStyle(colourProbe).color;}
  function palette() {return {fg:colour('--foreground'),muted:colour('--muted-foreground'),bg:colour('--background'),grid:colour('--border'),
    neutral:colour('--muted'),route:colour('--viz-series-1'),hit:colour('--viz-series-2'),memory:colour('--viz-series-3'),danger:colour('--destructive')};}
  function matchedProfile() {
    return planningProfiles.find(p=>Object.entries(p.config).every(([key,value])=>typeof value==='number'?Math.abs(config[key]-value)<1e-9:config[key]===value));
  }
  function profileSettings() {
    if(M.native && !el('[data-out="policy-source"]').dataset.stale) {
      el('[data-out="policy-source"]').textContent=`실차 YAML 기준 · RGB ${config.rgbHz} / Depth ${config.depthHz} / 계획 ${config.planHz} / 구동 ${config.controlHz} Hz · 정상 입력 · 정책 ${M.vehicleProfile.fingerprint}`;
    }
    if(!planningProfiles.length || (M.native && frenet)) {
      el('[data-field="planning-profile"]').closest('.viz-controls').hidden=true;
      el('[data-out="profile-detail"]').textContent='VFH 기본 설정 · '+config.speed.toFixed(2)+'m/s · 계획 '+config.horizon.toFixed(2)+'m · 차선 이탈 한도 없음 · 마킹은 평가 전용';
      return;
    }
    const select=el('[data-field="planning-profile"]');select.replaceChildren();
    select.closest('label').hidden=false;
    for(const preset of planningProfiles){const option=document.createElement('option');option.value=preset.id;option.textContent=preset.label;select.append(option);}
    const match=matchedProfile();
    if(!match){const option=document.createElement('option');option.value='custom';option.textContent='사용자 설정';option.disabled=true;select.append(option);}
    select.value=match?.id||'custom';
    if(M.native) {
      el('[data-out="profile-detail"]').textContent=`${match?.label||'사용자 설정'} · C++ VFH · ${config.planHz}Hz · 목표 ${config.speed.toFixed(2)}m/s · 마킹은 평가 전용`;
      return;
    }
    el('[data-out="profile-detail"]').textContent=`${match?.label||'사용자 설정'} · 계획 ${config.planHz}Hz · 경로 ${config.horizon}m · 검사 ${(config.sampleStep*100).toFixed(0)}cm · 각도 ${config.angularStep}° · 범위 ${config.range}m · 목표 ${config.speed.toFixed(1)}m/s`;
  }
  function settings() {
    const area=el('[data-area="settings"]');area.replaceChildren();
    for(const [key,label,min,max,step]of fields) {
      const l=document.createElement('label');l.className='form-label';l.textContent=label;
      const i=document.createElement('input');i.className='form-control';i.type='number';i.min=min;i.max=max;i.step=M.native && step<1?'any':step;i.value=config[key];i.dataset.param=key;
      if(key==='courseLength' && config.trackLayout==='loop'){i.value=M.trackLength.toFixed(2);i.disabled=true;}
      l.append(i);area.append(l);
    }
    el('[data-field="occlusion"]').checked=config.occlusion;
    el('[data-field="lane-visible"]').checked=config.laneVisible;
    el('[data-field="jetson-ratio"]').value=jetsonRatio;
    el('[data-object="x"]').max=config.trackLayout==='loop'?13:config.courseLength;
    el('[data-object="y"]').min=config.trackLayout==='loop'?0:-1.5;
    el('[data-object="y"]').max=config.trackLayout==='loop'?4:1.5;
    el('[data-canvas="world"]').setAttribute('aria-label',config.trackLayout==='loop'?'순환 트랙 장애물 편집 지도':'직선 코스 장애물 편집 지도');
    el('[data-out="terrain-size"]').textContent=M.native?
      `terrain 폭 ${config.terrainWidth}m · 전방 ${config.terrainForward}m`:'가로 1.2m · 전방 3m';
    profileSettings();
  }
  function parameterLock() {
    for(const input of all('[data-param]'))input.disabled=running || (input.dataset.param==='courseLength' && config.trackLayout==='loop');
    el('[data-act="apply"]').disabled=running;
    el('[data-act="track-tuning"]').disabled=running;
    el('[data-field="planning-profile"]').disabled=running;
  }
  function stopFrames() {running=false;parameterLock();if(frameRequest!==null)cancelAnimationFrame(frameRequest);frameRequest=null;lastFrame=0;}
  function reset() {stopFrames();el('[data-act="play"]').textContent='주행 시작';sim=new M.Simulation(config,objects, ...(marksEnabled?[markers]:[]));accumulator=0;lastPerfAt=-Infinity;renderTimes.length=0;draw();}
  function objectFields() {
    const o=selectedItem();
    el('[data-out="selected"]').textContent=o?`${o.shape==='marker'?'통과 영역 '+(markers.indexOf(o)+1):o.shape==='circle'?'원형':'사각형'} · X ${o.x.toFixed(2)}m · Y ${o.y.toFixed(2)}m · 지름/길이 ${o.w.toFixed(2)}m`:`선택 없음 · 장애물 ${objects.length}개`;
    for(const input of all('[data-object]')){input.disabled=!o||(o.shape!=='box'&&input.dataset.object==='h');input.value=o?o[input.dataset.object]:'';}
  }
  function mapBounds() {
    return config.trackLayout==='loop'?{xmin:-.4,xmax:13.4,ymin:-.4,ymax:4.4}:
      {xmin:-.5,xmax:config.courseLength,ymin:-1.5,ymax:1.5};
  }
  function worldToCanvas(p) {
    const bounds=mapBounds();
    return {x:worldMap.left+(p.x-bounds.xmin)*worldMap.scale,y:worldMap.top+(bounds.ymax-p.y)*worldMap.scale};
  }
  function canvasToWorld(event) {
    const rect=el('[data-canvas="world"]').getBoundingClientRect(),bounds=mapBounds();
    return {x:M.clamp(bounds.xmin+(event.clientX-rect.left-worldMap.left)/worldMap.scale,0,config.trackLayout==='loop'?13:config.courseLength),
      y:M.clamp(bounds.ymax-(event.clientY-rect.top-worldMap.top)/worldMap.scale,
        config.trackLayout==='loop'?0:-1.4,config.trackLayout==='loop'?4:1.4)};
  }
  function path(ctx,points,transform,stroke,width=1,dash=[]) {
    if(!points?.length)return;ctx.beginPath();points.forEach((p,i)=>{const q=transform(p);i?ctx.lineTo(q.x,q.y):ctx.moveTo(q.x,q.y);});
    ctx.strokeStyle=stroke;ctx.lineWidth=width;ctx.setLineDash(dash);ctx.stroke();ctx.setLineDash([]);
  }
  function worldDraw(pal) {
    const canvas=el('[data-canvas="world"]'),width=canvas.clientWidth,dpr=window.devicePixelRatio||1;
    const bounds=mapBounds(),loop=config.trackLayout==='loop';
    const scale=(width-55)/(bounds.xmax-bounds.xmin);
    worldMap={left:35,top:18,scale,height:Math.round(scale*(bounds.ymax-bounds.ymin)+48)};
    canvas.width=Math.round(width*dpr);canvas.height=Math.round(worldMap.height*dpr);canvas.style.height=worldMap.height+'px';
    const ctx=canvas.getContext('2d');ctx.scale(dpr,dpr);ctx.clearRect(0,0,width,worldMap.height);
    ctx.font='12px sans-serif';ctx.fillStyle=pal.fg;
    const tickStep=loop?1:Math.ceil(config.courseLength/8);
    for(let x=0;x<=(loop?13:config.courseLength);x+=tickStep){
      path(ctx,[{x,y:bounds.ymin},{x,y:bounds.ymax}],worldToCanvas,pal.grid);
      const a=worldToCanvas({x,y:bounds.ymin});ctx.fillText(String(x),a.x-3,a.y+16);
    }
    for(let y=Math.ceil(bounds.ymin);y<=bounds.ymax;y++){
      path(ctx,[{x:bounds.xmin,y},{x:bounds.xmax,y}],worldToCanvas,pal.grid);
      const a=worldToCanvas({x:bounds.xmin,y});ctx.fillText(String(y),3,a.y+4);
    }
    ctx.fillText('X(m)',Math.max(35,width-50),worldMap.height-2);ctx.fillText('Y(m)',0,12);
    if(loop){
      path(ctx,M.trackBoundary(0),worldToCanvas,pal.neutral,scale);
      for(const side of [-1,1])path(ctx,M.trackBoundary(side*(config.laneWidth/2+.015)),worldToCanvas,pal.fg,.03*scale);
      path(ctx,M.trackBoundary(0),worldToCanvas,pal.muted,1,[4,4]);
      path(ctx,[{x:11.5,y:.15},{x:11.5,y:.85}],worldToCanvas,pal.fg,2);
      for(const [text,x,y] of [['출발 →',10.5,.15],['← 지그재그 콘',9.1,4.1],['← 유도 콘',4.8,4.1]]){
        const p=worldToCanvas({x,y});ctx.fillStyle=pal.fg;ctx.fillText(text,p.x,p.y);
      }
    }else{
      const lane=worldToCanvas({x:0,y:config.laneWidth/2});ctx.globalAlpha=.35;ctx.fillStyle=pal.neutral;
      ctx.fillRect(lane.x,lane.y,config.courseLength*scale,config.laneWidth*scale);ctx.globalAlpha=1;
      for(const y of [-config.laneWidth/2,config.laneWidth/2])path(ctx,[{x:0,y},{x:config.courseLength,y}],worldToCanvas,pal.fg,1);
    }
    const half=M.native?config.terrainWidth/2:.6, forward=M.native?config.terrainForward:3;
    const viewport=[{x:0,y:half},{x:forward,y:half},{x:forward,y:-half},{x:0,y:-half},{x:0,y:half}].map(p=>M.world(p,sim.pose));
    ctx.save();ctx.beginPath();ctx.rect(worldMap.left,worldMap.top,(bounds.xmax-bounds.xmin)*scale,(bounds.ymax-bounds.ymin)*scale);ctx.clip();
    path(ctx,viewport,worldToCanvas,pal.route,1,[4,3]);
    for(const o of objects) {
      const q=worldToCanvas(o);ctx.fillStyle=pal.neutral;ctx.strokeStyle=o.id===selected?pal.fg:pal.muted;ctx.lineWidth=o.id===selected?2:1;
      ctx.beginPath();if(o.shape==='circle')ctx.arc(q.x,q.y,o.w/2*worldMap.scale,0,Math.PI*2);
      else ctx.rect(q.x-o.w/2*worldMap.scale,q.y-o.h/2*worldMap.scale,o.w*worldMap.scale,o.h*worldMap.scale);
      ctx.fill();ctx.stroke();
    }
    ctx.fillStyle=pal.memory;ctx.globalAlpha=.7;
    for(const p of sim.planner.memory.values()){const q=worldToCanvas(p);ctx.fillRect(q.x-1,q.y-1,2,2);}ctx.globalAlpha=1;
    ctx.fillStyle=pal.hit;for(const p of sim.observation.hits){const q=worldToCanvas(p);ctx.fillRect(q.x-1.5,q.y-1.5,3,3);}
    path(ctx,sim.trace,worldToCanvas,pal.route,2);
    path(ctx,sim.result.path,worldToCanvas,pal.route,1.5,[4,3]);
    path(ctx,sim.result.referencePath,worldToCanvas,pal.muted,1.2,[2,3]);
    markers.forEach((m,i)=>{
      const q=worldToCanvas(m),passed=i<sim.markerIndex,active=i===sim.markerIndex;
      ctx.strokeStyle=passed?'#16803c':'#9333aa';
      ctx.fillStyle=ctx.strokeStyle;
      ctx.lineWidth=m.id===selected?3:active?2:1;
      ctx.beginPath();ctx.arc(q.x,q.y,m.w*scale/2,0,Math.PI*2);
      ctx.globalAlpha=.18;ctx.fill();ctx.globalAlpha=1;ctx.stroke();
      ctx.fillText(passed?'✓'+(i+1):String(i+1),q.x+m.w*scale/2+3,q.y-3);
    });
    const car=M.corners(sim.pose,config);car.push(car[0]);path(ctx,car,worldToCanvas,sim.collided?pal.danger:pal.fg,2);
    path(ctx,[sim.pose,M.world({x:.30,y:0},sim.pose)],worldToCanvas,pal.fg,2);
    ctx.restore();
  }
  function bevDraw(pal) {
    const canvas=el('[data-canvas="bev"]'),ctx=canvas.getContext('2d');ctx.clearRect(0,0,120,300);
    if(M.native) {
      ctx.fillStyle=pal.neutral;ctx.fillRect(0,0,120,300);
      const g=sim.result.grid, sx=120/(g.height*g.res),sy=300/(g.width*g.res);
      const toPixel=p=>({x:120-(p.y-g.y0)*sx,y:300-(p.x-g.x0)*sy});
      const pt=p=>toPixel(M.local(p,g.pose));
      ctx.fillStyle=pal.hit;
      for(const index of sim.result.hazards) {
        const p=toPixel({x:g.x0+(index%g.width+1)*g.res,y:g.y0+(Math.floor(index/g.width)+1)*g.res});
        ctx.fillRect(p.x,p.y,g.res*sx,g.res*sy);
      }
      ctx.fillStyle=pal.memory;
      for(const index of sim.result.retainedHazards||[]) {
        const p=toPixel({x:g.x0+(index%g.width+1)*g.res,y:g.y0+(Math.floor(index/g.width)+1)*g.res});
        ctx.fillRect(p.x,p.y,g.res*sx,g.res*sy);
      }
      const body=M.corners(sim.pose,config);body.push(body[0]);path(ctx,body,pt,pal.fg,.8);
      path(ctx,sim.result.referencePath,pt,pal.fg,.9,[2,2]);
      path(ctx,sim.result.path,pt,pal.route,1.5);
      return;
    }
    ctx.fillStyle=pal.neutral;ctx.globalAlpha=.4;ctx.fillRect(0,0,120,300);ctx.globalAlpha=1;
    const pt=p=>{const q=M.local(p,sim.pose);return {x:60-q.y*100,y:300-q.x*100};};
    // Ray fans expose only measured free space; the unobserved region remains shaded.
    ctx.fillStyle=pal.bg;
    const rays=sim.observation.rays;
    for(let i=1;i<rays.length;i++) {
      const a=rays[i-1],b=rays[i];ctx.beginPath();ctx.moveTo(60,300);
      ctx.lineTo(60-a.distance*Math.sin(a.angle)*100,300-a.distance*Math.cos(a.angle)*100);
      ctx.lineTo(60-b.distance*Math.sin(b.angle)*100,300-b.distance*Math.cos(b.angle)*100);ctx.closePath();ctx.fill();
    }
    ctx.strokeStyle=pal.grid;ctx.lineWidth=.5;
    for(let x=10;x<120;x+=10){ctx.beginPath();ctx.moveTo(x,0);ctx.lineTo(x,300);ctx.stroke();}
    for(let y=0;y<300;y+=10){ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(120,y);ctx.stroke();}
    for(const y of [-config.laneWidth/2,config.laneWidth/2])path(ctx,[{x:sim.pose.x-4,y},{x:sim.pose.x+5,y}],pt,pal.fg,.7);
    ctx.fillStyle=pal.memory;for(const p of sim.planner.memory.values()){const q=pt(p);ctx.fillRect(Math.floor(q.x),Math.floor(q.y),1,1);}
    ctx.fillStyle=pal.hit;for(const p of sim.observation.hits)ctx.fillRect(p.px,p.py,1,1);
    path(ctx,sim.result.path,pt,pal.route,1.1);
    path(ctx,sim.result.referencePath,pt,pal.muted,.7,[2,2]);
    const car=M.corners(sim.pose,config);car.push(car[0]);path(ctx,car,pt,pal.fg,1);
    const y=300-config.range*100;ctx.setLineDash([2,2]);ctx.strokeStyle=pal.muted;ctx.beginPath();ctx.moveTo(0,y);ctx.lineTo(120,y);ctx.stroke();ctx.setLineDash([]);
  }
  function histDraw(pal) {
    const cv=el('[data-canvas="hist"]'),w=cv.clientWidth,dpr=window.devicePixelRatio||1;cv.width=w*dpr;cv.height=115*dpr;
    const ctx=cv.getContext('2d');ctx.scale(dpr,dpr);ctx.clearRect(0,0,w,115);
    const left=30,right=w-12,top=10,bottom=83,hist=sim.result.histogram,dx=(right-left)/hist.length;
    for(let i=0;i<hist.length;i++) {ctx.fillStyle=hist[i].blocked?pal.hit:pal.neutral;ctx.globalAlpha=hist[i].blocked?.6:1;
      ctx.fillRect(left+i*dx,bottom-hist[i].density*(bottom-top),dx+.2,hist[i].density*(bottom-top));}ctx.globalAlpha=1;
    ctx.strokeStyle=pal.grid;ctx.strokeRect(left,top,right-left,bottom-top);
    ctx.font='12px sans-serif';ctx.fillStyle=pal.fg;ctx.fillText('밀도',0,18);
    const sectorLimit=M.native?75:90;
    for(const a of [-sectorLimit,0,sectorLimit])ctx.fillText(String(a)+'°',M.clamp(left+(a+sectorLimit)/(2*sectorLimit)*(right-left)-10,left,right-28),103);
    const selectedAngle=sim.planner.lastDirection-(M.native?sim.result.grid.pose.yaw:sim.pose.yaw);const x=M.clamp(left+(selectedAngle/M.DEG+sectorLimit)/(2*sectorLimit)*(right-left),left,right);
    ctx.strokeStyle=pal.route;ctx.lineWidth=2;ctx.beginPath();ctx.moveTo(x,top);ctx.lineTo(x,bottom);ctx.stroke();
  }
  let lastStatus='';
  function performanceDraw(force=false) {
    const now=performance.now();if(!force&&now-lastPerfAt<500)return;lastPerfAt=now;
    if(M.native) {
      const stats=sim.performanceStats(), body=el('[data-out="perf-rows"]');
      body.replaceChildren();
      for(const [key,label] of [['planner',frenet?'C++ 프레넷 계획':'C++ VFH 계획'],['hazard','C++ hazard 확인·기억'],['sensor','브라우저 센서 모사'],['pipeline','센서 + 로컬 왕복 + 계획'],['integration','차량 모사 / 0.01초']]) {
        const row=document.createElement('tr'),sample=stats[key];
        for(const value of [label,sample.count?`${sample.meanMs.toFixed(3)} / ${sample.p95Ms.toFixed(3)} ms`:'—']) {
          const cell=document.createElement('td');cell.textContent=value;row.append(cell);
        }
        body.append(row);
      }
      el('[data-out="perf-source"]').textContent=`Mac 로컬 측정 · 워밍업 20회 제외 · 계획 ${stats.planner.count}회`;
      const cpu=(stats.planner.meanMs+stats.hazard.meanMs)*config.planHz/10;
      el('[data-out="perf-budget"]').textContent=stats.planner.count?
        `필터 + 계획의 단일 코어 시간 환산 ${cpu.toFixed(2)}% · ${config.planHz}Hz`:'주행 또는 0.1초 진행으로 표본을 수집합니다.';
      return;
    }
    const live=sim.performanceStats(),matches=baseline&&baseline.fingerprint===M.fingerprint(config,objects);
    const useLive=live.planner.count>=10,stats=useLive?live:matches?baseline.stages:null;
    const source=useLive?`현재 브라우저 · 워밍업 20회 제외 · 계획 ${live.planner.count}회`:
      matches?`${baseline.host.cpu} · ${baseline.host.memoryGiB}GB · Node ${baseline.method.runs}회 저장 실측 · 현재 배치 일치`:
      `현재 배치 측정 대기 · 주행 후 워밍업 20회 제외 · 표본 ${live.planner.count}/10회`;
    el('[data-out="perf-source"]').textContent=source;
    const body=el('[data-out="perf-rows"]');body.replaceChildren();
    for(const [key,label]of [['planner','회피 계획 / 주기'],['activePlanner','장애물 구간 계획 / 주기'],['sensor','센서 모사 / 주기'],['integration','주행·충돌 평가 / 0.01초'],['pipeline','센서+계획 / 주기']]) {
      const r=document.createElement('tr'),s=stats?.[key],values=[label,s?.count?`${s.meanMs.toFixed(3)} / ${s.p95Ms.toFixed(3)} ms`:'—',
        s?.count?`${(s.meanMs*jetsonRatio).toFixed(3)} / ${(s.p95Ms*jetsonRatio).toFixed(3)} ms`:'—'];
      for(const value of values){const td=document.createElement('td');td.textContent=value;r.append(td);}body.append(r);
    }
    const p=stats?.planner;
    el('[data-out="perf-budget"]').textContent=p?.count?
      `계획 ${config.planHz}Hz · 단일 코어 점유 환산: 맥 ${(p.meanMs*config.planHz/10).toFixed(1)}% / Jetson 가정 ${(p.meanMs*jetsonRatio*config.planHz/10).toFixed(1)}% · Jetson P95 ${(p.p95Ms*jetsonRatio/(1000/config.planHz)*100).toFixed(1)}% 주기 예산`:'주행을 시작하면 현재 배치의 측정값을 표시합니다.';
    if(renderTimes.length)el('[data-out="perf-budget"]').textContent+=` · 화면 평균 ${M.timingStats(renderTimes).meanMs.toFixed(2)}ms/그리기(환산 제외)`;
  }
  function draw() {
    const started=performance.now();
    const pal=palette();worldDraw(pal);bevDraw(pal);if(!frenet)histDraw(pal);
    el('[data-out="clock"]').textContent=config.trackLayout==='loop'?`${sim.time.toFixed(2)}초 · 한 바퀴 ${Math.max(0,sim.lapProgress).toFixed(2)}/${M.trackLength.toFixed(2)}m · ${sim.pose.v.toFixed(2)}m/s`:`${sim.time.toFixed(2)}초 · 전방 ${sim.pose.x.toFixed(2)}m · ${sim.pose.v.toFixed(2)}m/s`;
    let status=sim.collided?'충돌 · 주행 중단':sim.departureViolation&&!M.native&&!sim.result.laneMissing?'이탈 한도 초과 · 주행 중단':sim.done?
      (sim.summary().laneReturned?`${config.trackLayout==='loop'?'한 바퀴':config.courseLength+'m'} 완료 · 차체 전체 차선 내부`:`${config.trackLayout==='loop'?'한 바퀴':config.courseLength+'m'} 완료 · 차선 이탈 남음`):sim.result.status;
    const markerStatus=el('[data-out="markers"]');
    markerStatus.hidden=!marksEnabled || !markers.length;
    markerStatus.textContent=markers.length?
      `평가용 통과 영역 ${sim.markerIndex}/${markers.length} · ${sim.markerIndex<markers.length?`다음: ${sim.markerIndex+1}번`:'모두 통과'}`:'';
    if(sim.done && sim.markerIndex<markers.length)status+=' · 미통과 영역 남음';
    status+=` · 최대 이탈 ${sim.maxDeparture.toFixed(2)}m`;
    if(sim.done)status+=` · 차선 중심 오차 ${(config.trackLayout==='loop'?M.projectTrack(sim.pose).distance:Math.abs(sim.pose.y)).toFixed(2)}m`;
    if(status!==lastStatus){el('[data-out="status"]').textContent=status;lastStatus=status;}
    el('[data-out="visibility"]').textContent=M.native?`위험 셀 원본 ${sim.result.raw} · 확정 ${sim.result.confirmed} · 대기 ${sim.result.pending} · 깜빡임 유지 ${sim.result.heldHazards??0} · 근거리 기억 ${sim.result.nearBodyHazards??0}`:`관측 ${sim.observation.hits.length}점 · 기억 ${sim.planner.memory.size}점`;
    el('[data-out="limit"]').textContent=M.native?`차선 ${sim.result.laneTrust} · 차선 방향 기억 ${sim.result.hasLaneMemory?sim.result.memoryAge.toFixed(1)+'초':'없음'}`:`차선 ${config.laneVisible?'관측':'미관측'} · 차선 경계 밖 ${config.departure.toFixed(2)}m 제한`;
    el('[data-out="course-label"]').textContent=config.trackLayout==='loop'?`도면 순환 트랙 · 13×4m · 차선 안쪽 폭 ${config.laneWidth.toFixed(2)}m · 코너 R0.50m · 클릭 추가 / 드래그 이동`:`${config.courseLength}m 코스 · 클릭 배치 / 드래그 이동·콘 배열`;
    el('[data-object="x"]').max=config.trackLayout==='loop'?13:config.courseLength;
    el('[data-object="y"]').min=config.trackLayout==='loop'?0:-1.5;
    el('[data-object="y"]').max=config.trackLayout==='loop'?4:1.5;
    objectFields();
    renderTimes.push(performance.now()-started);if(renderTimes.length>4096)renderTimes.shift();performanceDraw();
  }
  function fail(e) {el('[data-out="error"]').textContent=e.message||String(e);}
  root.addEventListener('click',event=>{
    const action=event.target.closest('[data-act]')?.dataset.act;if(!action)return;
    el('[data-out="error"]').textContent='';
    try {
      if(action==='play'){if(sim.done||sim.collided)reset();if(running){stopFrames();el('[data-act="play"]').textContent='주행 계속';save();}
        else{running=true;parameterLock();lastFrame=0;el('[data-act="play"]').textContent='일시 정지';frameRequest=requestAnimationFrame(frame);}}
      if(action==='step'){stopFrames();el('[data-act="play"]').textContent='주행 계속';for(let i=0;i<10;i++)sim.step();draw();save();}
      if(action==='reset'){reset();save();}
      if(action==='delete'){el('[data-field="preset"]').value='user';objects=objects.filter(o=>o.id!==selected);markers=markers.filter(m=>m.id!==selected);selected=null;reset();save();}
      if(action==='clear'){el('[data-field="preset"]').value='user';objects=[];selected=null;reset();save();}
      if(action==='track-tuning' && M.native && !frenet) {
        config=M.validate({...config,...M.defaults,trackLayout:config.trackLayout,courseLength:config.courseLength});
        settings();reset();save();el('[data-out="error"]').textContent='';
      }
      if(action==='apply'){
        const next={...config};
        for(const input of all('[data-param]')) {
          if(M.native && !input.disabled && (!input.value || !input.checkValidity()))throw Error(input.dataset.param+': 설정 범위를 확인하세요.');
          if(!input.disabled)next[input.dataset.param]=Number(input.value);
        }
        next.occlusion=el('[data-field="occlusion"]').checked;config=M.validate(next);settings();reset();save();
      }
      if(action==='export'){el('[data-field="json"]').value=JSON.stringify(snapshot(),null,2);el('[data-field="json"]').focus();el('[data-field="json"]').select();}
      if(action==='import'){const data=parseScene(JSON.parse(el('[data-field="json"]').value));config=data.config;objects=data.obstacles;markers=data.markers;selected=null;settings();reset();save();}
    }catch(e){fail(e);}
  });
  el('[data-field="preset"]').value=initial||savedModel?.scene?'user':'track';
  el('[data-field="preset"]').addEventListener('change',event=>{
    if(event.target.value==='user')return;
    const layout=event.target.value==='track'?'loop':'straight';
    if(M.native && layout!==config.trackLayout) {
      const preset=layout==='loop'?M.trackDefaults:M.defaults;
      for(const key of Object.keys(M.trackDefaults))config[key]=preset[key];
    }
    if(config.trackLayout!==layout)markers=[];
    config.trackLayout=layout;
    objects=M.presets(event.target.value);selected=null;settings();reset();save();
  });
  el('[data-field="planning-profile"]').addEventListener('change',event=>{
    const preset=planningProfiles.find(p=>p.id===event.target.value);if(!preset)return;
    try {config=M.validate({...config,...preset.config});settings();reset();save();el('[data-out="error"]').textContent='';}
    catch(error){fail(error);profileSettings();}
  });
  el('[data-field="lane-visible"]').addEventListener('change',event=>{
    config.laneVisible=event.target.checked;sim.c.laneVisible=config.laneVisible;
    sim.senseAndPlan();draw();save();
  });
  el('[data-field="jetson-ratio"]').addEventListener('input',event=>{
    const value=Number(event.target.value);if(!Number.isFinite(value)||value<.1||value>30){event.target.value=jetsonRatio;return;}
    jetsonRatio=value;performanceDraw(true);save();
  });
  for(const input of all('[data-object]'))input.addEventListener('change',()=>{
    const o=selectedItem();if(!o)return;
    const value=Number(input.value);if(!Number.isFinite(value)||value<Number(input.min)||value>Number(input.max)){objectFields();return;}
    o[input.dataset.object]=value;if(o.shape==='circle')o.h=o.w;reset();save();
  });
  const canvas=el('[data-canvas="world"]');
  canvas.addEventListener('pointerdown',event=>{
    const p=canvasToWorld(event),tool=el('[data-field="tool"]').value;
    stopFrames();el('[data-act="play"]').textContent='주행 시작';canvas.setPointerCapture(event.pointerId);
    if(tool!=='move')el('[data-field="preset"]').value='user';
    if(tool==='move') {
      const hit=[...objects,...markers].reverse().find(o=>o.shape!=='box'?Math.hypot(o.x-p.x,o.y-p.y)<Math.max(o.w/2,.07):Math.abs(o.x-p.x)<Math.max(o.w/2,.07)&&Math.abs(o.y-p.y)<Math.max(o.h/2,.07));
      selected=hit?.id||null;drag=hit?{type:'move',id:hit.id}:null;
    } else if(tool==='marker' && marksEnabled) {
      const w=M.clamp(Number(el('[data-field="marker-size"]').value)||.20,.04,2);
      if(markers.length<180) {
        const m={id:'m'+Date.now()+markers.length,shape:'marker',x:p.x,y:p.y,w};
        markers.push(m);selected=m.id;reset();
      }
    } else {
      const size=M.clamp(Number(el('[data-field="size"]').value)||.13,.04,1.5);
      const o={id:'o'+Date.now()+objects.length,shape:tool==='box'?'box':'circle',x:p.x,y:p.y,w:size,h:size};
      if(objects.length<180){objects.push(o);selected=o.id;}
      drag=tool==='chain'?{type:'chain',point:p,size}:null;reset();
    }
    draw();
  });
  canvas.addEventListener('pointermove',event=>{
    if(!drag)return;const p=canvasToWorld(event);
    if(drag.type==='move'){const o=selectedItem();if(!o || o.id!==drag.id)return;o.x=p.x;o.y=p.y;reset();}
    else {
      const dx=p.x-drag.point.x,dy=p.y-drag.point.y,len=Math.hypot(dx,dy),spacing=drag.size*.95;
      const count=Math.floor(len/spacing);
      if(count>0){for(let i=1;i<=count&&objects.length<180;i++)objects.push({id:'o'+Date.now()+objects.length,shape:'circle',x:drag.point.x+dx/len*spacing*i,y:drag.point.y+dy/len*spacing*i,w:drag.size,h:drag.size});
        drag.point={x:drag.point.x+dx/len*spacing*count,y:drag.point.y+dy/len*spacing*count};reset();}
    }
  });
  canvas.addEventListener('pointerup',()=>{drag=null;save();});canvas.addEventListener('pointercancel',()=>{drag=null;save();});
  function frame(time) {
    frameRequest=null;
    if(!running)return;
    const delta=lastFrame?Math.min(.08,(time-lastFrame)/1000):0;lastFrame=time;accumulator+=delta;
    try {while(accumulator>=.01&&!sim.done&&!sim.collided){sim.step(.01);accumulator-=.01;}}
    catch(error){stopFrames();el('[data-act="play"]').textContent='주행 계속';fail(error);return;}
    if(!M.native || time-lastCanvasFrame>=1000/config.previewHz || sim.done || sim.collided) {
      draw();lastCanvasFrame=time;
    }
    if(sim.done||sim.collided){stopFrames();el('[data-act="play"]').textContent='다시 주행';save();}
    else if(running)frameRequest=requestAnimationFrame(frame);
  }
  window.addEventListener('openai:set_globals',event=>{
    const state=event.detail?.globals?.widgetState,model=state?.modelContent||state;if(!model?.scene)return;
    // State saves are echoed by the host. Do not reset the vehicle on our own save.
    if(JSON.stringify(model.scene)===JSON.stringify(snapshot()))return;
    try {const data=parseScene(model.scene);config=data.config;objects=data.obstacles;markers=data.markers;selected=null;settings();reset();}catch{}
  });
  new ResizeObserver(()=>{if(sim)draw();}).observe(root);
  new MutationObserver(()=>{if(sim)draw();}).observe(document.documentElement,{attributes:true,attributeFilter:['class','style','data-theme']});
  if(M.native) {
    el('[data-field="jetson-ratio"]').closest('label').hidden=true;
    const timing=el('[data-field="jetson-ratio"]').closest('details');
    timing.querySelector('summary').textContent='Mac 처리 시간';
    timing.querySelector('th:last-child').closest('tr').lastElementChild.remove();
    timing.querySelector('.viz-controls').hidden=true;
    for(const child of timing.querySelectorAll(':scope > div.text-small:not([data-out])'))child.hidden=true;
    el('.sim-legend').textContent='빨강: 확정 장애물 · 회색: 미관측 · 파랑: 계획 경로 · 점선: 차선';
    el('[data-canvas="bev"]').setAttribute('aria-label','차량 전방 terrain과 선택 경로');
    if(frenet) {
      const histogram=el('[data-canvas="hist"]');
      histogram.previousElementSibling.hidden=true;
      histogram.hidden=true;
      for(const summary of root.querySelectorAll('summary'))
        if(summary.textContent.includes('VFH 설정'))summary.textContent='주행·차량·프레넷 설정';
    }
    root.querySelector('.sim-footer').textContent=frenet?'프레넷 궤적 · 차체 충돌 검사 · 이상적인 차량 위치':'차선 신뢰도·복귀 정책 실험 · C++ VFH와 hazard 필터 · 이상적인 차량 위치';
  }
  try {settings();reset();}catch(error){fail(error);return;}
  // Small read-only diagnostic surface for the local smoke check.
  root.vfhDiagnostics=()=>({summary:sim.summary(),scene:snapshot(),planningProfile:matchedProfile()?.id||'custom',bev:[el('[data-canvas="bev"]').width,el('[data-canvas="bev"]').height]});
})();
