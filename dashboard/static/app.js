'use strict';
const $ = id => document.getElementById(id);
const esc = value => String(value ?? '').replace(/[&<>"']/g, c => ({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
const icon = name => `<svg aria-hidden="true"><use href="#i-${name}"/></svg>`;
const number = (v, digits=1) => Number.isFinite(v) ? v.toFixed(digits) : '—';
const timeLabel = value => value ? new Date(value).toLocaleTimeString([], {hour:'2-digit', minute:'2-digit'}) : '—';
let state = null, selected = null, priority = '', status = 'PENDING', inFlight = false;
let queuedRefresh = false;
let queueSignature = '', detailSignature = '', activitySignature = '', toastTimer;

function toast(message) {
  $('toast').textContent = message; $('toast').hidden = false;
  clearTimeout(toastTimer); toastTimer = setTimeout(() => {$('toast').hidden = true;}, 4500);
}
async function api(path, body) {
  const response = await fetch(path, body === undefined ? {} : {method:'POST', headers:{'Content-Type':'application/json'}, body:JSON.stringify(body)});
  const data = await response.json();
  if (!response.ok) throw new Error(data.error || 'The request could not be completed.');
  return data;
}
async function refresh(force=false) {
  if (inFlight) {if(force)queuedRefresh=true;return;}
  inFlight = true;
  try {
    const query = new URLSearchParams(); if (priority) query.set('priority', priority); if (status) query.set('status', status);
    state = await api('/api/state?' + query);
    if (!state.scene_targets.some(t => t.task_id === selected)) selected = state.active_task?.task_id || state.pending_queue[0]?.task_id || state.scene_targets[0]?.task_id || null;
    render();
  } catch (error) {
    $('monitor-health').classList.add('warning'); $('monitor-health').innerHTML = '<span class="dot"></span>Dashboard connection lost';
  } finally { inFlight = false;if(queuedRefresh){queuedRefresh=false;refresh();} }
}
async function action(path, body={}) {
  try { await api(path, body); await refresh(true); }
  catch (error) {toast(error.message);}
}
function locationText(t) {
  if (!t.location) return 'Location unavailable';
  const axis = t.surface === 'WALL' ? 'h' : 'y';
  return `${t.active ? '' : 'Last observed · '}X ${number(t.location.x)} · ${axis.toUpperCase()} ${number(t.location[axis])} cm`;
}
function statusText(value) { return value.replaceAll('_', ' ').toLowerCase(); }
function taskCard(t) {
  return `<button class="task-card prio-${t.priority.toLowerCase()} status-${t.task_status.toLowerCase()} ${selected===t.task_id?'selected':''}" data-task="${esc(t.task_id)}" aria-pressed="${selected===t.task_id}">
    <span class="task-top"><span class="task-target">TARGET #${String(t.target_id).padStart(2,'0')}</span><span class="priority-badge">${esc(t.priority)} PRIORITY</span></span>
    <span class="task-name"><span class="category-dot"></span>${esc(t.label)}</span>
    <span class="task-location">${icon('pin')} ${esc(t.surface)} <span>·</span> ${esc(locationText(t))}</span>
    <span class="task-bottom"><span class="status-badge">${esc(t.navigation_progress?.label || statusText(t.task_status))}${!t.active?' · not visible':''}</span><span>${t.completion_source?`<span class="source-badge">${esc(t.completion_source)}</span> `:''}${timeLabel(t.detected_at)}</span></span></button>`;
}
function render() {
  const s=state, demo=s.mode==='DEMO';
  document.querySelectorAll('[data-mode]').forEach(b=>{b.classList.toggle('selected',b.dataset.mode===s.mode);b.setAttribute('aria-pressed',String(b.dataset.mode===s.mode));});
  for (const [key,value] of Object.entries(s.metrics)) $('metric-'+key).textContent=value;
  $('nav-count').textContent=s.metrics.open; $('queue-count').textContent=s.metrics.open;
  $('completion-caption').textContent = s.metrics.completed ? `${s.metrics.completed} ${demo?'simulated':'manually recorded'} completion${s.metrics.completed===1?'':'s'}` : 'No completions recorded';
  $('monitor-health').classList.toggle('warning', ['UNAVAILABLE','ATTENTION'].includes(s.monitoring.status));
  $('monitor-health').innerHTML=`<span class="dot"></span>${demo?'Demo available':s.monitoring.status==='AVAILABLE'?'Monitoring available':s.monitoring.status==='CONNECTING'?'Connecting camera':s.monitoring.status==='ATTENTION'?'Monitoring needs review':'Monitoring unavailable'}`;
  $('scene-mode').textContent=demo?'SIMULATED VIEW':'LIVE CAMERA'; $('scene-mode').className='badge '+(demo?'demo-badge':'live-badge');
  $('zone-label').textContent=demo?'Demo ward · Zone A':'Monitored area';
  $('scene-source').textContent=demo?'DEMO SCENE':'LIVE MONITORING';
  $('scene-caption').textContent=demo?'Illustrative scene · not a live camera':'Actual camera feed · estimated locations';
  $('scene-resolution').textContent=s.arena.frame_size.join(' × ');
  $('demo-scene').toggleAttribute('hidden',!demo); $('demo-toolbar').hidden=!demo;
  document.querySelector('.scene-topline').hidden=!demo;document.querySelector('.scene-bottom').hidden=!demo;
  const cameraReady=!demo && ['AVAILABLE','ATTENTION'].includes(s.monitoring.status);
  $('live-frame').hidden=!cameraReady; $('camera-unavailable').hidden=demo || cameraReady;
  $('camera-reason').textContent=s.monitoring.message;
  if(cameraReady && !$('live-frame').hasAttribute('src')) {
    $('live-frame').src='/api/stream.mjpg';
  }
  if(!cameraReady) $('live-frame').removeAttribute('src');
  $('scenario-title').textContent=s.demo.step===0?'Ready when you are':`Scenario · step ${s.demo.step} of ${s.demo.total_steps}`;
  $('scenario-description').textContent=s.demo.description;
  $('demo-play').innerHTML=icon(s.demo.playing?'pause':'play')+`<span>${s.demo.playing?'Pause scenario':s.demo.step===s.demo.total_steps?'Replay scenario':'Play scenario'}</span>`;
  $('footer-mode').textContent=demo?'DEMO · Simulated actions only':'LIVE · Observation & manual workflow only';
  renderQueue(); renderDetails(); renderActivity(); renderScene();
}
function renderQueue() {
  const s=state, active=s.active_task;
  const shown=s.tasks.filter(t=>t.task_status!=='IN_PROGRESS');
  const project=t=>[t.task_id,t.category,t.task_status,t.location,t.active,t.completion_source,t.surface,t.navigation_progress];
  const signature=JSON.stringify([shown.map(project),active&&project(active),priority,status,selected]);
  if(signature===queueSignature)return;queueSignature=signature;
  $('active-task').innerHTML=active?`<div class="active-task-wrap"><div class="active-label"><span class="dot"></span>IN PROGRESS · NOT INTERRUPTED</div>${taskCard(active)}</div>`:'';
  $('queue-label').textContent=status==='PENDING'?'PENDING QUEUE':'TASK HISTORY & QUEUE';
  $('queue-note').textContent='High first · oldest first';
  $('task-list').innerHTML=shown.length?shown.map(taskCard).join(''):`<div class="empty-state">${icon('check')}<h3>${priority||(status&&status!=='PENDING')?'No matching tasks':'A clear space. A fresh start.'}</h3><p>${priority||(status&&status!=='PENDING')?'Try a different priority or status filter.':s.mode==='DEMO'?'Play the scenario to see detection and prioritization in action.':'New confirmed detections will appear here automatically.'}</p></div>`;
  document.querySelectorAll('[data-task]').forEach(b=>b.addEventListener('click',()=>{selected=b.dataset.task;render();}));
}
function renderDetails() {
  const t=state.scene_targets.find(t=>t.task_id===selected);
  const signature=JSON.stringify([t,state.arena,state.mode]);if(signature===detailSignature)return;detailSignature=signature;
  $('detail-id').textContent=t?`/ #${String(t.target_id).padStart(2,'0')}`:'';
  if(!t){$('detail-content').innerHTML=`<div class="empty-state">${icon('scan')}<h3>Every target gets a considered plan.</h3><p>Select a cleaning task to review its location, estimated size and proposed coverage.</p></div>`;return;}
  const g=t.geometry, axis=t.surface==='WALL'?'height_cm':'depth_cm';
  const size=g?`${number(g.width_cm)} × ${number(g[axis])} cm · ${number(g.area_cm2)} cm²`:'Geometry unavailable';
  const reach=t.reachability==='UNKNOWN'?'Configuration / assessment required':t.reachability.replaceAll('_',' ').toLowerCase();
  const reachClass=t.reachability==='UNKNOWN'?'unknown':['REACHABLE','FULLY_REACHABLE'].includes(t.reachability)?'reachable':'partial';
  let buttons='';
  if(t.task_status==='PENDING')buttons=`<button class="button primary" data-transition="IN_PROGRESS">${icon('play')}${state.mode==='DEMO'?'Start simulated task':'Record task start'}</button><button class="button secondary" data-transition="UNREACHABLE">Flag unreachable</button>`;
  if(t.task_status==='IN_PROGRESS')buttons=(t.navigation_progress?'':`<button class="button primary" data-transition="COMPLETED">${icon('check')}${state.mode==='DEMO'?'Complete simulation':'Record manual completion'}</button>`)+`<button class="button secondary" data-transition="FAILED">Record failure</button>`;
  if(['FAILED','UNREACHABLE'].includes(t.task_status))buttons='<button class="button secondary" data-transition="PENDING">Return to queue</button>';
  const plan=t.plan||{}, reason=(plan.reasons||[])[0];
  $('detail-content').innerHTML=`<div class="detail-layout"><div><div class="detail-label">${esc(t.surface)} · ${esc(t.priority)} PRIORITY</div><div class="detail-name">${esc(t.label)}</div><div class="detail-info"><div><span>Physical location</span><strong>${esc(locationText(t))}</strong></div><div><span>Estimated size</span><strong>${esc(size)}</strong></div><div><span>Cleaning mode</span><strong>${esc(t.cleaning_mode?.replaceAll('_',' ')||'Awaiting assessment')}</strong></div><div><span>Detected at</span><strong>${timeLabel(t.detected_at)} ${state.mode==='DEMO'?'· scenario time':''}</strong></div></div><div class="plan-state ${reachClass}">${icon(['REACHABLE','FULLY_REACHABLE'].includes(t.reachability)?'check':'alert')}${esc(reach)}${state.mode==='DEMO'?' · simulated geometry':''}</div>${reason?`<p class="small-note">${esc(reason)}</p>`:''}<div class="trust-note">${t.task_status==='COMPLETED'?`${esc(t.completion_source)} completion · physical verification pending.`:state.mode==='DEMO'?'Synthetic target & illustrative planning geometry.':`Location ${esc(t.localization_status)}${t.active?'':' · last observed'}.`} No physical execution is initiated.</div><div class="detail-actions">${buttons}</div></div><div class="mini-arena"><div class="mini-arena-title"><span>${esc(t.surface)} · TARGET BOUNDARY</span><span>cm</span></div>${arenaSVG(t)}<div class="mini-arena-caption">${esc(arenaCaption(t))}<br>${t.surface==='FLOOR'&&(plan.coverage_segments||[]).length?'Dashed lines: proposed coverage, not a navigation route.':'No confirmed cleaning reachability is inferred.'}</div></div></div>`;
  document.querySelectorAll('[data-transition]').forEach(b=>b.addEventListener('click',()=>{
    if(state.mode==='LIVE' && b.dataset.transition==='COMPLETED' && !confirm('Record a MANUAL completion? This does not verify physical cleaning or operate any equipment.'))return;
    action('/api/tasks/'+encodeURIComponent(t.task_id)+'/status',{status:b.dataset.transition});
  }));
}
function arenaCaption(t) {
  const a=t.surface==='WALL'?state.arena.wall:state.arena.floor;
  return a?`${number(a.width_cm)} × ${number(a.depth_cm??a.height_cm)} cm · loaded calibration`:'Surface calibration unavailable';
}
function pathString(points,project) {
  if(!Array.isArray(points)||points.length<2)return '';
  if(!points.every(p=>Array.isArray(p)&&p.length===2&&p.every(Number.isFinite)))return '';
  return points.map((p,i)=>`${i?'L':'M'}${project(p).map(v=>v.toFixed(2)).join(',')}`).join(' ')+' Z';
}
function arenaSVG(t) {
  const wall=t.surface==='WALL',a=wall?state.arena.wall:state.arena.floor;
  if(!a || !['FLOOR','WALL'].includes(t.surface))return '<svg viewBox="0 0 300 160"><text x="150" y="80" text-anchor="middle" fill="#708aa4" stroke="none" font-size="10">Geometry unavailable</text></svg>';
  const w=a.width_cm,d=a.depth_cm??a.height_cm,project=p=>[23+p[0]/w*254,wall?137-p[1]/d*110:27+p[1]/d*110];
  let paths='';
  for(const target of state.scene_targets.filter(x=>x.surface===t.surface))for(const ring of target.geometry?.contours||[]){
    const path=pathString(ring.points,project);if(path)paths+=`<path d="${path}" fill="${esc(target.color)}" fill-opacity="${target.task_id===selected?'.33':'.12'}" stroke="${esc(target.color)}" stroke-width="${target.task_id===selected?'2':'1'}"/>`;
  }
  if(!wall)for(const segment of t.plan?.coverage_segments||[]) {
    const start=segment.start_pose,end=segment.end_pose;
    if(start&&end&&[start.x_cm,start.y_cm,end.x_cm,end.y_cm].every(Number.isFinite)){
      const p=project([start.x_cm,start.y_cm]),q=project([end.x_cm,end.y_cm]);
      paths+=`<path d="M${p.join(',')} L${q.join(',')}" stroke="#65d6cd" stroke-width="1.2" stroke-dasharray="3 3"/>`;
    }
  }
  return `<svg viewBox="0 0 300 164" aria-label="Calibrated ${esc(t.surface.toLowerCase())} target map"><defs><pattern id="mini-grid" width="25.4" height="22" patternUnits="userSpaceOnUse"><path d="M25.4 0H0V22" stroke="#30465b" stroke-width=".5" fill="none"/></pattern></defs><rect x="23" y="27" width="254" height="110" fill="url(#mini-grid)" stroke="#3a576d" stroke-width="1"/>${paths}<text x="23" y="17" fill="#69879f" stroke="none" font-size="7">${wall?'H ↑':'Y ↓'}</text><text x="273" y="153" fill="#69879f" stroke="none" font-size="7" text-anchor="end">X → ${number(w)} cm</text></svg>`;
}
function renderActivity() {
  const events=state.events.slice(0,12),signature=JSON.stringify(events.map(e=>[e.id,e.title]));
  if(signature===activitySignature)return;activitySignature=signature;
  $('activity-list').innerHTML=events.map(e=>`<div class="activity-item ${esc(e.kind)}"><span class="activity-node">${icon(e.kind==='completed'?'check':e.kind==='high'?'alert':e.kind==='started'?'play':'scan')}</span><div><div class="activity-text">${esc(e.title)}</div><div class="activity-meta"><span>${timeLabel(e.timestamp)}</span><span>${e.source==='SIMULATED'?'DEMO · SIMULATED':esc(e.source)}</span></div></div></div>`).join('');
}
function renderScene() {
  const demo=state.mode==='DEMO',visible=state.scene_targets.filter(t=>t.active && (!demo || t.task_status!=='COMPLETED'));
  $('scene-count').textContent=`${visible.length} visible target${visible.length===1?'':'s'}`;
  const overlay=$('scene-overlay');overlay.setAttribute('viewBox',`0 0 ${demo?'1280 720':state.arena.frame_size.join(' ')}`);
  if(!demo){overlay.innerHTML='';return;} // LIVE overlays are the original test-vision pixels.
  let content='';
  if(!demo&&!['AVAILABLE','ATTENTION'].includes(state.monitoring.status)){overlay.innerHTML='';return;}
  for(const t of visible) {
    let rings,project;
    if(demo){
      const a=t.surface==='WALL'?state.arena.wall:state.arena.floor;if(!a)continue;
      rings=t.geometry?.contours||[];
      project=p=>t.surface==='WALL'?[270+p[0]/a.width_cm*730,280-p[1]/a.height_cm*193]:[270+(106-270)*(p[1]/a.depth_cm)+(730+(1074-730)*(p[1]/a.depth_cm))*p[0]/a.width_cm,280+p[1]/a.depth_cm*380];
    } else {rings=t.image_geometry?.contours||[];project=p=>p;}
    let anchor=null;
    for(const ring of rings){
      const path=pathString(ring.points,project);if(!path)continue;
      const coords=ring.points.map(project);anchor=coords.reduce((best,p)=>p[1]<best[1]?p:best,coords[0]);
      content+=`<path d="${path}" fill="${esc(t.color)}" fill-opacity=".23" stroke="${esc(t.color)}" stroke-width="${selected===t.task_id?'4':'2.5'}" stroke-linejoin="round" data-scene-task="${esc(t.task_id)}"/>`;
    }
    if(anchor){
      const label=`#${String(t.target_id).padStart(2,'0')}  ${t.priority} · ${t.surface}`;
      const x=Math.max(8,Math.min((demo?1280:state.arena.frame_size[0])-310,anchor[0]-20)),y=Math.max(42,anchor[1]-57);
      content+=`<g data-scene-task="${esc(t.task_id)}" role="button" tabindex="0" aria-label="Select target ${t.target_id}"><rect x="${x}" y="${y}" width="300" height="44" rx="5" fill="#0d1929" stroke="${esc(t.color)}" stroke-width="1"/><circle cx="${x+18}" cy="${y+22}" r="4" fill="${esc(t.color)}" stroke="none"/><text x="${x+34}" y="${y+29}" fill="${esc(t.color)}" stroke="none" font-size="20" font-weight="600" font-family="system-ui">${esc(label)}</text></g>`;
    }
  }
  overlay.innerHTML=content;
  overlay.querySelectorAll('[data-scene-task]').forEach(e=>{const choose=()=>{selected=e.dataset.sceneTask;render();};e.addEventListener('click',choose);e.addEventListener('keydown',event=>{if(event.key==='Enter'||event.key===' '){event.preventDefault();choose();}});});
}

document.querySelectorAll('[data-mode]').forEach(b=>b.addEventListener('click',()=>action('/api/mode',{mode:b.dataset.mode})));
document.querySelectorAll('[data-priority]').forEach(b=>b.addEventListener('click',()=>{priority=b.dataset.priority;document.querySelectorAll('[data-priority]').forEach(x=>x.classList.toggle('selected',x===b));refresh(true);}));
$('status-filter').addEventListener('change',event=>{status=event.target.value;refresh(true);});
function resetDemoFilters(){selected=null;priority='';status='PENDING';$('status-filter').value='PENDING';document.querySelectorAll('[data-priority]').forEach(b=>b.classList.toggle('selected',b.dataset.priority===''));}
$('demo-play').addEventListener('click',async()=>{try{if(!state)return;if(state.demo.step===state.demo.total_steps){resetDemoFilters();await api('/api/demo/reset',{});}await action('/api/demo/'+(state.demo.playing?'pause':'play'));}catch(error){toast(error.message);}});
$('demo-step').addEventListener('click',()=>action('/api/demo/step'));
$('demo-reset').addEventListener('click',()=>{resetDemoFilters();action('/api/demo/reset');});
$('retry-live').addEventListener('click',()=>{$('live-frame').removeAttribute('src');action('/api/monitor/retry');});
$('live-frame').addEventListener('error',()=>{if(state?.mode!=='LIVE')return;$('live-frame').removeAttribute('src');$('camera-reason').textContent='Camera image interrupted · retry monitoring if it does not recover.';$('camera-unavailable').hidden=false;});
function clock(){$('clock').textContent=new Date().toLocaleTimeString([], {hour:'2-digit',minute:'2-digit'});}
clock();setInterval(clock,10000);refresh();setInterval(refresh,900);
