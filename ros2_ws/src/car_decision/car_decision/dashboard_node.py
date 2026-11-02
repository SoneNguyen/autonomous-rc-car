"""Read-only loopback web dashboard for a live Obstacle course run."""

from __future__ import annotations

from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import signal
import threading
import time
from typing import Any
from urllib.parse import parse_qs, urlsplit

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String


COMPONENTS = ("lidar", "camera", "imu", "odometry", "contact")
ROLES = ("idle", "supporting", "trigger", "veto", "overruled")
HTML = """<!doctype html>
<html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width">
<title>RC obstacle course monitor</title><style>
:root{color-scheme:dark;--bg:#111820;--panel:#1b252e;--panel2:#222e38;--line:#3a4650;--text:#edf1f4;--muted:#a4afb8;--accent:#83b7d9;--green:#72c49a;--amber:#e8bd69;--red:#ed6a72;--purple:#bd9de0;--orange:#dfa064}
*{box-sizing:border-box}body{font:15px system-ui,sans-serif;margin:0;background:var(--bg);color:var(--text)}
main{max-width:1500px;margin:auto;padding:1.2rem 1.5rem 2rem}header{display:flex;justify-content:space-between;gap:1rem;align-items:center;border-bottom:1px solid var(--line);padding-bottom:1rem}
h1{font-size:1.15rem;letter-spacing:.02em;margin:0}.brand{display:flex;align-items:center;gap:.7rem}.mark{width:10px;height:10px;border-radius:2px;background:var(--accent)}
.live{display:flex;align-items:center;gap:.45rem;color:var(--green);font-size:.75rem;font-weight:700;text-transform:uppercase;letter-spacing:.06em}.dot{width:8px;height:8px;border-radius:50%;background:currentColor}.disconnected{color:var(--red)}
.runline{display:flex;gap:.8rem;flex-wrap:wrap;color:var(--muted);font: .82rem ui-monospace,monospace;padding:.8rem 0}.hero{display:grid;grid-template-columns:minmax(0,1.6fr) minmax(240px,.8fr);gap:1rem}
.panel{background:var(--panel);border:1px solid var(--line);border-radius:6px;padding:1rem}.behavior-name{font-size:clamp(2rem,5vw,3rem);font-weight:750;letter-spacing:.01em;line-height:1}.behavior-badge{font-size:.7rem;text-transform:uppercase;letter-spacing:.1em;color:var(--muted)}
.behavior-follow_route{color:var(--green)}.behavior-caution{color:var(--amber)}.behavior-avoid_left,.behavior-avoid_right{color:var(--orange)}.behavior-stop{color:var(--red)}.behavior-resume{color:var(--accent)}
.reason{font-size:1rem;margin-top:.8rem;line-height:1.45}.side-metrics{display:grid;align-content:center;gap:.8rem}.metric-label,.section-note{font-size:.72rem;color:var(--muted);text-transform:uppercase;letter-spacing:.1em}.latency{font:1.15rem ui-monospace,monospace}.latency-good{color:var(--green)}.latency-bad{color:var(--red)}
h2{font-size:.85rem;letter-spacing:.07em;text-transform:uppercase;margin:1.4rem 0 .65rem;color:#d3dbe0}.section-head{display:flex;justify-content:space-between;align-items:baseline;gap:.6rem}.section-note{font-weight:400;text-transform:none;letter-spacing:0}
#components{display:grid;grid-template-columns:repeat(auto-fit,minmax(205px,1fr));gap:.65rem}.card{width:100%;min-height:128px;text-align:left;color:var(--text);background:var(--panel);border:1px solid var(--line);border-radius:5px;padding:.8rem;cursor:pointer}
.card:hover{border-color:var(--accent)}.card:focus-visible{outline:2px solid var(--accent);outline-offset:2px}.card[aria-pressed=true]{outline:1px solid var(--accent)}.card.status-trigger{border:2px solid var(--red)}.card.status-supporting{border-color:var(--accent)}.card.status-veto{border-color:var(--purple)}.card.status-overruled{border-color:var(--amber)}.card.freshness-stale{border-style:dashed}
.card-top{display:flex;justify-content:space-between;align-items:center}.component-name{font-weight:700;text-transform:capitalize}.role{font-size:.66rem;font-weight:700;letter-spacing:.06em;text-transform:uppercase;border-radius:3px;padding:.2rem .4rem;background:#303a43;color:#d3dbe0}.role-trigger{background:#45252a;color:#ffb3b8}.role-supporting{background:#253844;color:#b4d7eb}.role-veto{background:#372d43;color:#d6c3eb}.role-overruled{background:#453a27;color:#ecd29a}
.freshness{margin:.65rem 0 .35rem;font-size:.68rem;text-transform:uppercase;letter-spacing:.06em}.freshness-fresh{color:var(--green)}.freshness-stale{color:var(--amber)}.freshness-unavailable{color:var(--muted)}.summary{color:#cbd3d9;font-size:.83rem;line-height:1.35;overflow-wrap:anywhere}
.preview{margin-top:.65rem}#preview[hidden]{display:none}.preview-head{display:flex;align-items:baseline;justify-content:space-between;gap:1rem;border-bottom:1px solid var(--line);padding-bottom:.65rem}.preview-title{font-size:1rem;font-weight:700;text-transform:capitalize}.preview-meta{font-size:.75rem;color:var(--muted)}.preview-layout{display:grid;grid-template-columns:minmax(0,1.4fr) minmax(220px,.8fr);gap:1rem;align-items:start;padding-top:.8rem}.camera-frame{display:block;width:100%;max-height:420px;object-fit:contain;background:#090d11;border:1px solid var(--line);border-radius:4px}.camera-frame-status{margin-top:.35rem}.lidar-map{display:block;width:min(100%,520px);height:auto;background:#111820;border:1px solid var(--line);border-radius:4px}.preview-metrics{display:grid;gap:.55rem}.preview-row{display:flex;justify-content:space-between;gap:1rem;border-bottom:1px solid var(--line);padding:.42rem 0}.preview-label{color:var(--muted);font-size:.8rem}.preview-value{font: .82rem ui-monospace,monospace;text-align:right}.preview-list{display:grid;gap:.35rem;margin-top:.5rem;color:#d4dbe0;font-size:.82rem}.preview-empty{color:var(--muted);padding:.75rem 0}.contact-active{color:var(--red);font-weight:700}.contact-clear{color:var(--green);font-weight:700}
.toolbar{display:flex;gap:.6rem;align-items:center;flex-wrap:wrap;margin:.8rem 0}.toolbar label{font-size:.8rem;color:var(--muted)}button,select{font:inherit;color:var(--text);background:var(--panel2);border:1px solid var(--line);border-radius:4px;padding:.48rem .7rem}button{cursor:pointer}button:hover,select:hover{border-color:var(--accent)}button:focus-visible,select:focus-visible{outline:2px solid var(--accent);outline-offset:2px}
#history{display:grid;gap:.5rem;max-height:480px;overflow:auto;padding-right:.25rem}.event{display:grid;grid-template-columns:84px 1fr;gap:.75rem;background:var(--panel);border:1px solid var(--line);border-left:2px solid #65727c;border-radius:4px;padding:.7rem}.event.event-triggered{border-left-color:var(--red)}.event-time{font: .75rem ui-monospace,monospace;color:var(--muted);padding-top:.1rem}.event-title{font-weight:700}.event-reason{font-size:.86rem;line-height:1.4;margin:.25rem 0;color:#d2d9de}.chips{display:flex;gap:.35rem;flex-wrap:wrap;margin-top:.45rem}.chip{font-size:.68rem;border-radius:3px;padding:.2rem .45rem;background:#303a43;color:#d3dbe0}.chip-trigger{background:#45252a;color:#ffb3b8}.chip-supporting{background:#253844;color:#b4d7eb}.chip-veto{background:#372d43;color:#d6c3eb}.chip-overruled{background:#453a27;color:#ecd29a}.empty{padding:1rem;color:var(--muted);background:var(--panel);border-radius:4px}
@media(max-width:760px){main{padding:.8rem}.hero,.preview-layout{grid-template-columns:1fr}.event{grid-template-columns:1fr;gap:.25rem}}
</style></head><body><main><header><div class="brand"><span class="mark"></span><h1>RC VEHICLE · BEHAVIOR MONITOR</h1></div><div id="connection" class="live"><span class="dot"></span><span id="connection-text">starting dashboard</span></div></header>
<div id="meta" class="runline">Waiting for a simulation run…</div><section class="hero"><div class="panel"><div class="behavior-badge">Active behavior</div><div id="state" class="behavior-name" aria-live="polite">—</div><div id="reason" class="reason" aria-live="polite">Waiting for decision evidence</div></div><div class="panel side-metrics"><div><div class="metric-label">Dashboard delivery</div><div id="latency" class="latency">waiting for decision events</div></div><div><div class="metric-label">Decision events</div><div id="event-count" class="latency">0</div></div></div></section>
<div class="section-head"><h2>Sensor and actuation evidence</h2><span class="section-note">Red border = trigger · select a card for its live preview and history</span></div><section id="components">
<button type="button" class="card status-idle freshness-unavailable" data-component="lidar"><div class="card-top"><span class="component-name">LiDAR</span><span class="role role-idle">idle</span></div><div class="freshness freshness-unavailable">unavailable</div><div class="summary">waiting for sample</div></button>
<button type="button" class="card status-idle freshness-unavailable" data-component="camera"><div class="card-top"><span class="component-name">Camera</span><span class="role role-idle">idle</span></div><div class="freshness freshness-unavailable">unavailable</div><div class="summary">waiting for sample</div></button>
<button type="button" class="card status-idle freshness-unavailable" data-component="imu"><div class="card-top"><span class="component-name">IMU</span><span class="role role-idle">idle</span></div><div class="freshness freshness-unavailable">unavailable</div><div class="summary">waiting for sample</div></button>
<button type="button" class="card status-idle freshness-unavailable" data-component="odometry"><div class="card-top"><span class="component-name">Odometry</span><span class="role role-idle">idle</span></div><div class="freshness freshness-unavailable">unavailable</div><div class="summary">waiting for sample</div></button>
<button type="button" class="card status-idle freshness-unavailable" data-component="contact"><div class="card-top"><span class="component-name">Contact</span><span class="role role-idle">idle</span></div><div class="freshness freshness-unavailable">unavailable</div><div class="summary">waiting for sample</div></button>
</section>
<section id="preview" class="panel preview" aria-live="polite" hidden></section>
<div class="toolbar"><button id="pause" type="button">Pause display</button><label for="filter">History component</label><select id="filter"><option value="all">All components</option><option value="lidar">LiDAR</option><option value="camera">Camera</option><option value="imu">IMU</option><option value="odometry">Odometry</option><option value="contact">Contact</option></select><button id="reset" type="button">Resume and reset view</button></div>
<section class="section-head"><h2>Decision history</h2><span class="section-note">Transition and attribution updates in simulation time</span></section><section id="history"></section></main>
<script>
// Keep a tiny independent watchdog so a syntax error in the dashboard client
// does not leave the page silently stuck on its initial "connecting" label.
window.__dashboardClientStarted=false;
window.setTimeout(()=>{if(window.__dashboardClientStarted)return;const status=document.querySelector('#connection');const label=document.querySelector('#connection-text');if(status)status.classList.add('disconnected');if(label)label.textContent='dashboard script failed · hard refresh';},3000);
</script><script>
const components=['lidar','camera','imu','odometry','contact'];let paused=false;let selected='all';let activePreview=null;let previewTimer=null;let cameraFrameTimer=null;let previewRequest=false;let cameraFrameRequest=false;let cameraPreviewGeneration=0;let cameraPreviewRunId=null;let cameraFrameSequence=0;let cameraFrameFailures=0;let latest=null;let source=null;let lastId=0;let historySignature='';const cardNodes=Object.create(null);
const $=s=>document.querySelector(s);const node=(tag,cls,text)=>{const e=document.createElement(tag);if(cls)e.className=cls;if(text!==undefined)e.textContent=String(text);return e;};
function refs(e,role){return (e.attribution||{})[role]||[];}function eventComponents(e){return ['triggers','supporters','vetoes','overruled'].flatMap(k=>refs(e,k)).map(x=>typeof x==='string'?x:x.component).filter(Boolean);}
function eventKey(e){return [e.event_id,e.event_kind,e.sim_time_ns,e.behavior,e.reason,JSON.stringify(e.attribution||{})].join('|');}
function detail(x){if(x===null||x===undefined)return '';if(typeof x==='string'||typeof x==='number')return String(x);if(Array.isArray(x))return x.map(detail).join(', ');if(typeof x==='object'){if(x.collision_a||x.collision_b)return `${x.collision_a||''} ↔ ${x.collision_b||''}`;return Object.entries(x).map(([k,v])=>`${k}: ${detail(v)}`).join(' · ');}return '';}
function refText(ref){const value=ref.value;return `${ref.component}${value===undefined?'':` · ${detail(value)}${ref.unit&&ref.unit!=='class'&&ref.unit!=='contact_pair'?` ${ref.unit}`:''}`}`;}
function stateClass(value){return `behavior-${String(value||'').toLowerCase()}`;}
function setConnected(connected){$('#connection').classList.toggle('disconnected',!connected);$('#connection-text').textContent=connected?'live stream':'reconnecting';}
function setText(element,value){const text=String(value);if(element.textContent!==text)element.textContent=text;}
function previewMessage(layout,text){if(activePreview==='camera'){const refs=mountCameraPreview(layout);setText(refs.evidenceStatus,text);return;}const first=layout.firstElementChild;if(first&&first.classList.contains('preview-empty')&&first.textContent===text)return;layout.replaceChildren(node('div','preview-empty',text));}
function makeCard(name){const card=node('button','card status-idle freshness-unavailable');card.type='button';card.dataset.component=name;const top=node('div','card-top');const title=node('span','component-name',name);const role=node('span','role role-idle','idle');top.append(title,role);const freshness=node('div','freshness freshness-unavailable','unavailable');const summary=node('div','summary','waiting for sample');card.append(top,freshness,summary);return card;}
function mountCards(){const grid=$('#components');if(!grid)return;for(const name of components){let card=cardNodes[name]&&cardNodes[name].card;if(!card||!card.isConnected||card.parentElement!==grid){card=grid.querySelector(`[data-component="${name}"]`);if(!card){card=makeCard(name);grid.append(card);}}cardNodes[name]={card,role:card.querySelector('.role'),freshness:card.querySelector('.freshness'),summary:card.querySelector('.summary')};}}
const componentGrid=$('#components');if(componentGrid)new MutationObserver(()=>mountCards()).observe(componentGrid,{childList:true});
function previewRow(parent,label,value){const row=node('div','preview-row');row.append(node('span','preview-label',label),node('span','preview-value',value));parent.append(row);}
function releaseCameraFrame(){cameraPreviewGeneration++;cameraPreviewRunId=null;cameraFrameSequence=0;cameraFrameFailures=0;if(cameraFrameTimer)clearInterval(cameraFrameTimer);cameraFrameTimer=null;}
function resetCameraPreviewForRun(runId){if(!runId||runId===cameraPreviewRunId)return;cameraPreviewRunId=runId;cameraPreviewGeneration++;cameraFrameSequence=0;cameraFrameFailures=0;const refs=$('#preview-layout')?.cameraRefs;if(refs?.canvas?.isConnected){const context=refs.canvas.getContext('2d');context?.clearRect(0,0,refs.canvas.width,refs.canvas.height);setText(refs.status,'Waiting for camera frame…');}}
function closePreview(){activePreview=null;if(previewTimer)clearInterval(previewTimer);previewTimer=null;releaseCameraFrame();$('#preview').hidden=true;selected='all';$('#filter').value='all';render(latest);}
function startCameraFramePolling(){if(cameraFrameTimer||paused||activePreview!=='camera')return;const refs=$('#preview-layout')?.cameraRefs;if(!refs)return;refreshCameraFrame(refs.canvas,refs.status);cameraFrameTimer=setInterval(()=>refreshCameraFrame(refs.canvas,refs.status),200);}
function openPreview(component){releaseCameraFrame();activePreview=component;selected=component;$('#filter').value=component;const panel=$('#preview');panel.hidden=false;panel.replaceChildren();const head=node('div','preview-head');const close=node('button','','Close preview');close.type='button';close.addEventListener('click',closePreview);head.append(node('div','preview-title',`${component} preview`),close);panel.append(head,node('div','preview-meta','Live simulation evidence · read only'));const layout=node('div','preview-layout');layout.id='preview-layout';panel.append(layout);if(previewTimer)clearInterval(previewTimer);if(component==='camera'){const refs=mountCameraPreview(layout);setText(refs.status,'Waiting for camera frame…');startCameraFramePolling();}refreshPreview();previewTimer=paused?null:setInterval(refreshPreview,500);}
function mountCameraPreview(layout){if(layout.dataset.cameraMounted==='true'&&layout.cameraRefs?.canvas?.isConnected)return layout.cameraRefs;const visual=node('div','');const canvas=node('canvas','camera-frame');canvas.setAttribute('role','img');canvas.setAttribute('aria-label','Live front camera image');canvas.width=640;canvas.height=480;const status=node('div','preview-meta camera-frame-status','Waiting for camera frame…');const details=node('div','preview-metrics');const evidenceStatus=node('div','preview-meta camera-evidence-status','Waiting for camera evidence…');const objectList=node('div','camera-object-list');const empty=node('div','preview-empty','No classified objects in the current frame.');objectList.append(empty);details.append(evidenceStatus,objectList);visual.append(canvas,status);layout.replaceChildren(visual,details);layout.dataset.cameraMounted='true';layout.cameraRefs={canvas,status,evidenceStatus,objectList,empty,objectRows:[]};return layout.cameraRefs;}
async function refreshCameraFrame(canvas,status){if(!activePreview||activePreview!=='camera'||paused||cameraFrameRequest)return;const generation=cameraPreviewGeneration;const controller=new AbortController();const timeout=setTimeout(()=>controller.abort(),2000);cameraFrameRequest=true;let bitmap=null;try{const response=await fetch(`/camera.jpg?since=${cameraFrameSequence}`,{cache:'no-store',signal:controller.signal});if(generation!==cameraPreviewGeneration||!canvas.isConnected)return;if(response.status===304){cameraFrameFailures=0;return;}if(!response.ok){if(cameraFrameSequence)cameraFrameFailures++;if(!cameraFrameSequence)setText(status,'Waiting for camera frame…');else if(cameraFrameFailures>=3)setText(status,`Camera reconnecting · last frame ${cameraFrameSequence}`);return;}const sequence=Number(response.headers.get('X-Preview-Sequence')||0);if(sequence&&sequence<=cameraFrameSequence){cameraFrameFailures=0;return;}bitmap=await createImageBitmap(await response.blob());if(generation!==cameraPreviewGeneration||!canvas.isConnected||!activePreview||activePreview!=='camera'||paused)return;if(canvas.width!==bitmap.width)canvas.width=bitmap.width;if(canvas.height!==bitmap.height)canvas.height=bitmap.height;const context=canvas.getContext('2d',{alpha:false});context.drawImage(bitmap,0,0);if(sequence)cameraFrameSequence=sequence;cameraFrameFailures=0;setText(status,`Live frame ${sequence||cameraFrameSequence}`);}catch(_){if(generation!==cameraPreviewGeneration)return;cameraFrameFailures++;if(cameraFrameFailures>=3)setText(status,cameraFrameSequence?`Camera reconnecting · last frame ${cameraFrameSequence}`:'Waiting for camera frame…');}finally{clearTimeout(timeout);if(bitmap)bitmap.close();cameraFrameRequest=false;}}
function updateCameraPreview(layout,record){resetCameraPreviewForRun(record.run_id);const refs=mountCameraPreview(layout);const freshness=record.freshness?.state||'unavailable';const simTime=(Number(record.sim_time_ns||0)/1e9).toFixed(2);setText(refs.evidenceStatus,`${freshness} evidence · sim ${simTime} s`);const objects=record.evidence?.objects||[];if(objects.length===0){if(refs.objectRows.length||refs.objectList.firstElementChild!==refs.empty){refs.objectList.replaceChildren(refs.empty);refs.objectRows=[];}}else{if(refs.empty.isConnected)refs.empty.remove();while(refs.objectRows.length>objects.length){refs.objectRows.pop().remove();}while(refs.objectRows.length<objects.length){const row=node('div','preview-row');const label=node('span','preview-label');const value=node('span','preview-value');row.append(label,value);refs.objectRows.push({row,label,value});refs.objectList.append(row);}objects.forEach((object,index)=>{const row=refs.objectRows[index];setText(row.label,object.class||'object');setText(row.value,`${Number(object.bearing_rad||0).toFixed(2)} rad · ${Number(object.confidence||0).toFixed(2)} confidence`);});}}
async function refreshPreview(){if(!activePreview||paused||previewRequest)return;previewRequest=true;const component=activePreview;const layout=$('#preview-layout');if(!layout){previewRequest=false;return;}try{const response=await fetch(`/evidence/${component}`,{cache:'no-store'});if(component!==activePreview)return;if(!response.ok){previewMessage(layout,response.status===503?'Waiting for a component sample…':'Component evidence is unavailable.');return;}const record=await response.json();if(component!==activePreview)return;if(component==='camera'){updateCameraPreview(layout,record);return;}const recordKey=`${record.run_id||''}:${record.sim_time_ns||0}:${JSON.stringify(record.freshness||{})}`;if(layout.dataset.recordKey===recordKey)return;layout.dataset.recordKey=recordKey;layout.replaceChildren();const data=record.evidence||{};const details=node('div','preview-metrics');const freshness=`${record.freshness?.state||'unavailable'} · sim ${(Number(record.sim_time_ns||0)/1e9).toFixed(2)} s`;details.append(node('div','preview-meta',freshness));
if(component==='lidar'){const map=document.createElementNS('http://www.w3.org/2000/svg','svg');map.setAttribute('viewBox','0 0 320 230');map.setAttribute('role','img');map.setAttribute('aria-label','Top-down forward LiDAR scan');map.classList.add('lidar-map');const svg=(name,attrs)=>{const el=document.createElementNS('http://www.w3.org/2000/svg',name);for(const [key,value] of Object.entries(attrs))el.setAttribute(key,String(value));map.append(el);return el;};const cx=160,cy=205,scale=36;for(const radius of [1,2,3]){const r=radius*scale;svg('path',{d:`M ${cx-r} ${cy} A ${r} ${r} 0 0 1 ${cx+r} ${cy}`,fill:'none',stroke:'#3a4650','stroke-width':1});svg('text',{x:cx+4,y:cy-r+13,fill:'#a4afb8','font-size':10}).textContent=`${radius}m`;}const points=(data.points_base_link||[]).filter(p=>Number(p.x_m)>0&&Number.isFinite(Number(p.y_m)));const stride=Math.max(1,Math.ceil(points.length/160));for(let i=0;i<points.length;i+=stride){const p=points[i],x=cx+Number(p.y_m)*scale,y=cy-Number(p.x_m)*scale;if(y<0||x<0||x>320)continue;svg('circle',{cx:x,cy:y,r:3,fill:'#e8bd69'});}svg('rect',{x:cx-8,y:cy-13,width:16,height:22,rx:3,fill:'#83b7d9'});layout.append(map,details);details.append(node('div','preview-meta',`${points.length} forward returns · plot range 3 m`));const sectors=data.sectors||{};for(const [name,value] of Object.entries(sectors))previewRow(details,name.replaceAll('_',' '),value===null?'clear':`${Number(value).toFixed(2)} m`);return;}
if(component==='imu'){for(const [label,key,unit] of [['Yaw rate','yaw_rate_rad_s','rad/s'],['Lateral acceleration','lateral_acceleration_mps2','m/s²'],['Longitudinal acceleration','longitudinal_acceleration_mps2','m/s²']])previewRow(details,label,`${Number(data[key]||0).toFixed(3)} ${unit}`);layout.append(details);return;}
if(component==='odometry'){const pose=data.start_relative_pose||{};for(const [label,value,unit] of [['Speed',data.speed_mps,'m/s'],['Route progress',data.route_progress_m,'m'],['Stopping distance',data.d_stop_m,'m'],['X position',pose.x_m,'m'],['Y position',pose.y_m,'m'],['Heading',pose.yaw_rad,'rad']])previewRow(details,label,`${Number(value||0).toFixed(3)} ${unit}`);layout.append(details);return;}
if(component==='contact'){const collision=Boolean(data.non_ground_collision);const status=node('div',collision?'contact-active':'contact-clear',collision?'NON-GROUND CONTACT':'CLEAR');details.append(status,node('div','preview-meta',data.ground_excluded?'Ground-only contact is excluded.':'Contact pairs are listed below.'));for(const pair of data.contacts||[]){const row=node('div','preview-row');row.append(node('span','preview-label',pair.ground?'ground':'entity'),node('span','preview-value',`${pair.collision_a||'—'} ↔ ${pair.collision_b||'—'}`));details.append(row);}if(!(data.contacts||[]).length)details.append(node('div','preview-empty','No contact pairs in the latest sample.'));layout.append(details);return;}
layout.append(details);
}catch(_){previewMessage(layout,'Preview disconnected; waiting for live evidence.');}finally{previewRequest=false;}}
function render(s){if(!s)return;latest=s;if(paused)return;
setText($('#meta'),`RUN ${s.run_id||'—'}  ·  ${s.scenario_id||'—'}  ·  ${s.run_purpose||'—'}  ·  SIM ${(Number(s.sim_time_ns||0)/1e9).toFixed(2)}s`);
const state=$('#state');setText(state,s.behavior||'—');const behaviorClass=`behavior-name ${stateClass(s.behavior)}`;if(state.className!==behaviorClass)state.className=behaviorClass;setText($('#reason'),s.reason||'No decision reason yet');
mountCards();const cards=s.components||{};for(const name of components){const c=cards[name]||{};const role=c.status||'idle';const freshness=c.freshness||'unavailable';const view=cardNodes[name];const cardClass=`card status-${role} freshness-${freshness}`;if(view.card.className!==cardClass)view.card.className=cardClass;const pressed=String(selected===name);if(view.card.getAttribute('aria-pressed')!==pressed)view.card.setAttribute('aria-pressed',pressed);const roleClass=`role role-${role}`;if(view.role.className!==roleClass)view.role.className=roleClass;setText(view.role,role);const freshnessClass=`freshness freshness-${freshness}`;if(view.freshness.className!==freshnessClass)view.freshness.className=freshnessClass;setText(view.freshness,freshness);setText(view.summary,c.summary||'no sample yet');}
const allHistory=(s.history||[]).slice(-50).reverse();const signature=`${selected}|${allHistory.map(eventKey).join('\\n')}`;if(signature!==historySignature){historySignature=signature;const history=allHistory.filter(e=>selected==='all'||eventComponents(e).includes(selected));const list=$('#history');list.replaceChildren();
for(const e of history){const item=node('article','event');if(refs(e,'triggers').length)item.classList.add('event-triggered');const sim=(Number(e.sim_time_ns||0)/1e9).toFixed(2);item.append(node('div','event-time',`${sim}s`));const body=node('div','');body.append(node('div','event-title',`${e.event_kind||'decision'}  ·  ${e.previous_behavior||'—'} → ${e.behavior||'—'}${e.rule_id?`  ·  ${e.rule_id}`:''}`),node('div','event-reason',e.reason||''));const chips=node('div','chips');for(const [key,role] of [['triggers','trigger'],['supporters','supporting'],['vetoes','veto'],['overruled','overruled']])for(const ref of refs(e,key))chips.append(node('span',`chip chip-${role}`,`${role}: ${refText(ref)}`));if(chips.childNodes.length)body.append(chips);item.append(body);list.append(item);}
if(!history.length)list.append(node('div','empty',selected==='all'?'No decision events received yet.':`No decisions attributed to ${selected} in this run yet.`));}setText($('#event-count'),(s.history||[]).length);}
$('#components').addEventListener('click',e=>{const card=e.target.closest('[data-component]');if(!card)return;openPreview(card.dataset.component);render(latest);});
$('#filter').addEventListener('change',e=>{selected=e.target.value;render(latest);});$('#pause').addEventListener('click',()=>{paused=!paused;$('#pause').textContent=paused?'Resume live display':'Pause display';if(paused&&previewTimer){clearInterval(previewTimer);previewTimer=null;}if(paused)releaseCameraFrame();if(!paused){render(latest);if(activePreview)previewTimer=setInterval(refreshPreview,500);if(activePreview==='camera')startCameraFramePolling();}});
$('#reset').addEventListener('click',()=>{selected='all';$('#filter').value='all';paused=false;$('#pause').textContent='Pause display';render(latest);if(activePreview&&!previewTimer)previewTimer=setInterval(refreshPreview,500);if(activePreview==='camera')startCameraFramePolling();});
function onMessage(e){try{const id=Number(e.lastEventId||0);if(lastId&&id&&id>lastId+1){source.close();lastId=0;connect(true);return;}if(id)lastId=id;const m=JSON.parse(e.data);if(m.type==='snapshot'){resetCameraPreviewForRun(m.snapshot.run_id);render(m.snapshot);}else if(m.type==='component'){latest=latest||{};latest.components=latest.components||{};latest.components[m.component]=m.card;render(latest);}else if(m.type==='state'){latest=m.state;latest.components=latest.components||{};render(latest);}else if(m.type==='decision'){latest=latest||{};latest.history=latest.history||[];if(!latest.history.some(x=>eventKey(x)===eventKey(m.event)))latest.history.push(m.event);latest.history=latest.history.slice(-50);render(latest);}else if(m.type==='run'){resetCameraPreviewForRun(m.run_id);$('#meta').textContent=`RUN ${m.run_id}  ·  ${m.scenario_id}  ·  ${m.run_purpose}`;}else if(m.type==='heartbeat'&&m.latency){const l=m.latency;const el=$('#latency');el.textContent=l.count?`max ${l.maximum_ms.toFixed(1)} ms · p95 ${l.p95_ms.toFixed(1)} ms · ${l.within_200_ms?'under':'over'} 200 ms`:'waiting for decision events';el.className=`latency ${l.count?(l.within_200_ms?'latency-good':'latency-bad'):''}`;}}catch(_){}}
function connect(refresh=false){source=new EventSource(refresh?'/events?refresh=1':'/events');source.onopen=()=>setConnected(true);source.onerror=()=>setConnected(false);source.onmessage=onMessage;}
try{if(typeof EventSource!=='function')throw new Error('EventSource is unavailable');connect();window.__dashboardClientStarted=true;}catch(error){$('#connection').classList.add('disconnected');$('#connection-text').textContent='dashboard client unavailable';$('#meta').textContent=`Dashboard could not connect: ${error.message}`;}
</script></body></html>"""


class LatencyMetrics:
    """Thread-safe measurement of decision publish to completed SSE flush."""

    def __init__(self, limit: int = 2048) -> None:
        self._values: deque[int] = deque(maxlen=limit)
        self._lock = threading.Lock()

    def observe(self, published_monotonic_ns: int, flushed_monotonic_ns: int) -> int:
        latency = max(0, int(flushed_monotonic_ns) - int(published_monotonic_ns))
        with self._lock:
            self._values.append(latency)
        return latency

    def summary(self) -> dict[str, float | int | bool]:
        with self._lock:
            values = sorted(self._values)
        if not values:
            return {"count": 0, "maximum_ms": 0.0, "p95_ms": 0.0, "within_200_ms": False}
        p95_index = max(0, math.ceil(0.95 * len(values)) - 1)
        return {
            "count": len(values),
            "maximum_ms": values[-1] / 1e6,
            "p95_ms": values[p95_index] / 1e6,
            "within_200_ms": values[-1] < 200_000_000,
        }


class DashboardStore:
    """Small bounded event store; callbacks never wait for browser clients."""

    def __init__(self, history_limit: int = 256) -> None:
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self._sequence = 0
        self._events: deque[dict[str, Any]] = deque(maxlen=history_limit)
        self._state: dict[str, Any] = {}
        self._evidence: dict[str, dict[str, Any]] = {}
        self._decisions: deque[dict[str, Any]] = deque(maxlen=50)
        self._run_id = ""
        self._scenario_id = ""
        self._run_purpose = "diagnostic"
        self._camera_jpeg: bytes | None = None
        self._camera_preview_sequence = 0
        self._last_state_event_ns = 0
        self._last_evidence_event_ns: dict[str, int] = {}
        self._evidence_freshness: dict[str, str] = {}

    def _append(self, kind: str, payload: dict[str, Any]) -> dict[str, Any]:
        self._sequence += 1
        event = {"id": self._sequence, "kind": kind, "payload": payload}
        self._events.append(event)
        self._condition.notify_all()
        return event

    def accept(self, kind: str, record: dict[str, Any]) -> bool:
        run_id = str(record.get("run_id", ""))
        scenario_id = str(record.get("scenario_id", ""))
        if not run_id or not scenario_id:
            return False
        with self._condition:
            if kind == "state":
                if run_id != self._run_id:
                    self._run_id, self._scenario_id = run_id, scenario_id
                    self._run_purpose = str(record.get("run_purpose", "diagnostic"))
                    self._state = {}
                    self._evidence.clear()
                    self._decisions.clear()
                    self._events.clear()
                    self._camera_jpeg = None
                    self._camera_preview_sequence = 0
                    self._last_state_event_ns = 0
                    self._last_evidence_event_ns.clear()
                    self._evidence_freshness.clear()
                    self._append("run", {
                        "run_id": run_id, "scenario_id": scenario_id,
                        "run_purpose": self._run_purpose,
                    })
                elif scenario_id != self._scenario_id:
                    return False
                state = dict(record)
                if self._state and state.get("behavior") == self._state.get("behavior"):
                    # Keep trigger/support roles anchored to this behavior's
                    # transition while sensor values update. Veto and overrule
                    # roles remain live so an attributed evaluation is visible.
                    attribution = dict(state.get("attribution", {}))
                    previous = self._state.get("attribution", {})
                    for role in ("triggers", "supporters"):
                        attribution[role] = previous.get(role, attribution.get(role, []))
                    state["attribution"] = attribution
                self._state = state
                now = time.monotonic_ns()
                if now - self._last_state_event_ns >= 100_000_000:
                    self._last_state_event_ns = now
                    self._append("state", dict(record))
                return True
            if run_id != self._run_id or scenario_id != self._scenario_id:
                return False
            if str(record.get("run_purpose", self._run_purpose)) != self._run_purpose:
                return False
            if kind == "evidence":
                component = str(record.get("component", ""))
                if component not in COMPONENTS:
                    return False
                self._evidence[component] = dict(record)
                now = time.monotonic_ns()
                freshness = str(record.get("freshness", {}).get("state", "unavailable"))
                changed = self._evidence_freshness.get(component) != freshness
                if changed or now - self._last_evidence_event_ns.get(component, 0) >= 250_000_000:
                    self._last_evidence_event_ns[component] = now
                    self._evidence_freshness[component] = freshness
                    self._append("component", {"component": component, "record": dict(record)})
                return True
            if kind == "decision":
                if record.get("event_kind") not in {"transition", "evaluation"}:
                    return False
                exact = dict(record)
                self._decisions.append(exact)
                self._append("decision", exact)
                return True
        return False

    def component_evidence(self, component: str) -> dict[str, Any] | None:
        if component not in COMPONENTS:
            return None
        with self._lock:
            record = self._evidence.get(component)
            if record is None:
                return None
            return {
                "component": component,
                "run_id": record.get("run_id"),
                "sim_time_ns": record.get("sim_time_ns"),
                "valid": bool(record.get("valid")),
                "freshness": record.get("freshness", {}).get("state", "unavailable"),
                "evidence": record.get("evidence", {}),
            }

    def accept_camera_preview(self, jpeg: bytes) -> int:
        """Keep only the most recent small JPEG; previews are never run evidence."""
        data = bytes(jpeg)
        if not data.startswith(b"\xff\xd8") or not data.endswith(b"\xff\xd9") or len(data) > 1_000_000:
            return self._camera_preview_sequence
        with self._lock:
            self._camera_jpeg = data
            self._camera_preview_sequence += 1
            return self._camera_preview_sequence

    def camera_preview(self) -> tuple[bytes | None, int]:
        with self._lock:
            return self._camera_jpeg, self._camera_preview_sequence

    def _snapshot_locked(self) -> dict[str, Any]:
        state = dict(self._state)
        attribution = state.get("attribution", {})
        roles: dict[str, str] = {component: "idle" for component in COMPONENTS}
        for field, role in (("supporters", "supporting"), ("triggers", "trigger"), ("vetoes", "veto"), ("overruled", "overruled")):
            for ref in attribution.get(field, []):
                component = ref.get("component")
                if component in roles:
                    roles[component] = role
        cards = {}
        freshness = state.get("freshness", {})
        for component in COMPONENTS:
            record = self._evidence.get(component, {})
            evidence = record.get("evidence", {})
            status = roles[component]
            summary = _evidence_summary(component, evidence)
            if status == "trigger":
                trigger = next((
                    ref for ref in attribution.get("triggers", [])
                    if ref.get("component") == component
                ), None)
                if trigger is not None:
                    trigger_summary = _trigger_summary(trigger)
                    # Some valid trigger references only identify the source
                    # component. Preserve its latest measured value in that case.
                    if trigger_summary != "active trigger":
                        summary = trigger_summary
            cards[component] = {
                "status": status,
                "freshness": record.get("freshness", {}).get("state", freshness.get(component, "unavailable")),
                "summary": summary,
            }
        state["components"] = cards
        state["history"] = list(self._decisions)
        return state

    def snapshot(self) -> dict[str, Any]:
        with self._lock:
            return self._snapshot_locked()

    def snapshot_with_cursor(self) -> tuple[dict[str, Any], int]:
        """Capture the dashboard state and its replay cursor atomically."""
        with self._lock:
            return self._snapshot_locked(), self._sequence

    def replay_after(self, event_id: int) -> tuple[bool, list[dict[str, Any]]]:
        with self._lock:
            if not self._events:
                return event_id == self._sequence, []
            earliest = int(self._events[0]["id"])
            if event_id < earliest - 1 or event_id > self._sequence:
                return False, []
            return True, [event for event in self._events if int(event["id"]) > event_id]

    def wait_after(self, event_id: int, timeout_s: float) -> list[dict[str, Any]]:
        with self._condition:
            self._condition.wait_for(lambda: self._sequence > event_id, timeout=max(0.0, timeout_s))
            return [event for event in self._events if int(event["id"]) > event_id]

    @property
    def current_event_id(self) -> int:
        with self._lock:
            return self._sequence


def _evidence_summary(component: str, evidence: dict[str, Any]) -> str:
    if component == "lidar":
        corridor = evidence.get("route_corridor", {})
        nearest = corridor.get("nearest_range_m")
        return f"front corridor {nearest:.2f} m" if isinstance(nearest, (int, float)) else "front corridor clear"
    if component == "camera":
        objects = evidence.get("objects", [])
        return ", ".join(str(obj.get("class", "object")) for obj in objects[:3]) or "no classified object"
    if component == "imu":
        return f"yaw {float(evidence.get('yaw_rate_rad_s', 0.0)):.2f} rad/s"
    if component == "odometry":
        return f"speed {float(evidence.get('speed_mps', 0.0)):.2f} m/s"
    if component == "contact":
        return "collision" if evidence.get("non_ground_collision") else "no non-ground contact"
    return "no sample yet"


def _trigger_summary(ref: dict[str, Any]) -> str:
    """Show the decision's trigger value even after the sensor's latest sample changes."""
    value = ref.get("value")
    unit = str(ref.get("unit", ""))
    if isinstance(value, list):
        pairs = [
            f"{str(row.get('collision_a', '')).split('::')[-1]} ↔ "
            f"{str(row.get('collision_b', '')).split('::')[-1]}"
            for row in value if isinstance(row, dict) and (row.get("collision_a") or row.get("collision_b"))
        ]
        if pairs:
            return "contact: " + ", ".join(pairs[:2])
    if isinstance(value, dict):
        if value.get("class"):
            return f"class: {value['class']}"
        if value.get("range_estimate_m") is not None:
            return f"estimated range: {float(value['range_estimate_m']):.2f} m"
        if value:
            return ", ".join(f"{key}={item}" for key, item in value.items())
    if isinstance(value, (int, float)) and unit:
        return f"{float(value):.2f} {unit}"
    return str(value) if value is not None else "active trigger"


def make_server(store: DashboardStore, metrics: LatencyMetrics, host: str = "127.0.0.1", port: int = 8765) -> ThreadingHTTPServer:
    if host not in {"127.0.0.1", "localhost", "::1"}:
        raise ValueError("dashboard must bind to a loopback address")

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.1"

        def _body(self, status: int, content_type: str, body: bytes) -> None:
            self.send_response(status)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)
            self.wfile.flush()

        def do_GET(self) -> None:
            path = urlsplit(self.path).path
            if path == "/":
                self._body(200, "text/html; charset=utf-8", HTML.encode("utf-8"))
            elif path == "/events":
                self._events()
            elif path == "/camera.jpg":
                jpeg, sequence = store.camera_preview()
                if jpeg is None:
                    self._body(503, "text/plain; charset=utf-8", b"camera preview is waiting for a frame\n")
                    return
                query = parse_qs(urlsplit(self.path).query)
                try:
                    since = int(query.get("since", ["0"])[0])
                except (TypeError, ValueError):
                    self._body(400, "text/plain; charset=utf-8", b"invalid preview sequence\n")
                    return
                if since >= sequence:
                    self.send_response(304)
                    self.send_header("Cache-Control", "no-store")
                    self.send_header("Content-Length", "0")
                    self.end_headers()
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(jpeg)))
                self.send_header("Cache-Control", "no-store")
                self.send_header("X-Preview-Sequence", str(sequence))
                self.end_headers()
                self.wfile.write(jpeg)
                self.wfile.flush()
            elif path.startswith("/evidence/"):
                component = path.removeprefix("/evidence/")
                payload = store.component_evidence(component)
                if payload is None:
                    self._body(404, "application/json; charset=utf-8", b'{"error":"component evidence unavailable"}\n')
                else:
                    self._body(200, "application/json; charset=utf-8", json.dumps(payload, separators=(",", ":")).encode("utf-8"))
            else:
                self._body(404, "text/plain; charset=utf-8", b"not found\n")

        def _events(self) -> None:
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream; charset=utf-8")
            self.send_header("Cache-Control", "no-cache")
            self.send_header("Connection", "keep-alive")
            self.send_header("X-Accel-Buffering", "no")
            self.end_headers()
            raw_id = self.headers.get("Last-Event-ID", "").strip()
            try:
                last_id = int(raw_id) if raw_id else None
            except ValueError:
                last_id = None
            try:
                if last_id is None:
                    state, cursor = store.snapshot_with_cursor()
                    snapshot = {"type": "snapshot", "snapshot": state}
                    self._send(None, snapshot)
                else:
                    replay_ok, replay = store.replay_after(last_id)
                    if replay_ok:
                        cursor = last_id
                        for item in replay:
                            self._send_event(item, measure_latency=False)
                            cursor = int(item["id"])
                    else:
                        state, cursor = store.snapshot_with_cursor()
                        self._send(None, {"type": "snapshot", "snapshot": state})
                while True:
                    pending = store.wait_after(cursor, 15.0)
                    if not pending:
                        self._send(None, {"type": "heartbeat", "latency": metrics.summary()})
                    for item in pending:
                        self._send_event(item, measure_latency=True)
                        cursor = int(item["id"])
            except (BrokenPipeError, ConnectionResetError, TimeoutError, OSError):
                return

        def _send_event(self, item: dict[str, Any], *, measure_latency: bool) -> None:
            payload = item["payload"]
            kind = item["kind"]
            if kind == "state":
                message = {"type": "state", "state": store.snapshot()}
            elif kind == "component":
                component = payload["component"]
                snapshot = store.snapshot()
                message = {"type": "component", "component": component,
                           "card": snapshot.get("components", {}).get(component, {})}
            elif kind == "decision":
                message = {"type": "decision", "event": payload}
            else:
                message = {"type": kind, **payload}
            self._send(int(item["id"]), message)
            if kind == "decision" and measure_latency:
                event = payload
                published = event.get("published_monotonic_ns")
                if isinstance(published, int):
                    metrics.observe(published, time.monotonic_ns())

        def _send(self, event_id: int | None, message: dict[str, Any]) -> None:
            prefix = f"id: {event_id}\n" if event_id is not None else ""
            data = json.dumps(message, separators=(",", ":"), allow_nan=False)
            self.wfile.write(f"{prefix}data: {data}\n\n".encode("utf-8"))
            self.wfile.flush()

        def do_POST(self) -> None:
            self._body(405, "text/plain; charset=utf-8", b"GET only\n")

        def do_PUT(self) -> None:
            self._body(405, "text/plain; charset=utf-8", b"GET only\n")

        def do_DELETE(self) -> None:
            self._body(405, "text/plain; charset=utf-8", b"GET only\n")

        def __getattr__(self, name: str):
            if name.startswith("do_"):
                return self._method_not_allowed
            raise AttributeError(name)

        def _method_not_allowed(self) -> None:
            self._body(405, "text/plain; charset=utf-8", b"GET only\n")

        def log_message(self, fmt: str, *args: Any) -> None:
            return

    return ThreadingHTTPServer((host, int(port)), Handler)


class DashboardNode(Node):
    def __init__(self) -> None:
        super().__init__("obstacle_course_dashboard")
        self.declare_parameter("host", "127.0.0.1")
        self.declare_parameter("port", 8765)
        host = str(self.get_parameter("host").value)
        port = int(self.get_parameter("port").value)
        self.store = DashboardStore()
        self.metrics = LatencyMetrics()
        self._subscriptions = [
            self.create_subscription(String, "/decision/state", self._callback("state"), 10),
            self.create_subscription(String, "/decision/event", self._callback("decision"), 50),
            *[
                self.create_subscription(String, f"/decision/evidence/{component}", self._callback("evidence"), qos_profile_sensor_data)
                for component in COMPONENTS
            ],
            self.create_subscription(
                CompressedImage, "/decision/preview/camera", self._on_camera_preview,
                qos_profile_sensor_data,
            ),
        ]
        self.server = make_server(self.store, self.metrics, host, port)
        self.server_thread = threading.Thread(target=self.server.serve_forever, name="dashboard-http", daemon=True)
        self.server_thread.start()
        self._metrics_timer = self.create_timer(5.0, self._report_metrics)
        self.get_logger().info(f"read-only dashboard listening at http://{host}:{self.server.server_port}/")

    def _callback(self, kind: str):
        def receive(message: String) -> None:
            try:
                self.store.accept(kind, json.loads(message.data))
            except (json.JSONDecodeError, TypeError, ValueError) as exc:
                self.get_logger().warning(f"dashboard ignored malformed {kind} message: {exc}")
        return receive

    def _on_camera_preview(self, message: CompressedImage) -> None:
        if message.format.lower().startswith("jpeg"):
            self.store.accept_camera_preview(bytes(message.data))

    def _report_metrics(self) -> None:
        summary = self.metrics.summary()
        if summary["count"]:
            self.get_logger().info(
                "decision-to-SSE latency "
                f"max={summary['maximum_ms']:.2f} ms p95={summary['p95_ms']:.2f} ms "
                f"count={summary['count']} within_200_ms={summary['within_200_ms']}"
            )

    def destroy_node(self):
        server = getattr(self, "server", None)
        if server is not None:
            server.shutdown()
            server.server_close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = DashboardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ROS 2 Lyrical's `ros2 run` wrapper and a terminal foreground process
        # can both forward SIGINT. Ignore a second interrupt during the HTTP
        # server's orderly shutdown so it cannot strand the server thread.
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
