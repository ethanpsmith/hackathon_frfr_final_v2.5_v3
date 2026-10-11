import * as THREE from 'three';
import {GLTFLoader} from './vendor/GLTFLoader.js';
import {OrbitControls} from './vendor/OrbitControls.js';
import {initMap,setRoute,setFlight} from './geographic.js';
import {Graphs,metricsFrom,diagnosticsFrom,trackDistance,bridgeRunForSelection} from './telemetry.js';

const $=id=>document.getElementById(id);
const BRIDGE='http://127.0.0.1:8765';
let routes={},selected='UAV-COLA-2047',archive=null,archiveIndex=0,archiveClock=0;
let token='',connected=false,activeLive=false,displaySource='none',lastRun=null,ownedRun=null,busy=false;
const graphs=new Graphs($('graphs'));
function text(id,value){$(id).textContent=String(value);}
function km(value){return `${(value/1000).toFixed(3)} km`;}
function samePath(a,b){return !!a&&!!b&&JSON.stringify(a)===JSON.stringify(b);}
function archiveForSelection(){return archive&&samePath(routes[selected]?.path,archive.route?.path);}
function resetSource(source){if(displaySource!==source){displaySource=source;graphs.clear();}}
function status(label,badge){text('flight-status',label);text('mapbadge',`●   ${badge}`);$('flight-dot').classList.toggle('is-idle',badge==='ROUTE PREVIEW'||badge==='BRIDGE OFFLINE');}
function selectRoute(id){
  if(!routes[id]||busy)return false;selected=id;ownedRun=null;const route=routes[id];$('tracking').value=id;
  text('store-name',route.store);text('store-address',route.storeAddress);text('destination-name',route.destination);text('destination-address',route.destinationAddress);
  text('route-distance',km(route.metrics.outbound_m));text('distance-caption',`${km(route.metrics.roundtrip_m)} round trip · WGS84 path`);
  text('map-route-name',`${route.store} → ${route.destination}`);text('uav-id',route.vehicle);text('package-id',id);
  for(const b of document.querySelectorAll('.demo-option')){const yes=b.dataset.tracking===id;b.classList.toggle('selected',yes);b.setAttribute('aria-pressed',String(yes));}
  $('tracking-error').classList.remove('show');$('tracking').removeAttribute('aria-invalid');setRoute(route.path);setFlight([],null);
  graphs.clear();displaySource='none';$('progressbar').setAttribute('aria-valuenow','0');$('progressfill').style.width='0%';text('progresspct','Route preview');
  if(!archiveForSelection()){showEmpty();status('READY FOR SITL','ROUTE PREVIEW');}
  else if(!activeLive){archiveIndex=0;archiveClock=0;showReplay(0,true);}
  return true;
}
function showEmpty(){for(const id of ['lat','lng','alt','velocity','health-mode','health-current','health-voltage','health-heading','health-gps','health-systems','health-aircraft','health-mission','health-age'])text(id,'—');text('gps-status','GPS —');text('updated','Start this route to see telemetry');text('graph-source','WAITING FOR DATA');text('health-source','WAITING FOR DATA');}
function renderTelemetry(t,track,source,runStatus,result,phase=null,age=null){
  resetSource(source);const p=t?.GLOBAL_POSITION_INT,gps=t?.GPS_RAW_INT,values=metricsFrom(t);
  const health=diagnosticsFrom(t);
  const valid=!!(p&&p.lat&&p.lon);const coordinates=valid?[p.lat/1e7,p.lon/1e7]:null;
  text('lat',coordinates?`${coordinates[0].toFixed(6)}° N`:'—');text('lng',coordinates?`${Math.abs(coordinates[1]).toFixed(6)}° W`:'—');
  text('alt',values.altitude==null?'—':`${values.altitude.toFixed(1)} m`);text('velocity',values.groundspeed==null?'—':`${values.groundspeed.toFixed(1)} m/s`);
  text('gps-status',gps?.fix_type>=3?`▂▄▆ ${gps.satellites_visible} SAT`:`GPS ${gps?.fix_type??'—'}`);
  text('health-mode',health.mode??'—');text('health-current',health.current==null?'—':`${health.current.toFixed(2)} A`);
  text('health-voltage',health.voltage==null?'—':`${health.voltage.toFixed(2)} V`);
  text('health-heading',values.heading==null?'—':`${values.heading.toFixed(1)}°`);
  text('health-gps',health.gpsFix==null?'—':`Fix ${health.gpsFix} · ${health.satellites??'—'} sat`);
  const word=x=>x==null?'—':x?'OK':'FAULT';text('health-systems',`${word(health.estimator)} / ${word(health.rc)}`);
  text('health-aircraft',`${health.armed==null?'—':health.armed?'Armed':'Disarmed'} / ${health.landed==null?'—':health.landed?'Landed':'Airborne'}`);
  text('health-mission',health.missionItem??'—');text('health-age',source==='replay'?'Recorded':Number.isFinite(age)?`${age.toFixed(1)} s`:'—');
  text('health-source',source==='replay'?'RECORDED SITL':runStatus==='RUNNING'?'LIVE SITL':`LAST SITL RUN · ${runStatus}`);
  text('health-note',t?.STATUSTEXT?.text?`Flight controller: ${t.STATUSTEXT.text}`:'Values come from simulated flight-controller messages.');
  text('updated',source==='replay'?'Recorded SITL telemetry · replay':runStatus==='RUNNING'?'Live simulator telemetry':`Last SITL run · ${runStatus}`);
  text('graph-source',source==='replay'?'RECORDED SITL':runStatus==='RUNNING'?'LIVE SITL':`LAST SITL RUN · ${runStatus}`);
  const cleaned=track.filter(x=>Number.isFinite(x[0])&&Number.isFinite(x[1])&&x[0]&&x[1]);setFlight(cleaned,coordinates);
  graphs.append(t);
  const roundtrip=routes[selected]?.metrics.roundtrip_m||1;
  const fraction=runStatus==='PASS'?1:Math.min(.99,trackDistance(cleaned)/roundtrip);
  const pct=Math.max(0,Math.round(fraction*100));$('progressbar').setAttribute('aria-valuenow',String(pct));$('progressfill').style.width=`${pct}%`;
  text('progresspct',source==='replay'?`${pct}% of recorded round trip`:`${pct}% of round trip`);
  if(source==='replay')status({PREPARING:'RECORDED PREPARATION',OUTBOUND:'RECORDED OUTBOUND',TARGET_LANDED:'RECORDED TARGET LANDING',RETURNING:'RECORDED RETURN'}[phase?.phase]||'RECORDED REPLAY','RECORDED SITL');
  else if(runStatus==='RUNNING'){
    const label={PREPARING:'PREPARING SITL',OUTBOUND:'OUTBOUND · SITL',TARGET_LANDED:'TARGET LANDED · SITL',RETURNING:'RETURNING · SITL',HOLD:'HOLD AT TARGET'}[phase?.phase]||'IN FLIGHT · SITL';
    status(label,'LIVE SITL GPS');
    if(phase?.phase==='TARGET_LANDED')text('launch-feedback','Target landing verified · payload retained · preparing return climb.');
    if(phase?.phase==='RETURNING')text('launch-feedback','Climbed from target · returning home · payload retained.');
  }
  else if(runStatus==='PASS')status('HOME · COMPLETE','SITL COMPLETE');
  else if(runStatus==='FAIL')status('MISSION STOPPED','SITL FAILED');
  if(runStatus==='PASS'&&result?.home_error_m!=null)text('launch-feedback',`COMPLETE · target landing within ${result.target_landing_error_m?.toFixed(2)??'—'} m · home within ${result.home_error_m.toFixed(2)} m · payload retained`);
  if(runStatus==='FAIL')text('launch-feedback',result?.error||'SITL run failed. See its local result.json.');
}
function showReplay(now,force=false){
  if(activeLive||ownedRun||!archiveForSelection()||!archive?.frames?.length)return;
  const interval=archive.frames[archiveIndex]?.phase==='TARGET_LANDED'?1000:250;
  if(!force&&archiveClock&&now-archiveClock<interval)return;archiveClock=now;
  if(!force){archiveIndex++;if(archiveIndex>=archive.frames.length){archiveIndex=0;graphs.clear();displaySource='none';}}
  const frame=archive.frames[archiveIndex];
  const t={GLOBAL_POSITION_INT:frame.position,SYS_STATUS:frame.battery,GPS_RAW_INT:frame.gps,ATTITUDE:frame.attitude,HEARTBEAT:frame.heartbeat,EXTENDED_SYS_STATE:frame.extended,MISSION_CURRENT:frame.mission,STATUSTEXT:frame.status};
  const track=archive.frames.slice(0,archiveIndex+1).map(x=>[x.position.lat/1e7,x.position.lon/1e7]);
  renderTelemetry(t,track,'replay','REPLAY',archive.result,{phase:frame.phase});
}
async function api(path,body){const response=await fetch(`${BRIDGE}${path}`,{method:body?'POST':'GET',cache:'no-store',headers:{'X-Flight-Token':token,...(body?{'Content-Type':'application/json'}:{})},body:body?JSON.stringify(body):undefined});const data=await response.json();if(!response.ok)throw Error(data.error||`HTTP ${response.status}`);return data;}
async function poll(){if(!token)return;
  try{const state=await api('/api/state');connected=true;text('bridge-message',`Connected · ${state.status}`);text('connection-chip',state.status==='RUNNING'?'LIVE SITL':'SIM CONNECTED');$('connection-chip').classList.add('is-live');$('connection-chip').classList.remove('is-offline');
    busy=state.status==='RUNNING';$('start-flight').disabled=busy;$('tracking').disabled=busy;document.querySelectorAll('.demo-option').forEach(b=>b.disabled=busy);
    const viewedRun=bridgeRunForSelection(state,routes[selected]?.path,ownedRun);
    if(viewedRun)ownedRun=viewedRun;
    activeLive=!!viewedRun&&state.status==='RUNNING';
    if(viewedRun){if(lastRun!==state.run){lastRun=state.run;graphs.clear();displaySource='none';setRoute(state.route.path);}
      const t=state.telemetry||{},track=(state.track||[]).map(p=>[p[0]/1e7,p[1]/1e7]);renderTelemetry(t,track,'live',state.status,state.result,state.phase,state.telemetry_age_s);
    }else if(activeLive){status('STARTING SITL','WAITING FOR GPS');text('launch-feedback','ArduCopter is checking its route and health.');}
    else if(!archiveForSelection()){status('READY FOR SITL','ROUTE PREVIEW');}
  }catch(error){connected=false;activeLive=false;busy=false;text('connection-chip','BRIDGE OFFLINE');$('connection-chip').classList.remove('is-live');$('connection-chip').classList.add('is-offline');
    text('bridge-message',`Bridge unavailable: ${error.message}. Restart it and use its new access code.`);$('start-flight').disabled=false;$('tracking').disabled=false;document.querySelectorAll('.demo-option').forEach(b=>b.disabled=false);
    if(!archiveForSelection())status('READY FOR SITL','BRIDGE OFFLINE');
  }
}
function init3d(){const host=$('model-view'),scene=new THREE.Scene(),camera=new THREE.PerspectiveCamera(36,1,.001,1000);
  const renderer=new THREE.WebGLRenderer({antialias:true,alpha:true});renderer.setPixelRatio(Math.min(devicePixelRatio,2));renderer.outputColorSpace=THREE.SRGBColorSpace;host.append(renderer.domElement);
  scene.add(new THREE.HemisphereLight(0xffffff,0x6b8b83,3));const key=new THREE.DirectionalLight(0xffffff,3);key.position.set(3,5,7);scene.add(key);const rim=new THREE.DirectionalLight(0x58ba8e,2);rim.position.set(-4,3,-4);scene.add(rim);
  const controls=new OrbitControls(camera,renderer.domElement);controls.enableDamping=true;controls.autoRotate=true;controls.autoRotateSpeed=.45;
  new GLTFLoader().load('./assets/quad_delivery_vtol.glb',gltf=>{const model=gltf.scene;model.rotation.x=-Math.PI/2;scene.add(model);const bounds=new THREE.Box3().setFromObject(model),size=bounds.getSize(new THREE.Vector3()),center=bounds.getCenter(new THREE.Vector3());model.position.sub(center);const span=Math.max(size.x,size.y,size.z)||1;camera.position.set(span*1.35,span*1.05,span*1.5);camera.near=span/1000;camera.far=span*100;camera.updateProjectionMatrix();controls.target.set(0,0,0);controls.update();},undefined,()=>{const e=document.createElement('p');e.className='model-error';e.textContent='3D model could not load.';host.append(e);});
  let last=0,width=0,height=0;function animate(now){requestAnimationFrame(animate);if(document.hidden||now-last<1000/24)return;last=now;const box=host.getBoundingClientRect();if(box.bottom<0||box.top>innerHeight)return;if(box.width!==width||box.height!==height){width=box.width;height=box.height;renderer.setSize(width,height,false);camera.aspect=width/height;camera.updateProjectionMatrix();}controls.update();renderer.render(scene,camera);}requestAnimationFrame(animate);
}
async function init(){initMap();init3d();
  $('track-form').addEventListener('submit',event=>{event.preventDefault();if(!selectRoute($('tracking').value.trim().toUpperCase())){$('tracking-error').classList.add('show');$('tracking').setAttribute('aria-invalid','true');}});
  $('connect-button').addEventListener('click',()=>{token=$('bridge-token').value.trim();if(!token){text('bridge-message','Paste the access code printed by the local simulator.');return;}poll();});
  $('start-flight').addEventListener('click',async()=>{if(!connected){text('launch-feedback','Connect the local simulator first.');return;}const altitude=Number($('route-altitude').value),speed=Number($('route-speed').value);if(!Number.isFinite(altitude)||!Number.isFinite(speed)||altitude<20||altitude>80||speed<.5||speed>5){text('launch-feedback','Use 20–80 m altitude and 0.5–5 m/s speed.');return;}
    try{$('start-flight').disabled=true;const result=await api('/api/geographic',{route_id:selected,altitude,speed});ownedRun=result.run;activeLive=true;text('launch-feedback',`Starting ${result.run}; waiting for flight-controller health and GPS.`);status('STARTING SITL','WAITING FOR GPS');poll();}catch(error){text('launch-feedback',error.message);$('start-flight').disabled=false;}});
  try{const [routesResponse,archiveResponse]=await Promise.all([fetch('./data/columbia-routes.json'),fetch('./data/archive.json')]);if(!routesResponse.ok||!archiveResponse.ok)throw Error('Route data unavailable');routes=await routesResponse.json();archive=await archiveResponse.json();
    let index=0;for(const [id,route] of Object.entries(routes)){index++;const button=document.createElement('button');button.type='button';button.className='demo-option';button.dataset.tracking=id;button.setAttribute('aria-pressed','false');button.innerHTML=`<span class="demo-num">${String(index).padStart(2,'0')}</span><span><strong></strong><small></small></span><span class="demo-arrow">↗</span>`;button.querySelector('strong').textContent=id;button.querySelector('small').textContent=`${route.store} → ${route.destination} · ${km(route.metrics.outbound_m)}`;button.onclick=()=>selectRoute(id);$('route-options').append(button);}selectRoute(selected);
  }catch(error){text('launch-feedback',error.message);$('start-flight').disabled=true;}
  setInterval(poll,1000);function loop(now){showReplay(now);requestAnimationFrame(loop);}requestAnimationFrame(loop);
}
init();
