const definitions=[
  {id:'battery',label:'Battery',unit:'%',low:0,high:100,color:'#43a364'},
  {id:'altitude',label:'Altitude',unit:'m',low:0,high:80,color:'#5189ad'},
  {id:'groundspeed',label:'Groundspeed',unit:'m/s',low:0,high:8,color:'#dc9250'},
  {id:'roll',label:'Roll',unit:'°',low:-30,high:30,color:'#8269c4'},
  {id:'pitch',label:'Pitch',unit:'°',low:-30,high:30,color:'#cf6e76'},
  {id:'heading',label:'Heading',unit:'°',low:0,high:360,color:'#348a82'}
];
export const graphDefinitions=definitions;
export function metricsFrom(t){
  const p=t?.GLOBAL_POSITION_INT,s=t?.SYS_STATUS,a=t?.ATTITUDE;
  const finite=x=>typeof x==='number'&&Number.isFinite(x);
  return {
    battery:finite(s?.battery_remaining)&&s.battery_remaining>=0?s.battery_remaining:null,
    altitude:finite(p?.relative_alt)?p.relative_alt/1000:null,
    groundspeed:finite(p?.vx)&&finite(p?.vy)?Math.hypot(p.vx,p.vy)/100:null,
    roll:finite(a?.roll)?a.roll*180/Math.PI:null,
    pitch:finite(a?.pitch)?a.pitch*180/Math.PI:null,
    heading:finite(p?.hdg)&&p.hdg>=0&&p.hdg<36000?p.hdg/100:null
  };
}
const flightModes={0:'STABILIZE',1:'ACRO',2:'ALT HOLD',3:'AUTO',4:'GUIDED',5:'LOITER',6:'RTL',9:'LAND',16:'POSHOLD',17:'BRAKE',20:'GUIDED NOGPS'};
export function diagnosticsFrom(t,nowSeconds){
  const s=t?.SYS_STATUS,h=t?.HEARTBEAT,g=t?.GPS_RAW_INT,e=t?.EXTENDED_SYS_STATE,m=t?.MISSION_CURRENT,p=t?.GLOBAL_POSITION_INT;
  const finite=x=>typeof x==='number'&&Number.isFinite(x);
  const sensor=(bit)=>finite(s?.onboard_control_sensors_health)?!!(s.onboard_control_sensors_health&bit):null;
  const age=finite(p?.wall)&&finite(nowSeconds)?Math.max(0,nowSeconds-p.wall):null;
  return {
    mode:finite(h?.custom_mode)?(flightModes[h.custom_mode]||`MODE ${h.custom_mode}`):null,
    current:finite(s?.current_battery)&&s.current_battery>=0?s.current_battery/100:null,
    voltage:finite(s?.voltage_battery)&&s.voltage_battery>0&&s.voltage_battery<65535?s.voltage_battery/1000:null,
    gpsFix:finite(g?.fix_type)?g.fix_type:null,
    satellites:finite(g?.satellites_visible)&&g.satellites_visible<255?g.satellites_visible:null,
    estimator:sensor(1<<21),rc:sensor(1<<16),
    armed:finite(h?.base_mode)?!!(h.base_mode&128):null,
    landed:finite(e?.landed_state)?e.landed_state===1:null,
    missionItem:finite(m?.seq)?m.seq:null,
    telemetryAge:age
  };
}
export function geographicDistance(a,b){const rad=Math.PI/180,dlat=(b[0]-a[0])*rad,dlon=(b[1]-a[1])*rad;return 6371008.8*2*Math.asin(Math.min(1,Math.sqrt(Math.sin(dlat/2)**2+Math.cos(a[0]*rad)*Math.cos(b[0]*rad)*Math.sin(dlon/2)**2)));}
export function trackDistance(points){let metres=0;for(let i=1;i<points.length;i++){const d=geographicDistance(points[i-1],points[i]);if(d>.35&&d<150)metres+=d;}return metres;}
export function bridgeRunForSelection(state,selectedPath,ownedRun){
  if(state?.kind!=='geographic'||!state.route||JSON.stringify(state.route.path)!==JSON.stringify(selectedPath))return null;
  if(state.status==='RUNNING')return !ownedRun||ownedRun===state.run?state.run:null;
  return ownedRun===state.run?state.run:null;
}

const $=id=>document.getElementById(id);
const svgNS='http://www.w3.org/2000/svg';
function svg(tag,attrs={}){const e=document.createElementNS(svgNS,tag);for(const [k,v] of Object.entries(attrs))e.setAttribute(k,v);return e;}
export class Graphs{
  constructor(host){this.host=host;this.samples=[];this.lastKey=null;this.cards={};
    for(const def of definitions){const card=document.createElement('article');card.className='graph-card';card.style.setProperty('--graph-color',def.color);
      const top=document.createElement('div');top.className='graph-top';const label=document.createElement('span');label.textContent=def.label;const value=document.createElement('strong');value.textContent='—';top.append(label,value);
      const plot=svg('svg',{viewBox:'0 0 320 90',preserveAspectRatio:'none','role':'img','aria-label':`${def.label} telemetry trend`});for(const y of [12,45,78])plot.append(svg('line',{x1:0,x2:320,y1:y,y2:y}));const fill=svg('path',{class:'graph-fill'}),line=svg('path',{class:'graph-line'});plot.append(fill,line);
      const bottom=document.createElement('div');bottom.className='graph-bottom';bottom.innerHTML='<span>OLDER</span><span>NEWEST →</span>';card.append(top,plot,bottom);host.append(card);this.cards[def.id]={value,line,fill};
    }
  }
  clear(){this.samples=[];this.lastKey=null;for(const def of definitions){const card=this.cards[def.id];card.value.textContent='—';card.line.setAttribute('d','');card.fill.setAttribute('d','');}}
  append(t){const key=t?.GLOBAL_POSITION_INT?.wall??t?.GLOBAL_POSITION_INT?.time_boot_ms;if(key==null||key===this.lastKey)return;this.lastKey=key;this.samples.push(metricsFrom(t));if(this.samples.length>100)this.samples.shift();this.draw();}
  draw(){for(const def of definitions){const card=this.cards[def.id],value=this.samples.at(-1)?.[def.id],number=Number.isFinite(value)?value:null;card.value.textContent=number==null?'—':`${number.toFixed(def.id==='battery'?0:1)} ${def.unit}`;
      const points=this.samples.map((sample,i)=>{const v=sample[def.id];return Number.isFinite(v)?[i*320/Math.max(1,this.samples.length-1),78-(Math.max(def.low,Math.min(def.high,v))-def.low)/(def.high-def.low)*66]:null;});
      let d='',segment=[];const flush=()=>{if(segment.length){d+=segment.map((p,i)=>`${i?'L':'M'}${p[0].toFixed(1)},${p[1].toFixed(1)}`).join(' ')+' ';segment=[];}};
      for(let i=0;i<points.length;i++){const p=points[i];if(p){if(def.id==='heading'&&i>0&&Number.isFinite(this.samples[i-1]?.heading)&&Math.abs(this.samples[i].heading-this.samples[i-1].heading)>180)flush();segment.push(p);}else flush();}flush();card.line.setAttribute('d',d.trim());
      const valid=points.filter(Boolean);card.fill.setAttribute('d',def.id!=='heading'&&valid.length>1?`${d}L${valid.at(-1)[0].toFixed(1)},78 L${valid[0][0].toFixed(1)},78 Z`:'');
    }}
}
