// Flightpath-style street map. Position and trace are supplied by SITL telemetry.
const $=id=>document.getElementById(id),NS='http://www.w3.org/2000/svg';
let map,tiles,svg,routeLine,traceLine,storePin,endPin,dronePin,zoom=15,center=[0,0],path=[],track=[],position=null,tileKey='';
const node=(tag,attrs)=>{const e=document.createElementNS(NS,tag);for(const [key,value] of Object.entries(attrs))e.setAttribute(key,value);return e;};
function project([lat,lon],z=zoom){const sine=Math.sin(lat*Math.PI/180),scale=256*2**z;return [(lon+180)/360*scale,(.5-Math.log((1+sine)/(1-sine))/(4*Math.PI))*scale];}
function screen(point){const p=project(point);return [p[0]-center[0]+map.clientWidth/2,p[1]-center[1]+map.clientHeight/2];}
function place(pin,point){const [x,y]=screen(point);pin.style.left=`${x}px`;pin.style.top=`${y}px`;}
function fit(){
  if(!map.clientWidth||!map.clientHeight||!path.length)return;
  const w=map.clientWidth,h=map.clientHeight;
  for(let z=17;z>=12;z--){const p=path.map(x=>project(x,z)),xs=p.map(x=>x[0]),ys=p.map(x=>x[1]),bounds=[Math.min(...xs),Math.max(...xs),Math.min(...ys),Math.max(...ys)];
    if(z===12||(bounds[1]-bounds[0]<w-130&&bounds[3]-bounds[2]<h-120)){zoom=z;center=[(bounds[0]+bounds[1])/2,(bounds[2]+bounds[3])/2];break;}
  }
  const key=JSON.stringify([zoom,center,w,h]);
  if(key!==tileKey){tileKey=key;const frag=document.createDocumentFragment();const limit=2**zoom;
    for(let y=Math.floor((center[1]-h/2)/256);y<=Math.floor((center[1]+h/2)/256);y++)for(let x=Math.floor((center[0]-w/2)/256);x<=Math.floor((center[0]+w/2)/256);x++){
      if(y<0||y>=limit)continue;const img=document.createElement('img');img.className='map-tile';img.alt='';img.draggable=false;
      img.style.left=`${x*256-center[0]+w/2}px`;img.style.top=`${y*256-center[1]+h/2}px`;
      img.src=`https://tile.openstreetmap.org/${zoom}/${((x%limit)+limit)%limit}/${y}.png`;
      img.onerror=()=>{$('map-error').hidden=false;};frag.append(img);
    }tiles.replaceChildren(frag);
  }
  svg.setAttribute('viewBox',`0 0 ${w} ${h}`);routeLine.setAttribute('points',path.map(screen).map(x=>x.join(',')).join(' '));
  place(storePin,path[0]);place(endPin,path.at(-1));drawTrace();
}
function drawTrace(){if(!map||!path.length)return;traceLine.setAttribute('points',track.map(screen).map(x=>x.join(',')).join(' '));dronePin.hidden=!position;if(position)place(dronePin,position);}
export function setRoute(next){path=next;track=[];position=null;tileKey='';fit();}
export function setFlight(nextTrack,nextPosition){track=nextTrack;position=nextPosition;drawTrace();}
export function initMap(){
  map=$('map');tiles=document.createElement('div');tiles.className='map-tiles';svg=node('svg',{class:'map-overlay'});
  routeLine=node('polyline',{fill:'none',stroke:'#3f8e61','stroke-width':4,'stroke-dasharray':'8 8','stroke-linecap':'round','stroke-linejoin':'round'});
  traceLine=node('polyline',{fill:'none',stroke:'#137b61','stroke-width':5,'stroke-linecap':'round','stroke-linejoin':'round'});svg.append(routeLine,traceLine);
  storePin=document.createElement('div');storePin.className='map-pin store';storePin.innerHTML='<div class="poi-pin">S</div>';
  endPin=document.createElement('div');endPin.className='map-pin end';endPin.innerHTML='<div class="poi-pin end">⌖</div>';
  dronePin=document.createElement('div');dronePin.className='map-pin drone';dronePin.innerHTML='<svg viewBox="0 0 28 28" fill="none" aria-hidden="true"><circle cx="5" cy="5" r="3" stroke="currentColor" stroke-width="2"/><circle cx="23" cy="5" r="3" stroke="currentColor" stroke-width="2"/><circle cx="5" cy="23" r="3" stroke="currentColor" stroke-width="2"/><circle cx="23" cy="23" r="3" stroke="currentColor" stroke-width="2"/><path d="M7 7l5 5m9-5l-5 5M7 21l5-5m9 5l-5-5M14 10l4 4-4-4 4-4Z" stroke="currentColor" stroke-width="2" stroke-linejoin="round"/></svg>';dronePin.hidden=true;
  const attribution=document.createElement('div');attribution.className='map-attribution';attribution.innerHTML='© <a href="https://www.openstreetmap.org/copyright" target="_blank" rel="noopener">OpenStreetMap contributors</a>';
  const error=document.createElement('div');error.id='map-error';error.className='map-error';error.hidden=true;error.textContent='Street tiles unavailable. Route and GPS trace remain visible.';
  map.append(tiles,svg,storePin,endPin,dronePin,attribution,error);new ResizeObserver(fit).observe(map);
}
