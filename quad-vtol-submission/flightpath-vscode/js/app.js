(() => {
  'use strict';
  const $ = id => document.getElementById(id);
  // Presentation coordinates only. These are not cleared or navigable flight paths.
  const shipments = {
    'UAV-COLA-2047': {
      vehicle: 'UAV-07', store: 'Publix · Gervais Place',
      storeAddress: '501 Gervais St · Columbia, SC',
      destination: 'Finlay Park', destinationAddress: '930 Laurel St · Columbia, SC',
      path: [
        [33.998339, -81.044467], [33.99905, -81.04415],
        [34.00005, -81.04380], [34.00128, -81.04343],
        [34.00240, -81.04324], [34.00338, -81.04292],
        [34.00455, -81.04248], [34.00548, -81.04212],
        [34.006317, -81.041680]
      ]
    },
    'UAV-COLA-3186': {
      vehicle: 'UAV-12', store: 'Mast General Store',
      storeAddress: '1601 Main St · Columbia, SC',
      destination: 'Riverfront Park', destinationAddress: '312 Laurel St · Columbia, SC',
      path: [
        [34.00667, -81.03611], [34.00655, -81.038],
        [34.00620, -81.040], [34.00550, -81.042],
        [34.00480, -81.044], [34.00420, -81.046],
        [34.00390, -81.048], [34.0037859, -81.0512424]
      ]
    },
    'UAV-COLA-5729': {
      vehicle: 'UAV-23', store: 'Publix · Gervais Place',
      storeAddress: '501 Gervais St · Columbia, SC',
      destination: 'Columbia Museum of Art',
      destinationAddress: '1515 Main St · Columbia, SC',
      path: [
        [33.998339, -81.044467], [33.99960, -81.04280],
        [34.00100, -81.04140], [34.00260, -81.03990],
        [34.00410, -81.03830], [34.005804, -81.0364504]
      ]
    }
  };
  let selectedId = 'UAV-COLA-2047';
  let path = shipments[selectedId].path;
  const preparationSeconds = 30;
  const flightSeconds = 118;
  let phase = 'preparing';
  let preparationLeft = preparationSeconds;
  let progress = 0;
  let paused = false;
  let speed = 1;
  let previousFrame = performance.now();

  let distance, totalDistance;
  function measureRoute() {
    distance = [0];
    for (let i = 1; i < path.length; i++) {
      const a = path[i - 1], b = path[i];
      distance.push(distance[i - 1] + Math.hypot(
        (b[1] - a[1]) * Math.cos(a[0] * Math.PI / 180), b[0] - a[0]
      ));
    }
    totalDistance = distance.at(-1);
  }
  measureRoute();
  function position(fraction) {
    const d = Math.max(0, Math.min(1, fraction)) * totalDistance;
    let i = 1;
    while (i < distance.length - 1 && distance[i] < d) i++;
    const f = (d - distance[i - 1]) / (distance[i] - distance[i - 1]);
    return [path[i - 1][0] + (path[i][0] - path[i - 1][0]) * f,
      path[i - 1][1] + (path[i][1] - path[i - 1][1]) * f];
  }

  // Small, local Web Mercator tile renderer. Tiles come from OpenStreetMap;
  // positioning and route layers do not depend on a third-party JS/CSS CDN.
  const mapElement = $('map');
  const tiles = document.createElement('div');
  tiles.className = 'map-tiles';
  const overlay = document.createElementNS('http://www.w3.org/2000/svg', 'svg');
  overlay.setAttribute('class', 'map-overlay');
  const routeBorder = document.createElementNS('http://www.w3.org/2000/svg', 'polyline');
  routeBorder.setAttribute('fill', 'none');
  routeBorder.setAttribute('stroke', '#fff');
  routeBorder.setAttribute('stroke-width', '10');
  routeBorder.setAttribute('stroke-linecap', 'round');
  routeBorder.setAttribute('stroke-linejoin', 'round');
  routeBorder.setAttribute('opacity', '.9');
  const routeGuide = routeBorder.cloneNode();
  routeGuide.setAttribute('stroke', '#488c61');
  routeGuide.setAttribute('stroke-width', '4');
  routeGuide.setAttribute('stroke-dasharray', '6 10');
  const routeTravelled = routeBorder.cloneNode();
  routeTravelled.setAttribute('stroke', '#15966a');
  routeTravelled.setAttribute('stroke-width', '5');
  overlay.append(routeBorder, routeGuide, routeTravelled);
  function pin(html, extra) {
    const e = document.createElement('div');
    e.className = 'map-pin ' + extra;
    e.innerHTML = html;
    e.setAttribute('aria-hidden', 'true');
    return e;
  }
  const storePin = pin('<div class="poi-pin">S</div>', 'store');
  const endPin = pin('<div class="poi-pin end">⌖</div>', 'end');
  const dronePin = pin('<div class="drone-pin"><svg viewBox="0 0 28 28" fill="none" aria-hidden="true"><circle cx="5" cy="5" r="3" stroke="currentColor" stroke-width="2"/><circle cx="23" cy="5" r="3" stroke="currentColor" stroke-width="2"/><circle cx="5" cy="23" r="3" stroke="currentColor" stroke-width="2"/><circle cx="23" cy="23" r="3" stroke="currentColor" stroke-width="2"/><path d="M7 7l5 5m9-5l-5 5M7 21l5-5m9 5l-5-5M14 10l4 4-4 4-4-4 4-4Z" stroke="currentColor" stroke-width="2" stroke-linejoin="round"/></svg></div>', 'drone');
  const zoomBox = document.createElement('div');
  zoomBox.className = 'map-zoom';
  zoomBox.innerHTML = '<button type="button" aria-label="Zoom in">+</button><button type="button" aria-label="Zoom out">−</button>';
  mapElement.append(tiles, overlay, storePin, endPin, dronePin, zoomBox);
  let zoom = 15;
  let center = [34.0023, -81.0432];
  let centerWorld = null;
  function project(latlng, z = zoom) {
    const [lat, lon] = latlng;
    const sine = Math.sin(lat * Math.PI / 180);
    const scale = 256 * Math.pow(2, z);
    return [(lon + 180) / 360 * scale,
      (0.5 - Math.log((1 + sine) / (1 - sine)) / (4 * Math.PI)) * scale];
  }
  function screen(latlng) {
    const p = project(latlng);
    return [p[0] - centerWorld[0] + mapElement.clientWidth / 2,
      p[1] - centerWorld[1] + mapElement.clientHeight / 2];
  }
  function fitRoute() {
    const width = mapElement.clientWidth, height = mapElement.clientHeight;
    for (let z = 17; z >= 13; z--) {
      const points = path.map(p => project(p, z));
      if (Math.max(...points.map(p => p[0])) - Math.min(...points.map(p => p[0])) < width - 145 &&
          Math.max(...points.map(p => p[1])) - Math.min(...points.map(p => p[1])) < height - 145) {
        zoom = z;
        const minX = Math.min(...points.map(p => p[0]));
        const maxX = Math.max(...points.map(p => p[0]));
        const minY = Math.min(...points.map(p => p[1]));
        const maxY = Math.max(...points.map(p => p[1]));
        centerWorld = [(minX + maxX) / 2, (minY + maxY) / 2];
        renderMap();
        return;
      }
    }
    zoom = 13;
    centerWorld = project(center);
    renderMap();
  }
  function renderMap() {
    if (!centerWorld || !mapElement.clientWidth || !mapElement.clientHeight) return;
    const w = mapElement.clientWidth, h = mapElement.clientHeight;
    const firstX = Math.floor((centerWorld[0] - w / 2) / 256);
    const lastX = Math.floor((centerWorld[0] + w / 2) / 256);
    const firstY = Math.floor((centerWorld[1] - h / 2) / 256);
    const lastY = Math.floor((centerWorld[1] + h / 2) / 256);
    const fragment = document.createDocumentFragment();
    const limit = Math.pow(2, zoom);
    for (let y = firstY; y <= lastY; y++) {
      for (let x = firstX; x <= lastX; x++) {
        if (y < 0 || y >= limit) continue;
        const image = document.createElement('img');
        image.className = 'map-tile';
        image.alt = '';
        image.draggable = false;
        image.style.left = (x * 256 - centerWorld[0] + w / 2) + 'px';
        image.style.top = (y * 256 - centerWorld[1] + h / 2) + 'px';
        image.src = `https://tile.openstreetmap.org/${zoom}/${((x % limit) + limit) % limit}/${y}.png`;
        fragment.append(image);
      }
    }
    tiles.replaceChildren(fragment);
    overlay.setAttribute('viewBox', `0 0 ${w} ${h}`);
    const allPoints = path.map(p => screen(p).join(',')).join(' ');
    routeBorder.setAttribute('points', allPoints);
    routeGuide.setAttribute('points', allPoints);
    for (const [element, p] of [[storePin, path[0]], [endPin, path.at(-1)]]) {
      const [x, y] = screen(p);
      element.style.left = x + 'px';
      element.style.top = y + 'px';
    }
    drawPosition();
  }
  function drawPosition() {
    if (!centerWorld) return;
    const [x, y] = screen(position(progress));
    dronePin.style.left = x + 'px';
    dronePin.style.top = y + 'px';
    const travelled = [path[0]];
    for (let i = 1; i < path.length - 1 && distance[i] / totalDistance < progress; i++) travelled.push(path[i]);
    travelled.push(position(progress));
    routeTravelled.setAttribute('points', travelled.map(p => screen(p).join(',')).join(' '));
  }
  function adjustZoom(delta) {
    const next = Math.max(12, Math.min(18, zoom + delta));
    if (next === zoom) return;
    const factor = Math.pow(2, next - zoom);
    const mapCenter = 256 * Math.pow(2, zoom) / 2;
    centerWorld = [(centerWorld[0] - mapCenter) * factor + mapCenter * factor,
      (centerWorld[1] - mapCenter) * factor + mapCenter * factor];
    zoom = next;
    renderMap();
  }
  zoomBox.children[0].addEventListener('click', () => adjustZoom(1));
  zoomBox.children[1].addEventListener('click', () => adjustZoom(-1));
  mapElement.addEventListener('wheel', event => {
    event.preventDefault();
    adjustZoom(event.deltaY < 0 ? 1 : -1);
  }, {passive: false});
  let drag = null;
  mapElement.addEventListener('pointerdown', event => {
    if (event.target.closest('button')) return;
    drag = [event.clientX, event.clientY];
    mapElement.classList.add('dragging');
    mapElement.setPointerCapture(event.pointerId);
  });
  mapElement.addEventListener('pointermove', event => {
    if (!drag) return;
    centerWorld[0] -= event.clientX - drag[0];
    centerWorld[1] -= event.clientY - drag[1];
    drag = [event.clientX, event.clientY];
    renderMap();
  });
  const endDrag = () => { drag = null; mapElement.classList.remove('dragging'); };
  mapElement.addEventListener('pointerup', endDrag);
  mapElement.addEventListener('pointercancel', endDrag);
  new ResizeObserver(fitRoute).observe(mapElement);

  function draw() {
    const p = position(progress);
    const remaining = phase === 'delivered' ? 0 : Math.ceil(
      ((phase === 'preparing' ? preparationLeft : 0) + (1 - progress) * flightSeconds) / speed
    );
    $('lat').textContent = p[0].toFixed(5) + '° N';
    $('lng').textContent = Math.abs(p[1]).toFixed(5) + '° W';
    $('alt').textContent = phase === 'flight' ? '43 m' : '0 m';
    $('velocity').textContent = phase === 'flight' && !paused ? '28 km/h' : '0 km/h';
    $('eta').textContent = phase === 'delivered' ? 'Delivered' : remaining >= 60
      ? `${Math.floor(remaining / 60)} min ${String(remaining % 60).padStart(2, '0')} sec`
      : `${remaining} sec`;
    $('eta-caption').textContent = phase === 'preparing'
      ? `Store preparing your item · flight begins in ${Math.ceil(preparationLeft / speed)} sec`
      : phase === 'delivered' ? 'Your package has reached the delivery point.'
      : 'Arriving at approximately ' + new Intl.DateTimeFormat('en-US', {
        hour: 'numeric', minute: '2-digit', timeZone: 'America/New_York'
      }).format(Date.now() + remaining * 1000) + ' ET';
    $('progressfill').style.width = (progress * 100).toFixed(1) + '%';
    $('progressbar').setAttribute('aria-valuenow', Math.round(progress));
    $('progresspct').textContent = phase === 'preparing' ? 'Preparing for pickup'
      : Math.round(progress * 100) + '% of route';
    $('flight-status').textContent = phase === 'delivered' ? 'DELIVERED'
      : paused ? 'PAUSED' : phase === 'preparing' ? 'PREPARING ORDER' : 'IN FLIGHT';
    $('mapbadge').innerHTML = phase === 'preparing' ? '● &nbsp; PREPARING AT STORE'
      : phase === 'delivered' ? '● &nbsp; DELIVERY COMPLETE' : '● &nbsp; SIMULATED LIVE GPS';
    $('updated').textContent = phase === 'delivered' ? 'Simulation complete'
      : paused ? 'Simulation paused' : phase === 'preparing' ? 'UAV waiting at the store'
      : 'Position updated just now';
    drawPosition();
  }
  function reset() {
    phase = 'preparing';
    preparationLeft = preparationSeconds;
    progress = 0;
    paused = false;
    $('pause').textContent = '❚❚ Pause flight';
    fitRoute();
    draw();
  }
  function selectShipment(id) {
    if (!Object.hasOwn(shipments, id)) return false;
    selectedId = id;
    const shipment = shipments[id];
    path = shipment.path;
    measureRoute();
    $('tracking').value = id;
    $('store-name').textContent = shipment.store;
    $('store-address').textContent = shipment.storeAddress;
    $('destination-name').textContent = shipment.destination;
    $('destination-address').textContent = shipment.destinationAddress;
    $('package-id').textContent = id;
    $('uav-id').textContent = shipment.vehicle;
    $('map').setAttribute('aria-label', `Street map of the route from ${shipment.store} to ${shipment.destination}`);
    for (const option of document.querySelectorAll('.demo-option')) {
      const active = option.dataset.tracking === id;
      option.classList.toggle('selected', active);
      option.setAttribute('aria-pressed', String(active));
    }
    $('tracking-error').classList.remove('show');
    $('tracking').removeAttribute('aria-invalid');
    reset();
    return true;
  }
  function tick(now) {
    const elapsed = Math.min((now - previousFrame) / 1000, .5) * speed;
    previousFrame = now;
    if (!paused) {
      if (phase === 'preparing') {
        preparationLeft = Math.max(0, preparationLeft - elapsed);
        if (!preparationLeft) phase = 'flight';
      } else if (phase === 'flight') {
        progress = Math.min(1, progress + elapsed / flightSeconds);
        if (progress === 1) {
          phase = 'delivered';
          $('pause').textContent = '✓ Delivered';
        }
      }
    }
    draw();
    requestAnimationFrame(tick);
  }
  $('track-form').addEventListener('submit', event => {
    event.preventDefault();
    if (!selectShipment($('tracking').value.trim().toUpperCase())) {
      $('tracking-error').classList.add('show');
      $('tracking').setAttribute('aria-invalid', 'true');
    }
  });
  for (const option of document.querySelectorAll('.demo-option')) {
    option.addEventListener('click', () => selectShipment(option.dataset.tracking));
  }
  $('pause').addEventListener('click', () => {
    if (phase === 'delivered') return;
    paused = !paused;
    $('pause').textContent = paused ? '▶ Resume flight' : '❚❚ Pause flight';
    draw();
  });
  $('replay').addEventListener('click', reset);
  $('speed').addEventListener('click', () => {
    speed = speed === 1 ? 2 : speed === 2 ? 4 : 1;
    $('speed').textContent = speed + '× speed';
    draw();
  });
  reset();
  requestAnimationFrame(tick);
})();
