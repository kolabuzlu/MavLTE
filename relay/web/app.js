// MavLTE's web page: the Aircraft card that mavweb.py works out (as MavLTE's window shows it), asked for every
// second while the page is in view.
"use strict";

(() => {
  const POLL_MS = 1000;
  const COLORS = ["text", "muted", "dim", "green", "blue", "amber", "red", "off"];
  const $ = (id) => document.getElementById(id);

  let size = 1; // the photo size chosen on this phone (Small, Medium, Large)
  try {
    const saved = localStorage.getItem("mavlte.size");
    if (saved === "0" || saved === "1" || saved === "2") size = Number(saved);
  } catch (e) { /* private window: Medium */ }

  let state = null; // the newest card from the server
  let timer = 0;
  let failures = 0;
  let lastNews = 0; // Date.now() of the last answer
  let voiceSending = false;
  let clickedAt = -1e9; // Snapshot: the photo that comes next opens in the viewer
  let shownPhoto = null; // the id on the card
  let photos = []; // the viewer's, newest first
  let at = 0; // the viewer's place in them

  // the moving map
  const TILE = 256;
  const ZOOM_MIN = 3;
  const ZOOM_MAX = 19;
  const TRACK_MOST = 2000;
  const TILES = "https://server.arcgisonline.com/ArcGIS/rest/services/World_Imagery/MapServer/tile/";
  const view = { zoom: 16, follow: true, lat: 0, lon: 0, fix: null, track: [], tiles: new Map() };
  try {
    const zoom = Number(localStorage.getItem("mavlte.zoom"));
    if (zoom >= ZOOM_MIN && zoom <= ZOOM_MAX) view.zoom = zoom;
  } catch (e) { /* 16: about a kilometre across */ }

  // -- painting

  function paint(el, text, color) {
    if (el.textContent !== text) el.textContent = text;
    for (const c of COLORS) el.classList.toggle("c-" + c, c === color);
  }

  function led(el, color) {
    el.className = "led " + color;
  }

  function age(seconds) { // as mavrelay.fmt_age
    const s = Math.max(0, Math.floor(seconds));
    if (s < 60) return s + " s";
    if (s < 3600) return Math.floor(s / 60) + " min";
    const m = Math.floor((s % 3600) / 60);
    return Math.floor(s / 3600) + " h" + (m ? " " + m + " min" : "");
  }

  function clock(unix) { // 07:18:01, in the phone's own time zone
    return new Date(unix * 1000).toLocaleTimeString("en-GB", { hour12: false });
  }

  function day(unix) { // 02 Oct 2026, 07:18:01
    const d = new Date(unix * 1000);
    return d.toLocaleDateString("en-GB", { day: "2-digit", month: "short", year: "numeric" }) + ", " + clock(unix);
  }

  function render(s) {
    state = s;
    $("signin").hidden = true;
    $("panel").hidden = false;
    document.title = s.name + " · MavLTE";
    $("name").textContent = s.name;
    led($("relay-led"), s.relay.led);
    $("relay-text").textContent = s.relay.text;
    $("version").textContent = s.version;

    led($("craft-led"), s.aircraft.led);
    paint($("craft-state"), s.aircraft.text, "text");
    [...$("bars").children].forEach((bar, i) => bar.classList.toggle("on", s.aircraft.bars !== null && i < s.aircraft.bars));
    paint($("link"), s.link, "text");
    paint($("loss"), s.loss, "text");

    paint($("position"), s.position.text, s.position.color);
    $("open-map").disabled = !s.position.map;
    $("copy").disabled = !s.position.copy;
    showFix(s);
    paint($("module"), s.module.text, s.module.color);
    paint($("temp"), s.temp.text, s.temp.color);

    const voice = $("voice");
    if (!voiceSending) voice.setAttribute("aria-checked", String(s.voice.on));
    voice.disabled = !s.voice.can;
    paint($("voice-text"), s.voice.text, s.voice.color);

    const cam = s.camera;
    paint($("cam-status"), cam.text, cam.color);
    $("progress").hidden = cam.fraction === null;
    if (cam.fraction !== null) $("progress-bar").style.width = Math.min(100, cam.fraction * 100).toFixed(1) + "%";
    $("snapshot").disabled = !cam.ready;
    for (const b of $("sizes").children) b.disabled = cam.busy;
    showPhoto(s.photo);
  }

  function showPhoto(p) {
    if (!p) {
      $("last").textContent = "";
      $("photo").hidden = true;
      return;
    }
    const parts = [];
    if (p.time) parts.push(clock(p.time));
    if (p.caption) parts.push(p.caption);
    $("last").textContent = parts.length ? "Last photo " + parts.join(" · ") : "Last photo";
    if (shownPhoto === p.id) return;
    shownPhoto = p.id;
    $("photo-img").src = "/photo/" + p.id + ".jpg";
    $("photo").hidden = false;
    if (Date.now() - clickedAt < 120000) { // the one asked for: into the viewer
      clickedAt = -1e9;
      openViewer(p.id);
    } else if (!$("viewer").hidden && at === 0) { // the viewer was on the newest: it stays on the newest
      openViewer(p.id);
    }
  }

  function paintSizes() {
    for (const b of $("sizes").children) b.setAttribute("aria-checked", String(Number(b.dataset.size) === size));
  }

  function showSignin() {
    clearTimeout(timer);
    state = null;
    shownPhoto = null;
    $("panel").hidden = true;
    closeViewer();
    closeMap();
    $("signin").hidden = false;
    $("name").textContent = "MavLTE";
    document.title = "MavLTE";
    led($("relay-led"), "off");
    $("relay-text").textContent = "Sign in to see the aircraft";
    $("password").focus();
  }

  function noNews(down) {
    const net = $("net");
    net.hidden = !down;
    document.body.classList.toggle("stale", down);
    if (down) {
      net.textContent = lastNews
        ? "No connection to the server for " + age((Date.now() - lastNews) / 1000) + ": trying again…"
        : "No connection to the server: trying again…";
    }
  }

  // -- talking to mavweb

  async function api(path, body) {
    const stop = new AbortController();
    const timeout = setTimeout(() => stop.abort(), 8000);
    const options = { cache: "no-store", credentials: "same-origin", signal: stop.signal };
    if (body !== undefined) {
      options.method = "POST";
      options.headers = { "Content-Type": "application/json", "X-MavLTE": "1" };
      options.body = JSON.stringify(body);
    }
    try {
      return await fetch(path, options);
    } finally {
      clearTimeout(timeout);
    }
  }

  async function why(response, otherwise) {
    try {
      return (await response.json()).error || otherwise;
    } catch (e) {
      return otherwise;
    }
  }

  async function poll() {
    clearTimeout(timer);
    if (document.hidden) return; // again when the page is back in view
    try {
      const r = await api("/api/state?size=" + size);
      if (r.status === 401) return showSignin();
      if (!r.ok) throw new Error("HTTP " + r.status);
      render(await r.json());
      failures = 0;
      lastNews = Date.now();
      noNews(false);
    } catch (e) {
      failures += 1;
      noNews(true);
    }
    timer = setTimeout(poll, failures ? Math.min(10000, 1000 * 2 ** Math.min(failures, 4)) : POLL_MS);
  }

  // -- what the phone does

  $("signin").addEventListener("submit", async (event) => {
    event.preventDefault();
    const button = $("signin-button");
    button.disabled = true;
    paint($("signin-error"), "", "red");
    try {
      const r = await api("/api/signin", { password: $("password").value });
      if (r.ok) {
        $("password").value = "";
        poll();
      } else {
        paint($("signin-error"), await why(r, "Cannot sign in"), "red");
      }
    } catch (e) {
      paint($("signin-error"), "No connection to the server", "red");
    } finally {
      button.disabled = false;
    }
  });

  $("signout").addEventListener("click", async () => {
    try {
      await api("/api/signout", {});
    } catch (e) { /* signed out here all the same */ }
    showSignin();
  });

  $("voice").addEventListener("click", async () => {
    const voice = $("voice");
    const on = voice.getAttribute("aria-checked") !== "true";
    voice.setAttribute("aria-checked", String(on)); // the relay confirms within a second or two
    voiceSending = true;
    try {
      const r = await api("/api/voice", { on });
      if (r.status === 401) return showSignin();
      if (!r.ok) paint($("voice-text"), await why(r, "Not connected to the relay"), "red");
    } catch (e) {
      paint($("voice-text"), "No connection to the server", "red");
    } finally {
      voiceSending = false;
    }
    poll();
  });

  $("snapshot").addEventListener("click", async () => {
    $("snapshot").disabled = true;
    try {
      const r = await api("/api/snapshot", { size });
      if (r.status === 401) return showSignin();
      if (r.ok) clickedAt = Date.now();
      else paint($("cam-status"), await why(r, "No photo"), "red");
    } catch (e) {
      paint($("cam-status"), "No connection to the server", "red");
    }
    poll();
  });

  for (const b of $("sizes").children) {
    b.addEventListener("click", () => {
      size = Number(b.dataset.size);
      try {
        localStorage.setItem("mavlte.size", String(size));
      } catch (e) { /* remembered until the page closes */ }
      paintSizes();
      poll();
    });
  }

  $("copy").addEventListener("click", async () => {
    const text = state && state.position.copy;
    if (!text) return;
    const copy = $("copy");
    try {
      await navigator.clipboard.writeText(text);
      copy.textContent = "Copied";
    } catch (e) { // no clipboard here: the coordinates, selected, for the phone's own Copy
      const range = document.createRange();
      range.selectNodeContents($("position"));
      getSelection().removeAllRanges();
      getSelection().addRange(range);
      copy.textContent = "Selected";
    }
    setTimeout(() => { copy.textContent = "Copy"; }, 1500);
  });

  // -- the viewer: a photo on the whole screen, and the others before it

  async function openViewer(id) {
    try {
      const r = await api("/api/photos");
      if (r.status === 401) return showSignin();
      if (!r.ok) return;
      photos = await r.json();
    } catch (e) {
      return;
    }
    if (!photos.length) return;
    at = Math.max(0, photos.findIndex((p) => p.id === id));
    $("viewer").hidden = false;
    document.body.classList.add("viewing");
    showViewer();
  }

  function closeViewer() {
    $("viewer").hidden = true;
    document.body.classList.toggle("viewing", !$("mapview").hidden);
  }

  function showViewer() {
    const p = photos[at];
    $("viewer-img").src = "/photo/" + p.id + ".jpg";
    const parts = [];
    if (p.time) parts.push(day(p.time));
    if (p.caption) parts.push(p.caption);
    $("viewer-caption").textContent = parts.join(" · ");
    $("older").disabled = at >= photos.length - 1;
    $("newer").disabled = at <= 0;
  }

  function step(delta) {
    const next = at + delta;
    if (next >= 0 && next < photos.length) {
      at = next;
      showViewer();
    }
  }

  $("photo").addEventListener("click", () => openViewer(shownPhoto));
  $("older").addEventListener("click", () => step(1));
  $("newer").addEventListener("click", () => step(-1));
  $("viewer-close").addEventListener("click", closeViewer);
  document.addEventListener("keydown", (event) => {
    if ($("viewer").hidden) return;
    if (event.key === "Escape") closeViewer();
    else if (event.key === "ArrowLeft") step(1);
    else if (event.key === "ArrowRight") step(-1);
  });
  let touchX = null;
  $("viewer").addEventListener("touchstart", (event) => { touchX = event.touches[0].clientX; }, { passive: true });
  $("viewer").addEventListener("touchend", (event) => {
    if (touchX === null) return;
    const dx = event.changedTouches[0].clientX - touchX;
    touchX = null;
    if (Math.abs(dx) > 50) step(dx > 0 ? 1 : -1); // swipe right: the older one, as a photo roll
  });

  // -- the moving map: the aircraft on Esri World Imagery, with its track (the page's server keeps the last
  // three hours or so). Drag it; pinch, the wheel or - and + zoom; Follow brings it back to the aircraft.

  function toPixel(lat, lon, zoom) { // where a point lies on the world map at that zoom, in pixels
    const size = TILE * 2 ** zoom;
    const s = Math.sin(Math.max(-85.05112878, Math.min(85.05112878, lat)) * Math.PI / 180);
    return [(lon + 180) / 360 * size, (0.5 - Math.log((1 + s) / (1 - s)) / (4 * Math.PI)) * size];
  }

  function toLatLon(x, y, zoom) {
    const size = TILE * 2 ** zoom;
    return [Math.atan(Math.sinh(Math.PI * (1 - 2 * y / size))) * 180 / Math.PI, x / size * 360 - 180];
  }

  function showFix(s) { // a new fix onto the track; the map's line as the card's Position
    const fix = s.fix;
    const last = view.track[view.track.length - 1];
    if (fix && (!last || fix.time > last[2])) {
      view.track.push([fix.lat, fix.lon, fix.time]);
      if (view.track.length > TRACK_MOST) view.track.shift();
    }
    view.fix = fix;
    paint($("map-where"), s.position.text, s.position.color);
    if (s.position.map) $("google").href = s.position.map;
    else $("google").removeAttribute("href");
    drawMap();
  }

  function drawMap() {
    if ($("mapview").hidden) return;
    const box = $("map");
    const w = box.clientWidth;
    const h = box.clientHeight;
    const zoom = view.zoom;
    const n = 2 ** zoom;
    const newest = view.track[view.track.length - 1];
    if (view.follow && newest) [view.lat, view.lon] = newest;
    const [cx, cy] = toPixel(view.lat, view.lon, zoom);
    const shown = new Set();
    const firstRow = Math.max(0, Math.floor((cy - h / 2) / TILE));
    const lastRow = Math.min(n - 1, Math.floor((cy + h / 2) / TILE));
    for (let row = firstRow; row <= lastRow; row++) {
      for (let col = Math.floor((cx - w / 2) / TILE); col <= Math.floor((cx + w / 2) / TILE); col++) {
        const id = zoom + "/" + col + "/" + row;
        shown.add(id);
        let img = view.tiles.get(id);
        if (!img) {
          img = new Image();
          img.alt = "";
          img.draggable = false;
          img.src = TILES + zoom + "/" + row + "/" + (((col % n) + n) % n); // Esri wants the row first
          $("tiles").append(img);
          view.tiles.set(id, img);
        }
        img.style.transform = "translate(" + Math.round(col * TILE - cx + w / 2) + "px, " +
          Math.round(row * TILE - cy + h / 2) + "px)";
      }
    }
    for (const [id, img] of view.tiles) {
      if (!shown.has(id)) {
        img.remove();
        view.tiles.delete(id);
      }
    }
    const points = view.track.map(([lat, lon]) => {
      const [x, y] = toPixel(lat, lon, zoom);
      return (x - cx + w / 2).toFixed(1) + "," + (y - cy + h / 2).toFixed(1);
    }).join(" ");
    $("trail").setAttribute("points", points);
    $("trail-casing").setAttribute("points", points);
    const plane = $("plane");
    plane.setAttribute("display", newest ? "inline" : "none");
    if (!newest) return;
    const [x, y] = toPixel(newest[0], newest[1], zoom);
    const now = view.fix && view.fix.time === newest[2] ? view.fix : null; // what the server said of the newest
    const heading = now ? now.heading : null; // an arrow where it points, while it moves; a dot when it does not
    plane.setAttribute("transform", "translate(" + (x - cx + w / 2).toFixed(1) + " " + (y - cy + h / 2).toFixed(1) +
      ") rotate(" + (heading || 0) + ")");
    plane.classList.toggle("live", Boolean(now && now.live)); // green; amber: the last known position
    $("plane-arrow").setAttribute("display", heading === null ? "none" : "inline");
    $("plane-dot").setAttribute("display", heading === null ? "inline" : "none");
  }

  function setFollow(on) {
    view.follow = on;
    $("follow").setAttribute("aria-pressed", String(on));
  }

  function zoomBy(step) {
    const zoom = Math.min(ZOOM_MAX, Math.max(ZOOM_MIN, view.zoom + step));
    if (zoom === view.zoom) return;
    view.zoom = zoom;
    try {
      localStorage.setItem("mavlte.zoom", String(zoom));
    } catch (e) { /* this time only */ }
    drawMap();
  }

  async function openMap() {
    setFollow(true);
    $("mapview").hidden = false;
    document.body.classList.add("viewing");
    drawMap();
    try { // the whole track: the page may have opened after the aircraft took off
      const r = await api("/api/track");
      if (r.status === 401) return showSignin();
      if (!r.ok) return;
      const fixes = (await r.json()).fixes;
      const end = fixes.length ? fixes[fixes.length - 1][2] : 0;
      view.track = fixes.concat(view.track.filter((p) => p[2] > end)).slice(-TRACK_MOST);
      drawMap();
    } catch (e) { /* the fixes this page has seen will do */ }
  }

  function closeMap() {
    $("mapview").hidden = true;
    document.body.classList.toggle("viewing", !$("viewer").hidden);
  }

  const fingers = new Map(); // the pointers on the map: one drags it, two pinch
  let spread = 0;
  let wheel = 0;

  function apart() {
    const [a, b] = [...fingers.values()];
    return Math.hypot(a[0] - b[0], a[1] - b[1]);
  }

  $("map").addEventListener("pointerdown", (event) => {
    $("map").setPointerCapture(event.pointerId);
    fingers.set(event.pointerId, [event.clientX, event.clientY]);
    if (fingers.size === 2) spread = apart();
  });
  $("map").addEventListener("pointermove", (event) => {
    const was = fingers.get(event.pointerId);
    if (!was) return;
    fingers.set(event.pointerId, [event.clientX, event.clientY]);
    if (fingers.size === 1) { // the map moves under the finger, and stops following
      const [cx, cy] = toPixel(view.lat, view.lon, view.zoom);
      [view.lat, view.lon] = toLatLon(cx - (event.clientX - was[0]), cy - (event.clientY - was[1]), view.zoom);
      setFollow(false);
      drawMap();
    } else if (fingers.size === 2 && spread) { // a zoom level for each 1.6 times further apart (or closer)
      const now = apart();
      if (now > spread * 1.6 || now < spread / 1.6) {
        zoomBy(now > spread ? 1 : -1);
        spread = now;
      }
    }
  });
  for (const type of ["pointerup", "pointercancel"]) {
    $("map").addEventListener(type, (event) => {
      fingers.delete(event.pointerId);
      if (fingers.size < 2) spread = 0;
    });
  }
  $("map").addEventListener("wheel", (event) => {
    event.preventDefault();
    wheel += event.deltaY;
    if (Math.abs(wheel) >= 50) { // one notch of a mouse wheel; a touchpad gets there in a few
      zoomBy(wheel < 0 ? 1 : -1);
      wheel = 0;
    }
  }, { passive: false });
  $("open-map").addEventListener("click", openMap);
  $("zoom-in").addEventListener("click", () => zoomBy(1));
  $("zoom-out").addEventListener("click", () => zoomBy(-1));
  $("follow").addEventListener("click", () => {
    setFollow(true);
    drawMap();
  });
  $("map-close").addEventListener("click", closeMap);
  window.addEventListener("resize", drawMap);
  document.addEventListener("keydown", (event) => {
    if ($("mapview").hidden) return;
    if (event.key === "Escape") closeMap();
    else if (event.key === "+") zoomBy(1);
    else if (event.key === "-") zoomBy(-1);
  });

  document.addEventListener("visibilitychange", () => {
    if (document.hidden) clearTimeout(timer);
    else poll();
  });

  paintSizes();
  poll();
})();
