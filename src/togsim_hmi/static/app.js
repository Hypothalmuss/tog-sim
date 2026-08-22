"use strict";
const $ = (id) => document.getElementById(id);
const fmt = (v, d = 1) => (v === undefined || v === null ? "–" : Number(v).toFixed(d));

async function post(url, body) {
  const r = await fetch(url, { method: "POST", headers: { "Content-Type": "application/json" }, body: JSON.stringify(body || {}) });
  return r.json();
}

function renderTrays(trays) {
  const box = $("trays");
  if (!trays.length) { box.innerHTML = '<div class="mono">no tray in view</div>'; return; }
  box.innerHTML = "";
  for (const t of trays) {
    const el = document.createElement("div");
    el.className = "tray";
    const free = t.occupied.filter((v) => !v).length;
    el.innerHTML = `<div class="title">tray ${t.id} &middot; x ${fmt(t.x, 2)} m &middot; ${free} free</div>`;
    const grid = document.createElement("div");
    grid.className = "grid";
    grid.style.gridTemplateColumns = `repeat(${t.cols || 4}, 34px)`;
    // row 1 (+y) on top, pocket index = row*cols + col
    for (let r = (t.rows || 2) - 1; r >= 0; r--) {
      for (let c = 0; c < (t.cols || 4); c++) {
        const p = document.createElement("div");
        p.className = "pocket" + (t.occupied[r * (t.cols || 4) + c] ? " full" : "");
        p.title = `pocket ${r * (t.cols || 4) + c}`;
        grid.appendChild(p);
      }
    }
    el.appendChild(grid);
    box.appendChild(el);
  }
}

function render(s) {
  const st = s.status || {};
  const state = s.running ? (st.state === "finished" ? "finished" : st.state || "running") : (st.state === "finished" ? "finished" : "idle");
  $("state").textContent = state;
  $("state").className = "pill " + state;
  $("cycles").textContent = st.cycles ?? 0;
  $("attempts").textContent = `${st.attempts ?? 0} attempts`;
  $("cpm").textContent = fmt(st.cpm);
  const att = st.attempts || 0, cyc = st.cycles || 0;
  $("success").textContent = att ? `${Math.round((100 * cyc) / att)}%` : "–";
  const fl = st.failures || {};
  const nf = Object.values(fl).reduce((a, b) => a + b, 0);
  $("failures").textContent = nf ? Object.entries(fl).map(([k, v]) => `${v} ${k}`).join(", ") : "no failures";
  $("placement").textContent = st.placement_mean_mm !== undefined ? `${fmt(st.placement_mean_mm)} mm` : "–";
  $("placement_p95").textContent = st.placement_p95_mm !== undefined ? `p95 ${fmt(st.placement_p95_mm)} mm` : "p95 –";
  $("motion").textContent = fmt(st.motion_s, 2);
  $("belt_in_m").textContent = `measured ${fmt(s.belts.infeed, 2)}`;
  $("belt_out_m").textContent = `measured ${fmt(s.belts.outfeed, 2)}`;
  renderTrays(s.trays || []);
  $("prod_n").textContent = s.products.n ?? 0;
  $("prod_pickable").textContent = s.products.pickable ?? 0;
  $("vacuum").textContent = s.vacuum.sealed ? `sealed (${s.vacuum.attached || "?"})` : "released";
  const j = s.joints || {};
  $("joints").textContent = Object.keys(j).length
    ? Object.entries(j).map(([k, v]) => `${k.replace("_joint", "")}=${fmt(v, 2)}`).join("  ")
    : "–";
  $("events").innerHTML = (s.events || []).map(([t, e]) => `<li><span>${t}</span>${e}</li>`).join("");
  $("log").textContent = (s.log || []).join("\n");
}

async function poll() {
  try {
    const r = await fetch("/api/state");
    render(await r.json());
  } catch (e) {
    $("state").textContent = "no connection";
    $("state").className = "pill stopped";
  }
  setTimeout(poll, 500);
}

$("btn_start").onclick = () =>
  post("/api/run", { cycles: Number($("run_cycles").value), perception: $("run_perception").value, belt: Number($("run_belt").value) });
$("btn_stop").onclick = () => post("/api/stop");
$("btn_belts").onclick = () => post("/api/belts", { infeed: Number($("belt_in").value), outfeed: Number($("belt_out").value) });
$("btn_belts_stop").onclick = () => post("/api/belts", { infeed: 0, outfeed: 0 });
$("belt_in").oninput = () => ($("belt_in_v").textContent = Number($("belt_in").value).toFixed(2));
$("belt_out").oninput = () => ($("belt_out_v").textContent = Number($("belt_out").value).toFixed(2));
poll();
