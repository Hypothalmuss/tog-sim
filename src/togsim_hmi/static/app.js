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
  $("cycles").textContent = (st.cycles || 0);
  $("attempts").textContent = `${(st.attempts || 0)} attempts`;
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
  $("prod_n").textContent = (s.products.n || 0);
  $("prod_pickable").textContent = (s.products.pickable || 0);
  $("vacuum").textContent = s.vacuum.sealed ? `sealed (${s.vacuum.attached || "?"})` : "released";
  const j = s.joints || {};
  $("joints").textContent = Object.keys(j).length
    ? Object.entries(j).map(([k, v]) => `${k.replace("_joint", "")}=${fmt(v, 2)}`).join("  ")
    : "–";
  $("events").innerHTML = (s.events || []).map(([t, e]) => `<li><span>${t}</span>${e}</li>`).join("");
  $("log").textContent = (s.log || []).join("\n");
  renderAlarms(s.alarms); renderHealth(s.health); sparkline(s.cpm_hist); renderHistory(s.history);
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

const RECIPES = {
  cartons_fast: { cycles: 40, perception: "vision", belt: 0.10, outfeed: 0.06 },
  mixed_smooth: { cycles: 20, perception: "vision", belt: 0.10, outfeed: 0.10 },
  gt_check: { cycles: 12, perception: "gt", belt: 0.10, outfeed: 0.10 },
};
$("run_recipe").onchange = () => {
  const r = RECIPES[$("run_recipe").value];
  if (!r) return;
  $("run_cycles").value = r.cycles; $("run_perception").value = r.perception;
  $("run_belt").value = r.belt.toFixed(2); $("run_outfeed").value = r.outfeed.toFixed(2);
};
$("btn_start").onclick = () =>
  post("/api/run", {
    cycles: Number($("run_cycles").value), perception: $("run_perception").value,
    belt: Number($("run_belt").value), outfeed: Number($("run_outfeed").value),
  });
$("btn_estop").onclick = () => { if (confirm("Emergency stop: stop the cycle and both belts?")) post("/api/estop"); };
function ack(id) { post("/api/ack", { id }); }
function sparkline(hist) {
  const c = $("spark"), ctx = c.getContext("2d");
  ctx.clearRect(0, 0, c.width, c.height);
  if (!hist || hist.length < 2) return;
  const ys = hist.map((h) => h[1]), max = Math.max(5, ...ys);
  ctx.strokeStyle = "#3498db"; ctx.lineWidth = 2; ctx.beginPath();
  hist.forEach((h, i) => {
    const x = (i / (hist.length - 1)) * (c.width - 4) + 2, y = c.height - 3 - (h[1] / max) * (c.height - 8);
    i ? ctx.lineTo(x, y) : ctx.moveTo(x, y);
  });
  ctx.stroke();
  ctx.fillStyle = "#8b9bab"; ctx.font = "11px sans-serif"; ctx.fillText(`${ys[ys.length - 1].toFixed(1)} now, max ${max.toFixed(0)}`, 4, 11);
}
function renderAlarms(alarms) {
  const box = $("alarms");
  const active = (alarms || []).filter((a) => a.active || !a.acked);
  box.className = "alarms" + (active.length ? "" : " hidden");
  box.innerHTML = active.map((a) =>
    `<div class="alarm ${a.active ? "" : "inactive"}"><span><b>${a.t}</b> ${a.text}${a.active ? "" : " (cleared)"}</span>` +
    `<button onclick="ack('${a.id}')">${a.acked ? "acked" : "acknowledge"}</button></div>`).join("") +
    (active.length > 1 ? `<div class="alarm"><span></span><button onclick="ack('all')">acknowledge all</button></div>` : "");
}
function renderHealth(h) {
  const limits = { joints: 2, products: 3, trays: 4, "infeed belt": 3, "outfeed belt": 3 };
  $("health").innerHTML = Object.entries(limits).map(([k, lim]) => {
    const age = h && h[k] !== undefined ? h[k] : null;
    const cls = age === null ? "" : age > lim ? "bad" : "ok";
    return `<span class="${cls}" title="${age === null ? "no data" : age + " s ago"}">${k}</span>`;
  }).join("");
}
function renderHistory(rows) {
  $("history").querySelector("tbody").innerHTML = (rows || []).map((r) => {
    const rec = r.recipe || {};
    const fl = Object.entries(r.failures || {}).map(([k, v]) => `${v} ${k}`).join(", ") || "–";
    return `<tr><td>${r.ended}</td><td>${rec.perception || ""} ${rec.belt || ""}/${rec.outfeed || ""} m/s</td>` +
      `<td>${r.cycles}/${r.attempts}</td><td>${fmt(r.cpm)}</td><td>${fmt(r.placement_mean_mm)} / ${fmt(r.placement_p95_mm)} mm</td><td>${fl}</td></tr>`;
  }).join("");
}
$("btn_stop").onclick = () => post("/api/stop");
$("btn_belts").onclick = () => post("/api/belts", { infeed: Number($("belt_in").value), outfeed: Number($("belt_out").value) });
$("btn_belts_stop").onclick = () => post("/api/belts", { infeed: 0, outfeed: 0 });
$("belt_in").oninput = () => ($("belt_in_v").textContent = Number($("belt_in").value).toFixed(2));
$("belt_out").oninput = () => ($("belt_out_v").textContent = Number($("belt_out").value).toFixed(2));
poll();
