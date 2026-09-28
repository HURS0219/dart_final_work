/* app.js — 页面组装 + 单一轮询 + 命令总线  (v0.2)
 *
 * C 端遥测帧:
 *   T,<mode>,<link>,<a0>,<p0>,<d0>,<t0>,<s0>, ... x4, <mp>,<my>,<mr>
 *   a=角度x10  p=脉宽us  d=方向  t=trim x10  s=scale x1000  m=mix x1000
 *
 * 要点: 命令串行发送 (匹配 ESP32 单连接服务器); 高频量合并+90ms 节流;
 *       滑块以网页为准, 遥测只更新"实际角"显示, 绝不回拽滑块。
 */

"use strict";

const CONFIG = { travelHalfDeg: 139.5 };
const MODE_NAMES = ["待机", "手动", "混控", "自检"];

/* ============================ 命令总线 ============================ */
const API = (() => {
  const pending = new Map();
  let sending = false;
  let timer = 0;
  let seq = 0;

  function qs(path, params) {
    const s = Object.entries(params || {})
      .filter(([, v]) => v !== undefined && v !== null)
      .map(([k, v]) => k + "=" + encodeURIComponent(v))
      .join("&");
    return path + (s ? "?" + s : "");
  }

  function send(url) {
    let ctrl = null, to = 0;
    const opt = {};
    if (typeof AbortController !== "undefined") {
      ctrl = new AbortController();
      opt.signal = ctrl.signal;
      to = setTimeout(() => ctrl.abort(), 4000);
    }
    return fetch(url, opt).catch(() => {}).then(() => { if (to) clearTimeout(to); });
  }

  function pump() {
    if (sending || pending.size === 0) return;
    const first = pending.entries().next().value;
    pending.delete(first[0]);
    sending = true;
    send(first[1]).then(() => { sending = false; pump(); });
  }

  function flush() { timer = 0; pump(); }

  return {
    flush,
    get(path, params) { pending.set("g" + (++seq), qs(path, params)); pump(); },
    queue(key, path, params) {
      pending.set(key, qs(path, params));
      if (!timer) timer = setTimeout(flush, 90);
    },
    servo(idx, op, v) {
      const p = { idx, op };
      if (v !== undefined) p.v = v;
      this.queue("s" + idx + ":" + op, "/api/servo", p);
    },
    servoNow(idx, op, v) {
      const p = { idx, op };
      if (v !== undefined) p.v = v;
      this.get("/api/servo", p);
    },
    task(op, extra) { this.get("/api/task", Object.assign({ op }, extra || {})); },
    taskQ(op, extra) { this.queue("t:" + op, "/api/task", Object.assign({ op }, extra || {})); },
    save() { this.get("/save"); },
    ping() { this.get("/ping"); },
  };
})();

/* ============================ 遥测解析 ============================ */
function parseFrame(text) {
  if (!text || text.charAt(0) !== "T") return null;
  const m = text.match(/-?\d+/g);
  if (!m) return null;
  const v = m.map(Number);
  if (v.length < 25) return null;
  let i = 0;
  const mode = v[i++], link = v[i++];
  const axes = [];
  for (let j = 0; j < 4; j++, i += 5) {
    axes.push({
      angle: v[i] / 10, pulse: v[i + 1], dir: v[i + 2],
      trim: v[i + 3] / 10, scale: v[i + 4] / 1000,
    });
  }
  const mix = { p: v[i] / 1000, y: v[i + 1] / 1000, r: v[i + 2] / 1000 };
  return { mode, link, axes, mix };
}

/* ============================ 提示 ============================ */
let toastTimer = 0;
function toast(msg) {
  const e = document.getElementById("toast");
  e.textContent = msg;
  e.classList.add("show");
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => e.classList.remove("show"), 1400);
}

/* ============================ 卡片实例 ============================ */
const TABS = ["总线", "右上", "左上", "左下", "右下", "总控"];
const servoCards = [
  new PTK7350Card(0, "右上舵面", "PWM1 · TIM1_CH1"),
  new PTK7350Card(1, "左上舵面", "PWM2 · TIM1_CH2"),
  new PTK7350Card(2, "左下舵面", "PWM3 · TIM1_CH3"),
  new PTK7350Card(3, "右下舵面", "PWM4 · TIM1_CH4"),
];
const taskCard = new TaskCard();
const pages = [];

/* ============================ 组装 ============================ */
function buildPages() {
  const main = document.getElementById("main");

  const p0 = document.createElement("section");
  p0.className = "page on";
  p0.innerHTML = `
    <div class="card">
      <h3>控制总线 <span class="tag">UART6 · 115200</span></h3>
      <div class="row"><label>当前模式</label><span class="v" id="stMode">--</span></div>
      <div class="row"><label>链路</label><span class="v" id="stLink">--</span></div>
      <div class="row"><label>保存</label><span class="v" id="stSave">--</span></div>
      <div class="btns c2">
        <button id="btn-test">自检扫舵</button>
        <button class="ghost" id="btn-idle">回待机</button>
        <button class="ghost" id="btn-save">保存标定</button>
        <button class="ghost" id="btn-ping">Ping</button>
      </div>
      <div class="hint" id="rawHint" style="margin-top:10px">等待遥测…</div>
    </div>
    <div class="card">
      <h3>四路舵面 <span class="tag">PWM1~4 · TIM1 CH1~4</span></h3>
      <div class="mgrid" id="overview"></div>
    </div>`;
  main.appendChild(p0);
  pages.push(p0);

  for (let s = 0; s < 4; s++) {
    const p = document.createElement("section");
    p.className = "page";
    p.appendChild(servoCards[s].el);
    main.appendChild(p);
    pages.push(p);
  }

  const p5 = document.createElement("section");
  p5.className = "page";
  p5.appendChild(taskCard.el);
  main.appendChild(p5);
  pages.push(p5);

  document.getElementById("overview").innerHTML = TABS.slice(1, 5)
    .map((t, i) => `<div class="cell"><b id="ov-a-${i}">--</b><span>${t}</span></div>`).join("");

  document.getElementById("btn-test").onclick = () => { API.task("mode", { v: 3 }); toast("自检扫舵"); };
  document.getElementById("btn-idle").onclick = () => { API.task("mode", { v: 0 }); toast("回待机"); };
  document.getElementById("btn-save").onclick = () => { API.save(); toast("已发送保存…"); };
  document.getElementById("btn-ping").onclick = () => { API.ping(); toast("Ping"); };
}

function buildTabs() {
  const bar = document.getElementById("tabbar");
  bar.innerHTML = TABS.map((t, i) => `<button data-i="${i}">${t}</button>`).join("");
  bar.querySelectorAll("button").forEach((b) => { b.onclick = () => showTab(+b.dataset.i); });
  showTab(0);
}

function showTab(i) {
  pages.forEach((p, k) => p.classList.toggle("on", k === i));
  document.querySelectorAll("#tabbar button").forEach((b, k) => b.classList.toggle("on", k === i));
  window.scrollTo(0, 0);
}

/* ============================ 刷新 ============================ */
let lastRx = 0;

function setConnected(live) {
  document.getElementById("pill").classList.toggle("ok", live);
  document.getElementById("pillTxt").textContent = live ? "已连接" : "未连接";
}

function render(s) {
  const live = (Date.now() - lastRx) < 1000;
  setConnected(live);
  if (!s) return;

  servoCards.forEach((c, i) => c.update(s.axes[i], live));
  taskCard.update(s.mode, s.mix, live);

  for (let i = 0; i < 4; i++) {
    const e = document.getElementById("ov-a-" + i);
    if (e) e.textContent = s.axes[i] ? s.axes[i].angle.toFixed(1) + "°" : "--";
  }
  const set = (id, val) => { const e = document.getElementById(id); if (e) e.textContent = val; };
  set("stMode", MODE_NAMES[s.mode] || "?");
  set("stLink", s.link ? "正常" : "超时");
  set("rawHint", "模式 " + (MODE_NAMES[s.mode] || "?") + " · 链路 " + (s.link ? "OK" : "LOST"));
}

function poll() {
  fetch("/state", { cache: "no-store" })
    .then((r) => r.text())
    .then((t) => {
      const s = parseFrame(t);
      if (s) lastRx = Date.now();
      render(s);
    })
    .catch(() => render(null));
}

function pollStatus() {
  fetch("/api/status", { cache: "no-store" })
    .then((r) => r.text())
    .then((t) => {
      const v = parseInt(t, 10) || 0;
      const e = document.getElementById("stSave");
      if (v === 1) { if (e) e.textContent = "成功"; toast("标定保存成功"); }
      else if (v === 2) { if (e) e.textContent = "失败"; toast("标定保存失败"); }
    })
    .catch(() => {});
}

/* ============================ 启动 ============================ */
buildPages();
buildTabs();
setInterval(poll, 300);
setInterval(pollStatus, 500);
setInterval(() => API.ping(), 1500);
poll();
