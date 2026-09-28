/* app.js — 组装: 4 路舵面卡 + 制导姿态滑块 + 遥测刷新 */

const STATE_NAME = ["BOOT", "中立", "制导", "扫描", "手动", "失效"];

window.toast = function (t) {
  const e = document.getElementById("toast");
  e.textContent = t; e.classList.add("show");
  clearTimeout(window._tt); window._tt = setTimeout(() => e.classList.remove("show"), 1300);
};

function parseState(t) {
  if (!t || t[0] !== "F") return null;
  const v = []; let num = "";
  for (const ch of t) {
    if ((ch >= "0" && ch <= "9") || ch === "-") num += ch;
    else { if (num) { v.push(+num); num = ""; } }
  }
  if (num) v.push(+num);
  if (v.length < 21) return null;
  let i = 0;
  const state = v[i++], failsafe = v[i++], link = v[i++], visok = v[i++];
  const defl = v.slice(i, i + 4); i += 4;
  const raw = v.slice(i, i + 4); i += 4;
  const mix = v.slice(i, i + 3); i += 3;
  const att = v.slice(i, i + 3); i += 3;
  const visx = v[i++], visy = v[i++], np = v[i++];
  const params = v.slice(i, i + np);
  return { state, failsafe, link, visok, defl, raw, mix, att, visx, visy, np, params };
}

const servoCards = [
  new ServoCard(0, "舵面 1 · 右上", "PWM1 · PE9"),
  new ServoCard(1, "舵面 2 · 左上", "PWM2 · PE11"),
  new ServoCard(2, "舵面 3 · 左下", "PWM3 · PE13"),
  new ServoCard(3, "舵面 4 · 右下", "PWM4 · PE14")
];

function buildServos() {
  const host = document.getElementById("servoList");
  servoCards.forEach(c => host.appendChild(c.el));
}

/* 制导姿态: 滑块即时下发混控, 并切到制导(ACTIVE) */
function sendGuid() {
  const p = document.getElementById("gP").value / 100;
  const y = document.getElementById("gY").value / 100;
  const r = document.getElementById("gR").value / 100;
  document.getElementById("gPv").textContent = p.toFixed(2);
  document.getElementById("gYv").textContent = y.toFixed(2);
  document.getElementById("gRv").textContent = r.toFixed(2);
  fetch("/api/surface?op=state&v=2").catch(() => {});
  fetch("/api/surface?op=mix&p=" + p + "&y=" + y + "&r=" + r).catch(() => {});
}

function buildGuid() {
  ["gP", "gY", "gR"].forEach(id => {
    document.getElementById(id).oninput = sendGuid;
  });
  document.getElementById("btn-guid").onclick = () => {
    sendGuid();
    toast("已进入制导模式");
  };
  document.getElementById("btn-mid").onclick = () => {
    ["gP", "gY", "gR"].forEach(id => { document.getElementById(id).value = 0; });
    sendGuid();
  };
}

let lastRx = 0;
function update(s) {
  const live = (Date.now() - lastRx) < 900;
  const pill = document.getElementById("pill");
  pill.className = "pill" + (live ? " ok" : "");
  document.getElementById("pillTxt").textContent = live ? "已连接" : "未连接";
  if (!s) return;

  servoCards.forEach(c => c.update(s, s.params));
  document.getElementById("modeTag").textContent =
    (STATE_NAME[s.state] || ("#" + s.state)) + (s.failsafe ? " · 失效" : "");
}

function poll() {
  fetch("/state").then(r => r.text()).then(t => {
    const s = parseState(t);
    if (s) lastRx = Date.now();
    update(s);
  }).catch(() => update(null));
}

buildServos();
buildGuid();
setInterval(poll, 300);
setInterval(() => fetch("/ping").catch(() => {}), 500);
poll();
