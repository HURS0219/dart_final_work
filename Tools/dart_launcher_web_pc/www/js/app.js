/* app.js — 页面组装: 用封装好的网页卡搭出六个主界面 + 轮询刷新 */

const TABS = ["识别", "拉簧", "扳机", "Yaw", "总控", "自动化"];

/* ---- 全局工具 ---- */
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
  const n = v[0]; let i = 1;
  const motors = [];
  for (let j = 0; j < n; j++) {
    motors.push({
      slot: v[i], type: v[i + 1], id: v[i + 2], online: v[i + 3], dir: v[i + 4],
      mode: v[i + 5], target100: v[i + 6], rpm: v[i + 7], angle100: v[i + 8],
      turns100: v[i + 9], temp: v[i + 10], cur: v[i + 11]
    });
    i += 12;
  }
  const servo = { cur10: v[i], state: v[i + 1], std10: v[i + 2], prep10: v[i + 3] }; i += 4;
  const task = { spring100: v[i], step: v[i + 1], yaw: v[i + 2], estop: v[i + 3], aim100: v[i + 4] }; i += 5;
  const vis = { x: v[i], ok: v[i + 1], center: v[i + 2], err: v[i + 3] }; i += 4;
  const params = [];
  for (let j = 0; j < n; j++) { params.push(v.slice(i, i + 11)); i += 11; }
  return { n, motors, servo, task, vis, params };
}

/* ---- 卡片实例 ---- */
const motorCards = [
  new M3508Card(0, "拉簧 A", "M3508 · ID2"),
  new M3508Card(1, "拉簧 B", "M3508 · ID3"),
  new M3508Card(2, "扳机丝杆", "M3508 · ID4"),
  new M2006Card(3, "Yaw 轴", "M2006 · ID1")
];
const servoCard = new PTK7350Card("扳机舵机", "PWM · TIM1_CH1");
const taskCard = new TaskCard();
const yawCard = new YawCard(3, "Yaw 轴", "M2006 · ID1");

/* ---- 界面1: 识别连接 ---- */
function buildIdentify() {
  const host = document.getElementById("pg0");
  const list = document.createElement("div");
  list.id = "idlist";
  host.appendChild(list);
  for (let s = 0; s < 4; s++) {
    const d = document.createElement("div");
    d.className = "mcard";
    d.innerHTML =
      '<div class="mhead"><span class="name">' + motorCards[s].name + '</span>' +
      '<span class="id">' + motorCards[s].sub + '</span><span class="led" id="id-led-' + s + '"></span></div>' +
      '<div class="mgrid">' +
      '<div class="cell"><b id="id-rpm-' + s + '">0</b><span>RPM</span></div>' +
      '<div class="cell"><b id="id-trn-' + s + '">0.0</b><span>圈</span></div>' +
      '<div class="cell"><b id="id-tmp-' + s + '">0</b><span>°C</span></div>' +
      '<div class="cell"><b id="id-cur-' + s + '">0.0</b><span>A</span></div></div>' +
      '<div class="btns c2"><button class="ghost" data-zero="' + s + '">设零</button>' +
      '<button class="ghost" data-stop="' + s + '">停止</button></div>';
    list.appendChild(d);
  }
  list.querySelectorAll("[data-zero]").forEach(b => b.onclick = () => { fetch("/api/motor?slot=" + b.dataset.zero + "&op=zero").catch(() => {}); toast("已设零"); });
  list.querySelectorAll("[data-stop]").forEach(b => b.onclick = () => { fetch("/api/motor?slot=" + b.dataset.stop + "&op=stop").catch(() => {}); });
  document.getElementById("btn-scan").onclick = () => {
    document.getElementById("scanRes").textContent = "扫描中...";
    fetch("/scan").catch(() => {});
    setTimeout(() => fetch("/scanres").then(r => r.text()).then(t => {
      t = t.trim();
      document.getElementById("scanRes").textContent = t ? ("CAN 反馈 ID: " + t) : "未发现电机";
    }).catch(() => {}), 1400);
  };
  document.getElementById("btn-zeroall").onclick = () => { fetch("/cmd?c=Z").catch(() => {}); toast("已全部设零"); };
  document.getElementById("btn-estop").onclick = () => { fetch("/api/task?op=estop").catch(() => {}); toast("急停"); };
  document.getElementById("btn-clear").onclick = () => { fetch("/api/task?op=clear").catch(() => {}); toast("解除急停"); };
}

/* ---- 界面2/3/4: 电机卡 ---- */
function buildMotorPages() {
  document.getElementById("pg1").appendChild(motorCards[0].el);
  document.getElementById("pg1").appendChild(motorCards[1].el);
  document.getElementById("pg2").appendChild(motorCards[2].el);
  document.getElementById("pg2").appendChild(servoCard.el);
  document.getElementById("pg3").appendChild(yawCard.el);
}

/* ---- 界面3: Yaw (由 yaw.js 的 YawCard 负责, 含自瞄/调试) ---- */
function buildYaw() {}

/* ---- 界面5: 总控 ---- */
function buildMaster() {
  const host = document.getElementById("master");
  let h = "";
  for (let s = 0; s < 4; s++) {
    h += '<div class="mcard"><div class="mhead"><span class="name">' + motorCards[s].name + '</span>' +
      '<span class="id">' + motorCards[s].sub + '</span><span class="led" id="m-led-' + s + '"></span></div>' +
      '<div class="row"><label>转速</label><span class="v" id="m-rpm-' + s + '">0</span></div>' +
      '<div class="row"><label>速度 RPM</label><input type="number" id="m-s-' + s + '" step="1" value="0">' +
      '<button class="ghost" data-spd="' + s + '">速度</button></div>' +
      '<div class="row"><label>角度 °</label><input type="number" id="m-a-' + s + '" step="1" value="0">' +
      '<button class="ghost" data-ang="' + s + '">角度</button>' +
      '<button class="ghost bad" data-stop2="' + s + '">停</button></div></div>';
  }
  host.innerHTML = h;
  host.querySelectorAll("[data-spd]").forEach(b => b.onclick = () => {
    fetch("/api/motor?slot=" + b.dataset.spd + "&op=speed&v=" + (parseFloat(document.getElementById("m-s-" + b.dataset.spd).value) || 0)).catch(() => {});
  });
  host.querySelectorAll("[data-ang]").forEach(b => b.onclick = () => {
    fetch("/api/motor?slot=" + b.dataset.ang + "&op=angle&v=" + (parseFloat(document.getElementById("m-a-" + b.dataset.ang).value) || 0)).catch(() => {});
  });
  host.querySelectorAll("[data-stop2]").forEach(b => b.onclick = () => {
    fetch("/api/motor?slot=" + b.dataset.stop2 + "&op=stop").catch(() => {});
  });
  document.getElementById("btn-stopall").onclick = () => { for (let s = 0; s < 4; s++) fetch("/api/motor?slot=" + s + "&op=stop").catch(() => {}); toast("已全部停止"); };
  document.getElementById("btn-springstd").onclick = () => { fetch("/api/task?op=springstd").catch(() => {}); };
  document.getElementById("btn-springprep").onclick = () => { fetch("/api/task?op=springprep").catch(() => {}); };
  document.getElementById("btn-servostd").onclick = () => { fetch("/api/task?op=servostd").catch(() => {}); };
  document.getElementById("btn-servoprep").onclick = () => { fetch("/api/task?op=servoprep").catch(() => {}); };
}

/* ---- 界面6: 自动化 ---- */
function buildAuto() {
  document.getElementById("pg5").appendChild(taskCard.el);
}

/* ---- 标签栏 ---- */
function buildTabs() {
  const bar = document.getElementById("tabbar");
  bar.innerHTML = TABS.map((t, i) => '<button data-i="' + i + '">' + t + '</button>').join("");
  bar.querySelectorAll("button").forEach(b => b.onclick = () => showTab(+b.dataset.i));
  showTab(0);
}
function showTab(i) {
  for (let k = 0; k < 6; k++) {
    document.getElementById("pg" + k).className = "page" + (k === i ? " on" : "");
    document.querySelectorAll("#tabbar button")[k].className = (k === i ? "on" : "");
  }
  window.scrollTo(0, 0);
}

/* ---- 刷新 ---- */
let lastRx = 0;
let LAST = null;
function update(s) {
  const live = (Date.now() - lastRx) < 900;
  const pill = document.getElementById("pill");
  pill.className = "pill" + (live ? " ok" : "");
  document.getElementById("pillTxt").textContent = live ? "已连接" : "未连接";
  if (!s) return;

  motorCards.forEach((c, i) => c.update(s.motors[i], s.params[i]));
  servoCard.update(s.servo);
  taskCard.update(s.task);
  yawCard.update(s.motors[3], s.params[3], s.task, s.vis);

  let on = 0;
  for (let i = 0; i < 4; i++) {
    const m = s.motors[i];
    if (m.online) on++;
    const set = (id, val) => { const e = document.getElementById(id); if (e) e.textContent = val; };
    set("id-rpm-" + i, m.rpm);
    set("id-trn-" + i, (m.turns100 / 100).toFixed(2));
    set("id-tmp-" + i, m.temp);
    set("id-cur-" + i, (m.cur / 1000).toFixed(1));
    const led = document.getElementById("id-led-" + i); if (led) led.className = "led" + (m.online ? " on" : "");
    const led2 = document.getElementById("m-led-" + i); if (led2) led2.className = "led" + (m.online ? " on" : "");
    set("m-rpm-" + i, m.rpm + " RPM");
  }
  document.getElementById("onlineCnt").textContent = on + " / 4";
  document.getElementById("busTag").textContent = s.task.estop ? "急停中" : "CAN1 · 1Mbps";
  const eb = document.getElementById("btn-estop2");
  if (eb) {
    eb.textContent = s.task.estop ? "解除" : "急停";
    eb.className = "estop" + (s.task.estop ? " clearing" : "");
  }

  document.getElementById("visTag").textContent = s.vis.ok ? "锁定" : "丢失";
  document.getElementById("visX").textContent = s.vis.x + " / " + s.vis.center;
  document.getElementById("visErr").textContent = (s.vis.err > 0 ? "+" : "") + s.vis.err + " px";
  const mk = document.getElementById("visMark");
  if (mk && s.vis.center > 0) mk.style.left = Math.max(2, Math.min(98, 50 + (s.vis.err / s.vis.center) * 50)) + "%";
}

function poll() {
  fetch("/state").then(r => r.text()).then(t => {
    const s = parseState(t);
    if (s) { LAST = s; lastRx = Date.now(); }
    update(s);
  }).catch(() => update(null));
}

/* ---- 启动 ---- */
buildTabs();
buildIdentify();
buildMotorPages();
buildYaw();
buildMaster();
buildAuto();
document.getElementById("btn-estop2").onclick = () => {
  const on = LAST && LAST.task && LAST.task.estop;
  if (on) { fetch("/api/task?op=clear").catch(() => {}); toast("解除急停"); }
  else { fetch("/api/task?op=estop").catch(() => {}); toast("急停"); }
};
setInterval(poll, 300);
setInterval(() => fetch("/ping").catch(() => {}), 500);
poll();
