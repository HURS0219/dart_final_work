/* task.js — 自动化发射任务网页卡 (调用封装好的状态机, 逐步执行)
 * 显示 8 步时序、开始/停止、发射参数、距离打击表。
 */

const TASK_STEPS = ["空闲", "舵机 → 标准位", "拉簧 → 标准位", "拉簧 → 预备位 (蓄力)",
  "舵机 → 预备位 (扣住)", "拉簧 → 复位", "舵机 → 复位 (发射)", "完成"];

class TaskCard {
  constructor() {
    this.el = document.createElement("div");
    this.edit = false;
    this.build();
  }

  api(op, extra) {
    let u = "/api/task?op=" + op;
    if (extra) u += "&" + extra;
    fetch(u).catch(() => {});
  }

  build() {
    this.el.innerHTML =
      '<div class="card"><h3>自动发射时序 <span class="tag step-tag">空闲</span></h3>' +
      '<div class="timeline"></div>' +
      '<div class="btns c2" style="margin-top:12px">' +
      '<button class="good b-start">开始发射</button>' +
      '<button class="bad b-stop">停止</button></div></div>' +
      '<div class="card"><h3>发射参数</h3>' +
      '<div class="row"><label>拉簧预备圈数</label><input type="number" class="turn" step="0.1" value="5">' +
      '<button class="ghost b-setturn">保存</button></div>' +
      '<div class="row"><label>舵机标准位 °</label><input type="number" class="ss" step="1" value="0">' +
      '<button class="ghost b-setss">保存</button></div>' +
      '<div class="row"><label>舵机预备位 °</label><input type="number" class="sp" step="1" value="90">' +
      '<button class="ghost b-setsp">保存</button></div>' +
      '<div class="btns c2">' +
      '<button class="ghost b-sstd">舵机标准位</button>' +
      '<button class="ghost b-sprep">舵机预备位</button></div></div>' +
      '<div class="card"><h3>距离打击表 <span class="tag dtag">点击即发射</span></h3>' +
      '<div class="dgrid"></div>' +
      '<div class="row" style="margin-top:12px">' +
      '<input type="number" class="nd" placeholder="距离 m" step="0.1">' +
      '<input type="number" class="nt" placeholder="圈数" step="0.1"></div>' +
      '<div class="btns c2"><button class="ghost b-add">添加靶点</button>' +
      '<button class="ghost b-edit">编辑</button></div></div>';

    const q = s => this.el.querySelector(s);
    q(".b-start").onclick = () => { this.api("start"); window.toast("开始自动发射"); };
    q(".b-stop").onclick = () => { this.api("stop"); window.toast("已停止"); };
    q(".b-setturn").onclick = () => { this.api("turns", "v=" + (parseFloat(q(".turn").value) || 0)); fetch("/save").catch(() => {}); window.toast("圈数已保存"); };
    q(".b-setss").onclick = () => { fetch("/api/servo?op=std&v=" + (parseFloat(q(".ss").value) || 0)).then(() => fetch("/save")).catch(() => {}); };
    q(".b-setsp").onclick = () => { fetch("/api/servo?op=prep&v=" + (parseFloat(q(".sp").value) || 0)).then(() => fetch("/save")).catch(() => {}); };
    q(".b-sstd").onclick = () => this.api("servostd");
    q(".b-sprep").onclick = () => this.api("servoprep");
    q(".b-add").onclick = () => {
      const d = parseFloat(q(".nd").value), t = parseFloat(q(".nt").value);
      if (isNaN(d) || isNaN(t)) { window.toast("请填距离和圈数"); return; }
      const a = this.loadDist(); a.push([d, t]); a.sort((x, y) => x[0] - y[0]);
      this.saveDist(a); this.renderDist();
    };
    q(".b-edit").onclick = () => {
      this.edit = !this.edit;
      q(".dgrid").className = "dgrid" + (this.edit ? " edit" : "");
      q(".b-edit").textContent = this.edit ? "完成" : "编辑";
      q(".dtag").textContent = this.edit ? "点右上角 × 删除" : "点击即发射";
    };
    this.renderTimeline();
    this.renderDist();
  }

  renderTimeline() {
    this.el.querySelector(".timeline").innerHTML = TASK_STEPS.map((t, i) =>
      '<div class="step" data-i="' + i + '"><span class="n">' + (i === 0 ? "●" : i) + '</span>' + t + '</div>').join("");
  }

  loadDist() {
    try { return JSON.parse(localStorage.getItem("dartDist") || "null") || [[2, 2], [2.5, 2.5], [3, 3], [3.5, 3.5], [4, 4]]; }
    catch (e) { return []; }
  }
  saveDist(a) { localStorage.setItem("dartDist", JSON.stringify(a)); }
  renderDist() {
    const d = this.loadDist();
    this.el.querySelector(".dgrid").innerHTML = d.map((r, i) =>
      '<button class="dbtn" data-i="' + i + '">' + r[0] + 'm<small>' + r[1] + ' 圈</small>' +
      '<span class="x">×</span></button>').join("") || '<div class="hint">还没有靶点</div>';
    this.el.querySelectorAll(".dbtn").forEach(b => {
      const i = +b.dataset.i;
      b.onclick = (e) => {
        if (e.target.classList.contains("x")) { const a = this.loadDist(); a.splice(i, 1); this.saveDist(a); this.renderDist(); return; }
        this.fire(i);
      };
    });
  }
  fire(i) {
    const r = this.loadDist()[i];
    if (!r) return;
    this.api("turns", "v=" + r[1]);
    window.toast("发射 " + r[0] + "m · " + r[1] + " 圈");
    setTimeout(() => this.api("start"), 180);
  }

  update(st) {
    if (!st) return;
    const step = st.step;
    this.el.querySelector(".step-tag").textContent = TASK_STEPS[step] || "?";
    this.el.querySelectorAll(".step").forEach(e => {
      const i = +e.dataset.i;
      e.className = "step" + (i === step ? " on" : (i < step ? " done" : ""));
    });
  }
}
