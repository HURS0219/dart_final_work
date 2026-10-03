/* servo.js — 舵机网页卡 (基类 ServoCard + PTK7350Card / PTK7465Card)
 * 角度模式 + 状态机切换 + 取零点 + x/y 角度输入 + PID/标定面板。
 */

class ServoCard {
  constructor(name, sub) {
    this.name = name;
    this.sub = sub;
    this.stdDeg = 0;
    this.prepDeg = 90;
    this.state = 0;
    this.el = document.createElement("div");
    this.el.className = "mcard";
    this.build();
  }

  api(op, extra) {
    let u = "/api/servo?op=" + op;
    if (extra) u += "&" + extra;
    fetch(u).catch(() => {});
  }

  build() {
    this.el.innerHTML =
      '<div class="mhead"><span class="name">' + this.name + '</span>' +
      '<span class="id">' + this.sub + '</span></div>' +
      '<div class="big v-deg">0.0<small>°</small></div>' +
      '<div class="seg seg-state">' +
      '<button data-s="0">标准位</button><button data-s="1">预备位</button></div>' +
      '<div class="row"><label>x 角度 (标准)</label><input type="number" class="x" step="1" value="0">' +
      '<button class="ghost b-setx">保存</button></div>' +
      '<div class="row"><label>y 角度 (预备)</label><input type="number" class="y" step="1" value="90">' +
      '<button class="ghost b-sety">保存</button></div>' +
      '<div class="row"><label>目标角度</label><input type="number" class="t" step="1" value="0">' +
      '<button class="ghost b-go">转到</button></div>' +
      '<div class="btns c2">' +
      '<button class="ghost b-zero">取零点</button>' +
      '<button class="b-fsm">状态机切换</button></div>' +
      '<details class="pid"><summary>PID / 标定 <span class="tag">PWM 舵机</span></summary>' +
      '<div class="body"><div class="hint">当前为 PWM 舵机（无角度反馈、无 PID）。' +
      '此处预留：接入 PTK 总线舵机后在此整定角度环 PID。</div></div></details>';

    const q = s => this.el.querySelector(s);
    this.el.querySelectorAll(".seg-state button").forEach(b => {
      b.onclick = () => {
        this.state = +b.dataset.s;
        this.api(this.state === 1 ? "goprep" : "gostd");
        this.renderState();
      };
    });
    q(".b-setx").onclick = () => { this.stdDeg = parseFloat(q(".x").value) || 0; this.api("std", "v=" + this.stdDeg); fetch("/save").catch(() => {}); window.toast("标准位 = " + this.stdDeg + "° 已保存"); };
    q(".b-sety").onclick = () => { this.prepDeg = parseFloat(q(".y").value) || 0; this.api("prep", "v=" + this.prepDeg); fetch("/save").catch(() => {}); window.toast("预备位 = " + this.prepDeg + "° 已保存"); };
    q(".b-go").onclick = () => this.api("deg", "v=" + (parseFloat(q(".t").value) || 0));
    q(".b-zero").onclick = () => { this.api("zero"); window.toast(this.name + " 已取零点"); };
    q(".b-fsm").onclick = () => {
      this.state = this.state ? 0 : 1;
      this.api(this.state === 1 ? "goprep" : "gostd");
      this.renderState();
      window.toast(this.name + " 状态机 → " + (this.state ? "预备位" : "标准位"));
    };
    this.renderState();
  }

  renderState() {
    this.el.querySelectorAll(".seg-state button").forEach(b =>
      b.className = (+b.dataset.s === this.state ? "on" : ""));
  }

  update(s) {
    if (!s) return;
    this.state = s.state;
    this.el.querySelector(".v-deg").innerHTML = (s.cur10 / 10).toFixed(1) + "<small>°</small>";
    this.renderState();
  }
}

class PTK7350Card extends ServoCard {}
class PTK7465Card extends ServoCard {}
