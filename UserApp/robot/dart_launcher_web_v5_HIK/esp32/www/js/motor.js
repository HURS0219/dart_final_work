/* motor.js — 电机网页卡 (基类 MotorCard + M3508Card / M2006Card / GM6020Card)
 * 三模式: 速度 / 角度 / 圈数; 同屏显示 转速·角度·圈数; 归零·取零点; 完整 PID 面板;
 * 圈数模式下带 状态机切换按钮 + 圈数 x 输入框。
 */

const PARAM_FIELDS = [
  [1, "速度 Kp", 100, 0.1], [2, "速度 Ki", 100, 0.1], [3, "速度 Kd", 100, 0.1],
  [4, "速度积分限幅", 100, 100], [5, "速度输出限幅", 100, 100],
  [6, "角度 Kp", 100, 0.1], [7, "角度 Ki", 100, 0.1], [8, "角度 Kd", 100, 0.1],
  [9, "角度死区", 100, 0.1], [10, "角度输出限幅", 100, 100],
  [11, "减速比", 100, 0.01]
];

class MotorCard {
  constructor(slot, name, sub) {
    this.slot = slot;
    this.name = name;
    this.sub = sub;
    this.mode = "speed";
    this.springTurns = 5;
    this.ratio = 1;
    this.angleMaxOut = 1000;  // 转子 deg/s
    this.pidDirty = false;
    this.el = document.createElement("div");
    this.el.className = "mcard";
    this.build();
  }

  /* 当前角度环限幅换算成“输出 RPM” */
  spdRpm() {
    const r = this.ratio > 0.01 ? this.ratio : 1;
    return Math.round(this.angleMaxOut / r / 6);
  }
  setMoveSpeed(input) {
    const rpm = parseFloat(input.value);
    if (isNaN(rpm)) return;
    const raw = Math.max(0, rpm) * this.ratio * 6;  // 输出rpm -> 转子deg/s
    this.api("param", "id=10&val=" + Math.round(raw * 100));
    window.toast(this.name + " 转速 ≈ " + Math.round(rpm) + " RPM");
  }
  speedRow() {
    return '<div class="row"><label>转速 RPM</label>' +
      '<input type="number" class="spd" step="1" value="' + this.spdRpm() + '">' +
      '<button class="ghost b-setspd">设定</button></div>';
  }

  api(op, extra) {
    let u = "/api/motor?slot=" + this.slot + "&op=" + op;
    if (extra) u += "&" + extra;
    fetch(u).catch(() => {});
  }

  build() {
    const pid = PARAM_FIELDS.map(f =>
      '<div class="row"><label>' + f[1] + '</label><input type="number" step="' + f[3] +
      '" data-pid="' + f[0] + '" oninput="void 0"></div>').join("");
    this.el.innerHTML =
      '<div class="mhead"><span class="name">' + this.name + '</span>' +
      '<span class="id">' + this.sub + '</span><span class="led"></span></div>' +
      '<div class="mgrid">' +
      '<div class="cell"><b class="v-rpm">0</b><span>RPM</span></div>' +
      '<div class="cell"><b class="v-ang">0</b><span>角度°</span></div>' +
      '<div class="cell"><b class="v-trn">0.0</b><span>圈数</span></div>' +
      '<div class="cell"><b class="v-tmp">0</b><span>°C</span></div>' +
      '</div>' +
      '<div class="seg seg-mode">' +
      '<button data-m="speed">速度</button><button data-m="angle">角度</button>' +
      '<button data-m="turns">圈数</button></div>' +
      '<div class="mode-body"></div>' +
      '<div class="btns c3">' +
      '<button class="ghost b-zero">归零</button>' +
      '<button class="ghost b-takezero">取零点</button>' +
      '<button class="ghost b-dir">方向反转</button></div>' +
      '<details class="pid"><summary>PID 参数 <span class="tag">独立保存</span></summary>' +
      '<div class="body">' + pid +
      '<div class="btns c2" style="margin-top:10px">' +
      '<button class="b-apply">应用并保存</button>' +
      '<button class="ghost b-reset">恢复默认</button></div></div></details>';

    const q = s => this.el.querySelector(s);
    this.el.querySelectorAll(".seg-mode button").forEach(b => {
      b.onclick = () => { this.mode = b.dataset.m; this.renderMode(); this.renderModeBtns(); };
    });
    q(".b-zero").onclick = () => {
      if (this.mode === "speed") this.api("speed", "v=0");
      else if (this.mode === "angle") this.api("angle", "v=0");
      else this.api("turns", "v=0");
    };
    q(".b-takezero").onclick = () => { this.api("zero"); window.toast(this.name + " 已取零点"); };
    q(".b-dir").onclick = () => { this.api("dir"); window.toast(this.name + " 方向已反转"); };
    q(".b-apply").onclick = () => this.applyPid();
    q(".b-reset").onclick = () => { this.pidDirty = false; this.api("reset"); window.toast("已恢复默认"); };
    this.el.querySelectorAll('input[data-pid]').forEach(inp => {
      inp.oninput = () => { this.pidDirty = true; };
    });
    this.renderMode();
    this.renderModeBtns();
  }

  renderModeBtns() {
    this.el.querySelectorAll(".seg-mode button").forEach(b =>
      b.className = (b.dataset.m === this.mode ? "on" : ""));
  }

  renderMode() {
    const body = this.el.querySelector(".mode-body");
    if (this.mode === "speed") {
      body.innerHTML =
        '<input type="range" class="rng" min="-300" max="300" value="0">' +
        '<div class="sub v-rpm-cmd">拖动滑块调速度</div>' +
        '<div class="row"><label>目标 RPM</label><input type="number" class="inp" value="0">' +
        '<button class="ghost b-go">转到</button></div>';
      const rng = body.querySelector(".rng"), inp = body.querySelector(".inp");
      rng.oninput = () => { inp.value = rng.value; body.querySelector(".v-rpm-cmd").textContent = rng.value + " RPM"; this.api("speed", "v=" + rng.value); };
      body.querySelector(".b-go").onclick = () => this.api("speed", "v=" + (parseFloat(inp.value) || 0));
    } else if (this.mode === "angle") {
      body.innerHTML =
        '<div class="row"><label>目标角度°</label><input type="number" class="inp" step="1" value="0">' +
        '<button class="ghost b-go">转到</button></div>' +
        this.speedRow() +
        '<div class="hint">相对“零点”。未取零点时以开机位置为 0。</div>';
      body.querySelector(".b-go").onclick = () => this.api("angle", "v=" + (parseFloat(body.querySelector(".inp").value) || 0));
      body.querySelector(".b-setspd").onclick = () => this.setMoveSpeed(body.querySelector(".spd"));
    } else {
      body.innerHTML =
        '<div class="row"><label>预备位 x 圈</label><input type="number" class="x" step="0.1" value="' + this.springTurns + '">' +
        '<button class="ghost b-setx">保存</button></div>' +
        '<div class="row"><label>目标圈数</label><input type="number" class="inp" step="0.01" value="0">' +
        '<button class="ghost b-go">转到</button></div>' +
        this.speedRow() +
        '<div class="btns c2">' +
        '<button class="ghost b-std">标准位 (0)</button>' +
        '<button class="b-prep">预备位 (x)</button></div>' +
        '<button class="wide b-fsm">状态机切换 (0 ↔ x)</button>';
      const xi = body.querySelector(".x"), inp = body.querySelector(".inp");
      body.querySelector(".b-setx").onclick = () => {
        this.springTurns = parseFloat(xi.value) || 0;
        fetch("/api/task?op=turns&v=" + this.springTurns).catch(() => {});
        window.toast(this.name + " 预备位 = " + this.springTurns + " 圈");
      };
      body.querySelector(".b-go").onclick = () => this.api("turns", "v=" + (parseFloat(inp.value) || 0));
      body.querySelector(".b-setspd").onclick = () => this.setMoveSpeed(body.querySelector(".spd"));
      body.querySelector(".b-std").onclick = () => { inp.value = 0; this.api("turns", "v=0"); };
      body.querySelector(".b-prep").onclick = () => { inp.value = this.springTurns; this.api("turns", "v=" + this.springTurns); };
      body.querySelector(".b-fsm").onclick = () => {
        const cur = this._lastTurns || 0;
        const tgt = Math.abs(cur) < 0.01 ? this.springTurns : 0;
        inp.value = tgt;
        this.api("turns", "v=" + tgt);
        window.toast(this.name + " 状态机 → " + tgt + " 圈");
      };
    }
  }

  applyPid() {
    const seq = [];
    this.el.querySelectorAll("input[data-pid]").forEach(inp => {
      const f = PARAM_FIELDS.find(x => x[0] == inp.dataset.pid);
      seq.push([f[0], Math.round((parseFloat(inp.value) || 0) * f[2])]);
    });
    let i = 0;
    const next = () => {
      if (i >= seq.length) {
        this.pidDirty = false;
        fetch("/save").catch(() => {});
        window.toast(this.name + " 参数已保存");
        return;
      }
      const q = seq[i++];
      this.api("param", "id=" + q[0] + "&val=" + q[1]);
      setTimeout(next, 30);
    };
    next();
  }

  update(m, params) {
    if (!m) return;
    const q = s => this.el.querySelector(s);
    q(".v-rpm").textContent = m.rpm;
    q(".v-ang").textContent = (m.angle100 / 100).toFixed(1);
    q(".v-trn").textContent = (m.turns100 / 100).toFixed(2);
    q(".v-tmp").textContent = m.temp;
    this._lastTurns = m.turns100 / 100;
    this.el.querySelector(".led").className = "led" + (m.online ? " on" : "");
    if (params && params.length === PARAM_FIELDS.length) {
      this.ratio = params[10] / 100;
      this.angleMaxOut = params[9] / 100;
      const sp = this.el.querySelector(".spd");
      if (sp && document.activeElement !== sp) sp.value = this.spdRpm();
    }
    if (params && params.length === PARAM_FIELDS.length && !this.pidDirty) {
      this.el.querySelectorAll("input[data-pid]").forEach(inp => {
        if (document.activeElement === inp) return;
        const idx = PARAM_FIELDS.findIndex(x => x[0] == inp.dataset.pid);
        inp.value = (params[idx] / PARAM_FIELDS[idx][2]).toFixed(3);
      });
    }
  }
}

class M3508Card extends MotorCard {}
class M2006Card extends MotorCard {}
class GM6020Card extends MotorCard {}
