/* yaw.js — Yaw 轴网页卡 (电机卡对象, 三模式: 手动 / 自瞄 / 制导)
 *   手动: 速度滑块 + 角度输入
 *   自瞄: 设定自瞄转速 RPM, 由 C 板按视觉误差闭环对准绿光中心
 *   制导: 回到 0° (交给飞镖自导)
 */

class YawCard {
  constructor(slot, name, sub) {
    this.slot = slot;
    this.name = name;
    this.sub = sub;
    this.mode = "manual";
    this.ratio = 36;
    this.angleMaxOut = 25920;
    this.aimRpm = 60;
    this.pidDirty = false;
    this.el = document.createElement("div");
    this.el.className = "mcard";
    this.build();
  }

  api(op, extra) {
    let u = "/api/motor?slot=" + this.slot + "&op=" + op;
    if (extra) u += "&" + extra;
    fetch(u).catch(() => {});
  }
  yaw(v) { fetch("/api/task?op=yaw&v=" + v).catch(() => {}); }

  build() {
    const pid = PARAM_FIELDS.map(f =>
      '<div class="row"><label>' + f[1] + '</label><input type="number" step="' + f[3] +
      '" data-pid="' + f[0] + '"></div>').join("");
    this.el.innerHTML =
      '<div class="mhead"><span class="name">' + this.name + '</span>' +
      '<span class="id">' + this.sub + '</span><span class="led"></span></div>' +
      '<div class="mgrid">' +
      '<div class="cell"><b class="v-rpm">0</b><span>RPM</span></div>' +
      '<div class="cell"><b class="v-ang">0</b><span>角度°</span></div>' +
      '<div class="cell"><b class="v-err">--</b><span>误差px</span></div>' +
      '<div class="cell"><b class="v-tmp">0</b><span>°C</span></div>' +
      '</div>' +
      '<div class="seg seg-mode">' +
      '<button data-m="manual">手动</button><button data-m="aim">自瞄</button>' +
      '<button data-m="guide">制导</button></div>' +
      '<div class="mode-body"></div>' +
      '<div class="btns c3">' +
      '<button class="ghost b-stop">停止</button>' +
      '<button class="ghost b-takezero">取零点</button>' +
      '<button class="ghost b-dir">方向反转</button></div>' +
      '<details class="pid"><summary>PID 参数 <span class="tag">独立保存</span></summary>' +
      '<div class="body">' + pid +
      '<div class="btns c2" style="margin-top:10px">' +
      '<button class="b-apply">应用并保存</button>' +
      '<button class="ghost b-reset">恢复默认</button></div></div></details>';

    const q = s => this.el.querySelector(s);
    this.el.querySelectorAll(".seg-mode button").forEach(b => {
      b.onclick = () => this.setMode(b.dataset.m);
    });
    q(".b-stop").onclick = () => { this.api("stop"); window.toast("Yaw 已停止"); };
    q(".b-takezero").onclick = () => { this.api("zero"); window.toast("Yaw 已取零点"); };
    q(".b-dir").onclick = () => { this.api("dir"); window.toast("Yaw 方向已反转"); };
    q(".b-apply").onclick = () => this.applyPid();
    q(".b-reset").onclick = () => { this.pidDirty = false; this.api("reset"); window.toast("已恢复默认"); };
    this.el.querySelectorAll('input[data-pid]').forEach(inp => {
      inp.oninput = () => { this.pidDirty = true; };
    });
    this.renderMode();
    this.renderModeBtns();
  }

  setMode(m) {
    this.mode = m;
    this.api("stop");  // 切换瞬间先停, 防止残留速度/乱转
    if (m === "manual") this.yaw(0);
    else if (m === "aim") this.yaw(1);
    else if (m === "guide") this.yaw(2);
    this.renderMode();
    this.renderModeBtns();
  }

  renderModeBtns() {
    this.el.querySelectorAll(".seg-mode button").forEach(b =>
      b.className = (b.dataset.m === this.mode ? "on" : ""));
  }

  renderMode() {
    const body = this.el.querySelector(".mode-body");
    if (this.mode === "manual") {
      body.innerHTML =
        '<input type="range" class="rng" min="-200" max="200" value="0">' +
        '<div class="sub v-cmd">拖动滑块调速度</div>' +
        '<div class="row"><label>目标角度°</label><input type="number" class="inp" step="1" value="0">' +
        '<button class="ghost b-go">转到</button></div>' +
        '<div class="hint">手动模式: 用滑块/角度直接控制 (相对零点)。</div>';
      const rng = body.querySelector(".rng"), inp = body.querySelector(".inp");
      rng.oninput = () => {
        body.querySelector(".v-cmd").textContent = rng.value + " RPM";
        this.api("speed", "v=" + rng.value);
      };
      body.querySelector(".b-go").onclick = () => this.api("angle", "v=" + (parseFloat(inp.value) || 0));
    } else if (this.mode === "aim") {
      body.innerHTML =
        '<div class="btns c2">' +
        '<button class="good b-aimon">启动自瞄</button>' +
        '<button class="bad b-aimoff">关闭自瞄</button></div>' +
        '<div class="hint v-vis" style="font-size:14px">视觉: 等待上位机坐标...</div>';
      body.querySelector(".b-aimon").onclick = () => { this.yaw(1); window.toast("自瞄已启动"); };
      body.querySelector(".b-aimoff").onclick = () => { this.yaw(0); window.toast("自瞄已关闭"); };
    } else {
      body.innerHTML =
        '<div class="btns c2">' +
        '<button class="b-guide">回到 0° (制导位)</button>' +
        '<button class="ghost b-stop2">停止</button></div>' +
        '<div class="hint">制导位: 发射架转回 0°, 发射后由飞镖自行制导。</div>';
      body.querySelector(".b-guide").onclick = () => { this.yaw(2); window.toast("转到制导位 0°"); };
      body.querySelector(".b-stop2").onclick = () => this.api("stop");
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
        window.toast("Yaw 参数已保存");
        return;
      }
      const q = seq[i++];
      this.api("param", "id=" + q[0] + "&val=" + q[1]);
      setTimeout(next, 30);
    };
    next();
  }

  update(m, params, task, vis) {
    if (!m) return;
    const q = s => this.el.querySelector(s);
    q(".v-rpm").textContent = m.rpm;
    q(".v-ang").textContent = (m.angle100 / 100).toFixed(1);
    q(".v-tmp").textContent = m.temp;
    q(".v-err").textContent = (vis && vis.ok) ? vis.err : "--";
    this.el.querySelector(".led").className = "led" + (m.online ? " on" : "");
    if (params && params.length === PARAM_FIELDS.length) {
      this.ratio = params[10] / 100;
      this.angleMaxOut = params[9] / 100;
      if (!this.pidDirty) {
        this.el.querySelectorAll("input[data-pid]").forEach(inp => {
          if (document.activeElement === inp) return;
          const idx = PARAM_FIELDS.findIndex(x => x[0] == inp.dataset.pid);
          inp.value = (params[idx] / PARAM_FIELDS[idx][2]).toFixed(3);
        });
      }
    }
    if (task && task.aim100 !== undefined) {
      this.aimRpm = task.aim100 / 100;
      const r = q(".rpm");
      if (r && document.activeElement !== r) r.value = this.aimRpm;
    }
    if (vis) {
      const v = q(".v-vis");
      if (v) v.textContent = "视觉: x=" + vis.x + " 中心=" + vis.center +
        " 误差=" + vis.err + " " + (vis.ok ? "✓" : "×");
    }
  }
}
