/* servo.js — 舵面卡片对象 (ServoCard 基类 + PTK7350Card / PTK7465Card)  v0.2
 *
 * 依赖 app.js 的全局 API / toast / CONFIG。
 * 逻辑角 -139.5 ~ +139.5 (279° 行程), 0 = 中位 1500us。
 * 滑块是"命令", 遥测只更新大字号"实际角"; 首次收到数据自动同步一次,
 * 之后可用"同步"按钮手动对齐, 绝不会自己回弹。
 */

function clampNum(v, lo, hi) {
  let n = parseFloat(v);
  if (isNaN(n)) n = 0;
  return Math.max(lo, Math.min(hi, n));
}

class ServoCard {
  constructor(idx, name, sub) {
    this.idx = idx;
    this.name = name;
    this.sub = sub;
    this.synced = false;
    this.dragging = false;
    this.last = null;
    this.el = document.createElement("div");
    this.el.className = "mcard";
    this._build();
  }

  _build() {
    const lo = -CONFIG.travelHalfDeg, hi = CONFIG.travelHalfDeg;
    this.el.innerHTML = `
      <div class="mhead">
        <span class="name">${this.name}</span>
        <span class="id">${this.sub}</span>
        <span class="led" data-led></span>
      </div>
      <div class="big" data-deg>--<small>°</small></div>
      <div class="sub" data-pulse>脉宽 -- us</div>

      <div class="row"><label>命令角 (${lo} ~ ${hi})</label><span class="v" data-cmd>0°</span></div>
      <input type="range" min="${lo}" max="${hi}" step="0.5" value="0" data-sl>
      <div class="row">
        <label>精确角度 °</label>
        <input type="number" step="0.5" value="0" data-num>
        <button class="ghost" data-go>转到</button>
      </div>

      <div class="btns c2">
        <button class="ghost" data-sync>同步当前角</button>
        <button class="ghost" data-center>回中位</button>
        <button class="ghost" data-zero>调零</button>
        <button class="ghost" data-dir>方向: 正</button>
      </div>

      <details class="pid">
        <summary>标定 <span class="tag">trim / scale</span></summary>
        <div class="body">
          <div class="hint">两步标定：① 手动拖到物理 0° 标记，点 [记 0°]；② 再拖到物理 90° 标记，点 [记 90°]，自动算出比例 scale。</div>
          <div class="row"><label>trim °</label><input type="number" step="0.1" value="0" data-trim></div>
          <div class="row"><label>scale</label><input type="number" step="0.001" value="1" data-scale></div>
          <div class="btns c2">
            <button class="ghost" data-rec0>记 0°</button>
            <button class="ghost" data-rec90>记 90°（算比例）</button>
          </div>
          <button class="ghost wide" data-cal>发送标定</button>
          <button class="ghost wide" data-resetcal>清除标定</button>
          <div class="sub" data-calinfo>trim 0.0° · scale 1.000</div>
        </div>
      </details>`;

    const q = (s) => this.el.querySelector(s);
    const sl = q("[data-sl]");
    const num = q("[data-num]");
    const enterManual = () => API.task("mode", { v: 1 });
    const send = (deg) => API.servo(this.idx, "angle", deg);

    sl.addEventListener("input", () => {
      q("[data-cmd]").textContent = sl.value + "°";
      num.value = sl.value;
      if (!this.dragging) { this.dragging = true; enterManual(); }
      send(sl.value);
    });
    sl.addEventListener("change", () => { this.dragging = false; API.flush(); });

    q("[data-go]").onclick = () => {
      const deg = clampNum(num.value, lo, hi);
      enterManual();
      API.servoNow(this.idx, "angle", deg);
      sl.value = deg;
      q("[data-cmd]").textContent = deg + "°";
    };
    q("[data-sync]").onclick = () => {
      if (this.last) {
        sl.value = this.last.angle;
        num.value = this.last.angle.toFixed(1);
        q("[data-cmd]").textContent = this.last.angle.toFixed(1) + "°";
        this.synced = true;
      }
      toast(this.name + " 已同步");
    };
    q("[data-center]").onclick = () => {
      enterManual();
      API.servoNow(this.idx, "angle", 0);
      sl.value = 0; num.value = 0; q("[data-cmd]").textContent = "0°";
      toast(this.name + " 回中位");
    };
    q("[data-zero]").onclick = () => {
      API.servoNow(this.idx, "zero");
      toast(this.name + " 当前位置记为 0°");
    };
    q("[data-dir]").onclick = () => { API.servoNow(this.idx, "dir"); };
    q("[data-cal]").onclick = () => {
      API.servoNow(this.idx, "trim", clampNum(q("[data-trim]").value, lo, hi));
      API.servoNow(this.idx, "scale", clampNum(q("[data-scale]").value, 0.1, 2));
      toast(this.name + " 标定已发送");
    };
    /* 两步标定:
     * ① 记 0°  -> 当前位置记为逻辑 0° (写 trim)
     * ② 记 90° -> 用户已拖到物理 90° 标记, 此时滑块值 L 对应物理 90°;
     *             比例 scale = scale旧 × L / 90, 再把滑块归到 90° 保持不动。
     */
    q("[data-rec0]").onclick = () => {
      API.servoNow(this.idx, "zero");
      toast(this.name + " 已把当前位置记为 0°");
    };
    q("[data-rec90]").onclick = () => {
      const L = Math.abs(parseFloat(q("[data-sl]").value) || 0);
      const sOld = (this.last && this.last.scale) ? this.last.scale : 1;
      const ns = clampNum(sOld * L / 90, 0.1, 3);
      API.servoNow(this.idx, "scale", ns);
      API.servoNow(this.idx, "angle", 90);
      q("[data-sl]").value = 90;
      q("[data-num]").value = 90;
      q("[data-cmd]").textContent = "90°";
      this.synced = true;
      toast(this.name + " 比例 scale = " + ns.toFixed(3));
    };
    q("[data-resetcal]").onclick = () => {
      API.servoNow(this.idx, "resetcal");
      API.servoNow(this.idx, "trim", 0);
      API.servoNow(this.idx, "scale", 1);
      toast(this.name + " 标定已清除");
    };
  }

  update(st, live) {
    const q = (s) => this.el.querySelector(s);
    q("[data-led]").classList.toggle("on", !!live && !!st);
    if (!st) return;
    this.last = st;

    q("[data-deg]").innerHTML = st.angle.toFixed(1) + "<small>°</small>";
    q("[data-pulse]").textContent = "脉宽 " + st.pulse + " us";
    q("[data-dir]").textContent = "方向: " + (st.dir ? "反" : "正");
    q("[data-calinfo]").textContent = "trim " + st.trim.toFixed(1) + "° · scale " + st.scale.toFixed(3);

    const sl = q("[data-sl]");
    const num = q("[data-num]");
    if (!this.synced && !this.dragging) {
      sl.value = st.angle;
      num.value = st.angle.toFixed(1);
      q("[data-cmd]").textContent = st.angle.toFixed(1) + "°";
      this.synced = true;
    }
    const tin = q("[data-trim]");
    const sc = q("[data-scale]");
    if (document.activeElement !== tin) tin.value = st.trim.toFixed(1);
    if (document.activeElement !== sc) sc.value = st.scale.toFixed(3);
  }
}

class PTK7350Card extends ServoCard {}
class PTK7465Card extends ServoCard {}
