/* task.js — 状态机 / 总控卡片  (v0.2)
 *
 * 模式: 0 待机 / 1 手动 / 2 混控 / 3 自检
 * 混控指令按百分比下发 (-100% ~ 100%)。混控滑块是命令, 不会被遥测回拽。
 */

class TaskCard {
  constructor() {
    this.mode = 0;
    this.el = document.createElement("div");
    this.el.className = "mcard";
    this._build();
  }

  _build() {
    this.el.innerHTML = `
      <div class="mhead"><span class="name">状态机 / 总控</span><span class="id" data-mode>待机</span></div>
      <div class="seg" data-seg>
        <button data-m="0">待机</button><button data-m="1">手动</button>
        <button data-m="2">混控</button><button data-m="3">自检</button>
      </div>

      <div class="hint" style="margin-top:10px">混控指令按百分比下发 (-100% ~ 100%)，仅“混控”模式生效。</div>
      <div class="row"><label>俯仰 pitch</label><span class="v" data-pv>0%</span></div>
      <input type="range" min="-100" max="100" value="0" data-p>
      <div class="row"><label>偏航 yaw</label><span class="v" data-yv>0%</span></div>
      <input type="range" min="-100" max="100" value="0" data-y>
      <div class="row"><label>滚转 roll</label><span class="v" data-rv>0%</span></div>
      <input type="range" min="-100" max="100" value="0" data-r>

      <div class="btns c3">
        <button class="ghost" data-save>保存标定</button>
        <button class="ghost" data-zero-mix>混控归零</button>
        <button class="ghost" data-test>自检扫舵</button>
      </div>
      <div class="hint" style="margin-top:10px">自检：四舵面缓慢同步摆动；待机：全部回中立。</div>`;

    const q = (s) => this.el.querySelector(s);
    this.el.querySelectorAll("[data-seg] button").forEach((b) => {
      b.onclick = () => {
        this.mode = +b.dataset.m;
        API.task("mode", { v: this.mode });
        this._render();
        toast("模式 → " + b.textContent);
      };
    });

    const sendMix = () => {
      const p = +q("[data-p]").value, y = +q("[data-y]").value, r = +q("[data-r]").value;
      q("[data-pv]").textContent = p + "%";
      q("[data-yv]").textContent = y + "%";
      q("[data-rv]").textContent = r + "%";
      if (this.mode !== 2) { this.mode = 2; API.task("mode", { v: 2 }); this._render(); }
      API.taskQ("mix", { p, y, r });
    };
    q("[data-p]").oninput = sendMix;
    q("[data-y]").oninput = sendMix;
    q("[data-r]").oninput = sendMix;

    q("[data-zero-mix]").onclick = () => {
      q("[data-p]").value = 0; q("[data-y]").value = 0; q("[data-r]").value = 0;
      q("[data-pv]").textContent = "0%"; q("[data-yv]").textContent = "0%"; q("[data-rv]").textContent = "0%";
      API.task("mode", { v: 2 });
      API.task("mix", { p: 0, y: 0, r: 0 });
      this.mode = 2; this._render();
      toast("混控归零");
    };
    q("[data-save]").onclick = () => { API.save(); toast("已发送保存…"); };
    q("[data-test]").onclick = () => { this.mode = 3; API.task("mode", { v: 3 }); this._render(); toast("自检扫舵"); };

    this._render();
  }

  _render() {
    this.el.querySelectorAll("[data-seg] button").forEach((b) =>
      b.classList.toggle("on", +b.dataset.m === this.mode));
    const n = this.el.querySelector("[data-mode]");
    if (n) n.textContent = MODE_NAMES[this.mode] || "?";
  }

  update(mode, mix, live) {
    if (mode !== undefined && mode !== null) { this.mode = mode; this._render(); }
    /* 混控滑块是命令, 不用遥测回写; 百分比文本已在本地维护 */
  }
}
