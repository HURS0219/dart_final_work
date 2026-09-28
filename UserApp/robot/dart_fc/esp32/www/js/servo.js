/* servo.js — 舵面网页卡: 手动滑块 + 微调(可保存)
 * 拖动滑块 -> 切到 MANUAL 并下发偏角 (V,idx,deg*10)
 * 微调保存 -> 写入参数表并保存 Flash (P, trim_id, val)
 */

const P_TRIM = 4;   /* 微调参数起始 ID (与 dart_cfg.h 一致) */

class ServoCard {
  constructor(idx, name, sub) {
    this.idx = idx;
    this.name = name;
    this.sub = sub;
    this.maxDeg = 35;
    this.el = document.createElement("div");
    this.el.className = "mcard";
    this.build();
  }

  build() {
    this.el.innerHTML =
      '<div class="mhead"><span class="name">' + this.name + '</span>' +
      '<span class="id">' + this.sub + '</span>' +
      '<span class="led"></span></div>' +
      '<div class="big"><span class="v-defl">0.0</span><small>°</small></div>' +
      '<input type="range" class="s-man" min="-35" max="35" step="1" value="0">' +
      '<div class="row"><label>手动偏角</label><span class="v v-man">0.0°</span></div>' +
      '<div class="row"><label>微调</label>' +
      '<input type="number" class="i-trim" step="0.5" value="0">' +
      '<button class="ghost b-trim">保存</button></div>';

    const q = s => this.el.querySelector(s);
    const slider = q(".s-man");
    slider.oninput = () => {
      const v = parseFloat(slider.value) || 0;
      q(".v-man").textContent = v.toFixed(1) + "°";
      fetch("/api/surface?op=state&v=4").catch(() => {});   /* 切到手动 */
      fetch("/api/servo?idx=" + this.idx + "&op=manual&v=" + v).catch(() => {});
    };
    q(".b-trim").onclick = () => {
      const t = parseFloat(q(".i-trim").value) || 0;
      fetch("/api/surface?op=param&id=" + (P_TRIM + this.idx) + "&v=" + t).catch(() => {});
      fetch("/save").catch(() => {});
      toast(this.name + " 微调 " + t + "° 已保存");
    };
  }

  update(s, params) {
    if (!s) return;
    const q = x => this.el.querySelector(x);
    q(".v-defl").textContent = (s.defl[this.idx] / 10).toFixed(1);

    if (params && params.length > P_TRIM + this.idx) {
      const m = params[8 + this.idx] / 100;   /* P_MAX = 8 */
      if (m > 1 && m !== this.maxDeg) {
        this.maxDeg = m;
        const sl = q(".s-man");
        sl.min = -Math.round(m);
        sl.max = Math.round(m);
      }
      const t = q(".i-trim");
      if (document.activeElement !== t) t.value = (params[P_TRIM + this.idx] / 100).toFixed(1);
    }
  }
}
