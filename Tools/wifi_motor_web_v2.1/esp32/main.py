# main.py — WiFi 多电机网页控制/调参 标准应用 (ESP32-S3 / MicroPython)
# 版本: wifi_motor_web v2.1
#
# 一条 CAN 多电机同时控制; 每个电机独立一套参数。
# ESP32 -> STM32:
#   C,<mode>,<value>   控制全部(0停/1速度RPM/2角度deg*10)
#   A,<type>,<id>      添加电机     X 清空     Z 全部设零
#   U,<slot>           选择要调参的电机(槽位)
#   P,<id>,<value>     设置“所选电机”的参数(浮点x100)
#   S / PING           扫描 / 自检
# STM32 -> ESP32:
#   F,<mode>,<count>[,<slot>,<type>,<id>,<rpm>]*count,<sel>,<10参数>,<limit>
#   S,<n>,<fb_id...>

import network
import socket
import time
from machine import UART

# ======================== 配置区 ========================
AP_SSID = "MOTOR_CTRL"
AP_PASS = "12345678"
UART_ID = 1
UART_TX = 17
UART_RX = 18
BAUD = 115200
MAX_RPM = 120
MAX_ANGLE = 180
PARAMS = [
    (1, "速度前馈 FF", 100, 0.1),
    (2, "速度 Kp", 100, 0.1),
    (3, "速度 Ki", 100, 0.1),
    (4, "积分限幅 I-limit", 100, 1),
    (5, "角度 Kp", 100, 1),
    (10, "减速比", 100, 0.01),
    (11, "角度死区(deg)", 100, 0.1),
    (12, "角度限速(输出RPM)", 100, 1),
]
LIMIT_ID = 6
TYPE_NAMES = ["GM6020", "M3508", "M2006"]
NP = len(PARAMS)
# =======================================================

PRESET_FILE = "preset.txt"

_rows = "".join('<div class="r"><label>%s</label><input id="p%d" type="number" step="%s"></div>' % (lab, pid, step)
                for (pid, lab, sc, step) in PARAMS)
_js_params = ",".join("[%d,%d]" % (pid, sc) for (pid, lab, sc, step) in PARAMS)

PAGE = """<!DOCTYPE html>
<html lang="zh">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>电机控制</title>
<style>
  body{font-family:-apple-system,Roboto,sans-serif;background:#111;color:#eee;margin:0;padding:14px;text-align:center}
  h2{margin:4px 0 12px;font-size:20px}
  .modes{display:flex;gap:8px;justify-content:center;margin-bottom:12px}
  .modes button{flex:1;max-width:100px;padding:12px 0;font-size:15px;border:1px solid #444;border-radius:10px;background:#222;color:#ddd}
  .modes button.on{background:#2d7dff;border-color:#2d7dff;color:#fff}
  .val{font-size:36px;font-weight:700;margin:4px 0}
  input[type=range]{width:94%;height:40px;accent-color:#2d7dff}
  .hint{color:#888;margin-top:8px;font-size:13px}
  .box{margin-top:14px;border-top:1px solid #333;padding-top:10px;text-align:left}
  .box h3{text-align:center;color:#2d7dff;margin:4px 0 8px;font-size:16px}
  .r{display:flex;justify-content:space-between;align-items:center;margin:6px 0;gap:6px}
  .r label{flex:1;color:#ccc;font-size:14px}
  .r input{width:96px;padding:6px;font-size:15px;border-radius:7px;border:1px solid #444;background:#222;color:#fff;text-align:right}
  .r select{padding:6px;border-radius:7px;border:1px solid #444;background:#222;color:#fff}
  button{border:none;border-radius:7px;background:#2d7dff;color:#fff}
  button:disabled{background:#555;color:#999}
  .wide{width:100%;padding:11px;margin-top:8px;font-size:15px}
  .green{background:#3a3}.red{background:#c33}.gray{background:#444}
  .small{padding:6px 10px;font-size:13px}
  .m{font-family:monospace;color:#8cf;font-size:14px;margin:4px 0}
</style>
</head>
<body>
<h2>多电机控制</h2>
<div class="modes">
  <button id="b0" onclick="setMode(0)">停止</button>
  <button id="b1" onclick="setMode(1)">速度</button>
  <button id="b2" onclick="setMode(2)">角度</button>
</div>
<div class="val" id="disp">--</div>
<input type="range" id="slider" min="-120" max="120" step="1" value="0" oninput="onSlide()">
<div class="hint" id="hint">拖动滑块 (同时控制所有电机)</div>

<div class="box">
  <h3>电机</h3>
  <div id="mlist" class="m">--</div>
  <button class="wide green" id="btnscan" onclick="doScan()">扫描电机</button>
  <div id="scanres" class="hint"></div>
  <button class="wide red" id="btnclear" onclick="clearMotors()">清空列表</button>
  <button class="wide gray" id="btnzero" onclick="zeroAll()">全部设零(角度0)</button>
</div>

<div class="box" id="tune">
  <h3>调参 <span id="selname" style="color:#8cf;font-size:13px"></span></h3>
  <div class="r"><label>限幅(电流/电压)</label><input id="lim" type="number" step="100"><button class="small" onclick="setLim()">设</button></div>
  __ROWS__
  <button class="wide" onclick="apply()">应用参数</button>
</div>

<div class="box">
  <h3>保存/恢复目标</h3>
  <div class="r"><label>速度(RPM)</label><input id="savspd" type="number" step="1"><button class="small" onclick="saveP('speed')">存</button><button class="small gray" onclick="loadP('speed')">取</button></div>
  <div class="r"><label>角度(deg)</label><input id="savang" type="number" step="1"><button class="small" onclick="saveP('angle')">存</button><button class="small gray" onclick="loadP('angle')">取</button></div>
</div>

<script>
const MAX_RPM=__MAX_RPM__, MAX_ANGLE=__MAX_ANGLE__, LIM_ID=__LIM_ID__;
const PARAMS=[__JS_PARAMS__], TYPES=["GM6020","M3508","M2006"];
let mode=1, filled=false, sel=-1, running=false;
function cfg(m){
  if(m===1) return {min:-MAX_RPM,max:MAX_RPM,unit:' RPM',scale:1};
  if(m===2) return {min:-MAX_ANGLE,max:MAX_ANGLE,unit:' deg',scale:10};
  return {min:-MAX_RPM,max:MAX_RPM,unit:'',scale:1};
}
function setMode(m){
  mode=m; const c=cfg(m), s=document.getElementById('slider');
  s.min=c.min; s.max=c.max; s.value=0;
  document.getElementById('b0').className=(m===0)?'on':'';
  document.getElementById('b1').className=(m===1)?'on':'';
  document.getElementById('b2').className=(m===2)?'on':'';
  document.getElementById('hint').textContent=(m===0)?'已停止':(m===1)?'拖动滑块 (同时控制所有电机)':'拖动滑块转到固定角度';
  onSlide();
}
function onSlide(){
  const v=parseInt(document.getElementById('slider').value), c=cfg(mode);
  document.getElementById('disp').textContent=(mode===0)?'--':(v+c.unit);
}
let lastSend=0,tmr=null;
function send(){
  const c=cfg(mode), v=parseInt(document.getElementById('slider').value);
  const value=(mode===0)?0:Math.round(v*c.scale), now=Date.now();
  const f=()=>{lastSend=Date.now();fetch('/set?mode='+mode+'&value='+value).catch(()=>{});};
  if(now-lastSend>80) f(); else if(!tmr) tmr=setTimeout(()=>{tmr=null;f();},80-(now-lastSend));
}
function setLim(){ fetch('/setp?id='+LIM_ID+'&value='+parseInt(document.getElementById('lim').value)).catch(()=>{}); }
function doScan(){ document.getElementById('scanres').textContent='扫描中...'; fetch('/scan').catch(()=>{}); setTimeout(loadScan,1300); }
function loadScan(){ fetch('/scanres').then(r=>r.text()).then(t=>{ renderScan(t.trim()?t.trim().split(',').map(Number):[]); }).catch(()=>{}); }
function renderScan(ids){
  const el=document.getElementById('scanres');
  if(ids.length===0){ el.textContent='未发现电机'; return; }
  let h='';
  ids.forEach(fid=>{
    let sug=0,id=fid-0x204;
    if(fid>=0x201&&fid<=0x204){ sug=1; id=fid-0x200; }
    else if(fid>=0x205&&fid<=0x208){ sug=0; id=fid-0x204; }
    else if(fid>=0x209&&fid<=0x20B){ sug=0; id=fid-0x204; }
    h+='<div class="r"><label>0x'+fid.toString(16).toUpperCase()+'</label>'+
       '<select id="t'+fid+'">'+TYPES.map((n,k)=>'<option value="'+k+'"'+(k===sug?' selected':'')+'>'+n+'</option>').join('')+'</select>'+
       '<input id="i'+fid+'" type="number" min="1" max="8" value="'+id+'" style="width:46px">'+
       '<button class="small" onclick="addMotor('+fid+')">添加</button></div>';
  });
  el.innerHTML=h;
}
function addMotor(fid){ fetch('/add?type='+document.getElementById('t'+fid).value+'&id='+document.getElementById('i'+fid).value).catch(()=>{}); }
function clearMotors(){ if(confirm('清空所有电机?')) fetch('/clearm').catch(()=>{}); }
function zeroAll(){ if(confirm('把当前方向设为所有电机的角度0?')) fetch('/zero').catch(()=>{}); }
function selectMotor(slot){ fetch('/select?idx='+slot).catch(()=>{}); filled=false; }
function fillTune(p){ PARAMS.forEach((a,i)=>{ const el=document.getElementById('p'+a[0]); if(el) el.value=(p[i]/a[1]).toFixed(2); }); }
function apply(){
  const seq=[];
  for(const [id,scale] of PARAMS){ const el=document.getElementById('p'+id); if(el) seq.push([id,Math.round(parseFloat(el.value)*scale)]); }
  let i=0; const next=()=>{ if(i>=seq.length){alert('参数已下发');return;} const [id,val]=seq[i++]; fetch('/setp?id='+id+'&value='+val).then(next).catch(next); };
  next();
}
function saveP(w){ const el=document.getElementById(w==='speed'?'savspd':'savang'); fetch('/savep?w='+w+'&v='+parseInt(el.value||0)).catch(()=>{}); }
function loadP(w){
  fetch('/getp').then(r=>r.text()).then(t=>{
    const p=t.split(','); const v=parseInt(w==='speed'?p[0]:p[1]);
    if(isNaN(v)) return;
    document.getElementById(w==='speed'?'savspd':'savang').value=v;
    setMode(w==='speed'?1:2);
    document.getElementById('slider').value=v; onSlide(); send();
  }).catch(()=>{});
}
function poll(){
  fetch('/state').then(r=>r.text()).then(t=>{
    if(!t) return;
    const p=t.split(',').map(Number);
    if(p.length<4) return;
    const m=p[0], count=p[1]; let idx=2, ml='';
    for(let i=0;i<count;i++){
      const slot=p[idx], ty=p[idx+1], id=p[idx+2], rpm=p[idx+3]; idx+=4;
      const on=(slot===sel);
      ml+='<div class="r"><label style="color:'+(on?'#2d7dff':'#8cf')+'">'+TYPES[ty]+' ID'+id+' : '+rpm+' RPM</label>'+
          '<button class="small '+(on?'':'gray')+'" onclick="selectMotor('+slot+')">'+(on?'调参中':'选')+'</button></div>';
    }
    document.getElementById('mlist').innerHTML = ml||'-- (先扫描并添加)';
    sel=p[idx]; idx+=1;
    const params=p.slice(idx,idx+10); idx+=10;
    const limit=p[idx];
    document.getElementById('selname').textContent = (sel>=0&&sel<4)?'':'(未选)';
    const le=document.getElementById('lim'); if(document.activeElement!==le) le.value=limit;
    const canEdit = (m===0);
    document.getElementById('tune').style.display = canEdit?'block':'none';
    document.getElementById('btnscan').disabled = !canEdit;
    document.getElementById('btnclear').disabled = !canEdit;
    document.getElementById('btnzero').disabled = !canEdit;
    if(canEdit && !filled && sel>=0){ fillTune(params); filled=true; }
  }).catch(()=>{});
}
setMode(1);
setInterval(send,250);
setInterval(poll,400);
</script>
</body>
</html>
"""
PAGE = (PAGE.replace("__ROWS__", _rows)
            .replace("__JS_PARAMS__", _js_params)
            .replace("__MAX_RPM__", str(MAX_RPM))
            .replace("__MAX_ANGLE__", str(MAX_ANGLE))
            .replace("__LIM_ID__", str(LIMIT_ID)))


def send_all(sock, data):
    mv = memoryview(data)
    while len(mv):
        n = sock.send(mv)
        if n is None:
            break
        mv = mv[n:]


def http_ok(cl, content_type, body):
    hdr = ("HTTP/1.1 200 OK\r\nContent-Type: %s\r\nContent-Length: %d\r\nConnection: close\r\n\r\n"
           % (content_type, len(body)))
    send_all(cl, hdr.encode())
    send_all(cl, body)


def parse_query(path):
    q = {}
    if "?" in path:
        for kv in path.split("?", 1)[1].split("&"):
            if "=" in kv:
                k, v = kv.split("=", 1)
                q[k] = v
    return q


uart = UART(UART_ID, baudrate=BAUD, tx=UART_TX, rx=UART_RX)

ap = network.WLAN(network.AP_IF)
ap.active(True)
ap.config(essid=AP_SSID, password=AP_PASS, max_clients=4)
while not ap.active():
    time.sleep_ms(50)
print("AP ready:", ap.ifconfig()[0])

srv = socket.socket()
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind(socket.getaddrinfo("0.0.0.0", 80)[0][-1])
srv.listen(4)
srv.settimeout(0.05)

mode = 0
value = 0
last_http = time.ticks_ms()
state_str = ""
scan_ids = []
rxbuf = b""
last_cmd = time.ticks_ms()
preset = [0, 0]


def load_preset():
    try:
        with open(PRESET_FILE) as f:
            v = [int(x) for x in f.read().strip().split(",")]
        preset[0], preset[1] = v[0], v[1]
    except Exception:
        pass


def save_preset():
    try:
        with open(PRESET_FILE, "w") as f:
            f.write("%d,%d" % (preset[0], preset[1]))
    except Exception:
        pass


load_preset()
print("HTTP server listening on :80")


def send_cmd():
    try:
        uart.write("C,%d,%d\n" % (mode, value))
    except Exception:
        pass


while True:
    while True:
        try:
            cl, _ = srv.accept()
        except OSError:
            break
        try:
            cl.settimeout(0.3)
            req = cl.recv(512).decode()
            path = req.split(" ", 2)[1] if " " in req else "/"
            if path.startswith("/setp"):
                q = parse_query(path)
                if "id" in q and "value" in q:
                    uart.write("P,%s,%s\n" % (q["id"], q["value"]))
                http_ok(cl, "text/plain", b"ok")
            elif path.startswith("/select"):
                q = parse_query(path)
                if "idx" in q:
                    uart.write("U,%s\n" % q["idx"])
                http_ok(cl, "text/plain", b"ok")
            elif path.startswith("/add"):
                q = parse_query(path)
                if "type" in q and "id" in q:
                    uart.write("A,%s,%s\n" % (q["type"], q["id"]))
                http_ok(cl, "text/plain", b"ok")
            elif path.startswith("/clearm"):
                uart.write("X\n"); http_ok(cl, "text/plain", b"ok")
            elif path.startswith("/scanres"):
                http_ok(cl, "text/plain", ",".join(str(x) for x in scan_ids).encode())
            elif path.startswith("/scan"):
                uart.write("S\n"); http_ok(cl, "text/plain", b"ok")
            elif path.startswith("/zero"):
                uart.write("Z\n"); http_ok(cl, "text/plain", b"ok")
            elif path.startswith("/savep"):
                q = parse_query(path)
                w = q.get("w", "speed"); v = int(q.get("v", "0"))
                if w == "speed": preset[0] = v
                else: preset[1] = v
                save_preset()
                http_ok(cl, "text/plain", b"ok")
            elif path.startswith("/getp"):
                http_ok(cl, "text/plain", ("%d,%d" % (preset[0], preset[1])).encode())
            elif path.startswith("/set"):
                q = parse_query(path)
                if "mode" in q: mode = int(q["mode"])
                if "value" in q: value = int(q["value"])
                last_http = time.ticks_ms()
                send_cmd()
                http_ok(cl, "text/plain", b"ok")
            elif path.startswith("/state"):
                http_ok(cl, "text/plain", state_str.encode())
            elif path.startswith("/favicon"):
                http_ok(cl, "image/x-icon", b"")
            else:
                http_ok(cl, "text/html; charset=utf-8", PAGE.encode())
        except Exception as e:
            print("http err:", e)
        finally:
            try:
                cl.close()
            except Exception:
                pass

    try:
        rx = uart.read()
        if rx:
            rxbuf += rx
            while b"\n" in rxbuf:
                line, rxbuf = rxbuf.split(b"\n", 1)
                line = line.strip()
                if line.startswith(b"F,"):
                    state_str = line[2:].decode()
                elif line.startswith(b"S,"):
                    p = line.decode().split(",")
                    if len(p) >= 2:
                        n = int(p[1]); scan_ids = [int(x) for x in p[2:2 + n]]
    except Exception:
        pass

    now = time.ticks_ms()
    if time.ticks_diff(now, last_http) > 1500:
        mode = 0; value = 0

    if time.ticks_diff(now, last_cmd) >= 200:
        last_cmd = now
        send_cmd()

    time.sleep_ms(10)
