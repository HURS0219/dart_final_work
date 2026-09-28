# web.py — HTTP 服务: 提供网页文件 + 结构化 API (内部调用 OO 驱动对象)

import socket
import time


def send_all(sock, data):
    mv = memoryview(data)
    while len(mv):
        n = sock.send(mv)
        if n is None:
            break
        mv = mv[n:]


def http_ok(cl, ctype, body):
    if isinstance(body, str):
        body = body.encode()
    hdr = ("HTTP/1.1 200 OK\r\nContent-Type: %s\r\nContent-Length: %d\r\n"
           "Connection: close\r\n\r\n" % (ctype, len(body)))
    send_all(cl, hdr.encode())
    send_all(cl, body)


def http_404(cl):
    send_all(cl, b"HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n")


def parse_query(path):
    q = {}
    if "?" in path:
        for kv in path.split("?", 1)[1].split("&"):
            if "=" in kv:
                k, v = kv.split("=", 1)
                q[k] = v
    return q


def urldecode(s):
    out = bytearray()
    i = 0
    n = len(s)
    while i < n:
        c = s[i]
        if c == "%" and i + 2 < n:
            try:
                out.append(int(s[i + 1:i + 3], 16))
                i += 3
                continue
            except Exception:
                pass
        if c == "+":
            out.append(32)
        else:
            out.append(ord(c))
        i += 1
    return out.decode()


_CTYPE = {".html": "text/html; charset=utf-8", ".css": "text/css; charset=utf-8",
          ".js": "application/javascript; charset=utf-8", ".json": "application/json"}


class WebServer:
    def __init__(self, bus, motors, servo, task):
        self.bus = bus
        self.motors = motors
        self.servo = servo
        self.task = task
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(socket.getaddrinfo("0.0.0.0", 80)[0][-1])
        self.srv.listen(4)
        self.srv.settimeout(0.004)

    # ---------- API ----------
    def _api_motor(self, q):
        try:
            slot = int(q.get("slot", "0"))
        except Exception:
            return
        if slot < 0 or slot >= len(self.motors):
            return
        m = self.motors[slot]
        op = q.get("op", "")
        if op == "stop":
            m.stop()
        elif op == "speed":
            m.speed(float(q.get("v", "0")))
        elif op == "angle":
            m.angle(float(q.get("v", "0")))
        elif op == "turns":
            m.turns(float(q.get("v", "0")))
        elif op == "zero":
            m.zero()
        elif op == "dir":
            m.toggle_dir()
        elif op == "param":
            m.set_param(int(q.get("id", "0")), int(q.get("val", "0")))
        elif op == "reset":
            m.reset_params()

    def _api_servo(self, q):
        op = q.get("op", "")
        v = float(q.get("v", "0"))
        if op == "std":
            self.servo.set_std(v)
        elif op == "prep":
            self.servo.set_prep(v)
        elif op == "gostd":
            self.servo.go_std()
        elif op == "goprep":
            self.servo.go_prep()
        elif op == "deg":
            self.servo.go_deg(v)
        elif op == "zero":
            self.servo.zero()

    def _api_task(self, q):
        op = q.get("op", "")
        v = float(q.get("v", "0"))
        if op == "start":
            self.task.start()
        elif op == "stop":
            self.task.stop()
        elif op == "estop":
            self.task.estop()
        elif op == "clear":
            self.task.clear_estop()
        elif op == "springstd":
            self.task.spring_std()
        elif op == "springprep":
            self.task.spring_prep()
        elif op == "servostd":
            self.task.servo_std()
        elif op == "servoprep":
            self.task.servo_prep()
        elif op == "turns":
            self.task.set_turns(v)
        elif op == "yaw":
            self.task.set_yaw(int(v))
        elif op == "aimrpm":
            self.bus.send("A,%d" % int(round(v * 100)))

    # ---------- 路由 ----------
    def _handle(self, path, cl):
        if path.startswith("/state"):
            http_ok(cl, "text/plain", self.bus.raw)
        elif path.startswith("/scanres"):
            http_ok(cl, "text/plain", ",".join(str(x) for x in self.bus.scan_ids))
        elif path.startswith("/scan"):
            self.bus.scan()
            http_ok(cl, "text/plain", b"ok")
        elif path.startswith("/ping"):
            self.bus.send("H")
            http_ok(cl, "text/plain", b"ok")
        elif path.startswith("/save"):
            self.bus.save()
            http_ok(cl, "text/plain", b"ok")
        elif path.startswith("/api/motor"):
            self._api_motor(parse_query(path))
            http_ok(cl, "text/plain", b"ok")
        elif path.startswith("/api/servo"):
            self._api_servo(parse_query(path))
            http_ok(cl, "text/plain", b"ok")
        elif path.startswith("/api/task"):
            self._api_task(parse_query(path))
            http_ok(cl, "text/plain", b"ok")
        elif path.startswith("/api/vision"):
            q = parse_query(path)
            self.bus.inject_vision(int(q.get("x", "160")), int(q.get("c", "160")))
            http_ok(cl, "text/plain", b"ok")
        elif path.startswith("/cmd"):
            q = parse_query(path)
            if "c" in q:
                self.bus.send(urldecode(q["c"]))
            http_ok(cl, "text/plain", b"ok")
        elif path.startswith("/favicon"):
            http_ok(cl, "image/x-icon", b"")
        else:
            # 静态文件
            rel = path.lstrip("/")
            if rel == "":
                rel = "index.html"
            if ".." in rel:
                http_404(cl)
                return
            try:
                with open("www/" + rel, "rb") as f:
                    body = f.read()
                ext = rel[rel.rfind("."):] if "." in rel else ""
                http_ok(cl, _CTYPE.get(ext, "application/octet-stream"), body)
            except Exception:
                http_404(cl)

    def poll_http(self):
        while True:
            try:
                cl, _ = self.srv.accept()
            except OSError:
                break
            try:
                cl.settimeout(0.3)
                req = cl.recv(1024).decode()
                path = req.split(" ", 2)[1] if " " in req else "/"
                self._handle(path, cl)
            except Exception as e:
                print("http err:", e)
            finally:
                try:
                    cl.close()
                except Exception:
                    pass
