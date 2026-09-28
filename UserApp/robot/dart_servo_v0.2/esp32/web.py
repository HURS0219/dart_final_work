# web.py — HTTP 服务: 静态文件 + 结构化 API (调用 OO 驱动对象)  v0.2

import socket


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
           "Cache-Control: no-store\r\nConnection: close\r\n\r\n" % (ctype, len(body)))
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
    def __init__(self, bus, servos, task):
        self.bus = bus
        self.servos = servos
        self.task = task
        self.srv = socket.socket()
        self.srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.srv.bind(socket.getaddrinfo("0.0.0.0", 80)[0][-1])
        self.srv.listen(4)
        self.srv.settimeout(0.004)

    # ---------- API ----------
    def _api_servo(self, q):
        try:
            idx = int(q.get("idx", "0"))
        except Exception:
            return
        if idx < 0 or idx >= len(self.servos):
            return
        s = self.servos[idx]
        op = q.get("op", "")
        try:
            if op == "angle":
                s.set_angle(float(q.get("v", "0")))
            elif op == "zero":
                s.zero()
            elif op == "dir":
                s.toggle_dir()
            elif op == "trim":
                s.set_trim(float(q.get("v", "0")))
            elif op == "scale":
                s.set_scale(float(q.get("v", "1")))
            elif op == "resetcal":
                s.reset_cal()
        except Exception:
            pass

    def _api_task(self, q):
        op = q.get("op", "")
        try:
            if op == "mode":
                self.task.set_mode(int(float(q.get("v", "0"))))
            elif op == "mix":
                self.task.set_mix(float(q.get("p", "0")), float(q.get("y", "0")),
                                  float(q.get("r", "0")))
            elif op == "save":
                self.task.save()
        except Exception:
            pass

    # ---------- 路由 ----------
    def _handle(self, path, cl):
        if path.startswith("/state"):
            http_ok(cl, "text/plain", self.bus.raw)
        elif path.startswith("/api/status"):
            sv = self.bus.saved
            self.bus.saved = 0
            http_ok(cl, "text/plain", str(sv))
        elif path.startswith("/ping"):
            self.task.ping()
            http_ok(cl, "text/plain", b"ok")
        elif path.startswith("/save"):
            self.task.save()
            http_ok(cl, "text/plain", b"ok")
        elif path.startswith("/api/servo"):
            self._api_servo(parse_query(path))
            http_ok(cl, "text/plain", b"ok")
        elif path.startswith("/api/task"):
            self._api_task(parse_query(path))
            http_ok(cl, "text/plain", b"ok")
        elif path.startswith("/cmd"):
            q = parse_query(path)
            if "c" in q:
                self.bus.send(urldecode(q["c"]))
            http_ok(cl, "text/plain", b"ok")
        elif path.startswith("/favicon"):
            http_ok(cl, "image/x-icon", b"")
        else:
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
                cl.settimeout(0.2)
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
