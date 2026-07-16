#!/usr/bin/env python3
"""极简 MJPEG over HTTP 推流服务(Python 标准库,零第三方依赖)。

在后台线程跑一个 http.server,把最新一帧 JPEG 以 multipart/x-mixed-replace 推给浏览器。
局域网内手机/电脑打开  http://<飞车IP>:<port>/  即可实时看标注画面;
或直接把  http://<飞车IP>:<port>/stream.mjpg  作为 <img> 源嵌到别处。

用法:
    srv = MjpegServer(host="0.0.0.0", port=8080, title="flycar rescuee")
    srv.start()
    ...
    srv.update_frame(jpeg_bytes)   # 每出一帧就喂一次
    ...
    srv.stop()
"""

import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

_BOUNDARY = "flycarframe"

_INDEX_HTML = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport"
 content="width=device-width,initial-scale=1">
<title>{title}</title>
<style>
 html,body{{margin:0;background:#111;height:100%;display:flex;
   flex-direction:column;align-items:center;justify-content:center;
   font-family:system-ui,sans-serif;color:#ddd}}
 h1{{font-size:15px;font-weight:500;margin:10px;opacity:.7}}
 img{{max-width:100%;max-height:88vh;background:#000;
   border:1px solid #333;border-radius:6px}}
</style></head>
<body>
 <h1>{title} &mdash; live</h1>
 <img src="/stream.mjpg" alt="stream">
</body></html>
"""


class _FrameHub:
    """保存最新一帧,并用 Condition 唤醒所有等待的推流客户端。"""

    def __init__(self):
        self._cond = threading.Condition()
        self._frame = None
        self._seq = 0
        self._closed = False

    def update(self, jpeg_bytes):
        with self._cond:
            self._frame = jpeg_bytes
            self._seq += 1
            self._cond.notify_all()

    def close(self):
        with self._cond:
            self._closed = True
            self._cond.notify_all()

    def wait_next(self, last_seq, timeout=2.0):
        """阻塞到有比 last_seq 新的一帧,返回 (frame, seq);超时/关闭返回 (None, last_seq)。"""
        with self._cond:
            if self._closed:
                return None, last_seq
            if self._seq == last_seq:
                self._cond.wait(timeout)
            if self._closed or self._seq == last_seq:
                return None, self._seq
            return self._frame, self._seq


def _make_handler(hub, title):
    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"

        def log_message(self, *args):  # 静音默认访问日志
            pass

        def do_GET(self):
            if self.path in ("/", "/index.html"):
                self._serve_index()
            elif self.path.startswith("/stream.mjpg"):
                self._serve_stream()
            else:
                self.send_error(404)

        def _serve_index(self):
            body = _INDEX_HTML.format(title=title).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def _serve_stream(self):
            self.send_response(200)
            self.send_header("Age", "0")
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header(
                "Content-Type",
                f"multipart/x-mixed-replace; boundary={_BOUNDARY}")
            self.end_headers()
            last_seq = 0
            try:
                while True:
                    frame, last_seq = hub.wait_next(last_seq)
                    if frame is None:
                        # 超时无新帧:发个心跳判断连接是否还在(会抛异常则退出)
                        if hub._closed:
                            break
                        continue
                    self.wfile.write(b"--" + _BOUNDARY.encode() + b"\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(
                        b"Content-Length: " + str(len(frame)).encode() + b"\r\n\r\n")
                    self.wfile.write(frame)
                    self.wfile.write(b"\r\n")
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass  # 客户端断开,正常退出

    return Handler


class MjpegServer:
    def __init__(self, host="0.0.0.0", port=8080, title="flycar stream"):
        self.host = host
        self.port = port
        self._hub = _FrameHub()
        self._httpd = ThreadingHTTPServer(
            (host, port), _make_handler(self._hub, title))
        self._httpd.daemon_threads = True
        self._thread = None

    def start(self):
        self._thread = threading.Thread(
            target=self._httpd.serve_forever, name="mjpeg-http", daemon=True)
        self._thread.start()
        return self

    def update_frame(self, jpeg_bytes):
        self._hub.update(jpeg_bytes)

    @property
    def url(self):
        return f"http://{self.host}:{self.port}/"

    def stop(self):
        self._hub.close()
        try:
            self._httpd.shutdown()
            self._httpd.server_close()
        except Exception:
            pass
