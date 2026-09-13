"""把车间看板实时推到网页，用来和 Gazebo 界面并排比对。

    ros2 run fleetflow_sim live_view --ros-args -p port:=8080 -p every_s:=1.5 -p size:=1280x800

然后浏览器打开 http://127.0.0.1:8080 。

存在的理由很具体：Gazebo 里看到的是三维场景，看板里看到的是同一批话题画出来的
二维平面图。两边都是同一份 `/fleet/robots`、`/factory/machines`、`/factory/summary`，
所以把两块屏并排放，就能一眼看出坐标、朝向、工位、任务流向的映射对不对 ——
这比盯着日志猜要快得多。

实现上刻意复用 `dashboard.Dashboard`：网页里看到的图和 README 里的截图是同一个
渲染器画的，不会出现"文档一张图、现场另一张图"。

三个端点：
  /            带自动重连的页面，内嵌下面的 MJPEG 流
  /stream      multipart/x-mixed-replace，浏览器当视频播，无需轮询
  /frame.png   最新一帧，方便脚本或 `<img>` 拉取
"""
from __future__ import annotations

import io
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import rclpy
from rclpy.executors import ExternalShutdownException

from .dashboard import Dashboard

PAGE = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>FleetFlow-ROS2 · 车间看板</title>
<style>
  html,body{margin:0;height:100%%;background:#f4f2ed;color:#1c1c1a;
            font:14px/1.5 system-ui,"Noto Sans CJK SC",sans-serif}
  header{padding:10px 16px;border-bottom:1px solid #c9c4b8;display:flex;
         align-items:baseline;gap:14px;background:#fbfaf7}
  header b{font-size:16px}
  header span{color:#6d6a63;font-size:12px}
  #wrap{height:calc(100%% - 46px);display:flex;align-items:center;justify-content:center}
  img{max-width:100%%;max-height:100%%;object-fit:contain;
      box-shadow:0 1px 4px rgba(0,0,0,.12);background:#fff}
</style></head>
<body>
<header>
  <b>FleetFlow-ROS2 · 车间看板</b>
  <span>实时渲染 · 与 Gazebo 界面同源话题 · %(size)s</span>
  <span id="st" style="margin-left:auto">连接中…</span>
</header>
<div id="wrap"><img id="v" src="/stream" alt="车间看板实时画面"></div>
<script>
  const img=document.getElementById('v'), st=document.getElementById('st');
  img.onload=()=>{st.textContent='已连接';st.style.color='#2e7d4f';};
  img.onerror=()=>{st.textContent='断开，重连中…';st.style.color='#c0392b';
                   setTimeout(()=>{img.src='/stream?t='+Date.now();},1500);};
</script>
</body></html>
"""


class _NamedBuffer(io.BytesIO):
    """BytesIO 带个名字，纯粹是为了让父类的 `frame -> {path}` 日志可读。"""

    def __str__(self):
        return "live"

    __repr__ = __str__


class _Frame:
    """最新一帧的线程安全容器。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.png: bytes | None = None
        self.n = 0
        self.t = 0.0

    def put(self, data: bytes):
        with self.lock:
            self.png = data
            self.n += 1
            self.t = time.time()

    def get(self):
        with self.lock:
            return self.png, self.n, self.t


class LiveView(Dashboard):
    """看板节点 + 内存出图。渲染仍走 Dashboard.render，保证与 README 图一致。"""

    def __init__(self):
        super().__init__()
        # 这些刻意做成 ROS 参数而不是 argparse：launch 里能直接传，
        # `ros2 run ... --ros-args -p port:=8099` 也能覆盖，和其它节点一致。
        self.declare_parameter("port", 8080)
        self.declare_parameter("host", "127.0.0.1")
        # 注意不能用 every_s：父类 Dashboard 已经声明过它（那是"落盘出图"的开关），
        # 重复声明会直接抛 ParameterAlreadyDeclaredException。
        self.declare_parameter("refresh_s", 1.5)
        self.declare_parameter("size", "1280x800")

        self.port = int(self.get_parameter("port").value)
        self.host = str(self.get_parameter("host").value)
        interval = float(self.get_parameter("refresh_s").value)
        raw = str(self.get_parameter("size").value).lower().replace(" ", "")
        try:
            w, h = (int(v) for v in raw.split("x"))
        except ValueError:
            self.get_logger().warn(f"size 解析失败：{raw!r}，回退 1280x800")
            w, h = 1280, 800
        self.frame = _Frame()
        self.interval = interval
        self.size = (w, h)
        self.rendering = False
        self.create_timer(interval, self._tick)
        self.get_logger().info(
            f"live view · {w}x{h} · 每 {interval:.1f}s 一帧 · "
            f"http://{self.host}:{self.port}")

    def _tick(self):
        # 出图比定时器慢时直接跳过这一拍，避免回调排队把执行器堵死
        if self.rendering:
            return
        self.rendering = True
        try:
            buf = _NamedBuffer()      # 让 Dashboard 的日志打印 "frame -> live" 而不是 repr
            self.render(path=buf, size=self.size)
            self.frame.put(buf.getvalue())
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn(f"render failed: {exc}")
        finally:
            self.rendering = False


def _handler_for(frame: _Frame, size):
    page = (PAGE % {"size": f"{size[0]}×{size[1]}"}).encode("utf-8")

    class Handler(BaseHTTPRequestHandler):
        protocol_version = "HTTP/1.0"

        def log_message(self, *a):                    # 静音，别刷日志
            pass

        def do_GET(self):                             # noqa: N802
            path = self.path.split("?")[0]
            if path in ("/", "/index.html"):
                self.send_response(200)
                self.send_header("Content-Type", "text/html; charset=utf-8")
                self.send_header("Content-Length", str(len(page)))
                self.end_headers()
                self.wfile.write(page)
            elif path == "/frame.png":
                png, _, _ = frame.get()
                if png is None:
                    self.send_error(503, "no frame yet")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/png")
                self.send_header("Content-Length", str(len(png)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(png)
            elif path == "/stream":
                # multipart/x-mixed-replace：浏览器把它当视频播，天然免轮询
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                last = -1
                try:
                    while True:
                        png, n, _ = frame.get()
                        if png is None or n == last:
                            time.sleep(0.15)
                            continue
                        last = n
                        self.wfile.write(b"--frame\r\nContent-Type: image/png\r\n")
                        self.wfile.write(
                            f"Content-Length: {len(png)}\r\n\r\n".encode())
                        self.wfile.write(png)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass
            else:
                self.send_error(404)

    return Handler


def main(argv=None):
    rclpy.init(args=argv)
    node = LiveView()
    srv = ThreadingHTTPServer((node.host, node.port),
                              _handler_for(node.frame, node.size))
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    node.get_logger().info(f"打开 http://{node.host}:{node.port} 查看实时看板")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        srv.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
