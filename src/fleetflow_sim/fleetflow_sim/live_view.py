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

from . import layout
import rclpy
from rclpy.executors import ExternalShutdownException

from .dashboard import DPI, FIG_W_IN, PAPER, Board, Dashboard

PAGE = """<!doctype html>
<html lang="zh-CN"><head><meta charset="utf-8">
<title>FleetFlow-ROS2 · 车间看板</title>
<style>
  html,body{margin:0;height:100%%;background:#f4f2ed;color:#1c1c1a;
            font:14px/1.5 system-ui,"Noto Sans CJK SC",sans-serif;overflow:hidden}
  header{padding:8px 14px;border-bottom:1px solid #c9c4b8;display:flex;
         align-items:center;gap:14px;background:#fbfaf7;height:30px}
  header b{font-size:15px}
  header span{color:#6d6a63;font-size:12px}
  #back{margin-left:auto;padding:3px 12px;border:1px solid #c9c4b8;background:#fff;
        border-radius:2px;cursor:pointer;font-size:12px;display:none}
  #st{color:#6d6a63;font-size:12px}
  #wrap{height:calc(100%% - 47px);display:flex;align-items:center;
        justify-content:center;cursor:zoom-in}
  #wrap.big{cursor:zoom-out}
  img{max-width:100%%;max-height:100%%;object-fit:contain;
      box-shadow:0 1px 4px rgba(0,0,0,.12);background:#fff}
</style></head>
<body>
<header>
  <b>FleetFlow-ROS2 · 车间看板</b>
  <span id="hint">点击画面进入大屏地图</span>
  <span id="st">连接中…</span>
  <button id="back">← 返回总览</button>
</header>
<div id="wrap"><img id="v" src="/stream" alt="车间看板实时画面"></div>
<script>
  const img=document.getElementById('v'), st=document.getElementById('st'),
        wrap=document.getElementById('wrap'), back=document.getElementById('back'),
        hint=document.getElementById('hint');
  let big=false;
  function setMode(next){
    big=next;
    img.src = big ? '/map/stream?t='+Date.now() : '/stream?t='+Date.now();
    wrap.classList.toggle('big',big);
    back.style.display = big ? 'block' : 'none';
    hint.textContent = big ? '大屏地图 · 再点一下返回' : '点击画面进入大屏地图';
  }
  wrap.onclick = ()=>setMode(!big);
  back.onclick = (e)=>{e.stopPropagation(); setMode(false);};
  document.onkeydown = (e)=>{ if(e.key==='Escape') setMode(false); };
  img.onload=()=>{st.textContent='已连接';st.style.color='#2e7d4f';};
  img.onerror=()=>{st.textContent='断开，重连中…';st.style.color='#c0392b';
                   setTimeout(()=>{img.src=(big?'/map/stream':'/stream')+'?t='+Date.now();},1500);};
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
        self.watchers = 0        # 有几个流连接在看这个视图

    def watch(self, delta: int):
        with self.lock:
            self.watchers = max(0, self.watchers + delta)

    def age(self) -> float:
        with self.lock:
            return time.time() - self.t

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
        self.declare_parameter("refresh_s", 0.25)   # 渲染能力上限约 4~6 fps
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
        self.map_frame = _Frame()
        self.interval = interval
        self.size = (w, h)
        # 大屏用同样的宽度、更高的像素，保证放大后仍然清晰
        self.map_size = (max(w, 1600), int(max(w, 1600) * h / w))
        self.rendering = False
        self.create_timer(interval, self._tick)
        self.get_logger().info(
            f"live view · {w}x{h} · 每 {interval:.1f}s 一帧 · "
            f"http://{self.host}:{self.port}")

    def render_map(self, path=None, size=None, fig_w_in: float = 11.0):
        """只画平面图，铺满整张画布 —— 网页里"点进去看大屏"用的就是它。

        复用 Dashboard._map_panel，所以大屏和总览图是同一份绘制代码。
        画布宽度取 11 in（总览图是 16 in）：同样像素下字号相对更大，
        放到全屏才读得清。
        """
        import matplotlib.pyplot as plt
        w_px, h_px = size if size else self.size
        dpi = max(20.0, w_px / fig_w_in)
        fig = plt.figure(figsize=(w_px / dpi, h_px / dpi), dpi=dpi, facecolor=PAPER)
        b = Board(fig)
        self._paper_grid(b)
        self._map_panel(b, region=(0.006, 0.006, 0.988, 0.988))
        fig.savefig(path, facecolor=PAPER)
        plt.close(fig)
        return path

    def _tick(self):
        """只渲染当前有人看的视图。

        这是帧率的关键：总览一帧 250 ms、大屏 160 ms，如果每拍都同时渲染两张，
        即使把间隔压到最小也只能到 ~2.5 fps。实际同一时刻只有一块屏在看，
        所以按观看者计数决定渲染谁 —— 单独看大屏时能跑到 6 fps 以上。
        出图比定时器慢时跳过这一拍，避免回调排队把执行器堵死。
        """
        if self.rendering:
            return
        want_board = self.frame.watchers > 0
        want_map = self.map_frame.watchers > 0
        if not (want_board or want_map):
            return
        self.rendering = True
        try:
            if want_board:
                buf = _NamedBuffer()  # 让 Dashboard 的日志打印 "frame -> live" 而不是 repr
                self.render(path=buf, size=self.size)
                self.frame.put(buf.getvalue())
            if want_map:
                mbuf = _NamedBuffer()
                self.render_map(path=mbuf, size=self.map_size)
                self.map_frame.put(mbuf.getvalue())
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn(f"render failed: {exc}")
        finally:
            self.rendering = False

    def ensure(self, which: str, stale_s: float = 0.4):
        """静图端点：缓存过期就现渲染一张，保证 /frame.png 永远有内容。"""
        f = self.frame if which == "board" else self.map_frame
        if f.age() < stale_s:
            return
        if self.rendering:
            return
        self.rendering = True
        try:
            buf = _NamedBuffer()
            if which == "board":
                self.render(path=buf, size=self.size)
            else:
                self.render_map(path=buf, size=self.map_size)
            f.put(buf.getvalue())
        except Exception as exc:                      # noqa: BLE001
            self.get_logger().warn(f"render failed: {exc}")
        finally:
            self.rendering = False


def _handler_for(frame: _Frame, map_frame: _Frame, size, ensure=None):
    page = (PAGE % {"size": f"{size[0]}×{size[1]}"}).encode("utf-8")
    sources = {"/frame.png": frame, "/map.png": map_frame,
               "/stream": frame, "/map/stream": map_frame}

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
            elif path in ("/frame.png", "/map.png"):
                if ensure is not None:
                    ensure("board" if path == "/frame.png" else "map")
                png, _, _ = sources[path].get()
                if png is None:
                    self.send_error(503, "no frame yet")
                    return
                self.send_response(200)
                self.send_header("Content-Type", "image/png")
                self.send_header("Content-Length", str(len(png)))
                self.send_header("Cache-Control", "no-store")
                self.end_headers()
                self.wfile.write(png)
            elif path in ("/stream", "/map/stream"):
                # multipart/x-mixed-replace：浏览器把它当视频播，天然免轮询
                src = sources[path]
                self.send_response(200)
                self.send_header("Content-Type",
                                 "multipart/x-mixed-replace; boundary=frame")
                self.end_headers()
                last = -1
                src.watch(+1)
                try:
                    while True:
                        png, n, _ = src.get()
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
                finally:
                    src.watch(-1)
            else:
                self.send_error(404)

    return Handler


def main(argv=None):
    layout.bootstrap()      # 先定布局，再建节点
    rclpy.init(args=argv)
    node = LiveView()
    srv = ThreadingHTTPServer((node.host, node.port),
                              _handler_for(node.frame, node.map_frame, node.map_size,
                                           ensure=node.ensure))
    srv.daemon_threads = True
    threading.Thread(target=srv.serve_forever, daemon=True).start()
    node.get_logger().info(f"打开 http://{node.host}:{node.port} 查看实时看板")
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        srv.shutdown()
        try:
            node.destroy_node()
        except Exception:                                     # noqa: BLE001
            pass
        if rclpy.ok():
            rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
