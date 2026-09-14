#!/usr/bin/env python3
"""从实时看板的多段流里抓**一帧** PNG。

看板由 `web:=true` 提供：`/` 是页面外壳，实时画面在 `/stream`（视频）与
`/map/stream`（地图）。这两个端点发的是 `multipart/x-mixed-replace`，
分片头是 `--frame` + `Content-Length`，**载荷是 PNG 不是 JPEG**
（第一次写这个工具时按 MJPEG 的 FFD8/FFD9 去切，结果 `/stream` 一直抓失败，
而 `/map/stream` 偶尔"成功" —— 那只是 PNG 里恰好出现了 FFD9 字节，
切出来的文件其实不是合法图片）。这里改成严格按分片头和 Content-Length 解析。

用法: python3 tools/grab_board.py <输出路径> [url]
"""
from __future__ import annotations

import sys
import time
import urllib.request

BOUNDARY = b"--frame"
HDR_END = b"\r\n\r\n"
PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


def _extract(buf: bytes) -> bytes | None:
    """从累积缓冲里切出一帧完整 PNG（分片头 -> Content-Length -> 载荷）。"""
    b = buf.find(BOUNDARY)
    if b < 0:
        return None
    h = buf.find(HDR_END, b)
    if h < 0:
        return None
    head = buf[b:h].decode("latin-1", "replace")
    n = None
    for line in head.split("\r\n"):
        if line.lower().startswith("content-length:"):
            try:
                n = int(line.split(":", 1)[1].strip())
            except ValueError:
                return None
    if n is None:
        return None
    start = h + len(HDR_END)
    if len(buf) < start + n:
        return None
    body = buf[start:start + n]
    return body if body.startswith(PNG_MAGIC) else None


def grab(url: str, out: str, budget_s: float = 15.0) -> int:
    buf = b""
    t0 = time.time()
    while time.time() - t0 < budget_s:
        try:
            with urllib.request.urlopen(url, timeout=8) as r:
                for _ in range(40):           # 读够一帧就停，别把流读干
                    chunk = r.read(32768)
                    if not chunk:
                        break
                    buf += chunk
                    body = _extract(buf)
                    if body is not None:
                        with open(out, "wb") as f:
                            f.write(body)
                        return len(body)
                    if len(buf) > 12_000_000:
                        buf = buf[-2_000_000:]
        except Exception as exc:              # noqa: BLE001
            print("retry:", exc, file=sys.stderr)
            time.sleep(0.5)
    return 0


def main() -> int:
    out = sys.argv[1] if len(sys.argv) > 1 else "/tmp/board.jpg"
    url = sys.argv[2] if len(sys.argv) > 2 else "http://127.0.0.1:8080/stream"
    n = grab(url, out)
    print(f"{out}: {n} bytes" if n else "抓取失败")
    return 0 if n else 1


if __name__ == "__main__":
    sys.exit(main())
