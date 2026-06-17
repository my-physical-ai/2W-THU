#!/usr/bin/env python3
# NUC에서 Pi5의 2대 카메라 MJPEG 스트림을 수신하여 디코딩/모니터링하는 서버
# [2026-06-15 작성] Pi5(camera_server_pi5_v2.py)와 짝을 이루는 NUC측 수신/모니터링
# [2026-06-15 v2] 카메라 교체(KINGSEN+USB Camera) 대응 — 짝 파일 camera_server_pi5_v2.py
"""
LeKiwi Camera Monitor (NUC)
===========================
Pi5의 camera_server_pi5_v2.py 에 접속하여 2대 카메라 MJPEG 프레임을 수신하고,
웹 브라우저로 모니터링할 수 있게 MJPEG-over-HTTP 로 다시 내보냅니다.

흐름
----
  [Pi5] TCP 8000(front), 8001(top) 으로 MJPEG 송출
   │
   ↓ NUC가 접속해서 수신
  [NUC] 수신 → (필요시 디코딩/추론/저장) → HTTP 8080 으로 브라우저에 표시

실행 (NUC)
  python3 camera_monitor_nuc_v2.py
  python3 camera_monitor_nuc_v2.py --pi5 192.168.50.111

확인
  브라우저: http://192.168.50.237:8080
"""

import argparse
import socket
import struct
import threading
import time
import logging
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

# ── 설정 상수 ────────────────────────────────────────────
PI5_IP     = "192.168.50.111"   # Pi5 IP (접속 대상)
NUC_IP     = "192.168.50.237"   # NUC IP (이 모니터가 바인딩할 주소)

FRONT_PORT = 8000               # Pi5 front 카메라 포트
TOP_PORT   = 8001               # Pi5 top 카메라 포트
HTTP_PORT  = 8080               # 브라우저 모니터링 HTTP 포트

RECONNECT_S = 3.0               # Pi5 재접속 대기

# 카메라별 최신 JPEG 프레임 보관 (브라우저 요청 시 여기서 꺼내 보냄)
_latest = {
    "front": {"jpg": None, "lock": threading.Lock(), "ts": 0.0},
    "top":   {"jpg": None, "lock": threading.Lock(), "ts": 0.0},
}


def recv_exact(sock: socket.socket, n: int) -> bytes | None:
    """소켓에서 정확히 n바이트를 읽는다. 연결이 끊기면 None."""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def receiver_loop(name: str, pi5_ip: str, port: int, stop_event: threading.Event) -> None:
    """Pi5의 한 카메라 포트에 접속해 MJPEG 프레임을 계속 수신한다.

    프로토콜: [4바이트 길이(big-endian)] + [JPEG 바이트]

    Args:
        name: "front" / "top"
        pi5_ip: Pi5 IP
        port: 접속할 TCP 포트
        stop_event: 종료 신호
    """
    while not stop_event.is_set():
        try:
            log.info(f"[{name}] Pi5 접속 시도: {pi5_ip}:{port}")
            sock = socket.create_connection((pi5_ip, port), timeout=5.0)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            log.info(f"[{name}] Pi5 연결 성공")
        except OSError as e:
            log.warning(f"[{name}] 연결 실패: {e} — {RECONNECT_S}초 후 재시도")
            stop_event.wait(RECONNECT_S)
            continue

        try:
            while not stop_event.is_set():
                header = recv_exact(sock, 4)
                if header is None:
                    break
                (length,) = struct.unpack(">I", header)
                data = recv_exact(sock, length)
                if data is None:
                    break

                # 최신 프레임 갱신 (브라우저가 여기서 꺼내감)
                slot = _latest[name]
                with slot["lock"]:
                    slot["jpg"] = data
                    slot["ts"] = time.time()

                # ── (선택) 여기서 NUC측 무거운 처리 가능 ──
                # 예: cv2.imdecode 후 YOLO 추론, 녹화 저장 등
                #   import cv2, numpy as np
                #   frame = cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)
        except OSError as e:
            log.warning(f"[{name}] 수신 오류: {e}")
        finally:
            sock.close()

        if not stop_event.is_set():
            log.info(f"[{name}] 연결 끊김 — {RECONNECT_S}초 후 재접속")
            stop_event.wait(RECONNECT_S)

    log.info(f"[{name}] 수신 종료")


# ── 브라우저 모니터링용 HTTP 핸들러 ──────────────────────
INDEX_HTML = """<!DOCTYPE html>
<html lang="ko"><head><meta charset="utf-8">
<title>LeKiwi Camera Monitor (NUC)</title>
<style>
  body{margin:0;background:#111;color:#eee;font-family:system-ui,sans-serif}
  h1{font-size:18px;padding:12px 16px;margin:0;border-bottom:1px solid #333}
  .grid{display:flex;flex-wrap:wrap;gap:16px;padding:16px}
  .cam{background:#1b1b1b;border:1px solid #333;border-radius:8px;padding:10px}
  .cam h2{font-size:14px;margin:0 0 8px;color:#7fd}
  img{display:block;width:640px;max-width:90vw;background:#000;border-radius:4px}
</style></head>
<body>
  <h1>📹 LeKiwi Camera Monitor — NUC</h1>
  <div class="grid">
    <div class="cam"><h2>FRONT / BOTTOM (ACT)</h2><img src="/stream/front"></div>
    <div class="cam"><h2>TOP (장애물 예측)</h2><img src="/stream/top"></div>
  </div>
</body></html>"""


class MonitorHandler(BaseHTTPRequestHandler):
    def log_message(self, *args):
        pass  # 액세스 로그 끔

    def do_GET(self):
        if self.path == "/" or self.path == "/index.html":
            body = INDEX_HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        if self.path.startswith("/stream/"):
            name = self.path.rsplit("/", 1)[-1]
            if name not in _latest:
                self.send_error(404)
                return
            self._serve_mjpeg(name)
            return

        self.send_error(404)

    def _serve_mjpeg(self, name: str):
        """multipart/x-mixed-replace 로 최신 프레임을 계속 밀어준다 (MJPEG 스트림)."""
        self.send_response(200)
        self.send_header("Content-Type",
                         "multipart/x-mixed-replace; boundary=frame")
        self.send_header("Cache-Control", "no-cache")
        self.end_headers()
        slot = _latest[name]
        try:
            while True:
                with slot["lock"]:
                    jpg = slot["jpg"]
                if jpg is None:
                    time.sleep(0.05)
                    continue
                self.wfile.write(b"--frame\r\n")
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode())
                self.wfile.write(jpg)
                self.wfile.write(b"\r\n")
                time.sleep(1.0 / 15)
        except (BrokenPipeError, ConnectionResetError):
            pass


def main() -> None:
    parser = argparse.ArgumentParser(description="LeKiwi Camera Monitor (NUC)")
    parser.add_argument("--pi5", default=PI5_IP, help="Pi5 IP 주소")
    parser.add_argument("--front-port", type=int, default=FRONT_PORT)
    parser.add_argument("--top-port",   type=int, default=TOP_PORT)
    parser.add_argument("--http-port",  type=int, default=HTTP_PORT)
    args = parser.parse_args()

    stop_event = threading.Event()

    # 카메라 2대 수신 스레드 시작
    threads = [
        threading.Thread(target=receiver_loop,
                         args=("front", args.pi5, args.front_port, stop_event),
                         daemon=True),
        threading.Thread(target=receiver_loop,
                         args=("top", args.pi5, args.top_port, stop_event),
                         daemon=True),
    ]
    for t in threads:
        t.start()

    # HTTP 모니터링 서버
    httpd = ThreadingHTTPServer(("0.0.0.0", args.http_port), MonitorHandler)

    print(f"\n{'='*60}")
    print(f"  🖥️  LeKiwi Camera Monitor (NUC)")
    print(f"  Pi5 접속 : {args.pi5}:{args.front_port}(front), {args.top_port}(top)")
    print(f"  모니터링 : http://{NUC_IP}:{args.http_port}")
    print(f"{'='*60}")
    print(f"  종료: Ctrl+C\n")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        log.info("종료 신호 수신...")
    finally:
        stop_event.set()
        httpd.shutdown()
        log.info("camera_monitor_nuc_v2.py 종료 완료")


if __name__ == "__main__":
    main()
