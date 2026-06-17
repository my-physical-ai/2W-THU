#!/usr/bin/env python3
# NUC 통합 모니터링 백엔드 — 카메라 소스(Pi5 웹캠2 + NUC직결 IP카메라1)를 모아 MJPEG/API로 제공하는 Flask 서버
# [2026-06-17 작성] 백엔드/프론트 분리판. 화면(UI)은 templates/ 의 HTML이 담당 → 팀이 자유롭게 디자인
#                   라즈베리파이쪽(camera_server_pi5_v2.py)·기존 NUC 수신은 손대지 않음. 이 파일은 NUC에서만 실행.
"""
NUC 통합 모니터링 백엔드 (공통)
================================
이 서버는 "데이터만" 책임진다. 화면 디자인은 templates/ 의 HTML 파일이 한다.
그래서 팀은 백엔드를 건드리지 않고 templates 의 HTML만 고쳐 UI를 자유롭게 꾸밀 수 있다.

기존 구성(손대지 않음)
  [Pi5 192.168.50.111] 웹캠 2개 → TCP 8000(front)/8001(top)   (camera_server_pi5_v2.py 그대로)
  [IP 카메라 192.168.50.123]    → NUC에 직결, RTSP
  ↓ 이 백엔드가 위 소스들을 받아 한 곳에서 MJPEG/JSON 으로 제공
  [NUC Flask] http://<NUC IP>:8080  → PC/폰 브라우저

엔드포인트
  GET /                 → templates/index.html (템플릿 고르는 시작 페이지)
  GET /control          → templates/template_control.html (관제센터형)
  GET /cards            → templates/template_cards.html  (카드형)
  GET /api/cameras      → 카메라 목록(JSON) — 템플릿 JS가 화면 구성에 사용
  GET /stream/<name>    → 카메라별 MJPEG 스트림 (종류 무관 동일 방식)
  GET /snapshot/<name>  → 최신 프레임 1장 저장 (기본 제공)

실행 (NUC)
  pip install flask opencv-python
  python3 nuc_monitor_server.py --pi5 192.168.50.111

확인
  PC/폰 브라우저:  http://<NUC IP>:8080
"""

import argparse
import os
import socket
import struct
import threading
import time
import logging

import cv2
from flask import Flask, Response, jsonify, render_template

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

# ── 설정 ─────────────────────────────────────────────────
PI5_IP       = "192.168.50.111"   # Pi5 IP (폐쇄망, 웹캠 송출)
HTTP_PORT    = 8080               # 브라우저 접속 포트
RECONNECT_S  = 3.0                # 끊겼을 때 재접속 대기(초)
JPEG_QUALITY = 80                 # RTSP 재인코딩 품질
FPS_LIMIT    = 15                 # 브라우저로 내보내는 최대 FPS
SNAPSHOT_DIR = "snapshots"        # 스냅샷 저장 폴더

# ── 카메라 목록 (소스 정의 — 화면과 무관) ────────────────
#   kind="pi5"  : Pi5가 TCP로 보내는 웹캠 (port=TCP 포트)   — Pi5 192.168.50.111 (폐쇄망)
#   kind="rtsp" : NUC에 직결된 IP 카메라 (url=rtsp 주소)    — IP캠 192.168.50.123
CAMERAS = [
    {"name": "front", "label": "Pi5 웹캠 — FRONT/BOTTOM (ACT)", "kind": "pi5", "port": 8000},
    {"name": "top",   "label": "Pi5 웹캠 — TOP (장애물 예측)",   "kind": "pi5", "port": 8001},
    {"name": "ipcam1", "label": "IP 카메라 (NUC 직결)",
     "kind": "rtsp", "url": "rtsp://admin:123456@192.168.50.123:554/Streaming/Channels/101"},
    # IP 카메라를 더 붙이려면 위 형식으로 줄 추가
]

# 카메라별 최신 JPEG 프레임 보관소
_frames = {
    cam["name"]: {"jpg": None, "lock": threading.Lock(), "ts": 0.0}
    for cam in CAMERAS
}
_stop = threading.Event()


# ── 공통 유틸 ────────────────────────────────────────────
def recv_exact(sock: socket.socket, n: int) -> bytes | None:
    """소켓에서 정확히 n바이트를 읽는다. 끊기면 None."""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            return None
        buf.extend(chunk)
    return bytes(buf)


def store_frame(name: str, jpg: bytes) -> None:
    """최신 프레임을 보관소에 저장한다."""
    slot = _frames[name]
    with slot["lock"]:
        slot["jpg"] = jpg
        slot["ts"] = time.time()


def get_frame(name: str) -> bytes | None:
    """보관소에서 최신 프레임을 꺼낸다."""
    slot = _frames[name]
    with slot["lock"]:
        return slot["jpg"]


# ── 소스 수신 1: Pi5 웹캠 (TCP, 이미 JPEG) ───────────────
def pi5_receiver(name: str, pi5_ip: str, port: int) -> None:
    """Pi5의 한 웹캠 포트에 접속해 [길이4+JPEG] 프레임을 계속 받는다."""
    while not _stop.is_set():
        try:
            log.info(f"[{name}] Pi5 접속 시도 {pi5_ip}:{port}")
            sock = socket.create_connection((pi5_ip, port), timeout=5.0)
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
            log.info(f"[{name}] Pi5 연결 성공")
        except OSError as e:
            log.warning(f"[{name}] 연결 실패: {e} — {RECONNECT_S}s 후 재시도")
            _stop.wait(RECONNECT_S)
            continue

        try:
            while not _stop.is_set():
                header = recv_exact(sock, 4)
                if header is None:
                    break
                (length,) = struct.unpack(">I", header)
                data = recv_exact(sock, length)
                if data is None:
                    break
                store_frame(name, data)
        except OSError as e:
            log.warning(f"[{name}] 수신 오류: {e}")
        finally:
            sock.close()

        if not _stop.is_set():
            log.info(f"[{name}] 연결 끊김 — {RECONNECT_S}s 후 재접속")
            _stop.wait(RECONNECT_S)
    log.info(f"[{name}] 수신 종료")


# ── 소스 수신 2: IP 카메라 (RTSP → OpenCV → JPEG) ────────
def rtsp_receiver(name: str, url: str) -> None:
    """RTSP 스트림을 OpenCV로 받아 JPEG로 인코딩해 보관한다(TCP 강제)."""
    os.environ["OPENCV_FFMPEG_CAPTURE_OPTIONS"] = "rtsp_transport;tcp"
    interval = 1.0 / FPS_LIMIT

    while not _stop.is_set():
        log.info(f"[{name}] RTSP 접속 시도 {url}")
        cap = cv2.VideoCapture(url, cv2.CAP_FFMPEG)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not cap.isOpened():
            log.warning(f"[{name}] RTSP 열기 실패 — {RECONNECT_S}s 후 재시도")
            cap.release()
            _stop.wait(RECONNECT_S)
            continue
        log.info(f"[{name}] RTSP 연결 성공")

        try:
            while not _stop.is_set():
                t0 = time.time()
                ok, frame = cap.read()
                if not ok:
                    log.warning(f"[{name}] 프레임 수신 실패 — 재접속")
                    break
                ok, jpg = cv2.imencode(
                    ".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
                if ok:
                    store_frame(name, jpg.tobytes())
                dt = time.time() - t0
                if dt < interval:
                    time.sleep(interval - dt)
        finally:
            cap.release()

        if not _stop.is_set():
            _stop.wait(RECONNECT_S)
    log.info(f"[{name}] RTSP 종료")


# ── Flask 앱 ─────────────────────────────────────────────
# templates/ 폴더의 HTML을 화면으로 사용 (백엔드와 분리)
app = Flask(__name__, template_folder="templates")


@app.route("/")
def index():
    """시작 페이지 — 두 템플릿(관제센터형/카드형) 중 고르게 안내."""
    return render_template("index.html", cameras=_public_cameras())


@app.route("/control")
def template_control():
    """관제센터형 템플릿 (큰 메인 + 사이드 썸네일)."""
    return render_template("template_control.html")


@app.route("/cards")
def template_cards():
    """카드형 템플릿 (동등한 카드 그리드)."""
    return render_template("template_cards.html")


def _public_cameras():
    return [
        {"name": c["name"], "label": c["label"], "kind": c["kind"]}
        for c in CAMERAS
    ]


@app.route("/api/cameras")
def api_cameras():
    """카메라 목록(JSON). 템플릿 JS가 화면 구성에 사용한다."""
    return jsonify(_public_cameras())


@app.route("/stream/<name>")
def stream(name):
    """카메라 종류와 무관하게 동일한 MJPEG 스트림으로 내보낸다."""
    if name not in _frames:
        return "unknown camera", 404

    def gen():
        interval = 1.0 / FPS_LIMIT
        while True:
            jpg = get_frame(name)
            if jpg is None:
                time.sleep(0.05)
                continue
            yield (b"--frame\r\n"
                   b"Content-Type: image/jpeg\r\n"
                   b"Content-Length: " + str(len(jpg)).encode() + b"\r\n\r\n"
                   + jpg + b"\r\n")
            time.sleep(interval)

    return Response(gen(),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/snapshot/<name>")
def snapshot(name):
    """현재 카메라의 최신 프레임 1장을 NUC에 파일로 저장한다(기본 제공)."""
    if name not in _frames:
        return jsonify({"ok": False, "error": "unknown camera"}), 404
    jpg = get_frame(name)
    if jpg is None:
        return jsonify({"ok": False, "error": "아직 영상이 없습니다"}), 503
    os.makedirs(SNAPSHOT_DIR, exist_ok=True)
    fname = f"{name}_{time.strftime('%Y%m%d_%H%M%S')}.jpg"
    path = os.path.join(SNAPSHOT_DIR, fname)
    with open(path, "wb") as f:
        f.write(jpg)
    log.info(f"[{name}] 스냅샷 저장: {path}")
    return jsonify({"ok": True, "path": path})


def main() -> None:
    parser = argparse.ArgumentParser(description="NUC 통합 모니터링 백엔드 (공통)")
    parser.add_argument("--pi5", default=PI5_IP, help="Pi5 IP 주소")
    parser.add_argument("--http-port", type=int, default=HTTP_PORT)
    args = parser.parse_args()

    # 소스별 수신 스레드 시작
    for cam in CAMERAS:
        if cam["kind"] == "pi5":
            threading.Thread(target=pi5_receiver,
                             args=(cam["name"], args.pi5, cam["port"]),
                             daemon=True).start()
        else:  # rtsp
            threading.Thread(target=rtsp_receiver,
                             args=(cam["name"], cam["url"]),
                             daemon=True).start()

    # 내 IP 안내 (폰에서 접속할 주소)
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        my_ip = s.getsockname()[0]
        s.close()
    except OSError:
        my_ip = "<NUC IP>"

    rtsp_cams = [c for c in CAMERAS if c["kind"] == "rtsp"]
    print(f"\n{'='*60}")
    print(f"  🖥️  NUC 통합 모니터링 백엔드 (공통)")
    print(f"  Pi5 웹캠 2개 : {args.pi5}:8000(front) / 8001(top)  [폐쇄망]")
    for c in rtsp_cams:
        print(f"  IP 카메라    : {c['url']}  [NUC 직결]")
    print(f"")
    print(f"  📱 폰/PC 접속:  http://{my_ip}:{args.http_port}")
    print(f"     - 시작/선택 :  /")
    print(f"     - 관제센터형 :  /control")
    print(f"     - 카드형     :  /cards")
    print(f"{'='*60}")
    print(f"  종료: Ctrl+C\n")

    try:
        app.run(host="0.0.0.0", port=args.http_port, threaded=True)
    except KeyboardInterrupt:
        pass
    finally:
        _stop.set()
        log.info("nuc_monitor_server.py 종료 완료")


if __name__ == "__main__":
    main()
