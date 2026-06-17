#!/usr/bin/env python3
# Pi5에서 2대 USB 카메라의 MJPEG 프레임을 재인코딩 없이 NUC로 전송하는 서버
# [2026-06-15 작성] GStreamer 제거 버전 — Pi5는 캡처+전송만, 인코딩/모니터링은 NUC 담당
#                   camera_server.py + camera_server_top.py 를 하나로 통합
# [2026-06-15 v2] 카메라 교체 반영: front=KINGSEN(/dev/video1), top=USB Camera(/dev/video2)
#                 둘 다 MJPG 지원 → 양쪽 mjpg 모드. --front-mode/--top-mode 인자 추가.
"""
LeKiwi Camera Server (Pi5) — GStreamer 미사용
=============================================
2대 USB 카메라의 MJPEG 프레임을 그대로(재인코딩 없이) NUC로 TCP 전송합니다.

설계 개념
--------
  [Pi5]  USB캠 MJPEG 캡처 → 그대로 TCP 전송 (CPU 부하 최소, H.264 인코딩 안 함)
   │
   ↓ TCP (포트별로 카메라 구분)
  [NUC]  MJPEG 수신 → 디코딩/모니터링/저장/추론  (무거운 작업은 NUC가 담당)

카메라 역할
  /dev/video0 (front/bottom) → ACT 데이터 수집용 (바닥 시점)   → TCP 8000
  /dev/video2 (top)          → 장애물 예측용 (전방 시점)        → TCP 8001

왜 GStreamer를 뺐나
  - 기존: Pi5에서 x264enc 로 H.264 인코딩 → Pi5 CPU 부하 큼
  - 변경: Pi5는 카메라 네이티브 MJPEG 프레임을 그대로 전달만 함
          (USB캠이 하드웨어로 이미 JPEG 압축해서 내보내므로 추가 인코딩 불필요)
  - 무거운 디코딩/H.264 변환/모니터링은 NUC(camera_monitor_nuc_v2.py)가 처리

실행 (Pi5)
  python3 camera_server_pi5_v2.py
  python3 camera_server_pi5_v2.py --front /dev/video1 --top /dev/video2

NUC에서 모니터링
  python3 camera_monitor_nuc_v2.py
  → 브라우저: http://192.168.50.237:8080
"""

import argparse
import os
import signal
import socket
import struct
import sys
import threading
import time
import logging

import cv2

logging.basicConfig(
    level=logging.INFO,
    format="[%(asctime)s] %(levelname)s %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger(__name__)

# ── 설정 상수 ────────────────────────────────────────────
PI5_IP        = "192.168.50.111"   # 이 Pi5의 IP (서버가 바인딩할 주소)
NUC_IP        = "192.168.50.237"   # NUC IP (참고용 — 실제 연결은 NUC가 접속해 옴)

FRONT_DEVICE  = "/dev/video1"      # front = KINGSEN CAM (ACT 데이터 수집)
TOP_DEVICE    = "/dev/video2"      # top   = USB Camera  (장애물 예측)

FRONT_PORT    = 8000               # front 카메라 TCP 포트
TOP_PORT      = 8001               # top 카메라 TCP 포트

WIDTH         = 640
HEIGHT        = 480
FRAMERATE     = 15                 # FPS (Pi5 부하 고려)
JPEG_QUALITY  = 80                 # JPEG 인코딩 품질

# ── 카메라별 캡처 포맷 (v4l2-ctl --list-formats-ext 로 확인) ──────────
#   둘 다 MJPG 지원 → "mjpg" 권장 (YUYV보다 USB 대역폭 적게 차지)
#
# CAP_MODE:
#   "mjpg"      → MJPG FOURCC로 캡처 (USB 대역폭 절약, 권장)
#   "yuyv_jpeg" → YUYV(raw)로 캡처 (MJPG 미지원 카메라용 폴백)
#
#   ※ 두 모드 모두 OpenCV가 디코딩 후 JPEG로 재인코딩해 NUC로 전송.
#     640x480 JPEG 인코딩은 가벼워 Pi5 부하 작음 (H.264 인코딩 없음).
FRONT_CAP_MODE = "mjpg"            # KINGSEN: MJPG 지원
TOP_CAP_MODE   = "mjpg"            # USB Camera: MJPG 지원

# 프레임 전송 프로토콜:  [4바이트 길이(big-endian)] + [JPEG 바이트]


def open_camera(device: str, cap_mode: str) -> cv2.VideoCapture:
    """USB 카메라를 캡처 모드에 맞게 연다.

    - "mjpg"      : MJPG FOURCC 강제. 카메라가 JPEG로 압축한 프레임을 받음
                    → Pi5에서 디코딩/인코딩 불필요 (패스스루, 부하 거의 0)
    - "yuyv_jpeg" : YUYV(raw) 캡처. oCam처럼 MJPG 미지원 카메라용.
                    → 프레임을 Pi5에서 JPEG로 압축해 전송 (가벼운 부하)

    Args:
        device:   카메라 장치 경로 (/dev/video0 등)
        cap_mode: "mjpg" 또는 "yuyv_jpeg"

    Returns:
        열린 VideoCapture 객체 (실패 시 isOpened()==False)
    """
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if cap_mode == "mjpg":
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    else:  # yuyv_jpeg
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"YUYV"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    cap.set(cv2.CAP_PROP_FPS,          FRAMERATE)
    cap.set(cv2.CAP_PROP_BUFFERSIZE,   1)   # 최신 프레임 우선 (지연 최소화)
    return cap


def grab_jpeg(cap: cv2.VideoCapture, cap_mode: str) -> bytes | None:
    """카메라에서 한 프레임을 읽어 JPEG 바이트로 반환한다.

    OpenCV cap.read()는 MJPG/YUYV 모두 디코딩한 BGR을 주므로,
    여기서 JPEG로 (재)인코딩한다. 640×480 JPEG 인코딩은 매우 가벼워
    H.264 인코딩과 비교할 수 없을 만큼 부하가 작다.

    Args:
        cap:      VideoCapture
        cap_mode: 캡처 모드 (로그/참고용)

    Returns:
        JPEG 바이트 (실패 시 None)
    """
    ok, frame = cap.read()
    if not ok:
        return None
    ok, jpg = cv2.imencode(".jpg", frame, [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY])
    if not ok:
        return None
    return jpg.tobytes()


def serve_camera(name: str, device: str, port: int, cap_mode: str,
                 stop_event: threading.Event) -> None:
    """단일 카메라용 TCP 서버. NUC가 접속하면 JPEG 프레임을 계속 보낸다.

    front(oCam)/top(USB Cam) 모두 OpenCV로 캡처 후 JPEG 인코딩해 전송한다.
    cap_mode는 캡처 FOURCC를 결정한다 ("mjpg" / "yuyv_jpeg").

    한 번에 한 클라이언트(NUC)만 받는다. NUC 연결이 끊기면 다음 접속을 기다린다.

    Args:
        name: 카메라 이름 (로그용)
        device: 장치 경로
        port: 바인딩할 TCP 포트
        cap_mode: "mjpg" 또는 "yuyv_jpeg"
        stop_event: 종료 신호 이벤트
    """
    srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    srv.bind(("0.0.0.0", port))
    srv.listen(1)
    srv.settimeout(1.0)   # accept 타임아웃 → stop_event 주기적 확인
    log.info(f"[{name}] TCP 서버 대기: 0.0.0.0:{port}  (장치 {device}, mode={cap_mode})")

    frame_interval = 1.0 / FRAMERATE

    while not stop_event.is_set():
        # ── NUC 접속 대기 ──
        try:
            conn, addr = srv.accept()
        except socket.timeout:
            continue
        except OSError:
            break

        log.info(f"[{name}] NUC 연결됨: {addr[0]}:{addr[1]}")
        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

        # ── 카메라 열기 ──
        cap = open_camera(device, cap_mode)
        if not cap.isOpened():
            log.error(f"[{name}] 카메라 열기 실패: {device}")
            cap.release()
            conn.close()
            stop_event.wait(2.0)
            continue

        try:
            while not stop_event.is_set():
                t0 = time.time()

                data = grab_jpeg(cap, cap_mode)
                if data is None:
                    log.warning(f"[{name}] 프레임 읽기 실패 — 재시도")
                    time.sleep(0.1)
                    continue

                # ── 전송 ──
                try:
                    conn.sendall(struct.pack(">I", len(data)) + data)
                except (BrokenPipeError, ConnectionResetError, OSError):
                    log.info(f"[{name}] NUC 연결 끊김 — 재접속 대기")
                    break

                # FPS 제한
                dt = time.time() - t0
                if dt < frame_interval:
                    time.sleep(frame_interval - dt)
        finally:
            cap.release()
            conn.close()

    srv.close()
    log.info(f"[{name}] 서버 종료")


def check_device(device: str) -> bool:
    """카메라 장치 존재 여부를 확인한다."""
    return os.path.exists(device)


def main() -> None:
    parser = argparse.ArgumentParser(description="LeKiwi Camera Server (Pi5, GStreamer 미사용)")
    parser.add_argument("--front", default=FRONT_DEVICE, help="Front 카메라 장치 (기본 KINGSEN /dev/video1)")
    parser.add_argument("--top",   default=TOP_DEVICE,   help="Top 카메라 장치 (기본 USB Camera /dev/video2)")
    parser.add_argument("--front-port", type=int, default=FRONT_PORT, help="Front TCP 포트")
    parser.add_argument("--top-port",   type=int, default=TOP_PORT,   help="Top TCP 포트")
    parser.add_argument("--front-mode", default=FRONT_CAP_MODE, choices=["mjpg", "yuyv_jpeg"],
                        help="Front 캡처 포맷 (기본 mjpg)")
    parser.add_argument("--top-mode",   default=TOP_CAP_MODE,   choices=["mjpg", "yuyv_jpeg"],
                        help="Top 캡처 포맷 (기본 mjpg)")
    args = parser.parse_args()

    # ── 사전 확인 ──
    for dev, name in [(args.front, "FRONT"), (args.top, "TOP")]:
        if not check_device(dev):
            log.error(f"❌ {name} 카메라 장치 없음: {dev}")
            log.error("   ls /dev/video*  또는  v4l2-ctl --list-devices 로 확인")
            sys.exit(1)

    stop_event = threading.Event()

    def handle_signal(sig, frame):
        log.info("종료 신호 수신...")
        stop_event.set()

    signal.signal(signal.SIGINT,  handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    threads = [
        threading.Thread(target=serve_camera,
                         args=("FRONT", args.front, args.front_port,
                               args.front_mode, stop_event),
                         daemon=True),
        threading.Thread(target=serve_camera,
                         args=("TOP", args.top, args.top_port,
                               args.top_mode, stop_event),
                         daemon=True),
    ]
    for t in threads:
        t.start()

    print(f"\n{'='*60}")
    print(f"  📹 LeKiwi Camera Server (Pi5) — GStreamer 미사용")
    print(f"  Pi5 IP : {PI5_IP}")
    print(f"  Front  : {args.front:14s} → TCP {args.front_port}  (KINGSEN/{args.front_mode})")
    print(f"  Top    : {args.top:14s} → TCP {args.top_port}  (USB Cam/{args.top_mode})")
    print(f"")
    print(f"  ℹ️  양쪽 다 OpenCV 캡처 + JPEG 인코딩으로 전송.")
    print(f"      640x480 JPEG라 부하 가벼움. H.264 인코딩 없음.")
    print(f"")
    print(f"  NUC에서 모니터링:")
    print(f"    python3 camera_monitor_nuc_v2.py")
    print(f"    → 브라우저: http://{NUC_IP}:8080")
    print(f"{'='*60}")
    print(f"  종료: Ctrl+C\n")

    stop_event.wait()
    log.info("모든 카메라 서버 종료 완료")


if __name__ == "__main__":
    main()
