#!/usr/bin/env python3
# check_wheels.py - LeKiwi 바퀴 모터(ID 7,8,9) 진단 및 작동 테스트 스크립트
# 2026-02-16 최초 작성
# 2026-02-16 수정1: 위치 읽기 제거, --port 옵션 제거, 이동 테스트 추가
# 2026-02-16 수정2: 이동 순서 변경(좌/우 먼저), 제자리 회전 추가
# 2026-02-16 수정3: 책상 테스트 제약 반영(저속/짧은시간), 옵션 3개 분리(--spin/--move/--turn)
# 2026-02-16 수정4: --10m 옵션 추가 (5m 전진 + 5m 후진 + 5m 회전, 시간 기반 거리 계산)
# 2026-02-16 수정5: --3m 으로 변경 (3m 전/후/회전), 책상 위 바퀴 띄움 방식 반영

"""
LeKiwi 단일 보드 구성에서 바퀴 모터(ID 7,8,9)를 진단하고 작동을 테스트합니다.

★ 책상 위 테스트 제약 ★
  로봇이 PC에 케이블로 연결된 채 책상에서 테스트하는 환경을 고려합니다.
  - 로봇이 책상에서 떨어지지 않도록, 바퀴를 공중에 띄운 상태(받침 위)에서 진행 권장
  - 짧은 동작은 1초, 속도가 낮습니다(저속)
  - 3m 주행(--3m)도 바퀴를 받침에 올려 띄운 상태로 진행 가능

확인 항목:
  1. 포트 존재 여부 (/dev/ttyACM0)
  2. 모터 버스 연결
  3. 바퀴 모터 ID(7,8,9) ping 응답
  4. 각 모터 전압/온도 확인
  5. 작동 테스트 (옵션 선택)

테스트 옵션 (4개):
  --spin   각 바퀴를 하나씩 개별 회전 (책상에서 가장 안전, 개별 작동 확인)
  --move   전후좌우 이동 (짧게, 바퀴를 띄운 상태 권장)
  --turn   제자리 회전 (좌회전/우회전, 짧게)
  --3m     3m 전진 + 3m 후진 + 3m 회전 (시간 기반, 책상 위 바퀴 띄운 상태 가능)

사용법:
  python3 check_wheels.py                 # 진단만 (모터 회전 X)
  python3 check_wheels.py --spin
  python3 check_wheels.py --move
  python3 check_wheels.py --turn
  python3 check_wheels.py --3m             # 3m 전/후진 + 3m 회전
"""

import argparse
import os
import sys
import time
import math

# 고정 포트 (단일 보드 구성에서는 항상 /dev/ttyACM0)
PORT = "/dev/ttyACM0"

# LeKiwi 바퀴 모터 ID 정의 (단일 보드 기준)
WHEEL_MOTORS = {
    7: "left_front (좌전)",
    8: "rear (후)",
    9: "right_front (우전)",
}

# 옴니휠 배치 각도 (LeKiwi 기준, 로봇 전방 = +X)
WHEEL_ANGLES = {
    7: 300,   # 좌전 바퀴
    8: 180,   # 후방 바퀴
    9: 60,    # 우전 바퀴
}

# ── 책상 테스트 제약: 저속 + 짧은 시간 ──
MOVE_SPEED = 400       # 짧은 이동 속도 (저속)
SPIN_SPEED = 350       # 개별/제자리 회전 속도 (저속)
ACTION_DURATION = 1.0  # 짧은 동작 지속 시간(초)

# ─────────────────────────────────────────────────────────
#  거리 주행(--3m) 파라미터 (시간 기반 + 실측 보정)
# ─────────────────────────────────────────────────────────
# 목표 거리 (m)
TARGET_DISTANCE_M = 3.0
# 주행에 사용할 raw 속도값 (전/후진, 회전 공통)
LONG_MOVE_RAW = 1500
LONG_SPIN_RAW = 1500
#
# ★★★ 가장 중요: 실측 속도 (m/s) ★★★
#   모터 raw값 → 실제 속도 변환은 펌웨어마다 달라 추정이 부정확합니다.
#   따라서 "직접 1번 측정"해서 이 값을 넣는 것이 정확합니다.
#
#   [책상 위 측정 방법 - 바퀴 회전수 활용]
#     바퀴를 띄운 상태라 실제 이동은 없으므로, 바퀴 회전수로 환산합니다.
#     1) 아래 MEASURED_SPEED_MPS 를 일단 0.3 정도로 두고 --3m 실행
#     2) 바퀴에 표시(테이프)를 붙이고, 동작 동안 몇 바퀴 도는지 센다
#     3) 굴러간 거리 = 회전수 × 바퀴둘레(0.314m, 지름 100mm 기준)
#     4) 굴러간거리 / 목표거리(3m) 비율로 보정:
#        새 속도 = 0.3 × (굴러간거리 ÷ 3)
#     5) 다시 실행하면 3m에 근접
#
#   기본값은 LeKiwi(100mm 옴니휠)의 대략적 속도 수준으로 보수적 설정
MEASURED_SPEED_MPS = 0.30    # ← 실측 후 이 값만 바꾸면 됩니다


def time_for_distance(distance_m):
    # 목표 거리(m) ÷ 실측 속도(m/s) = 필요 시간(초)
    if MEASURED_SPEED_MPS <= 0:
        return 0.0
    return distance_m / MEASURED_SPEED_MPS


def print_header(title):
    print("\n" + "=" * 55)
    print(f"  {title}")
    print("=" * 55)


def check_port_exists(port):
    # 1단계: 포트 존재 확인
    print_header("1단계: 포트 존재 확인")
    if os.path.exists(port):
        print(f"  ✅ 포트 발견: {port}")
        if os.access(port, os.R_OK | os.W_OK):
            print(f"  ✅ 읽기/쓰기 권한 있음")
        else:
            print(f"  ⚠️  권한 부족! 다음 명령으로 추가하세요:")
            print(f"      sudo usermod -aG dialout $USER")
            print(f"      (이후 재로그인 필요)")
        return True
    else:
        print(f"  ❌ 포트 없음: {port}")
        print(f"     USB 케이블과 12V 전원을 확인하세요.")
        return False


def connect_bus(port):
    # 2단계: 모터 버스 연결
    print_header("2단계: 모터 버스 연결")
    try:
        from lerobot.motors.feetech import FeetechMotorsBus
        from lerobot.motors import Motor, MotorNormMode
    except ImportError as e:
        print(f"  ❌ LeRobot import 실패: {e}")
        print(f"     conda activate lerobot 후 다시 시도하세요.")
        return None

    motors = {}
    for mid, name in WHEEL_MOTORS.items():
        motors[f"wheel_{mid}"] = Motor(mid, "sts3215", MotorNormMode.RANGE_M100_100)

    try:
        bus = FeetechMotorsBus(port=port, motors=motors)
        bus.connect()
        print(f"  ✅ 버스 연결 성공: {port}")
        return bus
    except Exception as e:
        print(f"  ❌ 버스 연결 실패: {e}")
        return None


def scan_motors(bus):
    # 3단계: 바퀴 모터 ID 스캔
    print_header("3단계: 바퀴 모터 ID 스캔")
    found_ids = []
    for mid in WHEEL_MOTORS:
        try:
            if bus.ping(mid) is not None:
                found_ids.append(mid)
        except Exception:
            pass

    wheel_found = [i for i in found_ids if i in WHEEL_MOTORS]
    print(f"  [바퀴 모터 7-9]  발견: {len(wheel_found)}/3")
    for mid in WHEEL_MOTORS:
        mark = "✅" if mid in found_ids else "❌"
        print(f"    {mark} ID {mid}: {WHEEL_MOTORS[mid]}")
    return found_ids


def read_wheel_status(bus, found_ids):
    # 4단계: 전압/온도 확인
    print_header("4단계: 바퀴 모터 상태 점검")
    all_ok = True
    for mid in WHEEL_MOTORS:
        name = WHEEL_MOTORS[mid]
        if mid not in found_ids:
            print(f"\n  ❌ ID {mid} ({name}): 응답 없음 - 건너뜀")
            all_ok = False
            continue
        print(f"\n  📍 ID {mid} ({name}):")
        motor_key = f"wheel_{mid}"
        try:
            voltage = bus.read("Present_Voltage", motor_key) / 10.0
            v_mark = "✅" if 10.0 <= voltage <= 14.0 else "⚠️"
            print(f"      {v_mark} 전압: {voltage:.1f}V (정상범위 11~13V)")
            if voltage < 10.0:
                print(f"         → 배터리 충전 필요 또는 전원 연결 확인")
            temp = bus.read("Present_Temperature", motor_key)
            t_mark = "✅" if temp < 50 else "⚠️"
            print(f"      {t_mark} 온도: {temp}°C (50°C 미만 권장)")
        except Exception as e:
            print(f"      ❌ 상태 읽기 실패: {e}")
            all_ok = False
    return all_ok


def compute_wheel_velocities(vx, vy, speed=MOVE_SPEED):
    # 옴니휠 운동학: vx(전후), vy(좌우) → 각 바퀴 속도
    velocities = {}
    for mid, angle_deg in WHEEL_ANGLES.items():
        rad = math.radians(angle_deg)
        v = vx * math.cos(rad) + vy * math.sin(rad)
        velocities[mid] = int(v * speed)
    return velocities


def compute_turn_velocities(direction, speed=SPIN_SPEED):
    # 제자리 회전: 모든 바퀴 같은 부호 속도
    return {mid: int(direction * speed) for mid in WHEEL_ANGLES}


def enable_wheels(bus, found_ids):
    for mid in found_ids:
        motor_key = f"wheel_{mid}"
        bus.write("Operating_Mode", motor_key, 1)
        bus.write("Torque_Enable", motor_key, 1)


def stop_wheels(bus, found_ids):
    for mid in found_ids:
        try:
            bus.write("Goal_Velocity", f"wheel_{mid}", 0)
        except Exception:
            pass


def disable_wheels(bus, found_ids):
    for mid in found_ids:
        try:
            bus.write("Goal_Velocity", f"wheel_{mid}", 0)
            bus.write("Torque_Enable", f"wheel_{mid}", 0)
        except Exception:
            pass


def desk_warning(extra=""):
    print("  ⚠️  책상 위 테스트 주의사항:")
    print("     - 로봇이 책상에서 떨어지지 않도록 바퀴를 받침 위에 올려")
    print("       공중에 띄운 상태에서 진행하는 것을 권장합니다.")
    print("     - PC 연결 케이블이 당겨지지 않도록 여유를 확보하세요.")
    if extra:
        print(f"     - {extra}")


def run_action(bus, found_ids, label, velocities, duration=ACTION_DURATION):
    # 한 동작 실행 (Enter 후, duration 초간)
    input(f"\n  ▶ [{label}] - Enter를 누르면 시작: ")
    print(f"    {label} 진행 중... ({duration:.1f}초)")
    for mid in found_ids:
        bus.write("Goal_Velocity", f"wheel_{mid}", velocities.get(mid, 0))
    time.sleep(duration)
    stop_wheels(bus, found_ids)
    print(f"    ✅ {label} 완료, 정지")
    time.sleep(0.3)


def spin_test(bus, found_ids):
    # [--spin] 각 바퀴 개별 회전
    print_header("테스트: 바퀴 개별 회전 (--spin)")
    desk_warning("바퀴 하나씩만 도므로 책상에서 가장 안전합니다.")
    input("\n  개별 회전 테스트 시작하려면 Enter (취소: Ctrl+C)... ")
    try:
        enable_wheels(bus, found_ids)
        for mid in WHEEL_MOTORS:
            if mid not in found_ids:
                print(f"\n  ⏭️  ID {mid} ({WHEEL_MOTORS[mid]}): 응답 없음 - 건너뜀")
                continue
            name = WHEEL_MOTORS[mid]
            vel = {m: 0 for m in found_ids}
            vel[mid] = SPIN_SPEED
            run_action(bus, found_ids, f"ID {mid} ({name}) 정방향 ↻", vel)
            vel[mid] = -SPIN_SPEED
            run_action(bus, found_ids, f"ID {mid} ({name}) 역방향 ↺", vel)
        print("\n  🎉 개별 회전 테스트 완료!")
    finally:
        disable_wheels(bus, found_ids)


def move_test(bus, found_ids):
    # [--move] 전후좌우 이동 (짧게)
    print_header("테스트: 전후좌우 이동 (--move)")
    desk_warning("바퀴를 띄운 상태에서 방향(좌/우/전/후)이 맞는지 눈으로 확인하세요.")
    input("\n  이동 테스트 시작하려면 Enter (취소: Ctrl+C)... ")
    try:
        enable_wheels(bus, found_ids)
        run_action(bus, found_ids, "좌측 이동 ⬅", compute_wheel_velocities(0.0, 1.0))
        run_action(bus, found_ids, "우측 이동 ➡", compute_wheel_velocities(0.0, -1.0))
        run_action(bus, found_ids, "전진 ⬆", compute_wheel_velocities(1.0, 0.0))
        run_action(bus, found_ids, "후진 ⬇", compute_wheel_velocities(-1.0, 0.0))
        print("\n  🎉 이동 테스트 완료!")
    finally:
        disable_wheels(bus, found_ids)


def turn_test(bus, found_ids):
    # [--turn] 제자리 회전 (짧게)
    print_header("테스트: 제자리 회전 (--turn)")
    desk_warning("제자리에서 회전합니다. 바퀴를 띄우면 가장 안전합니다.")
    input("\n  제자리 회전 테스트 시작하려면 Enter (취소: Ctrl+C)... ")
    try:
        enable_wheels(bus, found_ids)
        run_action(bus, found_ids, "좌회전 ↺", compute_turn_velocities(1))
        run_action(bus, found_ids, "우회전 ↻", compute_turn_velocities(-1))
        print("\n  🎉 제자리 회전 테스트 완료!")
    finally:
        disable_wheels(bus, found_ids)


def long_test(bus, found_ids):
    # [--3m] 3m 전진 + 3m 후진 + 3m 회전 (책상 위, 바퀴 띄운 상태)
    print_header("테스트: 3m 주행 (--3m)")
    print("  📏 시간 기반 거리 계산 (목표거리 ÷ 실측속도):")

    d = TARGET_DISTANCE_M
    t = time_for_distance(d)

    print(f"     - 목표 거리: {d:.0f}m")
    print(f"     - 실측 속도 설정값: {MEASURED_SPEED_MPS:.2f} m/s")
    print(f"     - {d:.0f}m 소요시간: 약 {t:.1f}초")
    print(f"     - 3m 회전도 동일하게 약 {t:.1f}초 적용")
    print()
    print("  ✅ 책상 위 테스트 OK: 바퀴를 받침에 올려 띄운 상태로 진행하세요.")
    print("     바퀴가 공중에서 도는 동안, 굴러간 거리만큼의 시간을 적용합니다.")
    print()
    print("  ⚠️  처음엔 시간이 부정확할 수 있습니다.")
    print("      → 바퀴에 테이프를 붙여 회전수를 세고,")
    print("        회전수 × 0.314m(바퀴둘레)로 실제 거리를 확인 후 보정하세요.")
    input("\n  3m 테스트를 시작하려면 Enter (취소: Ctrl+C)... ")

    try:
        enable_wheels(bus, found_ids)
        # 3m 전진
        run_action(bus, found_ids, "3m 전진 ⬆",
                   compute_wheel_velocities(1.0, 0.0, speed=LONG_MOVE_RAW),
                   duration=t)
        # 3m 후진
        run_action(bus, found_ids, "3m 후진 ⬇",
                   compute_wheel_velocities(-1.0, 0.0, speed=LONG_MOVE_RAW),
                   duration=t)
        # 3m 회전 (바퀴 궤적 길이 기준)
        run_action(bus, found_ids, "3m 회전 ↺ (좌회전)",
                   compute_turn_velocities(1, speed=LONG_SPIN_RAW),
                   duration=t)
        print("\n  🎉 3m 테스트 완료!")
        print(f"\n  💡 실제 굴러간 거리가 3m와 다르면 보정하세요:")
        print(f"     새 속도 = {MEASURED_SPEED_MPS} × (굴러간거리 ÷ 3)")
        print(f"     예) 실제 2m였다면 → {MEASURED_SPEED_MPS} × (2÷3) = {MEASURED_SPEED_MPS*2/3:.3f}")
        print(f"         실제 4m였다면 → {MEASURED_SPEED_MPS} × (4÷3) = {MEASURED_SPEED_MPS*4/3:.3f}")
        print(f"     → 코드 상단 MEASURED_SPEED_MPS 값을 수정")
    finally:
        disable_wheels(bus, found_ids)


def main():
    parser = argparse.ArgumentParser(description="LeKiwi 바퀴 모터 진단 및 작동 테스트")
    parser.add_argument("--spin", action="store_true", help="각 바퀴 개별 회전 (책상에서 가장 안전)")
    parser.add_argument("--move", action="store_true", help="전후좌우 이동 (짧게)")
    parser.add_argument("--turn", action="store_true", help="제자리 회전 (짧게)")
    parser.add_argument("--3m", dest="long", action="store_true",
                        help="3m 전진 + 3m 후진 + 3m 회전 (책상 위 바퀴 띄운 상태 가능)")
    args = parser.parse_args()

    print("\n" + "\u2588" * 55)
    print("  LeKiwi 바퀴 모터 진단 스크립트")
    print(f"  포트: {PORT}")
    print("\u2588" * 55)

    if not check_port_exists(PORT):
        print("\n진단 중단: 포트를 찾을 수 없습니다.")
        sys.exit(1)

    bus = connect_bus(PORT)
    if bus is None:
        print("\n진단 중단: 버스 연결 실패.")
        sys.exit(1)

    try:
        found_ids = scan_motors(bus)
        status_ok = read_wheel_status(bus, found_ids)

        wheel_count = len([i for i in found_ids if i in WHEEL_MOTORS])
        any_test = args.spin or args.move or args.turn or args.long
        if any_test:
            if wheel_count == 3:
                if args.spin:
                    spin_test(bus, found_ids)
                if args.move:
                    move_test(bus, found_ids)
                if args.turn:
                    turn_test(bus, found_ids)
                if args.long:
                    long_test(bus, found_ids)
            else:
                print("\n  ⚠️  바퀴 3개가 모두 인식되지 않아 작동 테스트를 건너뜁니다.")

        print_header("진단 결과 요약")
        if wheel_count == 3 and status_ok:
            print("  ✅ 바퀴 모터 3개 모두 정상입니다!")
        else:
            missing = [i for i in WHEEL_MOTORS if i not in found_ids]
            if missing:
                print(f"  ❌ 바퀴 모터 누락: ID {missing}")
                print(f"\n  점검 사항:")
                print(f"    1. 데이지 체인 케이블 연결 (... -> ID7 -> ID8 -> ID9)")
                print(f"    2. 모터 ID 충돌 여부 (새 모터는 기본 ID 1)")
                print(f"    3. 12V 전원 공급 상태")
                print(f"    4. lerobot-setup-motors로 ID 재설정")

        if not any_test:
            print("\n  ℹ️  작동 테스트를 하려면 옵션을 추가하세요:")
            print("       --spin  (각 바퀴 개별 회전, 책상에서 가장 안전)")
            print("       --move  (전후좌우 이동, 짧게)")
            print("       --turn  (제자리 회전, 짧게)")
            print("       --3m    (3m 전진 + 3m 후진 + 3m 회전, 책상 위 가능)")

    finally:
        try:
            bus.disconnect()
            print("\n  버스 연결 해제 완료.")
        except Exception:
            pass


if __name__ == "__main__":
    main()
