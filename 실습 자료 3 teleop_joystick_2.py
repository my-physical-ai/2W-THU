"""
teleop_joystick.py

Use an Xbox 360 (or similar) joystick on the PC to teleoperate a LeKiwi robot
running on a Raspberry Pi.

Usage:
  1) On the Raspberry Pi (robot side), run:

     python -m lerobot.robots.lekiwi.lekiwi_host \
        --host.port_zmq_cmd=5555 \
        --host.port_zmq_observations=5556 \
        --host.connection_time_s=86400 \
        --robot.id=my_lekiwi \
        --robot.cameras="{}"

  2) On the PC (where the joystick is connected), run:

     conda activate lerobot
     cd ~/lerobot/examples/lekiwi
     python teleop_joystick_1.py
"""

import os
import time
import pygame

# [수정] SSH 환경에서 pygame 비디오 오류 방지
os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig


# ==============================
# CONFIGURATION
# ==============================

DEFAULT_PI_IP    = "192.168.50.111"
LEKIWI_ROBOT_ID  = "my_lekiwi"

DEADZONE    = 0.15
MAX_LINEAR  = 0.3    # m/s  (lekiwi_client medium 기준)
# [수정] MAX_YAW 단위: deg/s (0.8 → 60)
# lekiwi_client speed_levels: slow=30, medium=60, fast=90
MAX_YAW     = 60.0   # deg/s


# ==============================
# 1) Joystick Teleop Class
# ==============================

class JoystickTeleop:
    def __init__(self,
                 deadzone: float = DEADZONE,
                 max_linear: float = MAX_LINEAR,
                 max_yaw: float = MAX_YAW):
        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            raise RuntimeError("❌ No joystick detected. Please connect a joystick and try again.")

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        print(f"🎮 Joystick Connected: {self.joystick.get_name()}")
        print(f"   축 수: {self.joystick.get_numaxes()}  버튼 수: {self.joystick.get_numbuttons()}")

        self.deadzone   = deadzone
        self.max_linear = max_linear
        self.max_yaw    = max_yaw

    def _apply_deadzone(self, value: float) -> float:
        if abs(value) < self.deadzone:
            return 0.0
        return value

    def read_teleop(self):
        pygame.event.pump()

        # Xbox 360 / Xbox One Linux 축 매핑
        # axis 0: 왼쪽 스틱 좌우 (y.vel 횡이동)
        # axis 1: 왼쪽 스틱 상하 (x.vel 전진/후진)
        # axis 3: 오른쪽 스틱 좌우 (theta.vel 회전)
        # [수정] axis 5 → axis 3 (axis 5는 오른쪽 트리거)
        lx = self._apply_deadzone(self.joystick.get_axis(0))
        ly = self._apply_deadzone(self.joystick.get_axis(1))
        rx = self._apply_deadzone(self.joystick.get_axis(3))  # 수정: 5 → 3

        # 위로 밀면 ly가 음수 → 전진(+x)
        x_vel   = -ly * self.max_linear
        y_vel   =  lx * self.max_linear
        yaw_vel =  rx * self.max_yaw

        return x_vel, y_vel, yaw_vel


# ==============================
# 2) LeKiwi Remote Client
# ==============================

class LeKiwiRemote:
    def __init__(self, ip: str = DEFAULT_PI_IP):
        cfg = LeKiwiClientConfig(
            remote_ip=ip,
            id=LEKIWI_ROBOT_ID,
        )
        self.robot = LeKiwiClient(cfg)

    def connect(self):
        print(f"🔌 Connecting to LeKiwi at {DEFAULT_PI_IP} (id='{LEKIWI_ROBOT_ID}') ...")
        self.robot.connect()
        print("✅ Connected.")

    def send_cmd(self, x_vel: float, y_vel: float, yaw_vel: float):
        action = {
            "x.vel":     x_vel,
            "y.vel":     y_vel,
            "theta.vel": yaw_vel,   # [수정] yaw.vel → theta.vel
        }
        self.robot.send_action(action)


# ==============================
# 3) Main loop
# ==============================

def main():
    print("🎮 Initializing joystick teleoperation...")
    teleop = JoystickTeleop()

    kiwi = LeKiwiRemote(ip=DEFAULT_PI_IP)
    kiwi.connect()

    print("\n🎮 Joystick teleoperation started.")
    print("   왼쪽 스틱 상하  : 전진/후진")
    print("   왼쪽 스틱 좌우  : 횡이동")
    print("   오른쪽 스틱 좌우: 회전")
    print("🛑 Ctrl+C to stop.\n")

    try:
        while True:
            x_vel, y_vel, yaw_vel = teleop.read_teleop()
            kiwi.send_cmd(x_vel, y_vel, yaw_vel)
            time.sleep(0.02)   # 50Hz

    except KeyboardInterrupt:
        print("\n🛑 Stopped.")
    finally:
        kiwi.send_cmd(0.0, 0.0, 0.0)
        pygame.quit()


if __name__ == "__main__":
    main()
