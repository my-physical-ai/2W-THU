import time

from lerobot.robots.lekiwi import LeKiwiClient, LeKiwiClientConfig
from lerobot.teleoperators.keyboard.teleop_keyboard import (
    KeyboardTeleop,
    KeyboardTeleopConfig,
)


def main():
    # 1) Remote LeKiwi connection config
    #    - remote_ip: Raspberry Pi's IP address
    #    - id: must match --robot.id used in lekiwi_host on the Pi
    robot_config = LeKiwiClientConfig(
        remote_ip="192.168.50.111",  # TODO: change to your Pi IP
        id="my_lekiwi",              # TODO: match --robot.id on Pi
    )

    # 2) Keyboard teleop config
    keyboard_config = KeyboardTeleopConfig(
        id="my_laptop_keyboard",
    )

    # 3) Create objects
    robot = LeKiwiClient(robot_config)
    teleop_keyboard = KeyboardTeleop(keyboard_config)

    # 4) Connect
    robot.connect()
    teleop_keyboard.connect()

    print("✅ LeKiwi keyboard teleoperation (BASE ONLY) started")
    print("   W/A/S/D : forward / left / backward / right (x, y)")
    print("   Z/X     : turn left / turn right (theta)")
    print("   R/F     : speed up / speed down")
    print("   Ctrl+C  : quit")

    try:
        while True:
            # Optional: read observation (e.g. for future use, logging, etc.)
            observation = robot.get_observation()
            # e.g., to access a camera:
            # front_img = observation.get("front", None)

            # 1) Read current keyboard state
            keyboard_keys = teleop_keyboard.get_action()

            # 2) Convert key presses -> base velocity action (x.vel, y.vel, theta.vel)
            base_action = robot._from_keyboard_to_base_action(keyboard_keys)

            # 3) Send action to the robot (BASE ONLY: wheels only)
            robot.send_action(base_action)

            # Small sleep to reduce CPU usage
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("\n🛑 Stopped by user (Ctrl+C).")

    finally:
        teleop_keyboard.disconnect()
        robot.disconnect()
        print("🔌 Disconnected from LeKiwi and keyboard teleop.")


if __name__ == "__main__":
    main()

