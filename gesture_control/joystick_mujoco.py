#!/usr/bin/env python3

import time
import threading

import mujoco
import mujoco.viewer

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


# ============================================================
# MUJOCO SCENE
# ============================================================

XML_PATH = "/home/robotics/scara_description_ws/urdf/scara_scene_mujoco.xml"


# ============================================================
# JOINT LIMITS
# ============================================================

BASE_MIN = -2.0
BASE_MAX = 2.0

SHOULDER_MIN = -0.15
SHOULDER_MAX = 0.02

FOREARM_MIN = -2.0
FOREARM_MAX = 2.0

WRIST_MIN = -4.712
WRIST_MAX = 4.712


# ============================================================
# GRIPPER
# ============================================================

GRIP_OPEN = -0.02
GRIP_CLOSED = 0.0


# ============================================================
# JOYSTICK AXES
#
# Confirmed from joystick_test.py
#
# ============================================================

LEFT_X = 0
LEFT_Y = 1

RIGHT_X = 3
RIGHT_Y = 4


# ============================================================
# BUTTONS
#
# A = 0
# B = 1
# X = 2
# Y = 3
# ============================================================

BUTTON_A = 0
BUTTON_B = 1
BUTTON_X = 2
BUTTON_Y = 3


# ============================================================
# SPEED
# ============================================================

BASE_SPEED = 0.45

# IMPORTANT:
# Shoulder has only +0.02 m upward travel.
SHOULDER_SPEED = 0.015

FOREARM_SPEED = 0.45

WRIST_SPEED = 0.60


# ============================================================
# CONTROL LOOP
# ============================================================

DT = 0.02


# ============================================================
# JOYSTICK DEADZONE
# ============================================================

DEADZONE = 0.12


# ============================================================
# ROS JOYSTICK
# ============================================================

class JoystickNode(Node):

    def __init__(self):

        super().__init__("scara_joystick")

        self.axes = [0.0] * 8

        self.buttons = [0] * 11

        self.lock = threading.Lock()

        self.create_subscription(
            Joy,
            "/joy",
            self.joy_callback,
            10
        )

        self.get_logger().info(
            "SCARA joystick connected to /joy"
        )


    def joy_callback(self, msg):

        with self.lock:

            self.axes = list(msg.axes)

            self.buttons = list(msg.buttons)


    def get_state(self):

        with self.lock:

            return (
                self.axes[:],
                self.buttons[:]
            )


# ============================================================
# CLAMP
# ============================================================

def clamp(value, low, high):

    return max(
        low,
        min(high, value)
    )


# ============================================================
# DEADZONE
# ============================================================

def apply_deadzone(value):

    if abs(value) < DEADZONE:

        return 0.0

    if value > 0:

        return (
            value - DEADZONE
        ) / (
            1.0 - DEADZONE
        )

    return (
        value + DEADZONE
    ) / (
        1.0 - DEADZONE
    )


# ============================================================
# MAIN
# ============================================================

def main():

    print("""
============================================================
             SCARA JOYSTICK TELEOPERATION
============================================================

LEFT STICK

    LEFT  -> BASE +
    RIGHT -> BASE -

    UP    -> SHOULDER / Z UP
    DOWN  -> SHOULDER / Z DOWN


RIGHT STICK

    LEFT  -> FOREARM +
    RIGHT -> FOREARM -

    UP    -> WRIST +
    DOWN  -> WRIST -


BUTTONS

    A -> GRIPPER CLOSE
    B -> GRIPPER OPEN
    X -> HOME
    Y -> STOP / HOLD


============================================================
             MUJOCO ONLY
             NO OPENCV
             NO CAMERA
============================================================
""")


    # ========================================================
    # ROS
    # ========================================================

    rclpy.init()

    joystick = JoystickNode()


    ros_thread = threading.Thread(
        target=rclpy.spin,
        args=(joystick,),
        daemon=True
    )

    ros_thread.start()


    # ========================================================
    # LOAD MUJOCO
    # ========================================================

    print("Loading MuJoCo:")

    print(XML_PATH)

    print()


    model = mujoco.MjModel.from_xml_path(
        XML_PATH
    )

    data = mujoco.MjData(
        model
    )


    # ========================================================
    # TARGET POSITIONS
    # ========================================================

    base = 0.0

    shoulder = 0.0

    forearm = 0.0

    wrist = 0.0

    gripper = GRIP_OPEN


    # ========================================================
    # BUTTON MEMORY
    # ========================================================

    old_buttons = [0] * 11


    # ========================================================
    # MUJOCO
    # ========================================================

    try:

        with mujoco.viewer.launch_passive(
            model,
            data
        ) as viewer:

            print("MuJoCo launched successfully.")
            print("Joystick control active.")
            print()


            while viewer.is_running():

                start = time.monotonic()


                # =================================================
                # READ JOYSTICK
                # =================================================

                axes, buttons = joystick.get_state()


                # =================================================
                # BUTTON A
                # CLOSE GRIPPER
                # =================================================

                if (
                    len(buttons) > BUTTON_A
                    and
                    buttons[BUTTON_A] == 1
                    and
                    old_buttons[BUTTON_A] == 0
                ):

                    gripper = GRIP_CLOSED

                    print(
                        "GRIPPER: CLOSE"
                    )


                # =================================================
                # BUTTON B
                # OPEN GRIPPER
                # =================================================

                if (
                    len(buttons) > BUTTON_B
                    and
                    buttons[BUTTON_B] == 1
                    and
                    old_buttons[BUTTON_B] == 0
                ):

                    gripper = GRIP_OPEN

                    print(
                        "GRIPPER: OPEN"
                    )


                # =================================================
                # BUTTON X
                # HOME
                # =================================================

                if (
                    len(buttons) > BUTTON_X
                    and
                    buttons[BUTTON_X] == 1
                    and
                    old_buttons[BUTTON_X] == 0
                ):

                    base = 0.0

                    shoulder = 0.0

                    forearm = 0.0

                    wrist = 0.0

                    print(
                        "SCARA: HOME"
                    )


                # =================================================
                # BUTTON Y
                # STOP
                # =================================================

                stop = (

                    len(buttons) > BUTTON_Y

                    and

                    buttons[BUTTON_Y] == 1
                )


                # =================================================
                # JOYSTICK MOVEMENT
                # =================================================

                if not stop:


                    # =============================================
                    # LEFT STICK X
                    # BASE
                    # =============================================

                    if len(axes) > LEFT_X:

                        x = apply_deadzone(
                            axes[LEFT_X]
                        )

                        base += (

                            -x
                            * BASE_SPEED
                            * DT
                        )


                    # =============================================
                    # LEFT STICK Y
                    # SHOULDER / Z
                    #
                    # joystick UP = negative
                    # joystick DOWN = positive
                    #
                    # therefore:
                    #
                    # UP   -> positive Z
                    # DOWN -> negative Z
                    # =============================================

                    if len(axes) > LEFT_Y:

                        y = apply_deadzone(
                            axes[LEFT_Y]
                        )

                        shoulder += (

                            -y
                            * SHOULDER_SPEED
                            * DT
                        )


                    # =============================================
                    # RIGHT STICK X
                    # FOREARM
                    # =============================================

                    if len(axes) > RIGHT_X:

                        x2 = apply_deadzone(
                            axes[RIGHT_X]
                        )

                        forearm += (

                            -x2
                            * FOREARM_SPEED
                            * DT
                        )


                    # =============================================
                    # RIGHT STICK Y
                    # WRIST
                    # =============================================

                    if len(axes) > RIGHT_Y:

                        y2 = apply_deadzone(
                            axes[RIGHT_Y]
                        )

                        wrist += (

                            -y2
                            * WRIST_SPEED
                            * DT
                        )


                # =================================================
                # APPLY LIMITS
                # =================================================

                base = clamp(
                    base,
                    BASE_MIN,
                    BASE_MAX
                )


                shoulder = clamp(
                    shoulder,
                    SHOULDER_MIN,
                    SHOULDER_MAX
                )


                forearm = clamp(
                    forearm,
                    FOREARM_MIN,
                    FOREARM_MAX
                )


                wrist = clamp(
                    wrist,
                    WRIST_MIN,
                    WRIST_MAX
                )


                # =================================================
                # SEND TO MUJOCO
                #
                # CONFIRMED ACTUATOR ORDER:
                #
                # 0 column
                # 1 shoulder
                # 2 forearm
                # 3 wrist
                # 4 left finger
                # 5 right finger
                # =================================================

                data.ctrl[0] = base

                data.ctrl[1] = shoulder

                data.ctrl[2] = forearm

                data.ctrl[3] = wrist

                data.ctrl[4] = gripper

                data.ctrl[5] = gripper


                # =================================================
                # PHYSICS
                # =================================================

                mujoco.mj_step(
                    model,
                    data
                )


                viewer.sync()


                # =================================================
                # BUTTON MEMORY
                # =================================================

                old_buttons = buttons[:]


                # =================================================
                # DEBUG OUTPUT
                # =================================================

                # Print only when useful
                if (
                    abs(base) > 0.001
                    or abs(shoulder) > 0.001
                    or abs(forearm) > 0.001
                    or abs(wrist) > 0.001
                ):

                    pass


                # =================================================
                # LOOP RATE
                # =================================================

                elapsed = (
                    time.monotonic()
                    - start
                )

                remaining = DT - elapsed

                if remaining > 0:

                    time.sleep(
                        remaining
                    )


    except KeyboardInterrupt:

        print()
        print(
            "SCARA joystick stopped."
        )


    finally:

        joystick.destroy_node()

        if rclpy.ok():

            rclpy.shutdown()


# ============================================================
# START
# ============================================================

if __name__ == "__main__":

    main()
