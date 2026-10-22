#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


# EXACT SAME LIMITS AS YOUR GUI
LIMITS = {
    'column_joint': (-2.0, 2.0),
    'shoulder_joint': (-0.15, 0.02),
    'forearm_joint': (-2.0, 2.0),
    'wrist_joint': (-4.712, 4.712),
    'left_finger_joint': (-0.06, 0.0)
}


class JoystickSCARA(Node):

    def __init__(self):
        super().__init__('joystick_scara_teleop')

        # Same ROS 2 controllers used by your GUI
        self.arm_pub = self.create_publisher(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            10
        )

        self.gripper_pub = self.create_publisher(
            JointTrajectory,
            '/gripper_controller/joint_trajectory',
            10
        )

        # Same joint order as GUI
        self.arm_joints = [
            'column_joint',
            'shoulder_joint',
            'forearm_joint',
            'wrist_joint'
        ]

        self.gripper_joints = [
            'left_finger_joint'
        ]

        # Same initial position as GUI
        self.arm_pos = [
            0.0,
            0.0,
            0.0,
            0.0
        ]

        self.gripper_pos = [
            0.0
        ]

        # ==================================================
        # JOYSTICK AXES
        # ==================================================

        # Left stick
        self.LEFT_X = 0
        self.LEFT_Y = 1

        # Right stick
        self.RIGHT_X = 3
        self.RIGHT_Y = 4

        # ==================================================
        # BUTTONS
        # ==================================================

        self.BUTTON_CLOSE = 0
        self.BUTTON_OPEN = 1
        self.BUTTON_HOME = 2
        self.BUTTON_STOP = 3

        # ==================================================
        # AXIS DEADZONE
        # ==================================================

        self.DEADZONE = 0.30

        # Latest joystick values
        self.axes = [0.0] * 8
        self.buttons = [0] * 11

        # Prevent a button from repeatedly triggering
        self.previous_buttons = [0] * 11

        # Stop flag
        self.stop_active = False

        self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        # 10 Hz = one GUI-style increment every 0.1 second
        self.timer = self.create_timer(
            0.1,
            self.update_robot
        )

        print("""
========================================================
             SCARA JOYSTICK TELEOPERATION
========================================================

LEFT STICK
  LEFT   -> Base +0.05 rad
  RIGHT  -> Base -0.05 rad

  UP     -> Shoulder/Z +0.005 m
  DOWN   -> Shoulder/Z -0.005 m


RIGHT STICK
  LEFT   -> Forearm +0.05 rad
  RIGHT  -> Forearm -0.05 rad

  UP     -> Wrist +0.10 rad
  DOWN   -> Wrist -0.10 rad


BUTTONS
  A / Button 0 -> CLOSE GRIPPER
  B / Button 1 -> OPEN GRIPPER
  X / Button 2 -> HOME
  Y / Button 3 -> STOP / HOLD


========================================================
 SAME JOINTS / INCREMENTS / LIMITS AS GUI
========================================================

Release stick = STOP
========================================================
""")

    # --------------------------------------------------
    # Clamp
    # --------------------------------------------------

    def clamp(self, value, low, high):
        return max(low, min(high, value))

    # --------------------------------------------------
    # Publish arm
    # --------------------------------------------------

    def publish_arm(self):

        msg = JointTrajectory()

        msg.joint_names = self.arm_joints

        point = JointTrajectoryPoint()

        point.positions = [
            float(position)
            for position in self.arm_pos
        ]

        point.time_from_start = Duration(
            sec=0,
            nanosec=200000000
        )

        msg.points = [point]

        self.arm_pub.publish(msg)

    # --------------------------------------------------
    # Publish gripper
    # --------------------------------------------------

    def publish_gripper(self):

        msg = JointTrajectory()

        msg.joint_names = self.gripper_joints

        point = JointTrajectoryPoint()

        point.positions = [
            float(self.gripper_pos[0])
        ]

        point.time_from_start = Duration(
            sec=0,
            nanosec=200000000
        )

        msg.points = [point]

        self.gripper_pub.publish(msg)

    # --------------------------------------------------
    # Update arm joint
    # --------------------------------------------------

    def update_arm_joint(self, index, delta):

        joint_name = self.arm_joints[index]

        low, high = LIMITS[joint_name]

        old_position = self.arm_pos[index]

        new_position = old_position + delta

        self.arm_pos[index] = self.clamp(
            new_position,
            low,
            high
        )

        if self.arm_pos[index] != old_position:
            self.publish_arm()

    # --------------------------------------------------
    # Gripper
    # --------------------------------------------------

    def set_gripper(self, position):

        low, high = LIMITS['left_finger_joint']

        self.gripper_pos[0] = self.clamp(
            position,
            low,
            high
        )

        self.publish_gripper()

    # --------------------------------------------------
    # HOME
    # --------------------------------------------------

    def home(self):

        self.arm_pos = [
            0.0,
            0.0,
            0.0,
            0.0
        ]

        self.gripper_pos = [
            0.0
        ]

        self.publish_arm()
        self.publish_gripper()

        print("HOME POSITION")

    # --------------------------------------------------
    # Deadzone
    # --------------------------------------------------

    def axis_direction(self, value):

        if abs(value) < self.DEADZONE:
            return 0

        if value > 0:
            return 1

        return -1

    # --------------------------------------------------
    # Joystick callback
    # --------------------------------------------------

    def joy_callback(self, msg):

        self.axes = list(msg.axes)
        self.buttons = list(msg.buttons)

    # --------------------------------------------------
    # Main joystick processing
    # --------------------------------------------------

    def update_robot(self):

        # ----------------------------------------------
        # BUTTONS
        # ----------------------------------------------

        # HOME
        if (
            len(self.buttons) > self.BUTTON_HOME
            and
            self.buttons[self.BUTTON_HOME]
            and
            not self.previous_buttons[self.BUTTON_HOME]
        ):
            self.home()

        # STOP
        if (
            len(self.buttons) > self.BUTTON_STOP
            and
            self.buttons[self.BUTTON_STOP]
        ):
            self.stop_active = True
        else:
            self.stop_active = False

        # GRIPPER CLOSE
        if (
            len(self.buttons) > self.BUTTON_CLOSE
            and
            self.buttons[self.BUTTON_CLOSE]
            and
            not self.previous_buttons[self.BUTTON_CLOSE]
        ):
            self.set_gripper(0.0)
            print("GRIPPER CLOSE")

        # GRIPPER OPEN
        if (
            len(self.buttons) > self.BUTTON_OPEN
            and
            self.buttons[self.BUTTON_OPEN]
            and
            not self.previous_buttons[self.BUTTON_OPEN]
        ):
            self.set_gripper(-0.05)
            print("GRIPPER OPEN")

        # Save button state
        self.previous_buttons = self.buttons[:]

        # ----------------------------------------------
        # STOP
        # ----------------------------------------------

        if self.stop_active:
            return

        # ----------------------------------------------
        # LEFT STICK X
        #
        # GUI:
        # A = +0.05
        # D = -0.05
        #
        # Joystick:
        # LEFT  = negative
        # RIGHT = positive
        # ----------------------------------------------

        if len(self.axes) > self.LEFT_X:

            value = self.axis_direction(
                self.axes[self.LEFT_X]
            )

            if value < 0:
                self.update_arm_joint(
                    0,
                    +0.05
                )

            elif value > 0:
                self.update_arm_joint(
                    0,
                    -0.05
                )

        # ----------------------------------------------
        # LEFT STICK Y
        #
        # GUI:
        # W = +0.005
        # S = -0.005
        #
        # Joystick:
        # UP   = negative
        # DOWN = positive
        # ----------------------------------------------

        if len(self.axes) > self.LEFT_Y:

            value = self.axis_direction(
                self.axes[self.LEFT_Y]
            )

            if value < 0:
                self.update_arm_joint(
                    1,
                    +0.005
                )

            elif value > 0:
                self.update_arm_joint(
                    1,
                    -0.005
                )

        # ----------------------------------------------
        # RIGHT STICK X
        #
        # GUI:
        # J = +0.05
        # L = -0.05
        # ----------------------------------------------

        if len(self.axes) > self.RIGHT_X:

            value = self.axis_direction(
                self.axes[self.RIGHT_X]
            )

            if value < 0:
                self.update_arm_joint(
                    2,
                    +0.05
                )

            elif value > 0:
                self.update_arm_joint(
                    2,
                    -0.05
                )

        # ----------------------------------------------
        # RIGHT STICK Y
        #
        # GUI:
        # I = +0.10
        # K = -0.10
        # ----------------------------------------------

        if len(self.axes) > self.RIGHT_Y:

            value = self.axis_direction(
                self.axes[self.RIGHT_Y]
            )

            if value < 0:
                self.update_arm_joint(
                    3,
                    +0.10
                )

            elif value > 0:
                self.update_arm_joint(
                    3,
                    -0.10
                )


def main():

    rclpy.init()

    node = JoystickSCARA()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
