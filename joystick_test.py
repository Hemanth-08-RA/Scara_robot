import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy


class JoystickTest(Node):

    def __init__(self):
        super().__init__('joystick_test')
        self.subscription = self.create_subscription(
            Joy,
            '/joy',
            self.joy_callback,
            10
        )

        self.last_axes = []
        self.last_buttons = []

    def joy_callback(self, msg):

        if self.last_axes != list(msg.axes):
            print("\nAXES:", [round(x, 2) for x in msg.axes])
            self.last_axes = list(msg.axes)

        if self.last_buttons != list(msg.buttons):
            print("BUTTONS:", msg.buttons)
            self.last_buttons = list(msg.buttons)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
