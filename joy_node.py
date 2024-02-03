import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import pygame
import time
from math import fabs

DEAD_ZONE = 0.1

class JoystickNode(Node):
    def __init__(self):
        super().__init__('joy_node')
        self.publisher_ = self.create_publisher(Joy, '/joy', 10)

        pygame.init()
        pygame.joystick.init()

        num_joysticks = pygame.joystick.get_count()

        print("Searching for Joystick")
        while not num_joysticks:
            time.sleep(1)  # Wait for 1 second before checking again
            num_joysticks = pygame.joystick.get_count()

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()

        name = self.joystick.get_name()
        id = self.joystick.get_id()
        guid = self.joystick.get_guid()
        power_level = self.joystick.get_power_level()
        self.get_logger().info(f"Connected to {name}")
        self.get_logger().info(f"ID: {id}")
        self.get_logger().info(f"Guid: {guid}")
        self.get_logger().info(f"Power Level: {power_level}")
        self.get_logger().info(f"Dead Zone: {DEAD_ZONE}")

        self.timer = self.create_timer(0.05, self.parse_and_publish_data)

    def parse_and_publish_data(self):
        try:
            joy_msg = Joy()
            pygame.event.pump()
            joy_msg.header.stamp = self.get_clock().now().to_msg()

            # Map buttons
            joy_msg.buttons = [
                self.joystick.get_button(0),  # Cross Button
                self.joystick.get_button(1),  # Circle Button
                self.joystick.get_button(2),  # Square Button
                self.joystick.get_button(3),  # Triangle Button
                self.joystick.get_button(4),  # Left Bumper
                self.joystick.get_button(5),  # Right Bumper
                self.joystick.get_button(9),  # Option Button
                self.joystick.get_button(8),  # Share Button
                self.joystick.get_button(7),  # Start Button
                self.joystick.get_button(10),  # Power Button
                self.joystick.get_button(11),  # Button Stick Left
                self.joystick.get_button(12)  # Button Stick Right
            ]

            # Map axes
            joy_msg.axes = [
                self.joystick.get_axis(0) if fabs(self.joystick.get_axis(0)) > DEAD_ZONE else 0.0,  # Left/Right Axis stick left
                self.joystick.get_axis(1) if fabs(self.joystick.get_axis(1)) > DEAD_ZONE else 0.0,  # Up/Down Axis stick left
                self.joystick.get_axis(3) if fabs(self.joystick.get_axis(3)) > DEAD_ZONE else 0.0,  # Left/Right Axis stick right
                self.joystick.get_axis(4) if fabs(self.joystick.get_axis(4)) > DEAD_ZONE else 0.0,  # Up/Down Axis stick right
                self.joystick.get_axis(2) if fabs(self.joystick.get_axis(2)) > DEAD_ZONE else 0.0,  # Left Trigger
                self.joystick.get_axis(5) if fabs(self.joystick.get_axis(5)) > DEAD_ZONE else 0.0,   # Right Trigger
                float(self.joystick.get_hat(0)[0]),  # Left/Right hat
                float(self.joystick.get_hat(0)[1])  # Down/Up hat
            ]

            power_level = self.joystick.get_power_level()

            self.publisher_.publish(joy_msg)
            # print(joy_msg)

        except pygame.error as e:
            self.get_logger().error(f"Pygame error: {e}")


def main(args=None):
    rclpy.init(args=args)
    joy = JoystickNode()
    rclpy.spin(joy)
    joy.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    pygame.quit()

if __name__ == '__main__':
    main()
