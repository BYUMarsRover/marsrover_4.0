#!/usr/bin/env python3
# Created by GitHub Copilot, Nov 2024

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_srvs.srv import Trigger

BACK, START, POWER = 6, 7, 8  # 6: Switch to auto, 7: Switch to teleop


class ModeSwitcher(Node):
    """
    This node handles mode switching between teleop and auto modes via joystick buttons.
    
    When START button is pressed, it calls trigger_teleop service to enable manual control.
    When BACK button is pressed, it calls trigger_auto service to enable autonomous mode.

    :author: GitHub Copilot
    :date: Nov 2024

    Subscribers:
    - joy (sensor_msgs/Joy)
    Clients:
    - trigger_teleop (std_srvs/Trigger)
    - trigger_auto (std_srvs/Trigger)
    """

    def __init__(self):
        super().__init__("mode_switcher")

        # Subscribe to joystick
        self.joy_sub = self.create_subscription(
            Joy, "joy", self.joy_callback, 10
        )

        # Service clients for mode switching
        self.teleop_client = self.create_client(Trigger, "trigger_teleop")
        self.auto_client = self.create_client(Trigger, "trigger_auto")

        # Track button states for edge detection (only trigger on button press, not hold)
        self.last_start_button = 0
        self.last_back_button = 0

        self.get_logger().info("Mode switcher node started")

    def joy_callback(self, msg):
        """Handle joystick input and trigger mode switches on button press"""
        
        # Check if START button (button 7) is pressed for teleop mode
        if len(msg.buttons) > START:
            start_button = msg.buttons[START]
            
            # Trigger teleop mode on button press (rising edge)
            if start_button == 1 and self.last_start_button == 0:
                self.get_logger().info("START button pressed - switching to teleop mode")
                self.call_trigger_teleop()
            
            self.last_start_button = start_button
        
        # Check if BACK button (button 6) is pressed for auto mode
        if len(msg.buttons) > BACK:
            back_button = msg.buttons[BACK]
            
            # Trigger auto mode on button press (rising edge)
            if back_button == 1 and self.last_back_button == 0:
                self.get_logger().info("BACK button pressed - switching to auto mode")
                self.call_trigger_auto()
            
            self.last_back_button = back_button

    def call_trigger_teleop(self):
        """Call the trigger_teleop service to switch to teleop mode"""
        if not self.teleop_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("trigger_teleop service not available")
            return
        
        request = Trigger.Request()
        future = self.teleop_client.call_async(request)
        future.add_done_callback(self.teleop_response_callback)

    def teleop_response_callback(self, future):
        """Handle response from trigger_teleop service"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Successfully switched to teleop mode")
            else:
                self.get_logger().warn("Failed to switch to teleop mode")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def call_trigger_auto(self):
        """Call the trigger_auto service to switch to auto mode"""
        if not self.auto_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("trigger_auto service not available")
            return
        
        request = Trigger.Request()
        future = self.auto_client.call_async(request)
        future.add_done_callback(self.auto_response_callback)

    def auto_response_callback(self, future):
        """Handle response from trigger_auto service"""
        try:
            response = future.result()
            if response.success:
                self.get_logger().info("Successfully switched to auto mode")
            else:
                self.get_logger().warn("Failed to switch to auto mode")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    mode_switcher = ModeSwitcher()
    
    try:
        rclpy.spin(mode_switcher)
    except KeyboardInterrupt:
        pass
    finally:
        mode_switcher.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
