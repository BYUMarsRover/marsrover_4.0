import rclpy
from rclpy.node import Node

from std_msgs.msg import Int8
from std_srvs.srv import Trigger
from geometry_msgs.msg import Twist


class DriveMux(Node):
    """
    Class for switching between drive states

    :author: Nelson Durrant
    :date: Mar 2025

    States:
    - Teleop (cmd_vel_teleop, blue LED)
    - Arrival (cmd_vel, flashing green LED)
    - Auto (cmd_vel, red LED)

    Subscribers:
    - cmd_vel (geometry_msgs/Twist) - from Nav2
    - cmd_vel_teleop (geometry_msgs/Twist) - from user operation
    Publishers:
    - cmd_vel_mux (geometry_msgs/Twist)
    - cmd_led (std_msgs/Int8)
    Services:
    - trigger_teleop (std_srvs/Trigger)
    - trigger_auto (std_srvs/Trigger)
    - trigger_arrival (std_srvs/Trigger)
    """

    def __init__(self):
        super().__init__("drive_mux")

        self.nav_sub = self.create_subscription(Twist, "cmd_vel", self.nav_callback, 10)
        self.nav_sub  # prevent unused variable warning
        self.teleop_sub = self.create_subscription(
            Twist, "cmd_vel_teleop", self.teleop_callback, 10
        )
        self.teleop_sub  # prevent unused variable warning

        self.teleop_service = self.create_service(
            Trigger, "trigger_teleop", self.teleop_service_callback
        )
        self.nav_service = self.create_service(
            Trigger, "trigger_auto", self.nav_service_callback
        )
        self.arrival_service = self.create_service(
            Trigger, "trigger_arrival", self.arrival_service_callback
        )

        self.cmd_vel_mux_pub = self.create_publisher(Twist, "cmd_vel_mux", 10)
        self.cmd_led_pub = self.create_publisher(Int8, "cmd_led", 10)
        self.create_timer(0.5, self.led_timer_callback)

        self.state = "teleop"

        self.get_logger().info("DriveMux node started")

    def nav_callback(self, msg):
        """
        Callback for cmd_vel subscription
        """

        if self.state == "auto":
            self.cmd_vel_mux_pub.publish(msg)
        else:
            self.get_logger().warn("Received cmd_vel while not in auto state")

    def teleop_callback(self, msg):
        """
        Callback for cmd_vel_teleop subscription
        """

        if self.state == "teleop":
            self.cmd_vel_mux_pub.publish(msg)
        else:
            self.get_logger().warn("Received cmd_vel_teleop while not in teleop state")

    def teleop_service_callback(self, request, response):
        """
        Callback for trigger_teleop service
        """

        self.state = "teleop"
        self.get_logger().info("Switched to teleop state")
        response.success = True
        return response

    def nav_service_callback(self, request, response):
        """
        Callback for trigger_auto service
        """

        self.state = "auto"
        self.get_logger().info("Switched to auto state")
        response.success = True
        return response

    def arrival_service_callback(self, request, response):
        """
        Callback for trigger_arrival service
        """

        self.state = "arrival"
        self.get_logger().info("Switched to arrival state")
        response.success = True
        return response

    def led_timer_callback(self):
        """
        Publish to cmd_led based on current state
        """

        # From Arduino code:
        # AUTONOMOUS_STATE = 0
        # TELEOPERATION_STATE = 1
        # ARRIVAL_STATE = 2

        if self.state == "teleop":
            led_state = 1
        elif self.state == "auto":
            led_state = 0
        elif self.state == "arrival":
            led_state = 2
        else:
            self.get_logger().error("Invalid state: " + self.state)
            return

        led_msg = Int8(data=led_state)
        self.cmd_led_pub.publish(led_msg)


def main(args=None):
    rclpy.init(args=args)
    drive_mux = DriveMux()
    rclpy.spin(drive_mux)
    drive_mux.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
