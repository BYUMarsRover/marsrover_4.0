import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, UInt16MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# from rover_msgs.msg import IWCMotors, Elevator, HeartbeatStatusRover, FPVServo

from std_srvs.srv import Trigger
import serial
import time
import queue
import threading

BACK, START, POWER = 6, 7, 8  # 6: Disable drive, 7: Enable drive
DPAD_HORIZONTAL, DPAD_VERTICAL = 6, 7

ELEVATOR_DIR_UP, ELEVATOR_DIR_DOWN = 1, 0

ELEVATOR_SPEED_CONSTANTS = [0.3, 0.5, 0.7, 1]
NUM_OF_ELEVATOR_SPEED_CONSTANTS = len(ELEVATOR_SPEED_CONSTANTS)


class MegaWrapper(Node):
    """
    This node acts as a middleman between the Orin and the Arduino Mega.

    :author: Braden Meyers
    :date: Mar 2025

    TODO: We need to add A LOT of functionality to this - elevator, speed changes, etc.

    Subscribers:
    - cmd_vel_mux (geometry_msgs/Twist)
    - joy (sensor_msgs/Joy)
    Publishers:
    - ArduinoDebug (std_msgs/String)
    Clients:
    - trigger_teleop (std_srvs/Trigger)
    """

    def __init__(self):
        super().__init__("mega_wrapper")

        # SUBSCRIBERS
        self.create_subscription(Twist, "cmd_vel_mux", self.send_wheel, 1)
        self.subscription = self.create_subscription(
            Joy, "joy", self.joy_callback, 10
        )  # direct elevator control
        # self.create_subscription(Elevator, '/elevator', self.send_elevator, 1)
        # self.create_subscription(Bool, '/arm_clicker', self.send_clicker, 1)
        # self.create_subscription(Bool, '/arm_laser', self.send_laser, 1)
        # self.create_subscription(FPVServo, '/fpv_servo', self.send_fpvsv, 1)
        # self.create_subscription(HeartbeatStatusRover, '/heartbeat_status_rover', self.send_heart, 1)

        # PUBLISHERS
        # self.pub_IR = self.create_publisher(UInt16MultiArray, '/IR', 1)
        self.pub_Debug = self.create_publisher(String, "/ArduinoDebug", 25)

        # Service clients
        self.client = self.create_client(Trigger, "trigger_teleop")

        self.latest_wheel_msg = None
        self.latest_heart_msg = None
        self.latest_elevator_msg = None
        self.latest_hands_msg = None

        # Joystick variable from elevator
        self.last_left_dpad = False
        self.last_right_dpad = False
        self.elevator_speed_multiplier_idx = 0
        self.elevator_speed_multiplier = ELEVATOR_SPEED_CONSTANTS[
            self.elevator_speed_multiplier_idx
        ]

        self.lock = threading.Lock()
        # Connect to Arduino
        self.disconnected = True
        self.handshake = False  # If a Serial object is good to communicate on the port
        self.connect()

        time.sleep(7.0)

        # Start writer thread
        self.writer_thread = threading.Thread(
            target=self.serial_writer_loop, daemon=True
        )
        self.writer_thread.start()

        # Timer for relay_mega
        self.create_timer(0.01, self.loop)  # 100 Hz
        self.get_logger().info("Mega wrapper started")

    def connect(self):
        failure_count = 0
        while failure_count < 10:
            try:
                arduino_port = "/dev/rover/onBoardMega"
                self.ser = serial.Serial(
                    arduino_port, 115200, timeout=0.3, write_timeout=1.0, dsrdtr=True
                )
                break
            except serial.SerialException as e:
                self.get_logger().warn(
                    f"Could not open serial port {arduino_port}: {e}"
                )
                failure_count += 1
            time.sleep(0.1)
        if failure_count >= 10:
            self.get_logger().warn(
                f"Could not open serial port {arduino_port} after {failure_count} attempts."
            )
            self.write_debug("Orin: Could not establish connection to Arduino.")
            self.disconnected = True
            self.disconnect()
        else:
            self.write_debug("Orin: Connection handshake.")
            self.disconnected = False
            try:
                self.ser.reset_input_buffer()
                self.ser.reset_output_buffer()
            except:
                self.write_debug("Orin: Could not reset buffers on connection.")

    def disconnect(self):
        self.disconnected = True
        self.handshake = False
        if self.ser is not None:
            self.ser.close()

    def serial_writer_loop(self):
        while rclpy.ok():  # Checks if the node is still running
            if not self.handshake:
                self.write_debug("Orin: Waiting for Arduino handshake")
                time.sleep(1)
                continue
            # msg = self.serial_queue.get()  # Blocking until a message is available
            if not self.disconnected:
                with self.lock:
                    # Ensure connection
                    if not self.ser.is_open:
                        self.get_logger().warn(
                            "Orin: Arduino port is not open at time of writing!"
                        )
                        self.write_debug(
                            "Orin: Arduino port is not open at time of writing!"
                        )
                        return

                    messages = []
                    if self.latest_wheel_msg is not None:
                        messages.append(str(self.latest_wheel_msg))
                        self.latest_wheel_msg = None
                    if self.latest_elevator_msg is not None:
                        messages.append(str(self.latest_elevator_msg))
                        self.latest_elevator_msg = None
                    if self.latest_heart_msg is not None:
                        messages.append(str(self.latest_heart_msg))
                        self.latest_heart_msg = None
                    if self.latest_hands_msg is not None:
                        messages.append(str(self.latest_hands_msg))
                        self.latest_hands_msg = None

                    # Send message
                    for msg in messages:
                        try:
                            self.ser.write((msg).encode("utf-8"))
                            self.write_debug("Orin: Thread output to Arduino: " + msg)
                        except serial.SerialTimeoutException as e:
                            self.get_logger().warn(
                                f"Failed to write to serial due to timeout: {e}"
                            )
                            self.write_debug(
                                "Orin: Failed to send message to Arduino in time."
                            )
                            # 2024-25 Experiments revealed these can help with reliability
                            time.sleep(2)
                            self.handshake = False
                            break  # Exit the loop after handling the exception
                        except serial.SerialException as e:
                            self.get_logger().warn(
                                f"Failed to write to serial due to exception: {e}"
                            )
                            self.write_debug(
                                "Orin: Error when trying to send message to Arduino."
                            )
                            self.disconnect()  # Not necessarily disconnected, but act like we are
                            break  # Exit the loop after handling the exception

                    # Give arduino breathing time
                    time.sleep(
                        0.05
                    )  # [s], delay between writes, only blocks write thread not main thread.
                    # Lowering this value can improve input latency, but lowering it too much
                    # risks overwhelming the Arduino and causing a I/O timeout -- resulting
                    # in a few seconds of disconnect (not worth shaving a few ms of each volley)
            else:
                self.write_debug("Orin: Not connected to Arduino; ignoring message.")

    def map_value(self, value, from_min, from_max, to_min, to_max):
        # Clamp the input value to the from_min and from_max range
        clamped_value = max(from_min, min(value, from_max))

        # Scale the clamped value to a 0-1 range
        scaled_value = (clamped_value - from_min) / (from_max - from_min)

        # Scale it to the new range
        mapped_value = to_min + (scaled_value * (to_max - to_min))

        return int(mapped_value)

    def send_wheel(self, msg):

        # Constants for differential drive
        self.wheel_radius = 0.1
        self.wheel_base = 0.5

        # Kinematics for differential drive
        left_wheels_speed = (
            msg.linear.x - (msg.angular.z * self.wheel_base / 2.0)
        ) / self.wheel_radius
        right_wheels_speed = (
            msg.linear.x + (msg.angular.z * self.wheel_base / 2.0)
        ) / self.wheel_radius

        # Get direction of the wheels
        if left_wheels_speed >= 0:
            left_wheels_forward = True
        else:
            left_wheels_forward = False

        if right_wheels_speed >= 0:
            right_wheels_forward = True
        else:
            right_wheels_forward = False

        # these need to be from 0 to 255 and ints
        # TODO: parameterize these values
        left_wheels_speed = self.map_value(abs(left_wheels_speed), 0, 10, 0, 255)
        right_wheels_speed = self.map_value(abs(right_wheels_speed), 0, 10, 0, 255)

        motor_params = [
            left_wheels_speed,
            left_wheels_forward,
            left_wheels_speed,
            left_wheels_forward,
            left_wheels_speed,
            left_wheels_forward,
            right_wheels_speed,
            right_wheels_forward,
            right_wheels_speed,
            right_wheels_forward,
            right_wheels_speed,
            right_wheels_forward,
        ]
        wheel_msg = (
            "$WHEEL," + ",".join(str(int(param)) for param in motor_params) + "*"
        )
        self.latest_wheel_msg = wheel_msg

    def send_elevator(self, speed, direction):
        eleva_params = [speed, direction]
        eleva_msg = (
            "$ELEVA," + ",".join(str(int(param)) for param in eleva_params) + "*"
        )
        # self.write_debug(f"Orin: Sending elevator message to Arduino. {eleva_msg}")
        self.latest_elevator_msg = eleva_msg

    # def send_clicker(self, msg):
    #     click_value = 1 if msg.data else 0
    #     click_msg = f"$CLICK,{click_value}*"
    #     self.serial_write(click_msg)

    # def send_laser(self, msg):
    #     laser_value = 1 if msg.data else 0
    #     laser_msg = f"$LASER,{laser_value}*"
    #     self.serial_write(laser_msg)

    # def send_fpvsv(self, msg):
    #     fpv_params = [msg.yaw, msg.pitch]
    #     fpvsv_msg = "$FPVSV," + ",".join(str(param) for param in fpv_params) + "*"
    #     self.serial_write(fpvsv_msg)

    def send_heart(self, msg):
        heart_msg = f"$HEART,{msg.elapsed_time}*"
        self.latest_heart_msg = heart_msg

    def read_nmea(self):
        # Watch for start of new message
        try:
            # Check if serial is open
            if not self.ser.is_open:
                self.get_logger().warn("Serial port is not open.")
                self.write_debug("WARNING: Serial port is not open")
                return -1, ""

            # Try reading one byte and decoding it
            x = self.ser.read(1).decode("ascii").strip()

            # Check if data was read
            if not x:
                # self.get_logger().info("No data read from serial port.")
                # self.write_debug("Orin: Nothing to read")  # Optional: uncomment if you want this log
                return 0, ""

        except serial.SerialException as e:
            # Handle SerialException (e.g., if the port is not available)
            self.get_logger().error(
                f"SerialException: Failed to read from serial port: {e}"
            )
            self.write_debug(f"ERROR: SerialException: {e}")

            # Try to reset the input buffer and disconnect
            try:
                self.ser.reset_input_buffer()  # Reset the buffer
                self.disconnect()  # Disconnect
                self.get_logger().info(
                    "Serial port input buffer reset and disconnected."
                )
            except Exception as reset_e:
                self.get_logger().warn(f"Could not flush input buffer: {reset_e}")
                self.write_debug(f"WARNING: Could not flush input buffer: {reset_e}")

            return -1, ""

        except Exception as e:
            # Catch all other unexpected errors
            self.get_logger().error(f"Unexpected error: {e}")
            self.write_debug(f"ERROR: Unexpected error: {e}")
            return -1, ""

        # Debug
        # self.write_debug("Orin: Reading a new message from the Arduino starting with " + x)
        # Verify message is in NMEA format
        if x != "$":
            return -1, ""

        # Read the rest of the sentence until '*'
        try:
            mega_msg = self.ser.read_until(b"*").decode("ascii").strip()
        except:
            self.get_logger().warn(f"Failed to read from serial - Second")
            self.write_debug("WARNING: Read failure - Second")
            try:
                self.ser.reset_input_buffer()
            except:
                self.write_debug("WARNING: Could not flush input buffer")
            return -1, ""

        # Ensure message does not contain encased messages
        if "$" in mega_msg:
            self.write_debug("WARNING: Nested messages: " + mega_msg)
            return -1, ""

        # Split message tag from data
        if "," in mega_msg:
            msg_parts = mega_msg.split(",", 1)
            tag, data = msg_parts[0], msg_parts[1][0:-2]  # Strip off trailing ,*
        else:
            tag, data = mega_msg, ""
        # Debug
        self.write_debug(
            "Orin: Reading a message from the Arduino w/ tag: "
            + tag
            + ", and data: "
            + data
        )

        return tag, data

    def relay_mega(self):
        # Read message
        ###################
        tag, msg = self.read_nmea()

        # Interpret message
        ###################
        if tag == "IRLIG":
            # try:
            #     sensorData = UInt16MultiArray(data=[int(x) for x in msg.split(',')])
            #     self.pub_IR.publish(sensorData)
            # except ValueError:
            #     self.write_debug("Orin: Failed to parse IR data")
            #     self.get_logger().warn("Failed to parse IR data")
            pass

        elif tag == "DEBUG":
            self.write_debug(msg)

        elif tag == "HANDS":
            self.handshake = True
            # self.serial_write("$HANDS,*")

            self.latest_hands_msg = "$HANDS,*"
            self.write_debug("Orin: Recieved Arduino connection handshake.")

        # Debug
        elif tag == -1:
            self.write_debug(
                "Orin: Read corrupt or incomplete message.  (This is expected after a read error)"
            )

    def write_debug(self, msg):
        message = String()
        message.data = msg
        self.pub_Debug.publish(message)

    def loop(self):
        if self.disconnected:
            self.connect()
        else:
            self.relay_mega()

    def joy_callback(self, msg):
        # Assuming the left joystick is represented by axes 0 and 1
        left_x = msg.axes[0]  # Left joystick horizontal
        left_y = msg.axes[1]  # Left joystick vertical

        self.get_logger().info(f"Left Joystick - X: {left_x}, Y: {left_y}")

        # create a case where it calls the trigger_telop when the enable button is pressed

    def elevator_commands(self, msg: Joy):

        elevator_speed = 0
        elevator_direction = 0

        elevator_input = msg.axes[DPAD_VERTICAL]
        elevator_speed_input = msg.axes[DPAD_HORIZONTAL]

        elevator_speed = int(self.elevator_speed_multiplier * 255)

        if elevator_input == 1.0:
            elevator_direction = ELEVATOR_DIR_UP
        elif elevator_input == -1.0:
            elevator_direction = ELEVATOR_DIR_DOWN
        else:
            elevator_speed = 0

        if elevator_speed_input == 1:
            left_dpad = True
            right_dpad = False
        elif elevator_speed_input == -1:
            right_dpad = True
            left_dpad = False
        else:
            left_dpad = False
            right_dpad = False

        if left_dpad and not self.last_left_dpad:
            if self.elevator_speed_multiplier_idx > 0:
                self.elevator_speed_multiplier_idx -= 1
                self.elevator_speed_multiplier = ELEVATOR_SPEED_CONSTANTS[
                    self.elevator_speed_multiplier_idx
                ]
            self.last_left_dpad = True

        elif right_dpad and not self.last_right_dpad:
            if self.elevator_speed_multiplier_idx < NUM_OF_ELEVATOR_SPEED_CONSTANTS - 1:
                self.elevator_speed_multiplier_idx += 1
                self.elevator_speed_multiplier = ELEVATOR_SPEED_CONSTANTS[
                    self.elevator_speed_multiplier_idx
                ]
            self.last_right_dpad = True

        if not left_dpad:
            self.last_left_dpad = False
        if not right_dpad:
            self.last_right_dpad = False

        self.send_elevator(elevator_speed, elevator_direction)

        self.check_drive_enabled(msg)

    def check_drive_enabled(self, msg: Joy):
        start_button = msg.buttons[START]
        back_button = msg.buttons[BACK]

        if back_button:
            # self.drive_enabled = False
            # TODO: HANDLE DRIVE DISABLED
            self.get_logger().info("Drive disabled NOT AVAILABLE")
        elif start_button:
            self.drive_enabled = True
            request = Trigger.Request()
            self.future = self.client.call_async(request)
            self.get_logger().info("Drive enabled")

            # Add a callback for when the future completes
            self.future.add_done_callback(self.service_response_callback)

    def service_response_callback(self, future):
        try:
            response = future.result()
            if response is not None:
                self.get_logger().info("Response: {}".format(response.message))
            else:
                self.get_logger().error("Service call failed with no response.")
        except Exception as e:
            self.get_logger().error(f"Exception while calling service: {e}")


def main(args=None):
    rclpy.init(args=args)
    mega_wrapper = MegaWrapper()
    try:
        rclpy.spin(mega_wrapper)
    except KeyboardInterrupt:
        pass
    finally:
        mega_wrapper.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
