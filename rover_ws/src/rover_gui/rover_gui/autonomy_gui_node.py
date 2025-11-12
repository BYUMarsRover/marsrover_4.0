import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus
from action_msgs.srv import CancelGoal
from rover_interfaces.action import AutonomyMission
from rover_interfaces.msg import AutonomyLeg
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped
from PyQt5.QtWidgets import (
    QApplication,
    QWidget,
    QVBoxLayout,
    QHBoxLayout,
    QLabel,
    QListWidget,
    QListWidgetItem,
    QPushButton,
    QComboBox,
    QLineEdit,
    QDialog,
    QFormLayout,
    QDialogButtonBox,
    QMessageBox,
    QFileDialog,
    QCheckBox,
)
from PyQt5.QtGui import QColor
from PyQt5.QtCore import QThread, pyqtSignal, pyqtSlot, QMetaObject, Qt, Q_ARG
import sys
import json
import os
import utm
import threading
import subprocess

from std_srvs.srv import SetBool, Trigger
from rover_interfaces.srv import (
    GetWaypoints,
    AddWaypoint,
    RemoveWaypoint,
    IsMissionRunning,
    SendWaypoint,
    GetFeedback,
)


class ROS2Thread(QThread):
    """
    Fix for occasional threading crashes

    :author: Nelson Durrant (w Gemini 2.5 Pro)
    :date: Apr 2025
    """

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.close_flag = False

    def run(self):
        # IMPORTANT! This must be run in a separate thread to avoid blocking the GUI
        while not self.close_flag and rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.node.destroy_node()
        rclpy.shutdown()

    def stop(self):
        self.close_flag = True


class WaypointDialog(QDialog):
    """
    Interactive dialog for adding or editing a waypoint

    :author: Nelson Durrant (w Gemini 2.5 Pro)
    :date: Apr 2025
    """

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Add/Edit Waypoint")
        self.layout = QFormLayout()

        # Create form fields
        self.name_edit = QLineEdit()
        self.type_combo = QComboBox()
        self.type_combo.addItems(["gps", "aruco", "obj"])
        self.latitude_edit = QLineEdit("0.0")
        self.longitude_edit = QLineEdit("0.0")
        self.tag_id_combo = QComboBox()
        self.tag_id_combo.addItems(["1", "2", "3"])
        self.object_combo = QComboBox()
        self.object_combo.addItems(["mallet", "bottle"])

        # Add fields to layout
        self.layout.addRow("Name:", self.name_edit)
        self.layout.addRow("Type:", self.type_combo)
        self.layout.addRow("Latitude:", self.latitude_edit)
        self.layout.addRow("Longitude:", self.longitude_edit)
        self.layout.addRow("Tag ID:", self.tag_id_combo)
        self.layout.addRow("Object:", self.object_combo)

        # Update visibility of fields based on selected type
        self.type_combo.currentIndexChanged.connect(self.update_fields_visibility)
        self.update_fields_visibility()

        self.buttons = QDialogButtonBox(QDialogButtonBox.Ok | QDialogButtonBox.Cancel)
        self.buttons.accepted.connect(self.accept)
        self.buttons.rejected.connect(self.reject)
        self.layout.addWidget(self.buttons)

        self.setLayout(self.layout)

    def update_fields_visibility(self):
        waypoint_type = self.type_combo.currentText()
        self.latitude_edit.setVisible(waypoint_type in ["gps", "aruco", "obj"])
        self.longitude_edit.setVisible(waypoint_type in ["gps", "aruco", "obj"])
        self.layout.labelForField(self.latitude_edit).setVisible(
            waypoint_type in ["gps", "aruco", "obj"]
        )
        self.layout.labelForField(self.longitude_edit).setVisible(
            waypoint_type in ["gps", "aruco", "obj"]
        )

        self.tag_id_combo.setVisible(waypoint_type == "aruco")
        self.layout.labelForField(self.tag_id_combo).setVisible(
            waypoint_type == "aruco"
        )

        self.object_combo.setVisible(waypoint_type == "obj")
        self.layout.labelForField(self.object_combo).setVisible(waypoint_type == "obj")

    def get_waypoint_data(self):
        waypoint = {}
        waypoint["name"] = self.name_edit.text()
        waypoint["type"] = self.type_combo.currentText()
        try:
            waypoint["latitude"] = float(self.latitude_edit.text())
            waypoint["longitude"] = float(self.longitude_edit.text())
        except ValueError:
            QMessageBox.critical(
                self, "Error", "Latitude and Longitude must be numbers."
            )
            return None
        if waypoint["type"] == "aruco":
            tag_id_text = self.tag_id_combo.currentText()
            waypoint["tag_id"] = int(tag_id_text)
        elif waypoint["type"] == "obj":
            waypoint["object"] = self.object_combo.currentText()
        return waypoint


class AutonomyGUI(Node, QWidget):
    """
    GUI for starting, stopping, and monitoring the rover's autonomy mission

    NOTE: This is quite the jumble of unorganized code, but Gemini 2.5 Pro one-shotted
    almost all the major functionality in under an hour, so I guess I can't complain too much
    haha. It does seem to work really well tho, and I've tested it a lot.

    :author: Nelson Durrant (w Gemini 2.5 Pro)
    :date: Apr 2025

    Subscribers:
    - /mapviz/clicked_point (geometry_msgs/PointStamped)
    Publishers:
    - /mapviz/preview (visualization_msgs/MarkerArray)
    Clients:
    - /exec_autonomy_mission/_action/cancel_goal (action_msgs/CancelGoal)
    Services:
    - /get_waypoints (rover_interfaces/GetWaypoints)
    - /add_waypoint (rover_interfaces/AddWaypoint)
    - /remove_waypoint (rover_interfaces/RemoveWaypoint)
    - /is_mission_running (rover_interfaces/IsMissionRunning)
    - /send_waypoint (rover_interfaces/SendWaypoint)
    - /send_all_waypoints (std_srvs/Trigger)
    - /get_feedback (rover_interfaces/GetFeedback)
    - /cancel_mission (std_srvs/Trigger)
    - /set_terrain_planning (std_srvs/SetBool)
    Action Clients:
    - exec_autonomy_mission (rover_interfaces/AutonomyMission)
    """

    # Signal to safely update the GUI from the ROS2 thread.
    # It will emit a callable (e.g., a lambda) to be executed in the GUI thread.
    ui_update_signal = pyqtSignal(object)

    def __init__(self):
        Node.__init__(self, "autonomy_gui")
        QWidget.__init__(self)

        self.resize(800, 800)

        # Lock for thread-safe access to shared data (waypoints, goal_handle, etc.)
        self.lock = threading.Lock()

        # Define the default directory for waypoints
        # ~/rover_ws/src/rover_gui/rover_gui/waypoints
        self.default_waypoints_dir = os.path.join(
            os.path.expanduser("~"),
            "rover_ws",
            "src",
            "rover_gui",
            "rover_gui",
            "waypoints",
        )

        self.callback_group = ReentrantCallbackGroup()
        self._action_client = ActionClient(
            self,
            AutonomyMission,
            "/exec_autonomy_mission",
            callback_group=self.callback_group,
        )

        self.preview_pub = self.create_publisher(
            MarkerArray,
            "/mapviz/preview",
            10,
            callback_group=self.callback_group,
        )

        self.mapviz_wp_count = 1
        self.clicked_point_sub = self.create_subscription(
            PointStamped,
            "/mapviz/clicked_point",
            self.mapviz_clicked_point_callback,
            10,
            callback_group=self.callback_group,
        )

        self.waypoints = []
        self.goal_handle = None
        self.close_flag = False
        self.rqt_image_process = None  # Track rqt_image_view process

        self.setWindowTitle("Autonomy Mission GUI")
        self.layout = QVBoxLayout()

        # Waypoint Display
        self.waypoint_label = QLabel("Waypoints:")
        self.layout.addWidget(self.waypoint_label)
        self.waypoint_list = QListWidget()
        self.layout.addWidget(self.waypoint_list)

        # Save/Load Layout
        json_layout = QHBoxLayout()
        self.preview_button = QPushButton("Preview WPs")
        self.preview_button.clicked.connect(self.preview_wps)
        json_layout.addWidget(self.preview_button)

        self.save_button = QPushButton("Save WPs to file")
        self.save_button.clicked.connect(self.save_waypoints_to_file)
        json_layout.addWidget(self.save_button)

        self.load_button = QPushButton("Load WPs from file")
        self.load_button.clicked.connect(self.load_waypoints_from_file)
        json_layout.addWidget(self.load_button)

        # Buttons Layout
        buttons_layout = QHBoxLayout()
        self.add_button = QPushButton("Add")
        self.add_button.clicked.connect(self.open_add_waypoint_dialog)
        buttons_layout.addWidget(self.add_button)

        self.edit_button = QPushButton("Edit")
        self.edit_button.clicked.connect(self.open_edit_waypoint_dialog)
        buttons_layout.addWidget(self.edit_button)

        self.duplicate_button = QPushButton("Duplicate")
        self.duplicate_button.clicked.connect(self.duplicate_waypoint)
        buttons_layout.addWidget(self.duplicate_button)

        self.remove_button = QPushButton("Remove")
        self.remove_button.clicked.connect(self.remove_waypoint)
        buttons_layout.addWidget(self.remove_button)

        self.clear_button = QPushButton("Clear")
        self.clear_button.clicked.connect(self.clear_waypoints)
        buttons_layout.addWidget(self.clear_button)

        # Start/Stop Layout
        mission_layout = QHBoxLayout()
        self.start_selected_button = QPushButton("Send WP")
        self.start_selected_button.clicked.connect(self.start_selected_mission)
        mission_layout.addWidget(self.start_selected_button)

        self.start_button = QPushButton("Send all WPs")
        self.start_button.clicked.connect(self.start_mission)
        mission_layout.addWidget(self.start_button)

        self.stop_button = QPushButton("Cancel Mission")
        self.stop_button.clicked.connect(self.stop_mission)
        mission_layout.addWidget(self.stop_button)

        # Annotated Image Layout
        annotated_image_layout = QHBoxLayout()
        self.get_annotated_image_button = QPushButton("Get Annotated Object Image")
        self.get_annotated_image_button.clicked.connect(self.get_annotated_image)
        annotated_image_layout.addWidget(self.get_annotated_image_button)
        annotated_image_layout.addStretch(1)

        # Object Detection Toggle Layout
        object_detection_layout = QHBoxLayout()
        self.object_detection_checkbox = QCheckBox("Enable Object Detection")
        self.object_detection_checkbox.setChecked(False)
        self.object_detection_checkbox.setToolTip(
            "If checked, the ZED camera will perform real-time object detection."
        )
        self.object_detection_checkbox.stateChanged.connect(self.toggle_object_detection)
        object_detection_layout.addWidget(self.object_detection_checkbox)
        object_detection_layout.addStretch(1)

        # Terrain Planning Layout
        terrain_planning_layout = QHBoxLayout()
        self.terrain_planning_checkbox = QCheckBox("Enable Terrain Path Planning")
        self.terrain_planning_checkbox.setChecked(True)
        self.terrain_planning_checkbox.setToolTip(
            "If checked, the rover will attempt to reference a pre-loaded terrain map for path planning."
        )
        terrain_planning_layout.addWidget(self.terrain_planning_checkbox)
        terrain_planning_layout.addStretch(1)

        self.layout.addLayout(json_layout)
        self.layout.addLayout(buttons_layout)
        self.layout.addLayout(mission_layout)
        self.layout.addLayout(annotated_image_layout)
        self.layout.addLayout(object_detection_layout)
        self.layout.addLayout(terrain_planning_layout)

        # Feedback Display
        self.feedback_label = QLabel("Mission Feedback:")
        self.layout.addWidget(self.feedback_label)
        self.feedback_display = QListWidget()
        self.layout.addWidget(self.feedback_display)

        self.layout.setStretch(0, 1)  # Waypoint Label
        self.layout.setStretch(1, 6)  # Waypoint List
        self.layout.setStretch(2, 1)  # Save/Load Buttons
        self.layout.setStretch(3, 1)  # Edit/Duplicate/Remove/Clear Buttons
        self.layout.setStretch(4, 1)  # Start/Stop Buttons
        self.layout.setStretch(5, 1)  # Annotated Image Layout
        self.layout.setStretch(6, 1)  # Object Detection Layout
        self.layout.setStretch(7, 1)  # Terrain Planning Layout
        self.layout.setStretch(8, 1)  # Feedback Label
        self.layout.setStretch(9, 20)  # Feedback Display

        self.setLayout(self.layout)

        # Connect the UI update signal to its slot
        self.ui_update_signal.connect(self.do_ui_update)

        ########################
        # MCP GUI INTEGRATIONS #
        ########################

        self.create_service(
            GetWaypoints,
            "~/get_waypoints",
            self.get_waypoints_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            AddWaypoint,
            "~/add_waypoint",
            self.add_waypoint_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            RemoveWaypoint,
            "~/remove_waypoint",
            self.remove_waypoint_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            IsMissionRunning,
            "~/is_mission_running",
            self.is_mission_running_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            SetBool,
            "~/set_terrain_planning",
            self.set_terrain_planning_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            SendWaypoint,
            "~/send_waypoint",
            self.send_waypoint_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            "~/send_all_waypoints",
            self.send_all_waypoints_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            GetFeedback,
            "~/get_feedback",
            self.get_feedback_callback,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            "~/cancel_mission",
            self.cancel_mission_callback,
            callback_group=self.callback_group,
        )
        self.get_logger().info("MCP GUI integration services are running.")

        ############################
        # END MCP GUI INTEGRATIONS #
        ############################

        self.ros2_thread = ROS2Thread(self)
        self.ros2_thread.start()

        self.load_default_waypoints()
        self.update_button_states()

    @pyqtSlot(object)
    def do_ui_update(self, func):
        """
        This slot executes a function passed from the ROS2 thread,
        ensuring it runs safely in the main GUI thread.
        """
        func()

    def is_waypoint_valid(self, leg_data):
        """Checks if a waypoint dictionary has all the required fields and valid types/bounds."""
        if not isinstance(leg_data, dict):
            return False, "Waypoint entry is not a valid dictionary."

        # Check for presence of base fields
        base_fields = ["name", "type", "latitude", "longitude"]
        for field in base_fields:
            if field not in leg_data:
                return (
                    False,
                    f"Waypoint '{leg_data.get('name', 'N/A')}' is missing required field: '{field}'.",
                )

        # Check name
        wp_name = leg_data.get("name", "").strip()
        if not wp_name:
            return False, "Waypoint name cannot be empty."

        # Check latitude and longitude bounds
        try:
            lat = float(leg_data["latitude"])
            if not -90.0 <= lat <= 90.0:
                return (
                    False,
                    f"Latitude for '{wp_name}' ({lat}) is out of bounds [-90, 90].",
                )

            lon = float(leg_data["longitude"])
            if not -180.0 <= lon <= 180.0:
                return (
                    False,
                    f"Longitude for '{wp_name}' ({lon}) is out of bounds [-180, 180].",
                )
        except (ValueError, TypeError):
            return (
                False,
                f"Waypoint '{wp_name}' has invalid latitude/longitude. They must be numbers.",
            )

        # Check type-specific fields and bounds
        wp_type = leg_data.get("type")
        if wp_type == "aruco":
            if "tag_id" not in leg_data:
                return (
                    False,
                    f"Aruco waypoint '{wp_name}' is missing required field: 'tag_id'.",
                )
            try:
                tag_id = int(leg_data["tag_id"])
                if tag_id not in [1, 2, 3]:
                    return (
                        False,
                        f"Tag ID for '{wp_name}' must be 1, 2, or 3, but was {tag_id}.",
                    )
            except (ValueError, TypeError):
                return False, f"Tag ID for '{wp_name}' must be an integer."
        elif wp_type == "obj":
            if "object" not in leg_data:
                return (
                    False,
                    f"Object waypoint '{wp_name}' is missing required field: 'object'.",
                )
            obj_name = leg_data.get("object", "").strip()
            if obj_name not in ["mallet", "bottle"]:
                return (
                    False,
                    f"Object for '{wp_name}' must be 'mallet' or 'bottle', but was '{obj_name}'.",
                )
        elif wp_type != "gps":
            return False, f"Waypoint '{wp_name}' has an unknown type: '{wp_type}'."

        return True, ""

    def load_default_waypoints(self):
        default_file_path = os.path.join(
            self.default_waypoints_dir, "sim_waypoints.json"
        )
        self.get_logger().info(
            f"Attempting to load default waypoints from: {default_file_path}"
        )

        if not os.path.exists(default_file_path):
            self.get_logger().warn(
                f"Default waypoints file not found at {default_file_path}. No default waypoints will be loaded."
            )
            return

        try:
            with open(default_file_path, "r") as f:
                default_waypoints_data = json.load(f)

            with self.lock:
                legs_data = default_waypoints_data.get("legs", [])

                # Validate each waypoint before proceeding
                for leg in legs_data:
                    is_valid, error_msg = self.is_waypoint_valid(leg)
                    if not is_valid:
                        self.get_logger().error(
                            f"Invalid data in default waypoints file: {error_msg}. Aborting load."
                        )
                        return

                # Check for duplicate names
                names_in_file = [
                    leg.get("name") for leg in legs_data if leg.get("name")
                ]
                if len(names_in_file) != len(set(names_in_file)):
                    self.get_logger().error(
                        f"Default waypoints file '{default_file_path}' contains duplicate names. Aborting load."
                    )
                    return

                self.waypoints = legs_data
                if self.waypoints:
                    self.get_logger().info(
                        f"Successfully loaded {len(self.waypoints)} default waypoints from {default_file_path}."
                    )

            self.ui_update_signal.emit(self.update_waypoint_list)

        except Exception as e:
            self.get_logger().error(
                f"Error loading default waypoints from {default_file_path}: {e}"
            )

    def format_feedback_text(self, text):
        item = QListWidgetItem(text)
        if "[SUCCESS]" in text:
            item.setForeground(QColor("green"))
        elif "[FATAL]" in text or "[ERROR]" in text:
            item.setForeground(QColor("red"))
        elif "[WARN]" in text:
            item.setForeground(QColor("orange"))
        elif "One small step" in text:
            item.setForeground(QColor("blue"))
        return item

    def closeEvent(self, event):
        self.close_flag = True
        self.stop_mission()
        
        # Clean up rqt_image_view process if running
        if self.rqt_image_process is not None and self.rqt_image_process.poll() is None:
            self.rqt_image_process.terminate()
            try:
                self.rqt_image_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.rqt_image_process.kill()
        
        self.ros2_thread.stop()
        self.ros2_thread.wait()
        event.accept()

    def update_button_states(self):
        with self.lock:
            mission_running = (
                self.goal_handle is not None
                and self.goal_handle.status == GoalStatus.STATUS_EXECUTING
            )
            self.edit_button.setEnabled(len(self.waypoints) > 0)
            self.duplicate_button.setEnabled(len(self.waypoints) > 0)
            self.remove_button.setEnabled(len(self.waypoints) > 0)
            self.clear_button.setEnabled(len(self.waypoints) > 0)
            self.start_button.setEnabled(
                not mission_running and len(self.waypoints) > 0
            )
            self.start_selected_button.setEnabled(
                not mission_running and len(self.waypoints) > 0
            )
            self.stop_button.setEnabled(mission_running)
            self.preview_button.setEnabled(len(self.waypoints) > 0)
            self.save_button.setEnabled(len(self.waypoints) > 0)

    def open_add_waypoint_dialog(self):
        self.get_logger().info("Adding waypoint...")
        dialog = WaypointDialog(self)
        if dialog.exec_() == QDialog.Accepted:
            waypoint_data = dialog.get_waypoint_data()
            if waypoint_data:
                is_valid, error_msg = self.is_waypoint_valid(waypoint_data)
                if not is_valid:
                    QMessageBox.critical(self, "Invalid Waypoint Data", error_msg)
                    return

                with self.lock:
                    existing_names = {wp["name"] for wp in self.waypoints}
                    if waypoint_data["name"] in existing_names:
                        QMessageBox.critical(
                            self,
                            "Error",
                            f"A waypoint with the name '{waypoint_data['name']}' already exists.",
                        )
                        return
                    self.waypoints.append(waypoint_data)
                self.update_waypoint_list()
        self.update_button_states()

    def open_edit_waypoint_dialog(self):
        self.get_logger().info("Editing waypoint...")
        with self.lock:
            selected_item = self.waypoint_list.currentItem()
            if not selected_item:
                return
            index = self.waypoint_list.row(selected_item)
            waypoint_data = self.waypoints[index]

        dialog = WaypointDialog(self)
        dialog.name_edit.setText(waypoint_data["name"])
        dialog.type_combo.setCurrentText(waypoint_data["type"])
        dialog.latitude_edit.setText(str(waypoint_data["latitude"]))
        dialog.longitude_edit.setText(str(waypoint_data["longitude"]))
        if waypoint_data["type"] == "aruco":
            dialog.tag_id_combo.setCurrentText(str(waypoint_data.get("tag_id", 0)))
        elif waypoint_data["type"] == "obj":
            dialog.object_combo.setCurrentText(waypoint_data.get("object", ""))

        dialog.update_fields_visibility()

        if dialog.exec_() == QDialog.Accepted:
            updated_waypoint_data = dialog.get_waypoint_data()
            if updated_waypoint_data:
                is_valid, error_msg = self.is_waypoint_valid(updated_waypoint_data)
                if not is_valid:
                    QMessageBox.critical(self, "Invalid Waypoint Data", error_msg)
                    return

                with self.lock:
                    existing_names = {
                        wp["name"] for i, wp in enumerate(self.waypoints) if i != index
                    }
                    if updated_waypoint_data["name"] in existing_names:
                        QMessageBox.critical(
                            self,
                            "Error",
                            f"A waypoint with the name '{updated_waypoint_data['name']}' already exists.",
                        )
                        return
                    self.waypoints[index] = updated_waypoint_data
                self.update_waypoint_list()
                self.waypoint_list.setCurrentRow(index)
        self.update_button_states()

    def duplicate_waypoint(self):
        self.get_logger().info("Duplicating waypoint...")
        with self.lock:
            selected_item = self.waypoint_list.currentItem()
            if not selected_item:
                return

            index = self.waypoint_list.row(selected_item)
            waypoint_data = self.waypoints[index]
            new_waypoint_data = waypoint_data.copy()

            # Find a unique name
            base_name = waypoint_data["name"]
            new_name_candidate = f"{base_name}_copy"
            counter = 1
            existing_names = {wp["name"] for wp in self.waypoints}
            while new_name_candidate in existing_names:
                counter += 1
                new_name_candidate = f"{base_name}_copy_{counter}"
            new_waypoint_data["name"] = new_name_candidate

            self.waypoints.insert(index + 1, new_waypoint_data)

        self.update_waypoint_list()
        self.waypoint_list.setCurrentRow(index + 1)
        self.update_button_states()

    def remove_waypoint(self):
        self.get_logger().info("Removing waypoint...")
        with self.lock:
            selected_item = self.waypoint_list.currentItem()
            if not selected_item:
                return

            index = self.waypoint_list.row(selected_item)
            del self.waypoints[index]

            new_index = -1
            if index < len(self.waypoints):
                new_index = index
            elif len(self.waypoints) > 0:
                new_index = index - 1

        self.update_waypoint_list()
        if new_index != -1:
            self.waypoint_list.setCurrentRow(new_index)
        self.update_button_states()

    def clear_waypoints(self):
        self.get_logger().info("Clearing waypoints...")
        reply = QMessageBox.question(
            self,
            "Clear Waypoints",
            "Are you sure you want to clear all waypoints?",
            QMessageBox.Yes | QMessageBox.No,
            QMessageBox.No,
        )
        if reply == QMessageBox.Yes:
            with self.lock:
                self.waypoints = []
            self.update_waypoint_list()
            self.update_button_states()

    def update_waypoint_list(self):
        with self.lock:
            current_row = self.waypoint_list.currentRow()
            self.waypoint_list.clear()
            for waypoint in self.waypoints:
                self.waypoint_list.addItem(str(waypoint))

            if self.waypoint_list.count() > 0:
                if 0 <= current_row < self.waypoint_list.count():
                    self.waypoint_list.setCurrentRow(current_row)
                else:
                    self.waypoint_list.setCurrentRow(0)
            self.waypoint_list.scrollToBottom()

    def save_waypoints_to_file(self):
        self.get_logger().info("Saving waypoints to file...")
        with self.lock:
            if not self.waypoints:
                QMessageBox.information(
                    self, "No Waypoints", "There are no waypoints to save."
                )
                return
            waypoints_copy = self.waypoints.copy()

        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getSaveFileName(
            self,
            "Save Waypoints File",
            self.default_waypoints_dir,
            "JSON Files (*.json);;All Files (*)",
            options=options,
        )
        if fileName:
            if not fileName.endswith(".json"):
                fileName += ".json"
            try:
                with open(fileName, "w") as f:
                    json.dump({"legs": waypoints_copy}, f, indent=4)
                self.get_logger().info(f"Waypoints saved to {fileName}")
                self.default_waypoints_dir = os.path.dirname(fileName)
            except Exception as e:
                self.get_logger().error(f"Error saving waypoints: {e}")
                QMessageBox.critical(
                    self, "Error", f"Could not save waypoints to file: {e}"
                )

    def load_waypoints_from_file(self):
        self.get_logger().info("Loading waypoints from file...")
        options = QFileDialog.Options()
        options |= QFileDialog.DontUseNativeDialog
        fileName, _ = QFileDialog.getOpenFileName(
            self,
            "Load Waypoints File",
            self.default_waypoints_dir,
            "JSON Files (*.json);;All Files (*)",
            options=options,
        )
        if fileName:
            try:
                with open(fileName, "r") as f:
                    data = json.load(f)

                with self.lock:
                    if "legs" in data and isinstance(data["legs"], list):
                        # Validate each waypoint in the file before proceeding
                        for leg in data["legs"]:
                            is_valid, error_msg = self.is_waypoint_valid(leg)
                            if not is_valid:
                                QMessageBox.critical(
                                    self,
                                    "Invalid Waypoint Data",
                                    f"Error in file '{os.path.basename(fileName)}':\n\n{error_msg}",
                                )
                                return

                        names_in_file = [
                            leg.get("name") for leg in data["legs"] if leg.get("name")
                        ]
                        if len(names_in_file) != len(set(names_in_file)):
                            QMessageBox.critical(
                                self,
                                "Error",
                                "The selected file contains duplicate waypoint names. Please fix the file and try again.",
                            )
                            return

                        self.waypoints = data["legs"]
                        self.get_logger().info(
                            f"Loaded {len(self.waypoints)} waypoints from {fileName}"
                        )
                        self.default_waypoints_dir = os.path.dirname(fileName)
                    else:
                        raise ValueError(
                            "Invalid waypoint file format. 'legs' array not found."
                        )

                self.ui_update_signal.emit(self.update_waypoint_list)
                self.ui_update_signal.emit(self.update_button_states)

            except Exception as e:
                self.get_logger().error(f"Error loading waypoints: {e}")
                QMessageBox.critical(
                    self, "Error", f"Could not load waypoints from file: {e}"
                )

    def preview_wps(self):
        self.get_logger().info("Previewing waypoints...")
        with self.lock:
            if not self.waypoints:
                QMessageBox.warning(self, "Warning", "No waypoints to preview.")
                return
            waypoints_copy = self.waypoints.copy()

        # Clear previous markers
        clear_marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.header.frame_id = "utm"
        clear_marker.header.stamp = self.get_clock().now().to_msg()
        clear_marker.ns = "preview"
        clear_marker.id = 0
        clear_marker.type = Marker.CUBE
        clear_marker.action = Marker.DELETEALL
        clear_marker.lifetime.sec = 0
        clear_marker.lifetime.nanosec = 0
        clear_marker_array.markers.append(clear_marker)
        self.preview_pub.publish(clear_marker_array)

        marker_array = MarkerArray()
        for i, wp in enumerate(waypoints_copy):
            # convert from lat/lon to UTM
            utm_coords = utm.from_latlon(wp["latitude"], wp["longitude"])

            marker = Marker()
            marker.header.frame_id = "utm"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "preview"
            marker.id = i
            marker.type = Marker.TEXT_VIEW_FACING
            marker.action = Marker.ADD
            marker.pose.position.x = utm_coords[0]
            marker.pose.position.y = utm_coords[1]
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 1.0
            marker.scale.y = 1.0
            marker.scale.z = 1.0
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 1.0
            marker.color.a = 1.0
            marker.text = f"{wp['name']}"
            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 0
            marker_array.markers.append(marker)
        self.preview_pub.publish(marker_array)

    def mapviz_clicked_point_callback(self, msg):
        self.get_logger().info(
            f"Received clicked point from mapviz: {msg.point.x}, {msg.point.y}"
        )
        lat, lon = msg.point.y, msg.point.x
        self.ui_update_signal.emit(lambda: self.add_mapviz_waypoint(lat, lon))

    def add_mapviz_waypoint(self, lat, lon):
        with self.lock:
            existing_names = {wp["name"] for wp in self.waypoints}
            name_candidate = f"mapviz{self.mapviz_wp_count}"
            while name_candidate in existing_names:
                self.mapviz_wp_count += 1
                name_candidate = f"mapviz{self.mapviz_wp_count}"

            waypoint_data = {
                "name": name_candidate,
                "type": "gps",
                "latitude": lat,
                "longitude": lon,
            }
            self.waypoints.append(waypoint_data)
            self.mapviz_wp_count += 1

        self.update_waypoint_list()
        self.update_button_states()

    def start_selected_mission(self):
        self.get_logger().info("Starting selected mission...")
        with self.lock:
            selected_item = self.waypoint_list.currentItem()
            if not selected_item:
                QMessageBox.warning(self, "Warning", "No waypoint selected to send.")
                return

            current_selection_index = self.waypoint_list.row(selected_item)
            if not (0 <= current_selection_index < len(self.waypoints)):
                QMessageBox.warning(
                    self, "Warning", "Invalid waypoint selection or list out of sync."
                )
                self.get_logger().error(
                    f"Invalid selection index: {current_selection_index} for waypoints len: {len(self.waypoints)}. "
                    f"Selected item text: {selected_item.text()}"
                )
                return

            waypoints_to_send = [self.waypoints[current_selection_index]]

        self._send_mission(waypoints_to_send)

    def start_mission(self):
        self.get_logger().info("Starting mission...")
        with self.lock:
            if not self.waypoints:
                QMessageBox.warning(self, "Warning", "No waypoints to send.")
                return
            waypoints_to_send = self.waypoints.copy()

        self._send_mission(waypoints_to_send)

    def _send_mission(self, waypoints_to_send):
        """Internal method to send a mission with a given list of waypoints."""
        if not self._action_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server not available after 2 seconds!")
            self.ui_update_signal.emit(
                lambda: QMessageBox.critical(
                    self, "Error", "Action server not available!"
                )
            )
            return

        with self.lock:
            goal_msg = AutonomyMission.Goal()
            goal_msg.enable_terrain = self.terrain_planning_checkbox.isChecked()
            goal_msg.legs = []
            for wp in waypoints_to_send:
                leg_msg = AutonomyLeg()
                leg_msg.name = wp.get("name", "")
                leg_msg.type = wp.get("type", "")
                leg_msg.latitude = float(wp.get("latitude", 0.0))
                leg_msg.longitude = float(wp.get("longitude", 0.0))

                if leg_msg.type == "aruco":
                    leg_msg.tag_id = int(wp.get("tag_id", 0))
                    leg_msg.object = ""
                elif leg_msg.type == "obj":
                    leg_msg.object = wp.get("object", "")
                    leg_msg.tag_id = 0
                else:
                    leg_msg.tag_id = 0
                    leg_msg.object = ""
                goal_msg.legs.append(leg_msg)

            self.sent_waypoints = waypoints_to_send

        self.ui_update_signal.emit(lambda: self.feedback_display.clear())
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.goal_feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)
        self.ui_update_signal.emit(self.update_button_states)

    def goal_response_callback(self, future):
        with self.lock:
            self.goal_handle = future.result()
        self.ui_update_signal.emit(self.handle_goal_response)

    def handle_goal_response(self):
        with self.lock:
            if not self.goal_handle.accepted:
                item = self.format_feedback_text(
                    "[ERROR] [gui] Goal rejected by action server"
                )
                self.feedback_display.addItem(item)
            else:
                self.feedback_display.addItem(
                    self.format_feedback_text("[gui] Goal accepted by action server:")
                )
                for wp in self.sent_waypoints:
                    self.feedback_display.addItem(" - " + str(wp))
                self.feedback_display.addItem(
                    self.format_feedback_text(
                        " - Terrain Path Planning: "
                        f"{'ENABLED' if self.terrain_planning_checkbox.isChecked() else 'DISABLED'}"
                    )
                )
                get_result_future = self.goal_handle.get_result_async()
                get_result_future.add_done_callback(self.get_result_callback)
        self.feedback_display.scrollToBottom()
        self.update_button_states()

    def goal_feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback.status
        self.ui_update_signal.emit(lambda: self.handle_goal_feedback(feedback))

    def handle_goal_feedback(self, feedback):
        formatted_text = self.format_feedback_text(f"{feedback}")
        self.feedback_display.addItem(formatted_text)
        self.feedback_display.scrollToBottom()
        self.update_button_states()

    def get_result_callback(self, future):
        result_msg = future.result().result.msg
        self.ui_update_signal.emit(lambda: self.handle_get_result(result_msg))

    def handle_get_result(self, result_msg):
        with self.lock:
            formatted_text = self.format_feedback_text(f"[gui] {result_msg}")
            self.feedback_display.addItem(formatted_text)
            self.goal_handle = None
        self.feedback_display.scrollToBottom()
        self.update_button_states()

    def stop_mission(self):
        self.get_logger().info("Stopping mission...")
        client = self.create_client(
            CancelGoal,
            "/exec_autonomy_mission/_action/cancel_goal",
            callback_group=self.callback_group,
        )

        if not client.wait_for_service(timeout_sec=2.0):
            if not self.close_flag:
                self.get_logger().error(
                    "CancelGoal service not available after 2 seconds!"
                )
                self.ui_update_signal.emit(
                    lambda: QMessageBox.critical(
                        self, "Error", "CancelGoal service not available!"
                    )
                )
            else:
                self.get_logger().warn(
                    "CancelGoal service not available after 2 seconds!"
                )
            return

        request = CancelGoal.Request()
        future = client.call_async(request)
        future.add_done_callback(self.cancel_response_callback)
        self.ui_update_signal.emit(self.update_button_states)

    def cancel_response_callback(self, future):
        """ROS2 Callback: Handles the response from a cancel request."""
        response = future.result()
        self.ui_update_signal.emit(lambda: self.handle_cancel_response(response))

    def handle_cancel_response(self, response):
        """Helper method to display cancel response. Runs in GUI thread."""
        if response.return_code == CancelGoal.Response.ERROR_NONE:
            self.feedback_display.addItem(
                self.format_feedback_text("[gui] Cancel request sent successfully")
            )
            with self.lock:
                self.goal_handle = None
        else:
            self.feedback_display.addItem(
                self.format_feedback_text("[ERROR] [gui] Failed to send cancel request")
            )
        self.feedback_display.scrollToBottom()
        self.update_button_states()

    def get_annotated_image(self):
        """Call the service to get annotated object image and display it in rqt_image_view"""
        self.get_logger().info("Requesting annotated object image...")
        
        # First, launch rqt_image_view to display the image
        try:
            # Kill any existing rqt_image_view process
            if self.rqt_image_process is not None and self.rqt_image_process.poll() is None:
                self.rqt_image_process.terminate()
                self.rqt_image_process.wait(timeout=2)
            
            # Launch rqt_image_view using ros2 run
            # This ensures it works even if rqt_image_view isn't directly in PATH
            self.rqt_image_process = subprocess.Popen(
                ['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/annotated_detections_static'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.get_logger().info("Launched rqt_image_view for /annotated_detections_static")
            
            # Give rqt_image_view a moment to start up
            import time
            time.sleep(0.5)
            
        except Exception as e:
            self.get_logger().error(f"Failed to launch rqt_image_view: {e}")
            QMessageBox.warning(
                self, "Warning", f"Failed to launch image viewer: {e}\n\nContinuing with service call..."
            )
        
        # Now call the service to publish the annotated image
        client = self.create_client(
            Trigger,
            "/get_annotated_detection",
            callback_group=self.callback_group,
        )

        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                "Annotated image service not available after 2 seconds!"
            )
            self.ui_update_signal.emit(
                lambda: QMessageBox.critical(
                    self, "Error", "Annotated image service not available!"
                )
            )
            return

        request = Trigger.Request()
        future = client.call_async(request)
        future.add_done_callback(self.annotated_image_response_callback)

    def annotated_image_response_callback(self, future):
        """ROS2 Callback: Handles the response from annotated image request."""
        try:
            response = future.result()
            self.ui_update_signal.emit(
                lambda: self.handle_annotated_image_response(response)
            )
        except Exception as e:
            self.get_logger().error(f"Annotated image service call failed: {e}")
            self.ui_update_signal.emit(
                lambda: QMessageBox.critical(
                    self, "Error", f"Annotated image service failed: {e}"
                )
            )

    def handle_annotated_image_response(self, response):
        """Helper method to display annotated image response. Runs in GUI thread."""
        if response.success:
            self.feedback_display.addItem(
                self.format_feedback_text(f"[SUCCESS] [gui] {response.message}")
            )
            QMessageBox.information(
                self, "Annotated Image", response.message
            )
        else:
            self.feedback_display.addItem(
                self.format_feedback_text(f"[ERROR] [gui] {response.message}")
            )
            QMessageBox.warning(
                self, "Annotated Image", response.message
            )
        self.feedback_display.scrollToBottom()

    def toggle_object_detection(self, state):
        """Toggle object detection on/off via service call"""
        enable = (state == 2)  # Qt.Checked == 2
        self.get_logger().info(f"Toggling object detection: {enable}")
        
        client = self.create_client(
            SetBool,
            "/zed/zed_node/enable_obj_det",
            callback_group=self.callback_group,
        )

        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().error(
                "Object detection toggle service not available after 2 seconds!"
            )
            self.ui_update_signal.emit(
                lambda: QMessageBox.critical(
                    self, "Error", "Object detection toggle service not available!"
                )
            )
            # Revert checkbox state
            self.ui_update_signal.emit(
                lambda: self.object_detection_checkbox.setChecked(not enable)
            )
            return

        request = SetBool.Request()
        request.data = enable
        future = client.call_async(request)
        future.add_done_callback(
            lambda f: self.object_detection_toggle_response_callback(f, enable)
        )

    def object_detection_toggle_response_callback(self, future, requested_state):
        """ROS2 Callback: Handles the response from object detection toggle request."""
        try:
            response = future.result()
            self.ui_update_signal.emit(
                lambda: self.handle_object_detection_toggle_response(response, requested_state)
            )
        except Exception as e:
            self.get_logger().error(f"Object detection toggle service call failed: {e}")
            self.ui_update_signal.emit(
                lambda: QMessageBox.critical(
                    self, "Error", f"Object detection toggle failed: {e}"
                )
            )
            # Revert checkbox state
            self.ui_update_signal.emit(
                lambda: self.object_detection_checkbox.setChecked(not requested_state)
            )

    def handle_object_detection_toggle_response(self, response, requested_state):
        """Helper method to display object detection toggle response. Runs in GUI thread."""
        if response.success:
            state_str = "enabled" if requested_state else "disabled"
            self.feedback_display.addItem(
                self.format_feedback_text(f"[SUCCESS] [gui] Object detection {state_str}")
            )
        else:
            # Service returns success=True even if already in that state
            # Only revert checkbox if there was an actual error
            if "already" not in response.message.lower():
                self.feedback_display.addItem(
                    self.format_feedback_text(f"[ERROR] [gui] Failed to toggle object detection: {response.message}")
                )
                QMessageBox.warning(
                    self, "Object Detection Toggle", f"Failed: {response.message}"
                )
                # Revert checkbox state on failure
                self.object_detection_checkbox.setChecked(not requested_state)
            else:
                # Already in the requested state - just log it
                self.feedback_display.addItem(
                    self.format_feedback_text(f"[gui] Object detection already {state_str}")
                )
        self.feedback_display.scrollToBottom()

    ########################
    # MCP GUI INTEGRATIONS #
    ########################

    def get_waypoints_callback(self, request, response):
        with self.lock:
            self.get_logger().info("Servicing request for all waypoints...")
            for wp_dict in self.waypoints:
                leg = AutonomyLeg()
                leg.name = wp_dict.get("name", "")
                leg.type = wp_dict.get("type", "")
                leg.latitude = float(wp_dict.get("latitude", 0.0))
                leg.longitude = float(wp_dict.get("longitude", 0.0))
                if leg.type == "aruco":
                    leg.tag_id = int(wp_dict.get("tag_id", 0))
                elif leg.type == "obj":
                    leg.object = wp_dict.get("object", "")
                response.legs.append(leg)
        return response

    def add_waypoint_callback(self, request, response):
        self.get_logger().info(f"Servicing request to add waypoint: {request.leg.name}")
        wp_data = {
            "name": request.leg.name,
            "type": request.leg.type,
            "latitude": request.leg.latitude,
            "longitude": request.leg.longitude,
        }
        if request.leg.type == "aruco":
            wp_data["tag_id"] = request.leg.tag_id
        elif request.leg.type == "obj":
            wp_data["object"] = request.leg.object

        is_valid, error_msg = self.is_waypoint_valid(wp_data)
        if not is_valid:
            response.success = False
            response.message = error_msg
            self.get_logger().warn(f"Invalid waypoint data from service: {error_msg}")
            return response

        with self.lock:
            existing_names = {wp["name"] for wp in self.waypoints}
            if wp_data["name"] in existing_names:
                response.success = False
                response.message = (
                    f"Waypoint with name '{wp_data['name']}' already exists."
                )
                self.get_logger().warn(response.message)
                return response

            self.waypoints.append(wp_data)

        self.ui_update_signal.emit(self.update_waypoint_list)
        self.ui_update_signal.emit(self.update_button_states)
        response.success = True
        response.message = "Waypoint added successfully."
        return response

    def remove_waypoint_callback(self, request, response):
        self.get_logger().info(f"Servicing request to remove waypoint: {request.name}")
        with self.lock:
            original_len = len(self.waypoints)
            self.waypoints = [
                wp for wp in self.waypoints if wp.get("name") != request.name
            ]
            if len(self.waypoints) < original_len:
                response.success = True
                response.message = f"Waypoint '{request.name}' removed."
                self.get_logger().info(response.message)
                self.ui_update_signal.emit(self.update_waypoint_list)
                self.ui_update_signal.emit(self.update_button_states)
            else:
                response.success = False
                response.message = f"Waypoint '{request.name}' not found."
                self.get_logger().warn(response.message)
        return response

    def is_mission_running_callback(self, request, response):
        with self.lock:
            response.is_running = (
                self.goal_handle is not None
                and self.goal_handle.status == GoalStatus.STATUS_EXECUTING
            )
        return response

    def set_terrain_planning_callback(self, request, response):
        self.get_logger().info(
            f"Servicing request to set terrain planning to: {request.data}"
        )
        # This must be run in the GUI thread.
        self.ui_update_signal.emit(
            lambda: self.terrain_planning_checkbox.setChecked(request.data)
        )
        response.success = True
        response.message = f"Terrain planning set to {request.data}"
        return response

    def send_waypoint_callback(self, request, response):
        self.get_logger().info(
            f"Servicing request to send waypoint by name: {request.name}"
        )
        with self.lock:
            if (
                self.goal_handle is not None
                and self.goal_handle.status == GoalStatus.STATUS_EXECUTING
            ):
                response.success = False
                response.message = "A mission is already running."
                self.get_logger().warn(response.message)
                return response

            waypoint_to_send = None
            for wp in self.waypoints:
                if wp.get("name") == request.name:
                    waypoint_to_send = wp
                    break

            if waypoint_to_send is None:
                response.success = False
                response.message = f"Waypoint '{request.name}' not found."
                self.get_logger().warn(response.message)
                return response

            waypoints_to_send_list = [waypoint_to_send]

        self._send_mission(waypoints_to_send_list)
        response.success = True
        response.message = "Mission sent successfully."
        return response

    def send_all_waypoints_callback(self, request, response):
        self.get_logger().info("Servicing request to send all waypoints.")
        with self.lock:
            if (
                self.goal_handle is not None
                and self.goal_handle.status == GoalStatus.STATUS_EXECUTING
            ):
                response.success = False
                response.message = "A mission is already running."
                self.get_logger().warn(response.message)
                return response

            if not self.waypoints:
                response.success = False
                response.message = "No waypoints to send."
                self.get_logger().warn(response.message)
                return response

            waypoints_to_send = self.waypoints.copy()

        self._send_mission(waypoints_to_send)
        response.success = True
        response.message = "Mission sent successfully."
        return response

    def get_feedback_callback(self, request, response):
        self.get_logger().info("Servicing request to get feedback log.")
        feedback_list = []

        def get_text_from_gui():
            nonlocal feedback_list
            for i in range(self.feedback_display.count()):
                feedback_list.append(self.feedback_display.item(i).text())

        # Blocking call to safely get data from the GUI thread
        QMetaObject.invokeMethod(
            self,
            "do_ui_update",
            Qt.BlockingQueuedConnection,
            Q_ARG(object, get_text_from_gui),
        )

        response.feedback_log = feedback_list
        return response

    def cancel_mission_callback(self, request, response):
        self.get_logger().info("Servicing request to cancel current mission.")
        with self.lock:
            if (
                self.goal_handle is None
                or self.goal_handle.status != GoalStatus.STATUS_EXECUTING
            ):
                response.success = False
                response.message = "No mission is currently running to cancel."
                self.get_logger().warn(response.message)
                return response

        self.stop_mission()
        response.success = True
        response.message = "Cancel request sent."
        return response

    ############################
    # END MCP GUI INTEGRATIONS #
    ############################


def main(args=None):
    rclpy.init(args=args)
    app = QApplication(sys.argv)
    gui = AutonomyGUI()
    gui.show()
    exit_code = app.exec_()
    sys.exit(exit_code)


if __name__ == "__main__":
    main()
