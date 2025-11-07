import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from zed_msgs.msg import ObjectsStamped
from std_srvs.srv import Trigger
from cv_bridge import CvBridge
import cv2

class ObjectDetectionService(Node):
    def __init__(self):
        super().__init__('object_detection_service')
        
        self.bridge = CvBridge()
        self.latest_image = None
        self.latest_objects = None

        self.declare_parameter('object_confidence', 65)
        
        # Declare object label mapping parameters
        self.declare_parameter('object_labels.0', 'Hammer')
        self.declare_parameter('object_labels.1', 'Water Bottle')
        self.declare_parameter('object_labels.2', 'Rock Pick')
        
        # Build label mapping dictionary
        self.object_labels = {
            0: self.get_parameter('object_labels.0').value,
            1: self.get_parameter('object_labels.1').value,
            2: self.get_parameter('object_labels.2').value,
        }
        
        # Subscribe to image and detections
        # Object detection bounding boxes are computed on the left camera image
        self.image_sub = self.create_subscription(
            Image,
            '/zed/zed_node/left/image_rect_color',
            self.image_callback,
            10
        )
        
        self.objects_sub = self.create_subscription(
            ObjectsStamped,
            '/zed/zed_node/obj_det/objects',
            self.objects_callback,
            10
        )
        
        # Publisher for static annotated image
        self.annotated_static_pub = self.create_publisher(
            Image,
            '/annotated_detections_static',
            10
        )
        
        # Service to trigger publishing one image
        self.service = self.create_service(
            Trigger,
            'get_annotated_detection',
            self.service_callback
        )
        
        self.get_logger().info('Object Detection Service ready. Call /get_annotated_detection service to publish annotated image.')
    
    def image_callback(self, msg):
        self.latest_image = msg
    
    def objects_callback(self, msg):
        self.latest_objects = msg
    
    def service_callback(self, request, response):
        """Service callback to publish one annotated image"""
        
        if self.latest_image is None:
            response.success = False
            response.message = "No image available yet"
            self.get_logger().warn("Service called but no image available")
            return response
        
        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
        image_height, image_width = cv_image.shape[:2]
        
        # Draw bounding boxes for each detected object (if available)
        num_objects = 0
        if self.latest_objects is not None:
            for obj in self.latest_objects.objects:
                if obj.confidence > self.get_parameter('object_confidence').value:
                    num_objects += 1
                    # BoundingBox2Di has 4 corners, each corner is a Keypoint2Di with a kp field [x, y]
                    corners = obj.bounding_box_2d.corners
                    
                    # Get all x and y coordinates from the 4 corners
                    # Note: Coordinates might be in grab resolution (HD720=1280x720)
                    # but image is in pub resolution (640x360), so we may need to scale
                    x_coords = [int(corners[i].kp[0]) for i in range(4)]
                    y_coords = [int(corners[i].kp[1]) for i in range(4)]
                    
                    # Calculate min and max to get the bounding rectangle
                    x_min = min(x_coords)
                    y_min = min(y_coords)
                    x_max = max(x_coords)
                    y_max = max(y_coords)
                    
                    # Check if coordinates need scaling (if they exceed image dimensions)
                    # Assuming grab resolution is HD720 (1280x720) and we're at 640x360
                    if x_max > image_width or y_max > image_height:
                        scale_x = image_width / 1280.0
                        scale_y = image_height / 720.0
                        x_min = int(x_min * scale_x)
                        y_min = int(y_min * scale_y)
                        x_max = int(x_max * scale_x)
                        y_max = int(y_max * scale_y)
                    
                    # Draw rectangle
                    cv2.rectangle(cv_image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
                    
                    # Parse class ID from label string (format: "Class ID: X")
                    # The label_id field is the tracking ID, not the class ID
                    class_id = None
                    if obj.label.startswith("Class ID: "):
                        try:
                            class_id = int(obj.label.split(": ")[1])
                        except (ValueError, IndexError):
                            pass
                    
                    # Get human-readable label from mapping, fallback to original label if not found
                    if class_id is not None:
                        object_name = self.object_labels.get(class_id, obj.label)
                    else:
                        object_name = obj.label
                    
                    label = f"{object_name}: {obj.confidence:.1f}%"
                    cv2.putText(cv_image, label, (x_min, y_min - 10),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Publish annotated image with proper header
        annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
        annotated_msg.header = self.latest_image.header  # Preserve timestamp and frame_id
        self.annotated_static_pub.publish(annotated_msg)
        
        response.success = True
        response.message = f"Published annotated image with {num_objects} detected objects"
        self.get_logger().info(response.message)
        
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
