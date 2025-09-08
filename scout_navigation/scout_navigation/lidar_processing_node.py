#############3working####################
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
import cv2
import numpy as np
from PIL import Image as PILImage
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
import struct
from builtin_interfaces.msg import Duration
from ultralytics import YOLO
import sys

# Ensure the path to sort.py is added
sys.path.append('/home/imad/scout_navigation_ws/src/scout_navigation/scout_navigation/sort')
from sort import Sort

class LidarProcessingNode(Node):
    def __init__(self):
        super().__init__('lidar_processing_node')

        self.bridge = CvBridge()
        self.marker_lifetime = Duration(sec=1)
        self.current_marker_ids = set()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.image_subscription = self.create_subscription(
            Image,
            '/ouster/signal_image',
            self.image_callback,
            qos_profile)

        self.pc_subscription = self.create_subscription(
            PointCloud2,
            '/ouster/points',
            self.point_cloud_callback,
            qos_profile
        )

        # Initialize YOLOv8 model and specify to detect only the "person" class (class index 0)
        self.model = YOLO('yolov8m.pt')
        self.model.overrides = {'classes': [0]}  # 0 corresponds to the "person" class in COCO dataset

        self.tracker = Sort(max_age=1, min_hits=3, iou_threshold=0.3)

        self.annotated_image_pub = self.create_publisher(Image, '/annotated_image', 10)
        self.bbox_pub = self.create_publisher(MarkerArray, '/ouster/bounding_boxes_3d', 10)

        self.latest_point_cloud = None

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            min_val, max_val, _, _ = cv2.minMaxLoc(cv_image)
            cv_image_8bit = cv2.convertScaleAbs(cv_image, alpha=(255.0 / max_val))
            cv_image_bgr = cv2.cvtColor(cv_image_8bit, cv2.COLOR_GRAY2BGR)

            results = self.model_infer(cv_image_bgr)

            # Get bounding box coordinates and tracking results
            dets = np.array([[*box.xyxy[0].tolist(), box.conf[0]] for box in results]) if len(results) > 0 else np.empty((0, 5))
            tracked_objects = self.tracker.update(dets)

            annotated_image = self.overlay_results(cv_image_bgr, tracked_objects)

            annotated_image_msg = self.bridge.cv2_to_imgmsg(annotated_image, encoding="bgr8")
            self.annotated_image_pub.publish(annotated_image_msg)

            if self.latest_point_cloud is not None:
                self.project_bboxes_to_3d(tracked_objects, self.latest_point_cloud, msg.header)
        except Exception as e:
            print(f"Error in image_callback: {e}")

    def point_cloud_callback(self, msg):
        self.latest_point_cloud = msg

    def model_infer(self, frame):
        image = PILImage.fromarray(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        results = self.model(image)
        # Filter the results to include only the 'person' class
        filtered_boxes = [box for box in results[0].boxes if box.cls[0] == 0]
        return filtered_boxes

    def overlay_results(self, image, tracked_objects):
        for obj in tracked_objects:
            x1, y1, x2, y2, obj_id = map(int, obj[:5])
            label = f"{obj_id}"  # Use only the tracking number without "ID"
            image = cv2.rectangle(image, (x1, y1), (x2, y2), (0, 165, 255), 2)
            image = cv2.putText(image, label, (x1, y1 - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 255), 2)
        return image

    def project_bboxes_to_3d(self, tracked_objects, point_cloud_msg, header):
        marker_array = MarkerArray()
        new_marker_ids = set()

        for obj in tracked_objects:
            x1, y1, x2, y2, obj_id = map(int, obj)
            xyz = self.point_cloud_to_xyz(point_cloud_msg, x1, y1, x2, y2)

            if xyz is None or xyz.size == 0:
                continue

            mean_point = np.mean(xyz, axis=0)

            # Define the 8 corners of the 3D bounding box
            bbox_size_x = 0.3
            bbox_size_y = 0.3
            bbox_size_z = 0.6
            corners = np.array([
                [mean_point[0] - bbox_size_x / 2, mean_point[1] - bbox_size_y / 2, mean_point[2] - bbox_size_z / 2],
                [mean_point[0] + bbox_size_x / 2, mean_point[1] - bbox_size_y / 2, mean_point[2] - bbox_size_z / 2],
                [mean_point[0] + bbox_size_x / 2, mean_point[1] + bbox_size_y / 2, mean_point[2] - bbox_size_z / 2],
                [mean_point[0] - bbox_size_x / 2, mean_point[1] + bbox_size_y / 2, mean_point[2] - bbox_size_z / 2],
                [mean_point[0] - bbox_size_x / 2, mean_point[1] - bbox_size_y / 2, mean_point[2] + bbox_size_z / 2],
                [mean_point[0] + bbox_size_x / 2, mean_point[1] - bbox_size_y / 2, mean_point[2] + bbox_size_z / 2],
                [mean_point[0] + bbox_size_x / 2, mean_point[1] + bbox_size_y / 2, mean_point[2] + bbox_size_z / 2],
                [mean_point[0] - bbox_size_x / 2, mean_point[1] + bbox_size_y / 2, mean_point[2] + bbox_size_z / 2]
            ])

            # Create a LINE_LIST marker for the wireframe
            line_marker = Marker()
            line_marker.header = header
            line_marker.header.frame_id = point_cloud_msg.header.frame_id
            line_marker.type = Marker.LINE_LIST
            line_marker.action = Marker.ADD
            line_marker.scale.x = 0.02  # Line width
            line_marker.color.a = 1.0
            line_marker.color.r = 1.0
            line_marker.color.g = 1.0
            line_marker.color.b = 0.0
            line_marker.id = obj_id
            line_marker.lifetime = self.marker_lifetime

            # Define lines between corners to form a wireframe box
            line_indices = [
                (0, 1), (1, 2), (2, 3), (3, 0),  # Bottom face
                (4, 5), (5, 6), (6, 7), (7, 4),  # Top face
                (0, 4), (1, 5), (2, 6), (3, 7)   # Vertical edges
            ]

            for start, end in line_indices:
                line_marker.points.append(Point(x=corners[start][0], y=corners[start][1], z=corners[start][2]))
                line_marker.points.append(Point(x=corners[end][0], y=corners[end][1], z=corners[end][2]))

            new_marker_ids.add(line_marker.id)
            marker_array.markers.append(line_marker)

            # Create a CUBE marker for the shading effect
            fill_marker = Marker()
            fill_marker.header = header
            fill_marker.header.frame_id = point_cloud_msg.header.frame_id
            fill_marker.type = Marker.CUBE
            fill_marker.action = Marker.ADD
            fill_marker.scale.x = bbox_size_x
            fill_marker.scale.y = bbox_size_y
            fill_marker.scale.z = bbox_size_z
            fill_marker.color.a = 0.5  # Semi-transparent
            fill_marker.color.r = 1.0
            fill_marker.color.g = 1.0
            fill_marker.color.b = 0.0
            fill_marker.pose.position.x = mean_point[0]
            fill_marker.pose.position.y = mean_point[1]
            fill_marker.pose.position.z = mean_point[2]
            fill_marker.id = obj_id + 5000  # Different ID for shading marker
            fill_marker.lifetime = self.marker_lifetime
            new_marker_ids.add(fill_marker.id)
            marker_array.markers.append(fill_marker)

            # Add a text label for the tracking number on top of the bounding box
            text_marker = Marker()
            text_marker.header = header
            text_marker.header.frame_id = point_cloud_msg.header.frame_id
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.scale.z = 0.3  # Text size
            text_marker.color.a = 1.0
            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.pose.position.x = mean_point[0]
            text_marker.pose.position.y = mean_point[1]
            text_marker.pose.position.z = mean_point[2] + bbox_size_z / 2 + 0.05  # Place the text just above the top surface
            text_marker.text = f"{obj_id}"  # Only the tracking number
            text_marker.id = obj_id + 1000  # Offset ID for text marker
            text_marker.lifetime = self.marker_lifetime
            new_marker_ids.add(text_marker.id)
            marker_array.markers.append(text_marker)

        # Delete old markers
        for marker_id in self.current_marker_ids - new_marker_ids:
            delete_marker = Marker()
            delete_marker.header = header
            delete_marker.header.frame_id = point_cloud_msg.header.frame_id
            delete_marker.action = Marker.DELETE
            delete_marker.id = marker_id
            marker_array.markers.append(delete_marker)

        self.current_marker_ids = new_marker_ids

        if len(marker_array.markers) > 0:
            self.bbox_pub.publish(marker_array)

    def point_cloud_to_xyz(self, pc_msg, x1, y1, x2, y2):
        xyz = []
        point_step = pc_msg.point_step
        row_step = pc_msg.row_step

        for v in range(y1, y2):
            for u in range(x1, x2):
                offset = v * row_step + u * point_step
                x, y, z = struct.unpack_from('fff', pc_msg.data, offset)
                if not np.isnan(x) and not np.isnan(y) and not np.isnan(z):
                    xyz.append([x, y, z])

        return np.array(xyz)

def main(args=None):
    rclpy.init(args=args)
    node = LidarProcessingNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

