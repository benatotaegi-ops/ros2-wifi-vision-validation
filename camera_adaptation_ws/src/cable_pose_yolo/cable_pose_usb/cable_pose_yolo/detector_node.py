import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Pose, PoseArray
from cv_bridge import CvBridge
from ultralytics import YOLO
from rclpy.qos import qos_profile_sensor_data
import math
import cv2
import numpy as np
import time

import tf2_ros
from rclpy.time import Time
from rclpy.duration import Duration


def quat_to_matrix(x, y, z, w):
    """Convert a quaternion (x, y, z, w) into a 3x3 rotation matrix."""
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        return np.eye(3)
    # Normalize quaternion
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([
        [1 - 2 * (y * y + z * z),     2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z),     2 * (y * z - x * w)],
        [2 * (x * z - y * w),         2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])





class DetectorCables3D(Node):
    def __init__(self):
        super().__init__('detector_cables_3d')
        self.bridge = CvBridge()

        # Declare parameters injected via YAML/Launch files (with valid default values)
        self.declare_parameter('model_path', '')
        self.declare_parameter('camera_topic', '/cam_3/camera/color/image_raw')
        self.declare_parameter('info_topic', '/cam_3/camera/color/camera_info')
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('camera_optical_frame', 'camera_color_optical_frame')
        self.declare_parameter('poses_topic', '/cam_3/cable_poses')
        self.declare_parameter('do_nothing', False)
        self.declare_parameter('debug_mode', True)
        self.debug_mode = self.get_parameter('debug_mode').value

        # NEW: instead of reading depth from the sensor (unreliable for thin objects
        # like a cable), we assume the cable rests on a plane of known height,
        # and compute its 3D position by intersecting the camera ray
        # (defined by the detected pixel + camera intrinsics) with that plane.
        self.declare_parameter('height_reference_frame', 'auxiliar_table')
        self.declare_parameter('height_value', 0.0)      # Value along that axis where the table plane lies

        # Read parameter values
        model_path = self.get_parameter('model_path').value
        camera_topic = self.get_parameter('camera_topic').value
        info_topic = self.get_parameter('info_topic').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value
        self.camera_optical_frame = self.get_parameter('camera_optical_frame').value
        poses_topic = self.get_parameter('poses_topic').value

        self.height_reference_frame = self.get_parameter('height_reference_frame').value
        self.height_axis_idx = 1  # Y-axis of auxiliar_table, hardcoded
        self.height_value = self.get_parameter('height_value').value

        # Load YOLO pose estimation model
        self.model = YOLO(model_path, task='pose')

        # Camera intrinsic parameters (populated by camera_info_callback)
        self.intrinsics = None

        # tf2: we need to know where the camera is relative to height_reference_frame
        # at the moment of each detection, in order to cast the ray and intersect it
        # with the table plane.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Publishers (raw annotated image output)
        self.pose_array_pub = self.create_publisher(PoseArray, poses_topic, 10)
        self.image_pub = self.create_publisher(Image, '/camera/color/image_annotated', 10)

        # Subscriptions
        self.create_subscription(CameraInfo, info_topic, self.camera_info_callback, 10)
        self.create_subscription(Image, camera_topic, self.image_callback, qos_profile_sensor_data)

        self.get_logger().info(
            f"YOLO 3D Brain started in Docker environment! "
            f"(fixed height: {self.height_reference_frame}[axis {self.height_axis_idx}] = {self.height_value})"
        )

    def camera_info_callback(self, msg):
        """Extract camera intrinsic parameters (focal lengths and principal point) from CameraInfo."""
        self.intrinsics = {
            'fx': msg.k[0], 'cx': msg.k[2],
            'fy': msg.k[4], 'cy': msg.k[5]
        }

    def _ray_plane_intersection(self, grip_u, grip_v, stamp):
        """
        Compute the 3D position (in the camera frame) of pixel (grip_u, grip_v),
        assuming the real point lies on the plane defined by
        height_reference_frame.height_axis = height_value.
        Returns None if it fails (e.g., TF unavailable, or the ray is parallel to the plane).
        """
        try:
            # Camera pose relative to the reference frame (to know where the ray
            # originates and how to rotate its direction)
            t = self.tf_buffer.lookup_transform(
                self.height_reference_frame,
                self.camera_optical_frame,
                Time(),  # Latest available transform
                timeout=Duration(seconds=0.2)
            )
        except Exception as e:
            self.get_logger().warn(f"Could not obtain TF {self.height_reference_frame} -> {self.camera_optical_frame}: {e}")
            return None

        # Camera origin and rotation matrix expressed in the reference frame
        cam_origin_ref = np.array([
            t.transform.translation.x,
            t.transform.translation.y,
            t.transform.translation.z,
        ])
        R_ref_cam = quat_to_matrix(
            t.transform.rotation.x,
            t.transform.rotation.y,
            t.transform.rotation.z,
            t.transform.rotation.w,
        )

        # Ray direction in the camera frame (standard pinhole convention,
        # optical frame: X right, Y down, Z forward)
        dir_cam = np.array([
            (grip_u - self.intrinsics['cx']) / self.intrinsics['fx'],
            (grip_v - self.intrinsics['cy']) / self.intrinsics['fy'],
            1.0,
        ])
        dir_cam = dir_cam / np.linalg.norm(dir_cam)

        # Transform ray direction into the reference frame
        dir_ref = R_ref_cam @ dir_cam

        # Check if the ray is parallel to the target plane
        denom = dir_ref[self.height_axis_idx]
        if abs(denom) < 1e-9:
            self.get_logger().warn("Ray is parallel to the table plane, no intersection.")
            return None

        # Solve for the ray parameter t where it intersects the plane height
        t_param = (self.height_value - cam_origin_ref[self.height_axis_idx]) / denom
        if t_param <= 0:
            # The intersection lies behind the camera, which is not physically valid
            self.get_logger().warn(f"Intersection behind the camera (t={t_param:.3f}), discarding.")
            return None

        point_ref = cam_origin_ref + t_param * dir_ref

        # Express the point back in the camera frame (as expected by
        # PoseArray for artificialy_tf_publisher)
        point_cam = R_ref_cam.T @ (point_ref - cam_origin_ref)

        return point_cam

    def image_callback(self, msg):
        """Process incoming raw frames, run YOLO pose inference, and publish poses and debug images."""
        self.get_logger().info("Image received in callback!")
        if self.get_parameter('do_nothing').value:
            return

        # Wait until camera intrinsics have been received
        if self.intrinsics is None:
            return

        t_start = time.perf_counter()
        # Convert ROS Image message to OpenCV BGR format
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Measure pure YOLO inference time
        t_yolo_0 = time.perf_counter()
        resultados = self.model(cv_image, conf=self.conf_thresh, verbose=False)
        t_yolo_ms = (time.perf_counter() - t_yolo_0) * 1000.0

        if self.debug_mode:
            annotated_frame = cv_image.copy()

        pose_array = PoseArray()
        # IMPORTANT: Preserve the original camera capture timestamp
        pose_array.header.stamp = msg.header.stamp
        pose_array.header.frame_id = self.camera_optical_frame

        # Process the first detected object if any bounding boxes exist
        if len(resultados[0].boxes) > 0:
            pose_result = resultados[0].keypoints.xy[0]

            # Ensure at least two keypoints are detected to compute midpoint and orientation
            if len(pose_result) >= 2:
                p1_x, p1_y = float(pose_result[0][0]), float(pose_result[0][1])
                p2_x, p2_y = float(pose_result[1][0]), float(pose_result[1][1])

                # Compute grip midpoint (u, v) and 2D orientation angle in radians
                grip_u = int((p1_x + p2_x) / 2)
                grip_v = int((p1_y + p2_y) / 2)
                angulo_rad = math.atan2((p2_y - p1_y), (p2_x - p1_x))

                # Draw 2D coordinate axes (X: red, Y: green, Z: blue) on the debug frame
                if self.debug_mode:
                    longitud_eje = 40
                    grosor_linea = 2
                    radio_punto = 3
                    x_end = (int(grip_u + longitud_eje * math.cos(angulo_rad)),
                             int(grip_v + longitud_eje * math.sin(angulo_rad)))
                    y_end = (int(grip_u + longitud_eje * math.cos(angulo_rad + math.pi / 2)),
                             int(grip_v + longitud_eje * math.sin(angulo_rad + math.pi / 2)))
                    z_end = (int(grip_u + (longitud_eje * 0.7) * math.cos(angulo_rad - math.pi * 0.75)),
                             int(grip_v + (longitud_eje * 0.7) * math.sin(angulo_rad - math.pi * 0.75)))

                    cv2.line(annotated_frame, (grip_u, grip_v), x_end, (0, 0, 255), grosor_linea)
                    cv2.line(annotated_frame, (grip_u, grip_v), y_end, (0, 255, 0), grosor_linea)
                    cv2.line(annotated_frame, (grip_u, grip_v), z_end, (255, 0, 0), grosor_linea)
                    cv2.circle(annotated_frame, (grip_u, grip_v), radio_punto, (255, 255, 255), -1)

                # Verify midpoint is within image bounds and populate Pose message
                if 0 <= grip_v < cv_image.shape[0] and 0 <= grip_u < cv_image.shape[1]:
                    # Temporary bypass: not using _ray_plane_intersection, assigning test values
                    pose = Pose()
                    pose.position.x = 0.0
                    pose.position.y = 0.0
                    pose.position.z = 1.0

                    # Convert 2D yaw angle into a quaternion around the Z-axis
                    pose.orientation.x = 0.0
                    pose.orientation.y = 0.0
                    pose.orientation.z = math.sin(angulo_rad / 2.0)
                    pose.orientation.w = math.cos(angulo_rad / 2.0)

                    pose_array.poses.append(pose)

        # Publish detected poses
        self.pose_array_pub.publish(pose_array)

        # Publish raw annotated image if debug mode is enabled
        if self.debug_mode:
            msg_annotated = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            self.image_pub.publish(msg_annotated)

        # Log inference and total callback latency
        t_total_ms = (time.perf_counter() - t_start) * 1000.0
        self.get_logger().info(f"YOLO Inference: {t_yolo_ms:.1f} ms | Total callback: {t_total_ms:.1f} ms")


def main(args=None):
    rclpy.init(args=args)
    nodo = DetectorCables3D()
    rclpy.spin(nodo)
    nodo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
