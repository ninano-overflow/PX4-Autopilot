#!/usr/bin/env python3
import sys
import os
import time
import math
import numpy as np
import cv2

# Set headless mode if no DISPLAY environment variable is found
headless = False
if 'DISPLAY' not in os.environ:
    os.environ['SDL_VIDEODRIVER'] = 'dummy'
    headless = True
    print("[ArUco Sim] Running in HEADLESS mode (no display detected)")
else:
    print("[ArUco Sim] Running in GUI mode")

import pygame
from pymavlink import mavutil

# --- Camera Parameters ---
WIDTH = 1920
HEIGHT = 1080
CX = WIDTH / 2
CY = HEIGHT / 2
FOV_H_DEG = 50.0
FOV_H_RAD = math.radians(FOV_H_DEG)
# Focal length in pixels
F_LEN = (WIDTH / 2) / math.tan(FOV_H_RAD / 2)

# Camera intrinsic matrix
K = np.array([
    [F_LEN, 0, CX],
    [0, F_LEN, CY],
    [0, 0, 1]
], dtype=np.float32)

# ==========================================
# [PART 1] ArUco Marker Setup & Placement
# Defines the physical size, center coordinate, and the 4 corners of the marker 
# in the world NED (North-East-Down) frame.
# ==========================================
# --- Marker Parameters ---
MARKER_SIZE = 1.0  # Physical size of ArUco marker in meters
MARKER_CENTER = np.array([0.0, 0.0, 0.0])  # Center in world NED frame [North, East, Down]

# 4 Corners of marker in world NED frame
# Clockwise order: Top-Left, Top-Right, Bottom-Right, Bottom-Left
# In NED: Top-Left is North-West, Top-Right is North-East, etc.
HALF_SIZE = MARKER_SIZE / 2.0
MARKER_CORNERS_W = np.array([
    [ HALF_SIZE, -HALF_SIZE, 0.0],  # NW
    [ HALF_SIZE,  HALF_SIZE, 0.0],  # NE
    [-HALF_SIZE,  HALF_SIZE, 0.0],  # SE
    [-HALF_SIZE, -HALF_SIZE, 0.0]   # SW
], dtype=np.float32)

# --- Rotation matrix functions ---
def rpy_to_R(roll, pitch, yaw):
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    R_z = np.array([
        [cy, -sy, 0],
        [sy,  cy, 0],
        [ 0,   0, 1]
    ], dtype=np.float32)
    R_y = np.array([
        [ cp, 0, sp],
        [  0, 1,  0],
        [-sp, 0, cp]
    ], dtype=np.float32)
    R_x = np.array([
        [1,  0,   0],
        [0, cr, -sr],
        [0, sr,  cr]
    ], dtype=np.float32)

    return R_z @ R_y @ R_x

def rotation_matrix_to_quaternion(R):
    tr = np.trace(R)
    if tr > 0:
        S = np.sqrt(tr + 1.0) * 2
        qw = 0.25 * S
        qx = (R[2, 1] - R[1, 2]) / S
        qy = (R[0, 2] - R[2, 0]) / S
        qz = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        qw = (R[2, 1] - R[1, 2]) / S
        qx = 0.25 * S
        qy = (R[0, 1] + R[1, 0]) / S
        qz = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        qw = (R[0, 2] - R[2, 0]) / S
        qx = (R[0, 1] + R[1, 0]) / S
        qy = 0.25 * S
        qz = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        qw = (R[1, 0] - R[0, 1]) / S
        qx = (R[0, 2] + R[2, 0]) / S
        qy = (R[1, 2] + R[2, 1]) / S
        qz = 0.25 * S
    return qw, qx, qy, qz

# --- Main simulation class ---
class ArUcoPrecisionLandSim:
    def __init__(self, connection_string='udpin:localhost:14540'):
        print(f"[ArUco Sim] Connecting to PX4 on {connection_string}...")
        self.mav = mavutil.mavlink_connection(connection_string)

        # Load ArUco marker texture
        script_dir = os.path.dirname(os.path.realpath(__file__))
        marker_path = os.path.join(script_dir, 'marker.png')
        if not os.path.exists(marker_path):
            print(f"[ArUco Sim] Error: marker.png not found at {marker_path}!")
            sys.exit(1)

        self.marker_img = cv2.imread(marker_path, cv2.IMREAD_GRAYSCALE)
        self.marker_h, self.marker_w = self.marker_img.shape

        # Initialize Pygame window
        pygame.init()
        pygame.display.set_caption("Virtual Camera - ArUco Tracking")
        self.screen = pygame.display.set_mode((WIDTH, HEIGHT))
        self.clock = pygame.time.Clock()

        # OpenCV ArUco detector setup
        # Using pre-defined 4x4 dictionary (matches generated image)
        self.dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.dictionary, self.parameters)

        # Drone State
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0  # NED: negative height
        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

        # Ground position calibration
        self.x_ground = None
        self.y_ground = None
        self.z_ground = None

        self.last_heartbeat = 0.0
        self.start_time = time.time()

        # Try to initialize ROS2 node for image/pose publishing (optional)
        self.ros2_enabled = False
        try:
            import rclpy
            from sensor_msgs.msg import Image
            from geometry_msgs.msg import PoseStamped
            
            # Check if rclpy is already initialized
            if not rclpy.ok():
                rclpy.init()
            self.ros2_node = rclpy.create_node('aruco_sim_image_publisher')
            self.image_pub = self.ros2_node.create_publisher(Image, '/camera/image_raw', 10)
            self.pose_pub = self.ros2_node.create_publisher(PoseStamped, '/drone/pose', 10)
            self.ros2_enabled = True
            print("[ArUco Sim] ROS2 image and pose publishers initialized. Publishing to /camera/image_raw and /drone/pose")
        except Exception as e:
            print(f"[ArUco Sim] ROS2 publishers disabled: {e}")


    def run(self):
        print("[ArUco Sim] Starting simulation loop (Press Ctrl+C to exit)...")
        running = True
        try:
            while running:
                # Handle Pygame events to keep the window responsive
                for event in pygame.event.get():
                    if event.type == pygame.QUIT:
                        running = False

                # 1. Update state from MAVLink
                self.read_mavlink()

                # 2. Render virtual camera frame
                frame = self.render_virtual_camera()

                # 3. Run ArUco detection on virtual frame
                detected_frame, detected, angle_x, angle_y, target_dist = self.detect_marker(frame)

                # 4. Send MAVLink feedback to PX4
                if detected:
                    self.send_landing_target(angle_x, angle_y, target_dist)

                # Send downward distance sensor measurement
                self.send_distance_sensor()

                # Send MAVLink heartbeat
                now = time.time()
                if now - self.last_heartbeat > 1.0:
                    self.send_heartbeat()
                    self.last_heartbeat = now

                # ==========================================
                # [PART 3] Image Display / Publishing Point
                # This section renders the processed frame to Pygame.
                # Note: Currently, this script runs standalone. If ROS/ROS2 is required,
                # you can publish the camera frame (e.g. 'detected_frame') from here
                # using a ROS2 Image Publisher (rclpy & sensor_msgs/Image).
                # ==========================================
                # 5. Display frame in Pygame
                if not headless:
                    # Convert grayscale frame to RGB for Pygame
                    rgb_frame = cv2.cvtColor(detected_frame, cv2.COLOR_GRAY2RGB)
                    # Rotate and flip image to match Pygame surface coordinates
                    rgb_frame = np.rot90(rgb_frame)
                    rgb_frame = np.flipud(rgb_frame)

                    pygame_surface = pygame.surfarray.make_surface(rgb_frame)
                    self.screen.blit(pygame_surface, (0, 0))
                    pygame.display.flip()

                # Limit rate to 30 FPS (camera frame rate)
                self.clock.tick(30)

                # Publish image and pose to ROS2 if enabled
                if self.ros2_enabled:
                    try:
                        from sensor_msgs.msg import Image
                        from geometry_msgs.msg import PoseStamped
                        
                        # 1. Publish Camera Frame
                        imgmsg = Image()
                        imgmsg.header.stamp = self.ros2_node.get_clock().now().to_msg()
                        imgmsg.header.frame_id = "camera_link"
                        imgmsg.height = detected_frame.shape[0]
                        imgmsg.width = detected_frame.shape[1]
                        imgmsg.encoding = "mono8"
                        imgmsg.is_bigendian = 0
                        imgmsg.step = detected_frame.shape[1]
                        imgmsg.data = detected_frame.tobytes()
                        self.image_pub.publish(imgmsg)
                        
                        # 2. Publish Drone Pose (ENU coordinates)
                        if self.x_ground is not None:
                            pose_msg = PoseStamped()
                            pose_msg.header.stamp = self.ros2_node.get_clock().now().to_msg()
                            pose_msg.header.frame_id = "map"
                            
                            # Local NED to ROS ENU conversion
                            x_ref, y_ref, z_ref = self.x_ground, self.y_ground, self.z_ground
                            pose_msg.pose.position.x = float(self.y - y_ref)  # East
                            pose_msg.pose.position.y = float(self.x - x_ref)  # North
                            pose_msg.pose.position.z = float(-(self.z - z_ref))  # Up
                            
                            # Transform rotation matrix from NED to ENU-FLU
                            T_earth = np.array([
                                [0, 1, 0],
                                [1, 0, 0],
                                [0, 0, -1]
                            ], dtype=np.float32)
                            T_body = np.array([
                                [1,  0,  0],
                                [0, -1,  0],
                                [0,  0, -1]
                            ], dtype=np.float32)
                            
                            R_px4 = rpy_to_R(self.roll, self.pitch, self.yaw)
                            R_ros = T_body @ R_px4 @ T_earth.T
                            
                            qw, qx, qy, qz = rotation_matrix_to_quaternion(R_ros)
                            pose_msg.pose.orientation.w = float(qw)
                            pose_msg.pose.orientation.x = float(qx)
                            pose_msg.pose.orientation.y = float(qy)
                            pose_msg.pose.orientation.z = float(qz)
                            
                            self.pose_pub.publish(pose_msg)
                        
                        import rclpy
                        rclpy.spin_once(self.ros2_node, timeout_sec=0.001)
                    except Exception as e:
                        print(f"[ArUco Sim] ROS2 publish failed: {e}")
                        self.ros2_enabled = False

        except KeyboardInterrupt:
            print("\n[ArUco Sim] Exiting...")
        finally:
            pygame.quit()

    def read_mavlink(self):
        """Read incoming MAVLink messages to update drone state."""
        # Non-blocking read of all queued messages
        while True:
            msg = self.mav.recv_match(blocking=False)
            if not msg:
                break

            msg_type = msg.get_type()
            if msg_type == 'LOCAL_POSITION_NED':
                self.x = msg.x
                self.y = msg.y
                self.z = msg.z
                if self.x_ground is None:
                    self.x_ground = msg.x
                    self.y_ground = msg.y
                    self.z_ground = msg.z
                    print(f"[ArUco Sim] Ground position calibrated: x={self.x_ground:.2f}, y={self.y_ground:.2f}, z={self.z_ground:.2f}")
            elif msg_type == 'ATTITUDE':
                self.roll = msg.roll
                self.pitch = msg.pitch
                self.yaw = msg.yaw

    # ==========================================
    # [PART 2] Downward-Facing Virtual Camera
    # Projects the 3D world corners of the ArUco marker into 2D camera coordinates
    # and performs perspective warping to simulate a gimbal-stabilized camera view.
    # ==========================================
    def render_virtual_camera(self):
        """Project the ArUco marker corners and warp the texture to simulate the camera frame."""
        # Canvas for the camera frame (start with pitch-black)
        frame = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)

        # If the drone altitude is zero or negative (on the ground or below it), we can't render
        # If the ground position hasn't been calibrated yet, we can't render
        if self.z_ground is None:
            return frame

        alt = self.z_ground - self.z
        if alt <= 0.05:
            return frame

        # Camera is mounted on a gimbal stabilized in roll and pitch (only rotates in yaw)
        R_gimbal_to_ned = rpy_to_R(0.0, 0.0, self.yaw)
        # Rotation matrix from NED to gimbal frame
        R_ned_to_gimbal = R_gimbal_to_ned.T

        drone_pos_w = np.array([self.x - self.x_ground, self.y - self.y_ground, self.z - self.z_ground])

        # Project marker corners into camera frame
        projected_pts = []
        for corner_w in MARKER_CORNERS_W:
            # 1. Transform from world NED to gimbal frame
            corner_g = R_ned_to_gimbal @ (corner_w - drone_pos_w)

            # 2. Transform from gimbal frame to downward-facing camera frame
            # Downward camera frame conventions:
            # X_cam = Y_gimbal (Right on screen)
            # Y_cam = -X_gimbal (Up/Down on screen)
            # Z_cam = Z_gimbal (optical axis, pointing down)
            x_c = corner_g[1]
            y_c = -corner_g[0]
            z_c = corner_g[2]

            # If the corner is behind the camera plane, we can't project it
            if z_c <= 0.05:
                return frame

            # 3. Project to 2D image coordinates using camera matrix K
            u = (F_LEN * x_c / z_c) + CX
            v = (F_LEN * y_c / z_c) + CY
            projected_pts.append([u, v])

        # Convert to numpy array of float32 for warpPerspective
        dst_pts = np.array(projected_pts, dtype=np.float32)

        # Original corners of the 2D marker texture image
        src_pts = np.array([
            [0, 0],
            [self.marker_w - 1, 0],
            [self.marker_w - 1, self.marker_h - 1],
            [0, self.marker_h - 1]
        ], dtype=np.float32)

        # Compute perspective warp matrix
        M = cv2.getPerspectiveTransform(src_pts, dst_pts)

        # Warp marker onto the camera frame
        # BorderValue=0 keeps the rest of the canvas black
        warped_marker = cv2.warpPerspective(self.marker_img, M, (WIDTH, HEIGHT), borderValue=0)
        return warped_marker

    def detect_marker(self, frame):
        """Run OpenCV ArUco detector on the virtual camera frame."""
        # Convert to BGR so we can draw colorful feedback on it
        annotated_frame = frame.copy()

        # Run detection
        corners, ids, rejected = self.detector.detectMarkers(frame)

        detected = False
        angle_x = 0.0
        angle_y = 0.0
        target_dist = 0.0

        if ids is not None and len(ids) > 0:
            # We only care about marker ID 0 for our precision landing
            for i, marker_id in enumerate(ids.flatten()):
                if marker_id == 0:
                    detected = True
                    # Get corners of detected marker
                    marker_corners = corners[i][0]

                    # Calculate center in pixel coordinates
                    center_u = np.mean(marker_corners[:, 0])
                    center_v = np.mean(marker_corners[:, 1])

                    # Calculate angular offsets in stabilized camera frame
                    angle_x_stab = (center_u - CX) / F_LEN
                    angle_y_stab = (center_v - CY) / F_LEN

                    # Transform from stabilized camera frame to drone's body frame (FRD)
                    # so that PX4's internal attitude-compensation recovers the correct coordinates.
                    # x_g = -angle_y, y_g = angle_x, z_g = 1.0 (vector in gimbal frame)
                    v_gimbal = np.array([-angle_y_stab, angle_x_stab, 1.0], dtype=np.float32)

                    # Rotation from gimbal to body: R_x(roll)^T * R_y(pitch)^T
                    cr = math.cos(self.roll)
                    sr = math.sin(self.roll)
                    cp = math.cos(self.pitch)
                    sp = math.sin(self.pitch)

                    R_x_T = np.array([
                        [1,   0,   0],
                        [0,  cr,  sr],
                        [0, -sr,  cr]
                    ], dtype=np.float32)
                    R_y_T = np.array([
                        [ cp, 0, -sp],
                        [  0, 1,   0],
                        [ sp, 0,  cp]
                    ], dtype=np.float32)

                    R_gimbal_to_body = R_x_T @ R_y_T
                    v_body = R_gimbal_to_body @ v_gimbal

                    # Convert back to body-frame camera offsets
                    if v_body[2] > 0.05:
                        angle_x = v_body[1] / v_body[2]
                        angle_y = -v_body[0] / v_body[2]
                    else:
                        angle_x = angle_x_stab
                        angle_y = angle_y_stab

                    # Approximate distance to target using the known physical marker size
                    # Average pixel side length of the marker in the image
                    side_px_1 = np.linalg.norm(marker_corners[0] - marker_corners[1])
                    side_px_2 = np.linalg.norm(marker_corners[1] - marker_corners[2])
                    avg_side_px = (side_px_1 + side_px_2) / 2.0

                    if avg_side_px > 0:
                        target_dist = (MARKER_SIZE * F_LEN) / avg_side_px
                    else:
                        target_dist = -self.z  # Fallback to altitude

                    # Draw green bounding box and center dot
                    cv2.polylines(annotated_frame, [marker_corners.astype(np.int32)], True, 180, 2)
                    cv2.circle(annotated_frame, (int(center_u), int(center_v)), 5, 255, -1)

                    if not headless:
                        # Draw some status text on the frame
                        text = f"ID: 0 Dist: {target_dist:.2f}m dx: {angle_x:.3f} dy: {angle_y:.3f}"
                        cv2.putText(annotated_frame, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, 255, 1)
                    break

        return annotated_frame, detected, angle_x, angle_y, target_dist

    def send_landing_target(self, angle_x, angle_y, distance):
        """Send LANDING_TARGET MAVLink message to PX4."""
        # Using MAV_FRAME_BODY_FRD (12) for the target coordinate frame
        # type = MAV_LANDING_TARGET_TYPE_VISION_FIDUCIAL (2)
        # position_valid = 0 (only angles are valid, letting PX4 fuse it with rangefinder)

        # size_x and size_y are angular sizes in radians
        size_x = MARKER_SIZE / distance
        size_y = MARKER_SIZE / distance

        self.mav.mav.landing_target_send(
            int(time.time() * 1e6),             # time_usec
            0,                                  # target_num (Signature)
            12,                                 # frame (MAV_FRAME_BODY_FRD)
            angle_x,                            # angle_x (rad, offset along camera X axis)
            angle_y,                            # angle_y (rad, offset along camera Y axis)
            distance,                           # distance (m)
            size_x,                             # size_x (rad)
            size_y,                             # size_y (rad)
            0.0, 0.0, 0.0,                      # x, y, z (ignored since position_valid = 0)
            [1.0, 0.0, 0.0, 0.0],               # q (quaternion orientation of target)
            2,                                  # type (MAV_LANDING_TARGET_TYPE_VISION_FIDUCIAL)
            0                                   # position_valid (0 = False)
        )
        print(f"[ArUco Sim] Sent LANDING_TARGET: angle_x={angle_x:.3f}, angle_y={angle_y:.3f}, dist={distance:.2f}m")

    def send_distance_sensor(self):
        """Send downward-facing DISTANCE_SENSOR MAVLink message to PX4."""
        if self.z_ground is None:
            alt = 0.05
        else:
            alt = self.z_ground - self.z

        if alt < 0.05:
            alt = 0.05

        # Compute line-of-sight distance straight down, accounting for pitch and roll attitude
        cos_roll = math.cos(self.roll)
        cos_pitch = math.cos(self.pitch)
        if cos_roll * cos_pitch > 0.1:
            distance = alt / (cos_roll * cos_pitch)
        else:
            distance = alt

        # Convert to centimeters
        dist_cm = int(distance * 100)

        # Clip to rangefinder limits (e.g. 10cm to 4000cm)
        dist_cm = max(10, min(dist_cm, 4000))

        # orientation = 25 (MAV_SENSOR_ROTATION_PITCH_270, pointing down)
        # type = 0 (MAV_DISTANCE_SENSOR_LASER)
        # covariance = 0
        self.mav.mav.distance_sensor_send(
            int((time.time() - self.start_time) * 1000) & 0xFFFFFFFF,  # time_boot_ms
            10,                 # min_distance (cm)
            4000,               # max_distance (cm)
            dist_cm,            # current_distance (cm)
            0,                  # type (laser)
            0,                  # id
            25,                 # orientation
            0                   # covariance
        )

    def send_heartbeat(self):
        """Send periodic MAVLink heartbeat from companion computer component."""
        self.mav.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_ONBOARD_CONTROLLER,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID,
            0, 0, 0
        )

if __name__ == '__main__':
    # Default connection string binds to udpin on 14540 (receives PX4 local broadcast)
    connection_string = 'udpin:localhost:14540'
    if len(sys.argv) > 1:
        connection_string = sys.argv[1]

    sim = ArUcoPrecisionLandSim(connection_string)
    sim.run()
