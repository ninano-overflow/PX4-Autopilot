# ArUco Precision Landing Simulation Documentation

이 문서에서는 [aruco_precland_sim.py](file:///home/sim/dev/PX4-Autopilot/Tools/simulation/jsbsim/scripts/aruco_precland_sim.py) 스크립트 내에서 Precision Landing(정밀 착륙)을 위해 사용되는 주요 구성 요소 3가지를 정리하고 설명합니다.

---

## 1. ArUco Marker Setup & Placement (아루코 마커 배치)
* **코드 위치**: [aruco_precland_sim.py L38-56](file:///home/sim/dev/PX4-Autopilot/Tools/simulation/jsbsim/scripts/aruco_precland_sim.py#L38-L56)
* **설명**: 
  시뮬레이션 환경 내에서 ArUco 마커의 크기와 월드 NED(North-East-Down) 좌표계 기준의 3D 공간 상 모서리(Corners) 위치를 정의합니다.
* **코드 내용**:
```python
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
```
* **수학적/기하학적 원리**:
  - `MARKER_SIZE`는 실제 마커의 1변 길이(미터 단위)를 의미합니다.
  - 마커 중심은 월드 NED 원점 `(0, 0, 0)`에 배치됩니다.
  - 모서리점 `MARKER_CORNERS_W`는 NED 좌표 기준 시계 방향(NW -> NE -> SE -> SW)으로 3D 좌표를 설정합니다.

---

## 2. Downward-Facing Virtual Camera (수직하방 가상 카메라)
* **코드 위치**: [aruco_precland_sim.py L200-268](file:///home/sim/dev/PX4-Autopilot/Tools/simulation/jsbsim/scripts/aruco_precland_sim.py#L200-L268) (`render_virtual_camera` 함수)
* **설명**: 
  드론의 3D 위치 및 자세(Yaw) 정보를 기반으로 월드 좌표계에 배치된 마커 모서리를 카메라 좌표계로 투영한 뒤, 2D 이미지 평면으로 변환하고 원근 변환(Perspective Warp)을 수행해 드론 하방 카메라 영상을 실시간으로 생성합니다.
* **코드 내용**:
```python
    def render_virtual_camera(self):
        """Project the ArUco marker corners and warp the texture to simulate the camera frame."""
        # Canvas for the camera frame (start with pitch-black)
        frame = np.zeros((HEIGHT, WIDTH), dtype=np.uint8)
        ...
```
* **구현 단계**:
  1. **회전 및 평행이동 변환**:
     드론의 Roll, Pitch가 물리적으로 보정되는 짐벌(Gimbal) 형태를 가정하므로, Yaw 회전만 고려된 회전 행렬 `R_ned_to_gimbal`을 통해 마커 모서리점의 상대 3D 위치를 계산합니다.
     ```python
     corner_g = R_ned_to_gimbal @ (corner_w - drone_pos_w)
     ```
  2. **카메라 좌표계 변환**:
     수직 하방 카메라 좌표계에 맞게 축을 매핑합니다.
     - $X_{cam} = Y_{gimbal}$ (우측 방향)
     - $Y_{cam} = -X_{gimbal}$ (전방/위쪽 방향)
     - $Z_{cam} = Z_{gimbal}$ (광학 축, 아래 방향)
  3. **핀홀 카메라 투영 (Pinhole Projection)**:
     카메라 내부 파라미터(초점거리 `F_LEN`, 중심점 `CX, CY`)를 이용하여 3D 좌표 $(x_c, y_c, z_c)$를 2D 픽셀 좌표 $(u, v)$로 변환합니다.
     $$u = f \cdot \frac{x_c}{z_c} + c_x$$
     $$v = f \cdot \frac{y_c}{z_c} + c_y$$
  4. **원근 변환 (Warp Perspective)**:
     원본 마커 이미지(`marker.png`)의 4개 모서리(`src_pts`)를 위에서 계산한 카메라 투영 픽셀 좌표(`dst_pts`)로 OpenCV `warpPerspective` 함수를 이용해 매핑시킵니다.

---

## 3. Image Display & Publishing Point (이미지 디스플레이 및 퍼블리싱 포인트)
* **코드 위치**: [aruco_precland_sim.py L157-178](file:///home/sim/dev/PX4-Autopilot/Tools/simulation/jsbsim/scripts/aruco_precland_sim.py#L157-L178) (`run` 함수 내부 루프)
* **설명**: 
  현재 시뮬레이션에서는 실시간으로 렌더링되고 아루코 검출이 완료된 이미지 프레임을 로컬 GUI 화면(Pygame)에 띄우는 역할을 합니다.
* **코드 내용**:
```python
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
```

### ROS2 Image & Pose Publisher (내장 기능)
현재 `aruco_precland_sim.py` 스크립트에는 ROS2 퍼블리셔 기능이 내장되어 있습니다. 시스템에 ROS2 환경이 소싱되어 있다면 자동으로 활성화되어 다음 토픽들을 발행합니다:

1. **`/camera/image_raw`** (`sensor_msgs/msg/Image`): 드론의 가상 하방 카메라 영상(ArUco 검출 박스 포함).
2. **`/drone/pose`** (`geometry_msgs/msg/PoseStamped`): PX4 NED 로컬 좌표계를 ROS ENU (East-North-Up) 좌표계로 변환한 드론의 3D 실시간 위치 및 자세(Orientation).

#### 코드 구현 분석
* **초기화 (`__init__`)**: `rclpy` 라이브러리가 로드 가능하고 활성화되어 있다면, 자동으로 `aruco_sim_image_publisher` 노드를 생성하고 퍼블리셔들을 초기화합니다.
* **퍼블리시 (`run` 루프)**: 루프 주기마다 `detected_frame`을 픽셀 바이트 배열로 변환하여 이미지 토픽으로 쏘고, `x_ground` 캘리브레이션 기준 편차와 회전 행렬을 쿼터니언으로 변환해 `/drone/pose`로 쏩니다.
* **수신 대기**: Foxglove Studio를 실행하고 **Foxglove WebSocket** 연결 방식으로 연결한 뒤, 해당 토픽들을 선택하면 바로 실시간 시뮬레이션 데이터를 볼 수 있습니다.

