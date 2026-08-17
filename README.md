<h1 align="center">Landfill Rover ROS 2 Workspace</h1>

Tập hợp các package ROS 2 cho pipeline cảm nhận của **Landfill Rover** với
camera Stereolabs ZED. Stack khởi chạy ZED ROS 2 Wrapper, biến đổi point cloud
sang một TF frame cấu hình được và hiển thị ảnh RGB cùng point cloud trong
RViz2.

`zed-ros2-wrapper` được quản lý dưới dạng Git submodule tại
`src/zed-ros2-wrapper`. Repository không phụ thuộc vào `zed-ros2-examples`.

```text
ros2_ws/
├── .gitmodules
├── README.md
└── src/
    ├── landfill-rover-ros2/
    │   ├── lr_bringup/
    │   ├── lr_display_rviz2/
    │   ├── lr_mavlink_replay/
    │   └── lr_pointcloud_transform/
    └── zed-ros2-wrapper/          # Git submodule
```

## Bắt đầu

### Yêu cầu

- Ubuntu 22.04 và ROS 2 Humble;
- [ZED SDK](https://www.stereolabs.com/developers/release/) cùng phiên bản CUDA
  tương thích;
- Git có hỗ trợ submodule;
- camera ZED được hỗ trợ, hoặc file ghi `.svo`/`.svo2`;
- RViz2 nếu cần giao diện trực quan hóa.

Tham khảo tài liệu chính thức của Stereolabs để
[cài ZED ROS 2 Wrapper](https://www.stereolabs.com/docs/ros2) và kiểm tra camera
bằng ZED Explorer trước khi chạy stack.

### Clone repository

Clone kèm submodule để lấy cả ZED ROS 2 Wrapper:

```bash
git clone --recurse-submodules <repository-url> ros2_ws
cd ros2_ws
```

Nếu đã clone repository mà chưa lấy submodule:

```bash
git submodule update --init --recursive
```

### Build workspace

Từ thư mục gốc của repository:

```bash
source /opt/ros/humble/setup.bash

sudo apt update
rosdep update
rosdep install --from-paths src --ignore-src -r -y --skip-keys scout_description
colcon build --symlink-install --cmake-args=-DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

Trong mỗi terminal mới, source lại ROS 2 và workspace:

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

## Các package

| Package | Chức năng |
| --- | --- |
| [`lr_bringup`](./src/landfill-rover-ros2/lr_bringup) | Entry point khởi chạy camera/SVO, node xử lý point cloud và RViz2. |
| [`lr_mavlink_replay`](./src/landfill-rover-ros2/lr_mavlink_replay) | Đọc GPS/attitude MAVLink CSV, nội suy pose ENU tại timestamp của từng point cloud SVO. |
| [`lr_pointcloud_transform`](./src/landfill-rover-ros2/lr_pointcloud_transform) | Biến đổi `sensor_msgs/msg/PointCloud2` sang frame đích bằng TF tại timestamp của message. |
| [`lr_display_rviz2`](./src/landfill-rover-ros2/lr_display_rviz2) | Khởi chạy RViz2 với cấu hình hiển thị ảnh RGB và registered point cloud. |

Luồng dữ liệu mặc định:

```text
ZED camera / SVO
       │
       ▼
  zed_wrapper ───────────── RGB image ───────────────────┐
       │                                                  │
       ├── registered PointCloud2                         │
       │              │                                   │
       │              ▼                                   │
       │   lr_pointcloud_transform ◄── /tf, /tf_static    │
       │              │                                   │
       │              └── /lr/point_cloud/cloud_in_map ─┐ │
       │                                             ▼      ▼
       └────────────────────────────────────── lr_display_rviz2
```

Khi SVO có một cặp CSV hợp lệ, nguồn pose được khóa sang MAVLink cho toàn
session:

```text
GPS CSV ─────┐
             ├─ mavlink_csv_pose_node ─ position + attitude + future Path
Attitude CSV ┘                                  (cùng stamp cloud) │
                                                                 ▼
ZED SVO ── registered PointCloud2 ── pointcloud_transform_node
                                                    │
                          TF: map → lr_base_link → zed_camera_link
                                                    │
                                                    ▼
                              /lr/point_cloud/cloud_in_map
```

## Sử dụng

### Camera trực tiếp

`camera_model` là tham số bắt buộc. Ví dụ với ZED 2i:

```bash
ros2 launch lr_bringup rover.launch.py camera_model:=zed2i
```

Lệnh trên khởi chạy một ZED wrapper, node transform point cloud và RViz2. Đổi
`camera_model` theo thiết bị đang dùng, chẳng hạn `zed2`, `zed2i`, `zedx` hoặc
`zedxm`.

### Phát lại SVO/SVO2

```bash
ros2 launch lr_bringup rover.launch.py \
  camera_model:=zed2i \
  svo_path:=/absolute/path/to/recording.svo2 \
  publish_svo_clock:=true
```

Khi `publish_svo_clock:=true`, ZED wrapper phát `/clock`; node transform và
RViz2 cùng sử dụng simulation time của bản ghi.

### Phát lại SVO với pose MAVLink CSV

```bash
ros2 launch lr_bringup rover.launch.py \
  camera_model:=zed2i \
  svo_path:=/workspace/svo/zed_20260710_092420_0001.svo2 \
  gps_path:=/workspace/data/session_20260710_0924_mavlink/gps.csv \
  attitude_path:=/workspace/data/session_20260710_0924_mavlink/attitude.csv \
  publish_svo_clock:=true \
  use_rviz:=true
```

Node replay đọc và kiểm tra toàn bộ hai file khi launch. GPS cần `lat`, `lon`,
`alt`, `fix_type`, ít nhất hai fix duy nhất có `fix_type >= 3`; attitude cần
`roll`, `pitch`, `yaw` radian và ít nhất hai timestamp duy nhất. Cả hai định
dạng thời gian `timestamp_unix_s` và `t_wall_epoch_us` đều được hỗ trợ.

Khi hợp lệ, trung bình tối đa 20 fix GPS đầu được dùng làm gốc local ENU
(X Đông, Y Bắc, Z lên); attitude MAVLink NED/FRD được đổi sang ENU/FLU (base
X trước, Y trái, Z lên), rồi nội suy tại đúng timestamp của point cloud. ZED
bị tắt `publish_tf` và
`publish_map_tf`; cây TF lúc này là:

```text
map → lr_base_link → zed_camera_link → zed_left_camera_frame
```

Nếu đang chạy camera live, thiếu một CSV, hoặc CSV sai path/schema/nội dung,
launch không tạo node replay và giữ pipeline TF của ZED:

```text
map → odom → zed_camera_link → zed_left_camera_frame
```

Sau khi một cặp CSV đã hợp lệ, session không fallback từng frame. Cloud nằm
ngoài khoảng log, nằm giữa hai GPS cách nhau hơn 2 giây, hoặc giữa hai attitude
cách nhau hơn 0,5 giây sẽ bị bỏ. Cách này tránh trộn hai hệ `world` khác nhau.

Tại mỗi cloud hợp lệ, node cũng publish đoạn GPS đã ghi nằm trong bán kính XY
10 m quanh vị trí hiện tại lên `/lr/mavlink/trajectory_future`, mặc định lấy
mẫu cách nhau 0,2 m theo quãng đường trên mặt đất. Orientation của mỗi pose
biểu diễn hướng tiếp tuyến của đường GPS. `Path.header.stamp` bằng timestamp
cloud, nên node xử lý địa hình có thể đồng bộ `Path` với
`/lr/point_cloud/cloud_in_map`. Path dừng khi lần đầu ra khỏi vòng tròn, gặp
gap GPS hoặc cuối log; nó không lấy lại đoạn đường quay vào vòng tròn và không
nối đường xuyên qua vùng thiếu dữ liệu. Tọa độ Z vẫn được giữ trong từng pose
nhưng không tham gia phép kiểm tra bán kính.

### Chạy headless

Tắt RViz2 khi chạy trên rover hoặc máy không có display:

```bash
ros2 launch lr_bringup rover.launch.py \
  camera_model:=zed2i \
  use_rviz:=false
```

### Chọn camera cụ thể

Với hệ thống có nhiều camera, truyền serial number hoặc camera ID:

```bash
ros2 launch lr_bringup rover.launch.py \
  camera_model:=zed2i \
  serial_number:=12345678
```

`serial_number:=0` và `camera_id:=-1` đều có nghĩa là chọn camera khả dụng đầu
tiên.

## Tham số của launch chính

| Tham số | Mặc định | Mô tả |
| --- | --- | --- |
| `camera_model` | Không có | Model camera ZED; bắt buộc phải truyền khi launch. |
| `camera_name` | `zed` | Namespace và tiền tố frame của camera. |
| `svo_path` | `live` | Đường dẫn tuyệt đối tới SVO/SVO2, hoặc `live` để dùng camera thật. |
| `publish_svo_clock` | `false` | Phát và sử dụng `/clock` khi đọc SVO. |
| `serial_number` | `0` | Serial number; `0` chọn camera khả dụng đầu tiên. |
| `camera_id` | `-1` | Camera ID; `-1` chọn camera khả dụng đầu tiên. |
| `ros_params_override_path` | Rỗng | File YAML ghi đè tham số của ZED wrapper. |
| `param_overrides` | Rỗng | Các tham số ZED wrapper inline, phân tách bằng dấu chấm phẩy. |
| `input_topic` | `/<camera_name>/zed_node/point_cloud/cloud_registered` | Point cloud đầu vào. |
| `output_topic` | `/lr/point_cloud/cloud_in_map` | Point cloud đã transform vào frame `map`. |
| `target_frame` | `map` | Frame cố định: do ZED cung cấp ở mode ZED, hoặc local ENU ở mode MAVLink. |
| `transform_timeout_sec` | `0.5` | Thời gian tối đa chờ transform, tính bằng giây. |
| `gps_path` | Rỗng | GPS MAVLink CSV; chỉ dùng khi phát SVO và có cả `attitude_path`. |
| `attitude_path` | Rỗng | Attitude MAVLink CSV radian; chỉ dùng khi phát SVO và có cả `gps_path`. |
| `mavlink_position_topic` | `/lr/mavlink/position_enu` | Vị trí ENU của rover base tại timestamp cloud. |
| `mavlink_attitude_topic` | `/lr/mavlink/attitude_enu` | Hướng ENU/FLU của rover base tại timestamp cloud. |
| `mavlink_future_path_topic` | `/lr/mavlink/trajectory_future` | Đoạn trajectory tương lai đã ghi, kiểu `nav_msgs/msg/Path`. |
| `future_path_radius_m` | `10.0` | Bán kính XY quanh rover của trajectory tương lai, tính bằng mét. |
| `future_path_step_m` | `0.2` | Quãng đường mặt đất giữa hai pose trên trajectory, tính bằng mét. |
| `mavlink_base_frame` | `lr_base_link` | Tên frame FLU của rover base. |
| `base_to_camera_x_m`, `base_to_camera_y_m`, `base_to_camera_z_m` | `0.0` | Tịnh tiến camera trong base FLU, đơn vị mét. |
| `base_to_camera_roll_deg`, `base_to_camera_pitch_deg`, `base_to_camera_yaw_deg` | `0.0` | Fixed-axis RPY của camera trong base, đơn vị độ. |
| `use_rviz` | `true` | Bật hoặc tắt RViz2. |

Sáu giá trị extrinsic bằng 0 vẫn chạy và tương đương giả sử base trùng
`zed_camera_link`, nhưng node sẽ cảnh báo để tránh quên hiệu chuẩn vị trí lắp
camera.

Xem danh sách trực tiếp từ launch file:

```bash
ros2 launch lr_bringup rover.launch.py --show-args
```

## Topic mặc định

Với `camera_name:=zed`, stack sử dụng các topic chính sau:

| Topic | Kiểu message | Nội dung |
| --- | --- | --- |
| `/zed/zed_node/rgb/color/rect/image` | `sensor_msgs/msg/Image` | Ảnh RGB đã rectification từ ZED wrapper. |
| `/zed/zed_node/point_cloud/cloud_registered` | `sensor_msgs/msg/PointCloud2` | Registered point cloud đầu vào. |
| `/lr/point_cloud/cloud_in_map` | `sensor_msgs/msg/PointCloud2` | Point cloud đã được đổi sang frame `map`. |
| `/lr/mavlink/position_enu` | `geometry_msgs/msg/PointStamped` | Vị trí rover base trong `map`; chỉ có ở chế độ MAVLink CSV. |
| `/lr/mavlink/attitude_enu` | `geometry_msgs/msg/QuaternionStamped` | Hướng rover base trong `map`; chỉ có ở chế độ MAVLink CSV. |
| `/lr/mavlink/trajectory_future` | `nav_msgs/msg/Path` | Trajectory đã ghi phía trước rover trong `map`; cùng stamp với cloud hiện tại. |
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Các transform cần để đổi hệ tọa độ. |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation clock khi phát SVO với `publish_svo_clock:=true`. |

Node `lr_pointcloud_transform` tra TF tại đúng `header.stamp` của point cloud,
biến đổi dữ liệu XYZ rồi cập nhật `header.frame_id`. Message sẽ bị bỏ qua và
phát cảnh báo có throttle nếu thiếu TF trong khoảng timeout cấu hình.

Trong chế độ MAVLink, hai topic pose có `header.stamp` giống chính xác cloud đã
kích hoạt nội suy. Transform node exact-sync cặp message này, phát dynamic TF
`map → lr_base_link`, phát static TF extrinsic `lr_base_link →
zed_camera_link`, rồi để TF MessageFilter xử lý cloud khi chuỗi transform đúng
timestamp đã đầy đủ.

RViz được cấu hình sẵn display `Future GPS Trajectory` màu cam. Vì point cloud
và Path cùng ở `map`, đường được vẽ trực tiếp trong không gian 3D hiện tại.
Một node detect plane sau này chỉ cần đồng bộ hai topic theo `header.stamp`, tạo
corridor quanh `Path`, rồi lọc điểm từ cloud nằm trong corridor đó.

Lưu ý: đây là phần tương lai đã tồn tại trong GPS CSV, phù hợp cho replay và
đánh giá offline; nó không phải trajectory được dự đoán từ dữ liệu live.

## Chạy từng thành phần

### Node transform point cloud

Khi nguồn point cloud và cây TF đã chạy:

```bash
ros2 run lr_pointcloud_transform pointcloud_transform_node --ros-args \
  --params-file \
  "$(ros2 pkg prefix lr_pointcloud_transform)/share/lr_pointcloud_transform/config/pointcloud_transform.yaml"
```

Cũng có thể ghi đè từng tham số:

```bash
ros2 run lr_pointcloud_transform pointcloud_transform_node --ros-args \
  -p input_topic:=/zed/zed_node/point_cloud/cloud_registered \
  -p output_topic:=/lr/point_cloud/cloud_in_map \
  -p target_frame:=map \
  -p transform_timeout_sec:=0.5
```

### RViz2

Khởi chạy ZED wrapper và RViz2:

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py camera_model:=zed2i
```

Nếu ZED wrapper đã chạy, chỉ mở RViz2 và hiển thị output của node transform:

```bash
ros2 launch lr_display_rviz2 display_zed_cam.launch.py \
  camera_model:=zed2i \
  start_zed_node:=false \
  pointcloud_topic:=/lr/point_cloud/cloud_in_map
```

Cấu hình mặc định của RViz2 dùng fixed frame `map` và target view
`zed_camera_link`. Vì vậy cây TF phải nối được frame của point cloud với `map`;
nếu dùng tên camera khác, có thể truyền một file RViz riêng bằng
`rviz_config:=/absolute/path/to/config.rviz`.

## Kiểm tra

Build riêng bốn package và chạy unit/integration/lint test:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select \
  lr_bringup lr_display_rviz2 lr_mavlink_replay lr_pointcloud_transform
source install/setup.bash
colcon test --packages-select \
  lr_bringup lr_display_rviz2 lr_mavlink_replay lr_pointcloud_transform
colcon test-result --verbose
```

Sau khi launch, có thể kiểm tra nhanh graph ROS 2:

```bash
ros2 node list
ros2 topic hz /lr/point_cloud/cloud_in_map
ros2 topic echo /lr/point_cloud/cloud_in_map --once --field header
```

Với SVO + CSV, kiểm tra thêm nguồn pose, timestamp và cây TF:

```bash
ros2 node list | grep mavlink_csv_pose_node
ros2 topic echo /lr/mavlink/position_enu --once
ros2 topic echo /lr/mavlink/attitude_enu --once
ros2 topic echo /lr/mavlink/trajectory_future --once --field header
ros2 run tf2_ros tf2_echo map zed_left_camera_frame
ros2 param get /zed/zed_node pos_tracking.publish_tf
```

Lệnh cuối phải trả về `False`; output cloud phải có `frame_id: map` và tiếp tục
có dữ liệu trên `ros2 topic hz /lr/point_cloud/cloud_in_map`.

## Giấy phép

Mã nguồn được phát hành theo
[Apache License 2.0](./src/landfill-rover-ros2/LICENSE).
