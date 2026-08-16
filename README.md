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
| `target_frame` | `map` | Frame cố định, được ZED căn theo trọng lực khi `set_gravity_as_origin` được bật. |
| `transform_timeout_sec` | `0.5` | Thời gian tối đa chờ transform, tính bằng giây. |
| `use_rviz` | `true` | Bật hoặc tắt RViz2. |

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
| `/tf`, `/tf_static` | `tf2_msgs/msg/TFMessage` | Các transform cần để đổi hệ tọa độ. |
| `/clock` | `rosgraph_msgs/msg/Clock` | Simulation clock khi phát SVO với `publish_svo_clock:=true`. |

Node `lr_pointcloud_transform` tra TF tại đúng `header.stamp` của point cloud,
biến đổi dữ liệu XYZ rồi cập nhật `header.frame_id`. Message sẽ bị bỏ qua và
phát cảnh báo có throttle nếu thiếu TF trong khoảng timeout cấu hình.

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

Build riêng ba package và chạy lint test:

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

colcon build --symlink-install --packages-select \
  lr_bringup lr_display_rviz2 lr_pointcloud_transform
source install/setup.bash
colcon test --packages-select \
  lr_bringup lr_display_rviz2 lr_pointcloud_transform
colcon test-result --verbose
```

Sau khi launch, có thể kiểm tra nhanh graph ROS 2:

```bash
ros2 node list
ros2 topic hz /lr/point_cloud/cloud_in_map
ros2 topic echo /lr/point_cloud/cloud_in_map --once --field header
```

## Giấy phép

Mã nguồn được phát hành theo
[Apache License 2.0](./src/landfill-rover-ros2/LICENSE).
