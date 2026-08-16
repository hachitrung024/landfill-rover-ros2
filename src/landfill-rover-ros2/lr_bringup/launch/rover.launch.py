# Copyright 2026 Landfill Rover Maintainers
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    camera_model = LaunchConfiguration('camera_model')
    camera_name = LaunchConfiguration('camera_name')
    svo_path = LaunchConfiguration('svo_path')
    publish_svo_clock = LaunchConfiguration('publish_svo_clock')
    serial_number = LaunchConfiguration('serial_number')
    camera_id = LaunchConfiguration('camera_id')
    ros_params_override_path = LaunchConfiguration('ros_params_override_path')
    param_overrides = LaunchConfiguration('param_overrides')
    input_topic = LaunchConfiguration('input_topic')
    output_topic = LaunchConfiguration('output_topic')
    target_frame = LaunchConfiguration('target_frame')
    transform_timeout_sec = LaunchConfiguration('transform_timeout_sec')
    use_rviz = LaunchConfiguration('use_rviz')

    zed_wrapper_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('zed_wrapper'),
                'launch',
                'zed_camera.launch.py',
            )
        ),
        launch_arguments={
            'camera_name': camera_name,
            'camera_model': camera_model,
            'svo_path': svo_path,
            'publish_svo_clock': publish_svo_clock,
            'serial_number': serial_number,
            'camera_id': camera_id,
            'ros_params_override_path': ros_params_override_path,
            'param_overrides': param_overrides,
        }.items(),
    )

    pointcloud_transform_node = Node(
        package='lr_pointcloud_transform',
        executable='pointcloud_transform_node',
        name='pointcloud_transform_node',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(
                publish_svo_clock, value_type=bool),
            'input_topic': input_topic,
            'output_topic': output_topic,
            'target_frame': target_frame,
            'transform_timeout_sec': ParameterValue(
                transform_timeout_sec, value_type=float),
        }],
    )

    display_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('lr_display_rviz2'),
                'launch',
                'display_zed_cam.launch.py',
            )
        ),
        launch_arguments={
            'camera_name': camera_name,
            'camera_model': camera_model,
            'start_zed_node': 'false',
            'publish_svo_clock': publish_svo_clock,
            'pointcloud_topic': output_topic,
        }.items(),
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'camera_model',
            description='ZED camera model.',
            choices=[
                'zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm',
                'zedxnano', 'zedxhdr', 'zedxhdrmini', 'zedxhdrmax',
                'virtual', 'zedxonegs', 'zedxone4k', 'zedxonehdr',
            ],
        ),
        DeclareLaunchArgument(
            'camera_name',
            default_value='zed',
            description='Camera namespace and frame prefix.',
        ),
        DeclareLaunchArgument(
            'svo_path',
            default_value='live',
            description='Absolute SVO/SVO2 input path, or live for a physical camera.',
        ),
        DeclareLaunchArgument(
            'publish_svo_clock',
            default_value='false',
            choices=['true', 'false'],
            description='Publish and use the SVO timestamp on /clock.',
        ),
        DeclareLaunchArgument(
            'serial_number',
            default_value='0',
            description='Camera serial number. Zero selects the first available camera.',
        ),
        DeclareLaunchArgument(
            'camera_id',
            default_value='-1',
            description='Camera ID. Minus one selects the first available camera.',
        ),
        DeclareLaunchArgument(
            'ros_params_override_path',
            default_value='',
            description='Optional YAML file overriding ZED wrapper parameters.',
        ),
        DeclareLaunchArgument(
            'param_overrides',
            default_value='',
            description='Semicolon-separated inline ZED wrapper parameter overrides.',
        ),
        DeclareLaunchArgument(
            'input_topic',
            default_value=[
                '/', camera_name, '/zed_node/point_cloud/cloud_registered',
            ],
            description='PointCloud2 topic transformed by the processing node.',
        ),
        DeclareLaunchArgument(
            'output_topic',
            default_value='/lr/pointcloud/odom',
            description='Transformed PointCloud2 output topic.',
        ),
        DeclareLaunchArgument(
            'target_frame',
            default_value='odom',
            description='TF frame into which the point cloud is transformed.',
        ),
        DeclareLaunchArgument(
            'transform_timeout_sec',
            default_value='0.5',
            description='Maximum time to wait for the matching TF transform.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='true',
            choices=['true', 'false'],
            description='Start the RGB and transformed-point-cloud RViz UI.',
        ),
        zed_wrapper_launch,
        pointcloud_transform_node,
        display_launch,
    ])
