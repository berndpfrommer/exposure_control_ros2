# -----------------------------------------------------------------------------
# Copyright 2022 Bernd Pfrommer <bernd.pfrommer@gmail.com>
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
#
#
import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration as LaunchConfig
from launch.actions import DeclareLaunchArgument as LaunchArg


def generate_launch_description():
    """Create exposure control node."""
    container = ComposableNodeContainer(
            name='exposure_control_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='exposure_control_ros2',
                    plugin='exposure_control_ros2::ExposureControl',
                    name=LaunchConfig('node_name'),
                    parameters=[{'cam_name': LaunchConfig('camera_name'),
                                 'max_gain': 10.0,
                                 'gain_priority': False,
                                 'brightness_target': 100,
                                 'max_exposure_time': 9000.0,
                                 'min_exposure_time': 3000.0}],
                    remappings=[
                        ('~/meta',
                         ['/', LaunchConfig('camera_name'), '/meta'])],
                    extra_arguments=[{'use_intra_process_comms': True}],
                ),
            ],
            output='screen',
    )
    name_arg = LaunchArg('node_name', default_value=['exposure_control'],
                         description='name of node')
    cam_arg = LaunchArg('camera_name', default_value=['cam_0'],
                        description='name of camera')
    return launch.LaunchDescription([name_arg, cam_arg, container])
