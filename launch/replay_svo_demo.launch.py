# Copyright 2025 Stereolabs
#
# Licensed under the Apache License, Version 2.0 (the 'License');
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an 'AS IS' BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os

from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    IncludeLaunchDescription
)
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    TextSubstitution
)
from launch_ros.actions import Node

default_svo_path = os.path.join(
        get_package_share_directory('ros2_replay_data'),
        'examples/person_walking_demo',
        'person_walking_demo.svo2'
)

def launch_setup(context, *args, **kwargs):

    # Launch configuration variables
    start_zed_node = LaunchConfiguration('start_zed_node')
    namespace = LaunchConfiguration('namespace')
    camera_name = LaunchConfiguration('camera_name')
    camera_model = LaunchConfiguration('camera_model')
    svo_path = LaunchConfiguration('svo_path')
    replay_rate = LaunchConfiguration('svo_replay_rate')
    replay_rate_increment = LaunchConfiguration('svo_replay_rate_increment')

    namespace_val = namespace.perform(context)
    camera_name_val = camera_name.perform(context)
    camera_model_val = camera_model.perform(context)
    svo_path_val = svo_path.perform(context)
    replay_rate_val = replay_rate.perform(context)
    replay_rate_increment_val = replay_rate_increment.perform(context)

    if (camera_name_val == ''):
        camera_name_val = 'zed'

    camera_type = ''
    if( camera_model_val=='zed' or
        camera_model_val=='zedm' or
        camera_model_val=='zed2' or
        camera_model_val=='zed2i' or
        camera_model_val=='zedx' or
        camera_model_val=='zedxm' or
        camera_model_val=='virtual'):
        camera_type = 'stereo'
    else: # 'zedxonegs' or 'zedxone4k')
        camera_type = 'mono'

    # RVIZ2 Configurations to be loaded by ZED Node
    config_rviz2 = os.path.join(
        get_package_share_directory('ros2_replay_data'),
        'rviz',
        'replay_svo.rviz'
    )

    # RVIZ2 node
    rviz2_node = Node(
        package='rviz2',
        namespace=camera_name_val,
        executable='rviz2',
        name=camera_model_val +'_rviz2',
        output='screen',
        arguments=[['-d'], [config_rviz2]],
    )

    # ZED Wrapper launch file
    zed_wrapper_launch = IncludeLaunchDescription(
        launch_description_source=PythonLaunchDescriptionSource([
            get_package_share_directory('zed_wrapper'),
            '/launch/zed_camera.launch.py'
        ]),
        launch_arguments={
            'camera_name': camera_name_val,
            'camera_model': camera_model_val,
            'svo_path': svo_path_val,
            'namespace': namespace_val
        }.items(),
        condition=IfCondition(start_zed_node)
    )
    
    control_svo_replay_node = Node(
            package='ros2_replay_data',
            executable='control_svo_node',
            name='control_svo_replay_node',
            output='screen',
            parameters=[
                {'svo_replay_rate': replay_rate},
                {'svo_replay_rate_increment': replay_rate_increment},
                {'namespace': namespace_val},
                {'camera_name': camera_name_val}
            ],
        )

    return [
        rviz2_node,
        zed_wrapper_launch,
        control_svo_replay_node
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                'start_zed_node',
                default_value='True',
                description='Set to `False` to start only RVIZ2 if a ZED node is already running.'),
            DeclareLaunchArgument(
                'namespace',
                default_value=TextSubstitution(text=''),
                description='Current namespace`.'),
            DeclareLaunchArgument(
                'camera_name',
                default_value=TextSubstitution(text=''),
                description='The name of the camera. It can be different from the camera model and it will be used as node `namespace`.'),
            DeclareLaunchArgument(
                'camera_model',
                default_value=TextSubstitution(text='zedx'),
                description='[REQUIRED] The model of the camera. Using a wrong camera model can disable camera features.',
                choices=['zed', 'zedm', 'zed2', 'zed2i', 'zedx', 'zedxm', 'virtual', 'zedxonegs', 'zedxone4k']),
            DeclareLaunchArgument(
                'svo_path',
                default_value=TextSubstitution(text=default_svo_path),
                description='The svo file path to be used to replay the svo data'),
            DeclareLaunchArgument(
                'svo_replay_rate',
                default_value='1.0',
                description='The svo current replay rate'),
            DeclareLaunchArgument(
                'svo_replay_rate_increment',
                default_value='0.1',
                description='The svo current replay rate increment'),
            OpaqueFunction(function=launch_setup)
        ]
    )
