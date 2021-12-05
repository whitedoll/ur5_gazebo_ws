# Copyright 2021 Stogl Robotics Consulting UG (haftungsbeschränkt)
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
import xacro
from ament_index_python.packages import get_package_share_directory, get_package_prefix

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution 
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    ld = LaunchDescription()

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    pkg_ur5_description = get_package_share_directory('ur5_gazebo')

    # Sart World
    start_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ur5_description, 'launch', 'start_world_launch.py'),
        )
    )

    # gazebo
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    xacro_file = os.path.join(get_package_share_directory('ur5_gazebo'), 'urdf/', 'ur5_joint_limited_robot.urdf.xacro')
    # xacro_file = os.path.join(get_package_share_directory('ur5_gazebo'), 'urdf/', 'ur5_all.urdf.xacro')    
    assert os.path.exists(xacro_file), "The ur5_joint.xacro doesnt exist in "+str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    install_dir = get_package_prefix('ur5_gazebo')
    if 'GAZEBO_MODEL_PATH' in os.environ:
        os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    else:
        os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share"

    if 'GAZEBO_PLUGIN_PATH' in os.environ:
        os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    else:
        os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'


    # print(robot_desc)

    start_steering_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ur5_description, 'launch', 'gazebo_control.launch.py'),
        )
    ) 
    
    # Declare arguments
    declared_arguments = []

    use_sim_time_descp = DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true')

    # rviz
    rviz_display_config_file = os.path.join(
        get_package_share_directory('ur5_gazebo'),
        'rviz',
        'view_robot.rviz')
    
    # rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_display_config_file],
    )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )

    # spawn_ur5_py = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                     arguments=['-topic', 'robot_description',
    #                                '-entity', 'ur5'],
    #                     output='screen')

    # spawn ur5 robot
    spawn_ur5_py = Node(package='ur5_gazebo', executable='spawn_ur5.py', arguments=[robot_desc], output='screen')

    robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {'use_sim_time': False},
                {"robot_description": robot_desc}],
            output="screen")

    

    spawn_robot_controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ur5_description, 'launch', 'gazebo_control.launch.py'),
        )
    )     

    # start world
    ld.add_action(start_world)

    # gazebo
    # ld.add_action(gazebo)

    # ld.add_action(use_sim_time_descp)
    ld.add_action(spawn_ur5_py)
    # ld.add_action(joint_state_broadcaster)
    ld.add_action(robot_state_publisher)

    # rviz gui
    # ld.add_action(joint_state_publisher_node)
    ld.add_action(rviz_node)

    # controller
    ld.add_action(spawn_robot_controller)

    # return LaunchDescription(declared_arguments + nodes)
    return ld