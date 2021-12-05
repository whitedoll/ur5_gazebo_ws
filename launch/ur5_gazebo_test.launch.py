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

    # install_dir = get_package_prefix('ur5_gazebo')
    # if 'GAZEBO_MODEL_PATH' in os.environ:
    #     os.environ['GAZEBO_MODEL_PATH'] =  os.environ['GAZEBO_MODEL_PATH'] + ':' + install_dir + '/share'
    # else:
    #     os.environ['GAZEBO_MODEL_PATH'] =  install_dir + "/share"

    # if 'GAZEBO_PLUGIN_PATH' in os.environ:
    #     os.environ['GAZEBO_PLUGIN_PATH'] = os.environ['GAZEBO_PLUGIN_PATH'] + ':' + install_dir + '/lib'
    # else:
    #     os.environ['GAZEBO_PLUGIN_PATH'] = install_dir + '/lib'

    # try:
    #     envs = {}
    #     for key in os.environ.__dict__["_data"]:
    #         key = key.decode("utf-8")
    #         if (key.isupper()):
    #             envs[key] = os.environ[key]
    # except Exception as e:
    #     print("Error with Envs: " + str(e))
    #     return None

    # gazebo = ExecuteProcess(
    #         cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'], output='screen',
    #         env=envs
    #     )

    # urdf = os.path.join(get_package_share_directory('box_car_description'), 'robot/', 'box_bot.urdf')    
    # assert os.path.exists(urdf), "Thebox_bot.urdf doesnt exist in "+str(urdf)

    xacro_file = os.path.join(get_package_share_directory('ur5_gazebo'), 'urdf/', 'ur5_joint_limited_robot2.urdf.xacro')    
    assert os.path.exists(xacro_file), "The ur5_joint.xacro doesnt exist in "+str(xacro_file)

    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

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

    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         'use_sim_time',
    #         default_value='false',
    #         description='Use simulation (Gazebo) clock if true')
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "description_package",
    #         default_value="ur5_gazebo",
    #         description="Description package with robot URDF/xacro files. Usually the argument \
    #     is not set, it enables use of a custom description.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "description_file",
    #         default_value="ur5_joint_limited_robot.urdf.xacro",
    #         # default_value="rrbot.urdf.xacro",
    #         description="URDF/XACRO description file with the robot.",
    #     )
    # )
    # declared_arguments.append(
    #     DeclareLaunchArgument(
    #         "prefix",
    #         default_value='""',
    #         description="Prefix of the joint names, useful for \
    #     multi-robot setup. If changed than also joint names in the controllers' configuration \
    #     have to be updated.",
    #     )
    # )
    # # declared_arguments.append(
    # #     DeclareLaunchArgument(
    # #         'use_sim_time',
    # #         default_value='false',
    # #         description='Use simulation (Gazebo) clock if true'
    # #     )
    # # )

    # # Initialize Arguments
    # description_package = LaunchConfiguration("description_package")
    # description_file = LaunchConfiguration("description_file", default=None)
    # prefix = LaunchConfiguration("prefix")

    # use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    # xacro_path = LaunchConfiguration('xacro_path', default=None)

    # rviz
    rviz_display_config_file = os.path.join(
        get_package_share_directory('ur5_gazebo'),
        'rviz',
        'view_robot_test.rviz')
    # urdf_file = os.path.join(
    #     get_package_share_directory('ur5_gazebo'),
    #     'urdf',
    #     'ur5_joint_limited_robot.urdf')

    # urdf_file = os.path.join(
    #     get_package_share_directory('cart_test'),
    #     'urdf',
    #     'cartpole2.xacro')

    # with open(urdf_file, 'r') as infp:
    #     robot_description_file = infp.read()

    gazebo_ros2_control_demos_path = os.path.join(
        get_package_share_directory('ur5_gazebo'))

    xacro_file = os.path.join(gazebo_ros2_control_demos_path,
                              'urdf',
                              'ur5_joint_limited_robot2.urdf.xacro')
    # print('aaaaaa', xacro_file)

    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}

    # robot_state_publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[
            {'use_sim_time': False},
            {'robot_description': params}
        ],
        output='screen')
    
    # rviz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_display_config_file],
    )

    # # path_to_urdf = get_package_share_path('pr2_description') / 'robots' / 'pr2.urdf.xacro'
    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     parameters=[
    #         {'use_sim_time': False},
    #         {xacro.process_file(xacro_file).toxml()}
    #         ]
    # )

    # # Get URDF via xacro
    # robot_description_content = Command(
    #     [
    #         PathJoinSubstitution([FindExecutable(name="xacro")]),
    #         " ",
    #         PathJoinSubstitution(
    #             [FindPackageShare(description_package), "urdf", description_file]
    #         ),
    #         " ",
    #         "prefix:=",
    #         prefix,
    #     ]
    # )
    # # print('Description :', robot_description_content)
    # robot_description = {"robot_description": robot_description_content}

    # robot_state_publisher_node = Node(
    #     package="robot_state_publisher",
    #     executable="robot_state_publisher",
    #     output="both",
    #     parameters=[robot_description],
    # )

    # gazebo_ros2_control_demos_path = os.path.join(
    #     get_package_share_directory('ur5_gazebo'))

    # xacro_file = os.path.join(gazebo_ros2_control_demos_path,
    #                           'urdf',
    #                           'ur5_joint_limited_robot.urdf.xacro')
    # # print('aaaaaa', xacro_file)

    # doc = xacro.parse(open(xacro_file))
    # xacro.process_doc(doc)
    # params = {'robot_description': doc.toxml()}
    # # print('Paramsssss', params)

    # robot_state_publisher_node = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     output='screen',
    #     parameters=[params]
    # )

    # rviz_config_file = PathJoinSubstitution(
    #     [FindPackageShare(description_package), "config", "view_robot.rviz"]
    # )

    joint_state_publisher_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
    )
    
    # rviz_node = Node(
    #     package="rviz2",
    #     executable="rviz2",
    #     name="rviz2",
    #     output="screen",
    #     arguments=["-d", rviz_config_file],
    # )

    # # cartpole model spawn
    # spawn_entity = Node(package='gazebo_ros', executable='spawn_entity.py',
    #                     arguments=['-topic', 'robot_description',
    #                                '-entity', 'ur5'],
    #                     output='screen')

    # nodes = [
    #     # joint_state_publisher_node,
    #     robot_state_publisher,
    #     # robot_state_publisher_node,
    #     rviz_node,
    #     # spawn_entity,
    # ]

    # gazebo
    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('gazebo_ros'), 'launch'), '/gazebo.launch.py']),
             )

    spawn_ur5_py = Node(package='gazebo_ros', executable='spawn_entity.py',
                        arguments=['-topic', 'robot_description',
                                   '-entity', 'ur5'],
                        output='screen')

    # spawn_ur5_py = Node(package='ur5_gazebo', executable='spawn_ur5.py', arguments=[robot_desc], output='screen')

    robot_state_publisher = Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            parameters=[
                {"robot_description": robot_desc}],
            output="screen")

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )
    
    # robot_controllers = PathJoinSubstitution(
    #     [
    #         FindPackageShare("ur5_gazebo"),
    #         "config",
    #         "cart_controller.yaml",
    #     ]
    # )

    # gazebo
    ld.add_action(gazebo)

    ld.add_action(use_sim_time_descp)
    ld.add_action(spawn_ur5_py)
    ld.add_action(joint_state_broadcaster)
    ld.add_action(robot_state_publisher)

    ld.add_action(joint_state_publisher_node)
    # ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)
    # ld.add_action(description_package)
    # ld.add_action(description_file)
    # ld.add_action(prefix)

    # return LaunchDescription(declared_arguments + nodes)
    return ld