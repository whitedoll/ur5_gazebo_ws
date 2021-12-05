from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch_ros.actions import Node

def generate_launch_description():
    
    load_joint_state_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start',
             'joint_state_broadcaster'],
        output='screen'
    )

    load_joint_trajectory_controller = ExecuteProcess(
        cmd=['ros2', 'control', 'load_controller', '--set-state', 'start', 'joint_trajectory_controller'],
        output='screen'
    )

    spawn_controller = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    spawn_controller_traj = Node(
        package="controller_manager",
        executable="spawner.py",
        arguments=["joint_trajectory_controller"],
        output="screen",
    )

    # ros2 control load_controller --set-state start joint_state_broadcaster
    
    # # Velocity
    # ros2 control load_controller --set-state start velocity_controller
    # # Position 
    # ros2 control load_controller --set-state start joint_trajectory_controller

    # ros2 interface proto std_msgs/msg/Float64MultiArray
    #     ros2 topic pub /velocity_controller/commands std_msgs/msg/Float64MultiArray "layout:
    #   dim: []
    #   data_offset: 0
    # data: [10,10]
    # "

    return LaunchDescription([        
        # RegisterEventHandler(
        #     event_handler=OnProcessExit(
        #         target_action=load_joint_state_controller,
        #         on_exit=[load_joint_trajectory_controller],
        #     )
        # ),
        spawn_controller,
        spawn_controller_traj
    ])