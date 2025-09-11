from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    
    plant = Node(
        package="plant",
        executable="plant",  
        name="plant",
        output="screen"
    )

    wrench_controller = Node(
        package="palletrone_controller",
        executable="wrench_controller",
        name="wrench_controller",
        output="screen"
    )

    allocator_controller = Node(
        package="palletrone_controller",
        executable="allocator_controller",
        name="allocator_controller",
        output="screen"
    )

    position_cmd = Node(
        package="palletrone_cmd",
        executable="position_cmd",
        name="position_cmd",
        output="screen"
    )

    start_controllers_after_plant = RegisterEventHandler(
        OnProcessStart(
            target_action=plant,
            on_start=[wrench_controller, allocator_controller]
        )
    )

    start_cmd_after_allocator = RegisterEventHandler(
        OnProcessStart(
            target_action=allocator_controller,
            on_start=[position_cmd]
        )
    )

    return LaunchDescription([
        plant,
        start_controllers_after_plant,
        start_cmd_after_allocator
    ])
