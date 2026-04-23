from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='navigation_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            # 1. Load the Action Server
            ComposableNode(
                package='nav_project',
                plugin='nav_project::NavigationServer',
                name='navigation_server'
            ),
            # 2. Load the UI Action Client
            ComposableNode(
                package='nav_project',
                plugin='nav_project::UIClient',
                name='ui_client'
            )
        ],
        output='screen',
        emulate_tty=True  # This is important so the terminal displays your UI correctly
    )

    return LaunchDescription([container])
