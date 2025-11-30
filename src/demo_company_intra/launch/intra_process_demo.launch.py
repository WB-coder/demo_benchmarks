"""Launch file for intra-process communication demo."""

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    """Generate launch description with intra-process communication enabled."""
    container = ComposableNodeContainer(
        name='latency_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='demo_company_intra',
                plugin='demo_company_intra::LatencyPublisher',
                name='latency_publisher',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
            ComposableNode(
                package='demo_company_intra',
                plugin='demo_company_intra::LatencyListener',
                name='latency_listener',
                extra_arguments=[{'use_intra_process_comms': True}],
            ),
        ],
        output='screen',
    )

    return LaunchDescription([container])
