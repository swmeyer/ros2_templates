import os
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

	config_path = os.path.join(
		get_package_share_directory('rosbag2_api_examples'),
		'config'
		)

	template_node_description = Node(
			package='rosbag2_api_examples',
			namespace='rosbag2_api',
			executable='bagread_node',
			name='bagread_node',
			# output='screen', #Uncomment to allow print statements to print to terminal
			parameters=[os.path.join(config_path, 'bag_params.yaml')]
		)

	return LaunchDescription([
		template_node_description,
	])
