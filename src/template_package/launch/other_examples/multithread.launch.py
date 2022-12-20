import os
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

	config_path = os.path.join(
		get_package_share_directory('template_package'),
		'config'
		)

	template_node_description = Node(
			package='template_package',
			namespace='templates',
			executable='multithreaded_node',
			name='multithreaded_node',
			# output='screen', #Uncomment to allow print statements to print to terminal
			parameters=[os.path.join(config_path, 'other_examples', 'multithread_params.yaml')]
		)

	return LaunchDescription([
		template_node_description,
	])
