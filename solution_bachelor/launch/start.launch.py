import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='solution_bachelor',
            executable='mapper.py',  # Python script
            # executable='solution_node',  # C++ program
            name='mapper',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='solution_bachelor',
            executable='lidar_v2.py',  # Python script
            # executable='solution_node',  # C++ program
            name='lidar',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='solution_bachelor',
            executable='director.py',  # Python script
            # executable='solution_node',  # C++ program
            name='director',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='solution_bachelor',
            executable='pid_controller.py',  # Python script
            # executable='solution_node',  # C++ program
            name='pid_controller',
            output='screen'
        ),
        launch_ros.actions.Node(
            package='solution_bachelor',
            executable='control_summator.py',  # Python script
            # executable='solution_node',  # C++ program
            name='control_summator',
            output='screen'
        ),
    ])