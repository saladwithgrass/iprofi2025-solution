import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='solution_bachelor',
            executable='solution.py',  # Python script
            # executable='solution_node',  # C++ program
            name='solution_node',
            output='screen'
        )
    ])