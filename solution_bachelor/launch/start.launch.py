import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='solution_bachelor',
            executable='control_scheduler.py',  # Python script
            # executable='solution_node',  # C++ program
            name='control_scheduler',
            output='screen'
        )
    ])