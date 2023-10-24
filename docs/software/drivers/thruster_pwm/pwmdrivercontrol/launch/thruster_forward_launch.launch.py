from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    ld = LaunchDescription()
    duty_cycle = Node(
        package='pwmdriver',
        executable='duty_cycle',
        output='screen',
)
    ld.add_action(duty_cycle)
    pwm_control_node = Node(
        package='pwmdrivercontrol',
        executable='pwm_control_node',
    )
    ld.add_action(pwm_control_node)
    return ld
