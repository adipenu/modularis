from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, Command
from launch.actions import ExecuteProcess
def generate_launch_description():
    ld = LaunchDescription()
    duty_cycle = Node(
        package='pwmdriver',
        executable='duty_cycle',
        output='screen',
)    
    new_term = ExecuteProcess(
    	cmd=[
    		'gnome-terminal',
    		'--',
    		'ros2', 'run', 'pwmdriver', 'duty_cycle'
    		],
    		output='screen')
    ld.add_action(new_term)	
    pwm_control_node = Node(
        package='pwmdriver',
        executable='pwm_control_node',
    )
    ld.add_action(pwm_control_node)
    return ld
