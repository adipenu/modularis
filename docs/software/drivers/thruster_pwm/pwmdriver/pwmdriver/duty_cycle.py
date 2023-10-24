import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32

def publish_duty_cycle(node):
    while rclpy.ok():
        duty_cycle = float(input("Enter duty cycle (0.0 to 1.0): "))
        if 0.0 <= duty_cycle <= 1.0:
            node.get_logger().info(f'Publishing duty cycle: {duty_cycle}')
            msg = Float64()
            msg.data = duty_cycle
            node.duty_cycle_pub.publish(msg)
        else:
            node.get_logger().warn('Duty cycle must be between 0.0 and 1.0')

def publish_thruster(node):
    while rclpy.ok():
        thrust_num = int(input("Enter Thruster number: "))
        node.get_logger().info(f'Publishing thruster: {thrust_num}')
        msg = Int32()
        msg.data = thrust_num
        node.thrust_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = Node('input_publisher')

    node.duty_cycle_pub = node.create_publisher(Float64, 'duty_cycle', 10)
    node.thrust_pub = node.create_publisher(Int32, 'thrust', 10)

    rclpy.get_global_executor().add_node(node)

    try:
        publish_duty_cycle(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
