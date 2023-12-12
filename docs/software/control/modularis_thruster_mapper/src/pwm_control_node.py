import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32
import smbus
import time

class PWMControlNode(Node):
    def __init__(self):
        super().__init__('pwm_control_node')
        self.bus = smbus.SMBus(1)
        time.sleep(1)

        # PCA9685 register addresses
        self.PCA9685_ADDR = 0x70
        self.MODE1 = 0x00
        self.PRESCALE = 0xFE
        self.LED0_ON_L = 0x06
        self.OSC_CLK = 25000000

        self.pwm_init()
        self.duty_cycle = 0.293 #0.293 is duty cycle used to achieve 1500us pulse, which is the needed initialization pulse for ESCs
        
        self.thruster_numbers = [5, 1, 2, 3, 4, 6] 
        #Initializing all the thrusters with needed 1500us pulse
        for thruster_num in self.thruster_numbers:
            self.thrust_num = thruster_num
            self.set_pwm()

        self.thrust_sub = self.create_subscription(Int32, 'thrust', self.thrust_callback, 10) 
        self.duty_cycle_sub = self.create_subscription(Float32, 'duty_cycle', self.duty_cycle_callback, 10)

    def pwm_init(self):
      """Initializes pwm chip"""
        self.bus.write_byte_data(self.PCA9685_ADDR, self.MODE1, 0xA1)
        time.sleep(1)
        prescale_val = int(round((self.OSC_CLK / (4096 * 200)) - 1)) #Can change prescale by writing to self.Prescale
        time.sleep(1)

    def set_pwm(self):
      """Sets output PWM from desired out port from chip"""
        duty_tick_on = int(round((self.duty_cycle * 4095) / 2))
        duty_tick_off = int(round(self.duty_cycle * 4095)) + duty_tick_on
        if duty_tick_off > 4095:
            duty_tick_off = duty_tick_off - 4095

        duty_tick_array = [
            duty_tick_on & 0xFF,
            duty_tick_on >> 8,
            duty_tick_off & 0xFF,
            duty_tick_off >> 8,
        ]

        base_register = self.LED0_ON_L + 4 * self.thrust_num
        self.bus.write_i2c_block_data(self.PCA9685_ADDR, base_register, duty_tick_array)
        time.sleep(1)

    def thrust_callback(self, msg):
      """Callback for thruster number topic, logs thruster number received"""
        self.thrust_num = msg.data
        self.get_logger().info(f'Received Thrust Number: {self.thrust_num}')

    def duty_cycle_callback(self, msg):
      """Callback for duty cycle topic, logs duty cycle received"""
        self.duty_cycle = msg.data
        self.get_logger().info(f'Received Duty Cycle: {self.duty_cycle}')
        self.set_pwm()

def main(args=None):
    rclpy.init(args=args)
    node = PWMControlNode() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
