import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64, Int32
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

        self.thrust_num = 0 #initializing thruster 0 and 1 for 1500us initialization pwm
        self.duty_cycle = 0.293
        
        self.set_pwm()
        
        self.thrust_num = 1
        self.duty_cycle = 0.293
        
        self.set_pwm()

        #self.thrust_sub = self.create_subscription(Int32, 'thrust', self.thrust_callback, 10) #getting rid of choosing thruster to send pwm signal to for now since not needed for forward
        self.duty_cycle_sub = self.create_subscription(Float64, 'duty_cycle', self.duty_cycle_callback, 10)

    def pwm_init(self):
        self.bus.write_byte_data(self.PCA9685_ADDR, self.MODE1, 0xA1)
        time.sleep(1)
        prescale_val = int(round((self.OSC_CLK / (4096 * 200)) - 1))
        time.sleep(1)

    def set_pwm(self):
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

    #def thrust_callback(self, msg):
     #   self.thrust_num = msg.data
      #  self.get_logger().info(f'Received Thrust Number: {self.thrust_num}')

    def duty_cycle_callback(self, msg):
        self.duty_cycle = msg.data
        self.get_logger().info(f'Received Duty Cycle: {self.duty_cycle}')
        self.thrust_num = 0 #so that when duty cycle is changed it sends same duty cycle to whichever thrusters are back thrusters
        self.set_pwm()
        self.thrust_num = 1
        self.set_pwm()

def main(args=None):
    rclpy.init(args=args)
    node = PWMControlNode() 
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
