import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8MultiArray
import serial

data = [100, 100, 100, 100]

class serial_receive(Node):
    def __init__(self):
        super().__init__('serial_node')
        #self.ser = serial.Serial("/dev/ttyACM0", 115200, timeout=1)
        self.pub = self.create_publisher(Int8MultiArray, 'receive_plan_sub', 10)
        self.sub = self.create_subscription(Int8MultiArray, 'send_plan_pub', self.callback, 10)
        self.timer = self.create_timer(0.001, self.timer_callback)  # タイマーの生成

    def callback(self, sub_msg):
        print(sub_msg.data)


    def timer_callback(self):
        msg =Int8MultiArray()
        msg.data = data
        self.pub.publish(msg)

def main():
    print('START!!')
    rclpy.init()
    node = serial_receive()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()