import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import can
import struct

class SerialToPositionNode(Node):
    def __init__(self):
        super().__init__('serial_receive_node')

        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        self.publisher_ = self.create_publisher(Float32MultiArray, 'robot_position', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # より頻繁にバッファをチェックするためのタイマー

    def timer_callback(self):
        msg = self.bus.recv(timeout=0.1)
        if msg is not None and msg.arbitration_id == 0x150:
            try:
                command_data = struct.unpack('>BBhhh', msg.data)
                command_msg = Float32MultiArray()
                command_msg.data = [
                    float(command_data[0]),  # 指示番号
                    float(command_data[1]),  # モード
                    float(command_data[2]),  # Vx
                    float(command_data[3]),  # Vy
                    float(command_data[4])   # ω
                ]
                self.publisher_.publish(command_msg)
                self.get_logger().info(f"Published command data: {command_msg.data}")
            except struct.error as e:
                self.get_logger().error(f"Unpacking error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialToPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()