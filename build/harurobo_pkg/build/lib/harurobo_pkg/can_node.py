import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import can
import struct

class CANNode(Node):
    def __init__(self):
        super().__init__('can_node')

        # CANバスの設定
        try:
            self.bus = can.Bus(interface='socketcan', channel='can0')
        except OSError as e:
            self.get_logger().error(f"Could not access SocketCAN device can0: {e}")
            rclpy.shutdown()
            return

        self.publisher_ = self.create_publisher(Float32MultiArray, 'robot_position', 10)
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'send_can_message',
            self.send_can_message_callback,
            10
        )
        self.timer = self.create_timer(0.001, self.timer_callback)  # 1msに一回
        self.action_number = 0

    def send_can_message_callback(self, msg):
        if len(msg.data) == 3:
            vx = int(msg.data[0])  # Vx
            vy = int(msg.data[1])  # Vy
            omega = int(msg.data[2])  # ω

            # CANデータのパッキング
            data = struct.pack('>hhh', vx, vy, omega)
            can_msg = can.Message(arbitration_id=0x360, data=data, is_extended_id=False)
            try:
                self.bus.send(can_msg)  # CANバスに送信
                self.get_logger().info(f"Sent packed data 0x360: {data.hex()}")
                self.get_logger().debug(f"Sent CAN message 0x360: {can_msg}")
                self.get_logger().debug(f"Data sent - Vx: {vx}, Vy: {vy}, Omega: {omega}")
            except can.CanError:
                self.get_logger().error("Failed to send CAN message")

    def timer_callback(self):
        msg = self.bus.recv(timeout=0.1)
        if msg is not None:
            if msg.arbitration_id == 0x360:
                self.get_logger().debug(f"Received CAN message: {msg}")
                try:
                    # CANデータのアンパッキング
                    x = struct.unpack('>h', msg.data[0:2])[0]
                    y = struct.unpack('>h', msg.data[2:4])[0]
                    theta = struct.unpack('>h', msg.data[4:6])[0]
                    command_msg = Float32MultiArray()
                    command_msg.data = [float(x), float(y), float(theta), float(self.action_number)]
                    self.publisher_.publish(command_msg)
                    self.get_logger().info(f"Published command data: {command_msg.data}")
                except struct.error as e:
                    self.get_logger().error(f"Unpacking error: {e}")
            elif msg.arbitration_id == 0x151:
                self.get_logger().debug(f"Received CAN message: {msg}")
                try:
                    self.action_number = struct.unpack('>B', msg.data)[0]
                    self.get_logger().info(f"Updated action number: {self.action_number}")
                except struct.error as e:
                    self.get_logger().error(f"Unpacking error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = CANNode()
    if node.bus:
        rclpy.spin(node)
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
