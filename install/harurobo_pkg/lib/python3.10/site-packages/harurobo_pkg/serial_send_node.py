import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import can
import struct

class SerialSendNode(Node):
    def __init__(self):
        super().__init__('serial_send_node')

        # CANバスの設定
        try:
            self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        except OSError as e:
            self.get_logger().error(f"Could not access SocketCAN device can0: {e}")
            rclpy.shutdown()
            return

        self.subscription = self.create_subscription(
            Float32MultiArray,
            'send_can_message',
            self.send_can_message_callback,
            10
        )
        self.timer = self.create_timer(0.001, self.timer_callback)  # 1msに一回

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
            except can.CanError as e:
                self.get_logger().error(f"Failed to send CAN message 0x360: {e}")

    def listener_callback(self, msg):
        if len(msg.data) == 5:
            vx = int(msg.data[0])  # Vx
            vy = int(msg.data[1])  # Vy
            omega = int(msg.data[2])  # ω
            action_number = int(msg.data[4])  # 指示番号

            # CANデータのパッキング
            data_160 = struct.pack('>hhh', vx, vy, omega)
            msg_160 = can.Message(arbitration_id=0x160, data=data_160, is_extended_id=False)
            try:
                self.bus.send(msg_160)  # CANバスに送信
                self.get_logger().info(f"Sent packed data 0x160: {data_160.hex()}")
                self.get_logger().debug(f"Sent CAN message 0x160: {msg_160}")
                self.get_logger().debug(f"Data sent - Vx: {vx}, Vy: {vy}, Omega: {omega}")
            except can.CanError as e:
                self.get_logger().error(f"Failed to send CAN message 0x160: {e}")

            data_161 = struct.pack('>B', action_number)
            msg_161 = can.Message(arbitration_id=0x161, data=data_161, is_extended_id=False)
            try:
                self.bus.send(msg_161)  # CANバスに送信
                self.get_logger().info(f"Sent packed data 0x161: {data_161.hex()}")
                self.get_logger().debug(f"Sent CAN message 0x161: {msg_161}")
                self.get_logger().debug(f"Data sent - Action Number: {action_number}")
            except can.CanError as e:
                self.get_logger().error(f"Failed to send CAN message 0x161: {e}")

            self.get_logger().info(f"Sent data: {msg.data}")

    def timer_callback(self):
        pass  # タイマーコールバックの追加

def main(args=None):
    rclpy.init(args=args)
    node = SerialSendNode()
    if node.bus:
        rclpy.spin(node)
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()