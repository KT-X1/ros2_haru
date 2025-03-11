import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import can
import struct

class SerialSendNode(Node):
    def __init__(self):
        super().__init__('serial_send_node')

        # CANバスの設定
        self.bus = can.interface.Bus(channel='can0', bustype='socketcan')
        self.subscription = self.create_subscription(
            Float32MultiArray,
            'cmd_vel',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        if len(msg.data) == 5:
            action_number = int(msg.data[0])  # 動作番号
            team_color = int(msg.data[1])  # チームカラー
            x = int(msg.data[2])  # X座標
            y = int(msg.data[3])  # Y座標
            theta = int(msg.data[4])  # 角度

            # CANデータのパッキング
            data = struct.pack('>BBhhh', action_number, team_color, x, y, theta)
            msg = can.Message(arbitration_id=0x160, data=data, is_extended_id=False)
            self.bus.send(msg)  # CANバスに送信
            self.get_logger().info(f"Sent data: {msg.data}")
            self.get_logger().info(f"Sent packed data: {data.hex()}")

def main(args=None):
    rclpy.init(args=args)
    node = SerialSendNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()