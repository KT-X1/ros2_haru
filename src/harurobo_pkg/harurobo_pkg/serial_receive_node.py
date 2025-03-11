import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray
import serial
import struct

class SerialToPositionNode(Node):
    def __init__(self):
        super().__init__('serial_receive_node')

        self.ser = serial.Serial("/dev/ttyACM0", baudrate=9600)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'robot_position', 10) #(serial_receive_node->(robot_position)->position_node
        self.timer = self.create_timer(0.1, self.timer_callback)  # より頻繁にバッファをチェックするためのタイマー
        self.buffer = b''

    def read_frame(ser):
        while True:
            first_byte = ser.read(1)
            if (first_byte == b'\xA5'):
                second_byte = ser.read(1)
                if (second_byte == b'\xA5'):
                    break
        payload = ser.read(4)
        return payload

    def timer_callback(self):
        if self.ser.in_waiting > 0:
            self.buffer += self.ser.read(self.ser.in_waiting)

        # バッファに完全なパケット（8バイト、6バイトのデータ + 2バイトのプレフィックス）が含まれているか確認
        while len(self.buffer) >= 8:
            # パケットの開始を見つける
            start = self.buffer.find(b'\xA5\xA5')
            if start != -1 and len(self.buffer[start:]) >= 8:
                data = self.buffer[start+2:start+8]
                self.buffer = self.buffer[start+8:]
                try:
                    command_data = struct.unpack('>BBhhh', data)
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
            else:
                # プレフィックスが正しくない場合、またはデータが不十分な場合、最初のバイトを破棄して再試行
                self.buffer = self.buffer[start+1:] if start != -1 else self.buffer[1:]

def main(args=None):
    rclpy.init(args=args)
    node = SerialToPositionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()