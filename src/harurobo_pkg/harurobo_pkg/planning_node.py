import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray
import math

class ControllerNode(Node):
    def __init__(self):
        super().__init__('planning_node')  # ノード名を変更
        self.subscription = self.create_subscription(
            String,
            'web_socket_pub',
            self.listener_callback,
            10)
        self.publisher_ = self.create_publisher(Float32MultiArray, 'cmd_vel', 10)

        # 自己位置を更新するためのサブスクリプション
        self.position_subscription = self.create_subscription(
            Float32MultiArray,
            'robot_position', #estimated_position->robot_positionに変更
            self.update_position_callback,
            10)

        self.timer = self.create_timer(0.001, self.timer_callback)  # 1msに一回

        # 地点の座標を設定
        self.locations = {
            '1': [500, 500, 0],  # [x, y, theta]
            '2': [0, 500, 0],
            '3': [0, 0, 0],
            '4': [100, 100, 0],
            '5': [200, 200, 0],
            '6': [300, 300, 0],
            '7': [400, 400, 0],
            '8': [500, 500, 0],
            '9': [600, 600, 0],
            '10': [700, 700, 0],
            '11': [800, 800, 0],
            '12': [900, 900, 0],
            '13': [1000, 1000, 0],
            '14': [1100, 1100, 0],
            '15': [1200, 1200, 0],
            '16': [1300, 1300, 0]
        }
        self.current_position = [0.0, 0.0, 0.0]  # 初期位置 [x, y, theta]
        self.max_speed = 500.0  # 最大速度 [mm/s]
        self.max_accel = 50.0  # 最大加速度 [mm/s^2]
        self.max_angular_speed = 30.0  # 最大角速度 [deg/s]
        self.mode = 0  # モード初期値
        self.current_step = 0  # 現在のステップ

    def update_position_callback(self, msg):
        # 推定された自己位置を更新
        self.current_position = [msg.data[0], msg.data[1], msg.data[2]]
        self.get_logger().info(f"Updated current position: {self.current_position}")

    def listener_callback(self, msg):
        data = msg.data.split('_')
        if len(data) < 2:
            self.get_logger().error("Received data is too short")
            return
        command_type, command_value = data[0], data[1]

        self.get_logger().info(f"Received command: {command_type}_{command_value}")

        if command_type == 'action':
            self.send_velocity_command(0.0, 0.0, 0.0, self.mode, float(command_value))
        elif command_type == 'move':
            self.move_to_target(self.locations[command_value], float(self.mode), float(0))

    def move_to_target(self, target, team_color, action_number):
        x, y, target_theta = target
        dx = x - self.current_position[0]
        dy = y - self.current_position[1]
        distance = math.sqrt(dx**2 + dy**2)  # 距離をミリメートル単位で計算
        direction = (math.degrees(math.atan2(dy, -dx)) - 90) % 360  # 方向を0-360度に設定

        self.get_logger().info(f"Moving to target: {target} with direction {direction} and distance {distance}")

        if distance < 5:  # 例えば5mm以下になったら次のステップへ
            self.current_step += 1
            return

        speed = min(self.max_speed, distance)

        # 角速度を計算
        dtheta = (target_theta - self.current_position[2] + 360) % 360
        if dtheta > 180:
            dtheta -= 360
        angular_speed = max(min(dtheta, self.max_angular_speed), -self.max_angular_speed)

        self.send_velocity_command(speed, direction, angular_speed, team_color, action_number)

    def send_velocity_command(self, speed, direction, angular_speed, team_color, action_number):
        msg = Float32MultiArray()
        msg.data = [float(speed), float(direction), float(angular_speed), float(team_color), float(action_number)]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sent velocity command: {msg.data}")
        self.get_logger().debug(f"Velocity command details - Speed: {speed}, Direction: {direction}, Angular Speed: {angular_speed}, Team Color: {team_color}, Action Number: {action_number}")

    def timer_callback(self):
        steps = [
            ('move', '1'), ('move', '2'), ('move', '3'), ('move', '4'), ('move', '5'), ('move', '6'), ('move', '7'),
            ('action', 1), ('wait', 2),
            ('move', '8'), ('move', '9'), ('move', '10'), ('move', '11'),
            ('action', 2), ('wait', 3),
            ('move', '12'), ('move', '13'), ('move', '14'), ('move', '15'), ('move', '16'),
            ('action', 3), ('wait', 4)
        ]

        if self.current_step >= len(steps):
            return

        step_type, step_value = steps[self.current_step]

        if step_type == 'move':
            self.move_to_target(self.locations[step_value], float(self.mode), float(0))
        elif step_type == 'action':
            self.send_velocity_command(0.0, 0.0, 0.0, self.mode, float(step_value))
            self.current_step += 1
        elif step_type == 'wait':
            if self.action_number == step_value:
                self.current_step += 1

def main(args=None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()