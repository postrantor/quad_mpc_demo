import rclpy
from rclpy.node import Node
from unitree_msgs.msg import MotorCmd


class PeriodicPublisher(Node):
    def __init__(self):
        super().__init__("periodic_publisher")

        # 初始化发布者
        self.publisher_ = self.create_publisher(MotorCmd, "/desired_state", 10)

        # 初始化定时器
        self.timer_ = self.create_timer(0.05, self.timer_callback)  # 50ms间隔

    def timer_callback(self):
        cmd = MotorCmd()
        cmd.q = 3.14
        cmd.kp = 0.0
        cmd.dq = 0.0  # 已经由hardware按照传动比转换为输出轴
        cmd.kd = 0.02
        cmd.tau = 0.0

        self.get_logger().info(f"publishing velocity: {cmd.dq}")
        self.publisher_.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PeriodicPublisher()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
