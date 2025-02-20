'''
  @brief
  @author postrantor@gmail.com
  @date 2025-02-04 19:28:16
  @copyright Copyright (c) 2025
'''


import rclpy
from rclpy.node import Node
from unitree_msgs.msg import MotorCmd

class PeriodicPublisher(Node):
  def __init__(self):
    super().__init__("periodic_publisher")

    # 初始化发布者
    self.publisher_calf_ = self.create_publisher(MotorCmd, "/FL_calf_controller/desired_state", 10)
    self.publisher_hip_ = self.create_publisher(MotorCmd, "/FL_hip_controller/desired_state", 10)
    self.publisher_thigh_ = self.create_publisher(MotorCmd, "/FL_thigh_controller/desired_state", 10)

    # 初始化定时器
    self.timer_ = self.create_timer(0.05, self.timer_callback)  # 50ms间隔

  def send_zero_command(self):
    cmd = MotorCmd()
    cmd.mode.value = MotorCmd.MODE_BRAKE

    self.get_logger().info("Sending zero command before shutdown")
    self.publisher_calf_.publish(cmd)
    self.publisher_hip_.publish(cmd)
    self.publisher_thigh_.publish(cmd)

  def timer_callback(self):
    cmd = MotorCmd()
    cmd.mode.value = MotorCmd.MODE_FOC
    cmd.q = 3.14
    cmd.kp = 0.0
    cmd.dq = 0.0  # 已经由hardware按照传动比转换为输出轴
    cmd.kd = 0.02
    cmd.tau = 0.0

    self.get_logger().info(f"publishing velocity: {cmd.dq}")
    self.publisher_calf_.publish(cmd)
    self.publisher_hip_.publish(cmd)
    self.publisher_thigh_.publish(cmd)

def main(args=None):
  rclpy.init(args=args)

  node = PeriodicPublisher()

  # 添加关闭前的回调
  def shutdown_callback():
      node.send_zero_command()

  rclpy.signals.add_signal_handler(signal.SIGINT, shutdown_callback)
  rclpy.signals.add_signal_handler(signal.SIGTERM, shutdown_callback)

  rclpy.spin(node)

if __name__ == "__main__":
  main()
