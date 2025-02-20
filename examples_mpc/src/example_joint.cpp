/**
 * @brief
 *
 * @author postrantor@gmail.com
 * @author GPT4-o
 * @date 2024-11-09 17:44:52
 *
 * @copyright MIT License
 *
 */

#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "unitree_msgs/msg/motor_cmd.hpp"

using namespace std::chrono_literals;

class PeriodicPublisher : public rclcpp::Node {
public:
  PeriodicPublisher() : Node("periodic_publisher") {
    publisher_calf_ = this->create_publisher<unitree_msgs::msg::MotorCmd>("/FL_calf_controller/desired_state", 10);
    publisher_hip_ = this->create_publisher<unitree_msgs::msg::MotorCmd>("/FL_hip_controller/desired_state", 10);
    publisher_thigh_ = this->create_publisher<unitree_msgs::msg::MotorCmd>("/FL_thigh_controller/desired_state", 10);

    timer_ = this->create_wall_timer(50ms, std::bind(&PeriodicPublisher::timer_callback, this));
  }

  void send_zero_command() {
    auto cmd = unitree_msgs::msg::MotorCmd();
    cmd.mode.value = unitree_msgs::msg::MotorMode::MODE_BRAKE;

    RCLCPP_INFO_STREAM(this->get_logger(), "Sending zero command before shutdown");
    publisher_calf_->publish(cmd);
    publisher_hip_->publish(cmd);
    publisher_thigh_->publish(cmd);
  }

private:
  void timer_callback() {
    auto cmd = unitree_msgs::msg::MotorCmd();
    cmd.mode.value = unitree_msgs::msg::MotorMode::MODE_FOC;
    cmd.q = .5f;
    cmd.kp = .0f;
    cmd.dq = .0f;  // 已经由hardware按照传动比转换为输出轴
    cmd.kd = .0f;
    cmd.tau = .0f;

    RCLCPP_INFO_STREAM(this->get_logger(), "publishing velocity: " << cmd.dq);
    publisher_calf_->publish(cmd);
    publisher_hip_->publish(cmd);
    publisher_thigh_->publish(cmd);
  }

  rclcpp::Publisher<unitree_msgs::msg::MotorCmd>::SharedPtr publisher_calf_;
  rclcpp::Publisher<unitree_msgs::msg::MotorCmd>::SharedPtr publisher_hip_;
  rclcpp::Publisher<unitree_msgs::msg::MotorCmd>::SharedPtr publisher_thigh_;
  rclcpp::TimerBase::SharedPtr timer_;
};

bool
add_shutdown_callback(rclcpp::Node::SharedPtr node) {
  auto context = rclcpp::contexts::get_global_default_context();
  if (!context->is_valid()) {
    RCLCPP_ERROR(node->get_logger(), "Context is not valid, cannot add shutdown callback");
    return false;
  }

  // 将节点转换为 PeriodicPublisher 类型以访问 send_zero_command 方法
  auto periodic_publisher = std::dynamic_pointer_cast<PeriodicPublisher>(node);
  if (!periodic_publisher) {
    RCLCPP_ERROR(node->get_logger(), "Failed to cast node to PeriodicPublisher");
    return false;
  }

  // 添加关闭前的回调函数
  context->add_pre_shutdown_callback([periodic_publisher]() { periodic_publisher->send_zero_command(); });

  return true;
}

int
main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  auto node = std::make_shared<PeriodicPublisher>();
  rclcpp::executors::SingleThreadedExecutor executor;

  if (!add_shutdown_callback(node)) {
    RCLCPP_ERROR(node->get_logger(), "Failed to add shutdown callback");
    return 1;
  }

  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}
