/**
 * @brief
 * @date 2024-06-23 02:32:03
 * @author GPT4-o
 * @copyright Copyright(c) 2018 - 2019,
 *   Unitree Robotics.Co.Ltd.All rights reserved.Use of this source code is governed by the MPL
 * - 2.0 license,
 */

#include <memory>
#include <string>

#include "gazebo/common/Events.hh"
#include "gazebo/gazebo.hh"
#include "gazebo/sensors/sensors.hh"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "rclcpp/rclcpp.hpp"

namespace gazebo {

class UnitreeFootContactPlugin : public SensorPlugin {
public:
  // 构造函数
  UnitreeFootContactPlugin() = default;

  // 析构函数
  ~UnitreeFootContactPlugin() override = default;

  /**
   * @brief 插件加载函数
   * @param sensor 传感器指针
   * @param sdf SDF 元素指针
   */
  void Load(sensors::SensorPtr sensor, sdf::ElementPtr sdf) override {
    // 确保父传感器有效
    parent_sensor_ = std::dynamic_pointer_cast<sensors::ContactSensor>(sensor);
    if (!parent_sensor_) {
      gzerr << "UnitreeFootContactPlugin 需要一个 ContactSensor。\n";
      return;
    }

    contact_namespace_ = "contact/";
    ros_node_ = std::make_shared<rclcpp::Node>(contact_namespace_);

    // 发布力信息的话题
    force_pub_ = ros_node_->create_publisher<geometry_msgs::msg::WrenchStamped>("/visual/" + sensor->Name() + "/the_force", 100);

    // 连接到传感器更新事件
    update_connection_ = parent_sensor_->ConnectUpdated(std::bind(&UnitreeFootContactPlugin::OnUpdate, this));

    parent_sensor_->SetActive(true);  // 确保父传感器处于活动状态

    count_ = 0;
    Fx_ = 0;
    Fy_ = 0;
    Fz_ = 0;

    RCLCPP_INFO(ros_node_->get_logger(), "加载 %s 插件。", sensor->Name().c_str());
  }

private:
  /**
   * @brief 传感器更新事件回调函数
   */
  void OnUpdate() {
    auto contacts = parent_sensor_->Contacts();
    count_ = contacts.contact_size();
    Fx_ = Fy_ = Fz_ = 0;

    for (auto i = 0; i < count_; ++i) {
      if (contacts.contact(i).position_size() != 1) {
        RCLCPP_ERROR(ros_node_->get_logger(), "接触数量不正确！");
      }
      for (auto j = 0; j < contacts.contact(i).position_size(); ++j) {
        Fx_ += contacts.contact(i).wrench(0).body_1_wrench().force().x();
        Fy_ += contacts.contact(i).wrench(0).body_1_wrench().force().y();
        Fz_ += contacts.contact(i).wrench(0).body_1_wrench().force().z();
      }
    }

    auto force_msg = geometry_msgs::msg::WrenchStamped();
    if (count_ != 0) {
      force_msg.wrench.force.x = Fx_ / static_cast<double>(count_);
      force_msg.wrench.force.y = Fy_ / static_cast<double>(count_);
      force_msg.wrench.force.z = Fz_ / static_cast<double>(count_);
    } else {
      force_msg.wrench.force.x = 0;
      force_msg.wrench.force.y = 0;
      force_msg.wrench.force.z = 0;
    }

    force_msg.header.stamp = ros_node_->get_clock()->now();
    force_pub_->publish(force_msg);
  }

  // ROS 节点
  rclcpp::Node::SharedPtr ros_node_;
  // ROS 话题发布器
  rclcpp::Publisher<geometry_msgs::msg::WrenchStamped>::SharedPtr force_pub_;
  // 事件连接指针
  event::ConnectionPtr update_connection_;
  // 接触传感器命名空间
  std::string contact_namespace_;
  // 父传感器指针
  sensors::ContactSensorPtr parent_sensor_;
  // 计算接触力的相关变量
  int count_ = 0;
  double Fx_ = 0, Fy_ = 0, Fz_ = 0;
};

// 注册插件
GZ_REGISTER_SENSOR_PLUGIN(UnitreeFootContactPlugin)

}  // namespace gazebo
