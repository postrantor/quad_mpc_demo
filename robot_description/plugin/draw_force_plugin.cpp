/**
 * @brief UnitreeDrawForcePlugin 插件用于在 Gazebo 中绘制受力方向和大小的动态线条。
 * @author GPT4-o
 * @date 2024-06-23 02:31:54
 * @copyright Copyright(c) 2018 - 2019,
 * Unitree Robotics.Co.Ltd.All rights reserved.Use of this source code is governed by the MPL - 2.0
 * license, see LICENSE.
 */

#include <memory>
#include <string>

#include "gazebo/common/Events.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/transport/Node.hh"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "ignition/math/Color.hh"
#include "rclcpp/rclcpp.hpp"

namespace gazebo {

class UnitreeDrawForcePlugin : public VisualPlugin {
public:
  // 构造函数
  UnitreeDrawForcePlugin() = default;

  // 析构函数
  ~UnitreeDrawForcePlugin() override {
    if (visual_ && line_) {
      visual_->DeleteDynamicLine(line_);
    }
  }

  /**
   * @brief 插件加载函数
   * @param parent 父视觉对象指针
   * @param sdf SDF 元素指针
   */
  void Load(rendering::VisualPtr parent, sdf::ElementPtr sdf) override {
    visual_ = parent;
    visual_namespace_ = "visual/";

    if (!sdf->HasElement("topicName")) {
      RCLCPP_INFO(rclcpp::get_logger("UnitreeDrawForcePlugin"), "缺少 <topicName> 元素，默认使用 /default_force_draw");
      topic_name_ = "/default_force_draw";
    } else {
      topic_name_ = sdf->Get<std::string>("topicName");
    }

    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    line_ = visual_->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
    line_->AddPoint(ignition::math::Vector3d(0, 0, 0));
    line_->AddPoint(ignition::math::Vector3d(1, 1, 1));
    line_->setMaterial("Gazebo/Purple");
    line_->setVisibilityFlags(GZ_VISIBILITY_GUI);
    visual_->SetVisible(true);

    ros_node_ = std::make_shared<rclcpp::Node>(visual_namespace_);
    force_sub_ = ros_node_->create_subscription<geometry_msgs::msg::WrenchStamped>(
        topic_name_ + "/" + "the_force", 30, std::bind(&UnitreeDrawForcePlugin::GetForceCallback, this, std::placeholders::_1));

    update_connection_ = event::Events::ConnectPreRender(std::bind(&UnitreeDrawForcePlugin::OnUpdate, this));

    RCLCPP_INFO(ros_node_->get_logger(), "加载 %s 受力绘制插件。", topic_name_.c_str());
  }

private:
  /**
   * @brief 更新函数，在每次渲染之前调用
   */
  void OnUpdate() { line_->SetPoint(1, ignition::math::Vector3d(Fx_, Fy_, Fz_)); }

  /**
   * @brief 回调函数，用于接收并处理 WrenchStamped 消息
   * @param msg 接收到的 WrenchStamped 消息指针
   */
  void GetForceCallback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
    Fx_ = msg->wrench.force.x;
    Fy_ = msg->wrench.force.y;
    Fz_ = msg->wrench.force.z;
  }

  // ROS 节点
  rclcpp::Node::SharedPtr ros_node_;
  // 话题名称
  std::string topic_name_;
  // 父视觉对象
  rendering::VisualPtr visual_;
  // 动态线条
  rendering::DynamicLines* line_ = nullptr;
  // 视觉命名空间
  std::string visual_namespace_;
  // 订阅器
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr force_sub_;
  // 力的分量
  double Fx_ = 0, Fy_ = 0, Fz_ = 0;
  // 连接事件
  event::ConnectionPtr update_connection_;
};

// 注册插件
GZ_REGISTER_VISUAL_PLUGIN(UnitreeDrawForcePlugin)

}  // namespace gazebo
