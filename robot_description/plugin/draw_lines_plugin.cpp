/**
 * @brief DrawLinesPlugin 插件用于在 Gazebo 中绘制动态线条。
 * @author GPT4-o
 * @date 2024-06-23 02:31:58
 * @copyright Copyright(c) 2018 - 2019,
 *   Unitree Robotics.Co.Ltd.All rights reserved.Use of this source code is governed by the MPL
 * - 2.0 license,
 */

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

#include "gazebo/common/Events.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/transport/Node.hh"
#include "ignition/math/Color.hh"
#include <memory>
#include <string>

namespace gazebo {

class DrawLinesPlugin : public VisualPlugin {
public:
  // 构造函数
  DrawLinesPlugin() = default;

  // 析构函数
  ~DrawLinesPlugin() override {
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
      RCLCPP_INFO(rclcpp::get_logger("DrawLinesPlugin"), "缺少 <topicName> 元素，默认使用 /default_force_draw");
      topic_name_ = "/default_force_draw";
    } else {
      topic_name_ = sdf->Get<std::string>("topicName");
    }

    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }

    line_ = visual_->CreateDynamicLine(rendering::RENDERING_LINE_STRIP);
    line_->AddPoint(ignition::math::Vector3d(0, 0, 0));
    line_->AddPoint(ignition::math::Vector3d(100, 100, 100));
    line_->setMaterial("Gazebo/Red");
    line_->setVisibilityFlags(GZ_VISIBILITY_GUI);
    visual_->SetVisible(true);

    ros_node_ = std::make_shared<rclcpp::Node>(visual_namespace_);
    pose_sub_ =
        ros_node_->create_subscription<geometry_msgs::msg::Pose>(topic_name_ + "/" + "xyz_line", 30, std::bind(&DrawLinesPlugin::GetLineCallback, this, std::placeholders::_1));

    update_connection_ = event::Events::ConnectPreRender(std::bind(&DrawLinesPlugin::OnUpdate, this));

    RCLCPP_INFO(ros_node_->get_logger(), "加载 %s 画线插件。", topic_name_.c_str());
  }

private:
  /**
   * @brief 更新函数，在每次渲染之前调用
   */
  void OnUpdate() {
    // 更新线条的终点
    line_->SetPoint(1, ignition::math::Vector3d(x_, y_, z_));
  }

  /**
   * @brief 回调函数，用于接收并处理 Pose 消息
   * @param msg 接收到的 Pose 消息指针
   */
  void GetLineCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    x_ = msg->position.x;
    y_ = msg->position.y;
    z_ = msg->position.z;
    RCLCPP_INFO(ros_node_->get_logger(), "x = %f, y = %f, z = %f", x_, y_, z_);
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
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
  // 连接事件
  event::ConnectionPtr update_connection_;
  // 线条终点坐标
  double x_ = 1, y_ = 1, z_ = 1;
};

// 注册插件
GZ_REGISTER_VISUAL_PLUGIN(DrawLinesPlugin)

}  // namespace gazebo
