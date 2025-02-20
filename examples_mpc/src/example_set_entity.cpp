/**
 * @author postrantor@gmail.com
 * @author deepseek-reasoner
 * @brief
 * @version 0.2
 * @date 2025-01-30 16:57:19
 *
 * @copyright Copyright (c) 2025
 *
 */

#include <array>
#include <memory>
#include <optional>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/empty.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"

class GazeboEntityState : public rclcpp::Node {
public:
  GazeboEntityState() : Node("gazebo_entity_client") {}

  bool reset_simulation() {
    return call_service_bool<std_srvs::srv::Empty>("/reset_simulation", [this](auto request) { RCLCPP_INFO(get_logger(), "Simulation reset successfully"); });
  }

  bool set_entity_state(
      const std::string& entity_name,
      const std::string& reference_frame,
      const std::array<double, 3>& position,
      const std::array<double, 4>& orientation,
      const std::array<double, 3>& linear,
      const std::array<double, 3>& angular) {
    return call_service_bool<gazebo_msgs::srv::SetEntityState>("/set_entity_state", [&](auto request) {
      request->state.name = entity_name;
      request->state.reference_frame = reference_frame;
      request->state.pose.position = to_point(position);
      request->state.pose.orientation = to_quaternion(orientation);
      request->state.twist.linear = to_vector3(linear);
      request->state.twist.angular = to_vector3(angular);
    });
  }

  std::optional<gazebo_msgs::msg::EntityState> get_entity_state(const std::string& entity_name, const std::string& reference_frame = "world") {
    auto response = call_service_optional<gazebo_msgs::srv::GetEntityState>("/get_entity_state", [&](auto request) {
      request->name = entity_name;
      request->reference_frame = reference_frame;
    });

    if (response && response.value()->success) {
      return response.value()->state;
    }
    return std::nullopt;
  }

  void log_entity_state(const gazebo_msgs::msg::EntityState& state) const {
    const auto& pos = state.pose.position;
    const auto& orie = state.pose.orientation;
    const auto& linear = state.twist.linear;
    const auto& angular = state.twist.angular;

    RCLCPP_INFO_STREAM(
        get_logger(),
        "\n\tpose.position \t- x: " << pos.x << ", y: " << pos.y << ", z: " << pos.z                                                          //
                                    << "\n\tpose.oriention \t- w: " << orie.w << ", x: " << orie.x << ", y: " << orie.y << ", z: " << orie.z  //
                                    << "\n\tpose.linear \t- x: " << linear.x << ", y: " << linear.y << ", z: " << linear.z                    //
                                    << "\n\tpose.angular \t- x: " << angular.x << ", y: " << angular.y << ", z: " << angular.z);
  }

private:
  template <typename ServiceT, typename RequestCallback>
  bool call_service_bool(const std::string& service_name, RequestCallback&& callback) {
    auto client = create_client<ServiceT>(service_name);
    if (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(get_logger(), "Service %s not available", service_name.c_str());
      return false;
    }

    auto request = std::make_shared<typename ServiceT::Request>();
    callback(request);

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Service call to %s failed", service_name.c_str());
      return false;
    }

    return true;
  }

  template <typename ServiceT, typename RequestCallback>
  auto call_service_optional(const std::string& service_name, RequestCallback&& callback) -> std::optional<typename ServiceT::Response::SharedPtr> {
    auto client = create_client<ServiceT>(service_name);
    if (!client->wait_for_service(std::chrono::seconds(1))) {
      RCLCPP_ERROR(get_logger(), "Service %s not available", service_name.c_str());
      return std::nullopt;
    }

    auto request = std::make_shared<typename ServiceT::Request>();
    callback(request);

    auto future = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(get_node_base_interface(), future) != rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(get_logger(), "Service call to %s failed", service_name.c_str());
      return std::nullopt;
    }

    return future.get();
  }

  static geometry_msgs::msg::Point to_point(const std::array<double, 3>& arr) {
    geometry_msgs::msg::Point point;
    point.x = arr[0];
    point.y = arr[1];
    point.z = arr[2];
    return point;
  }

  static geometry_msgs::msg::Quaternion to_quaternion(const std::array<double, 4>& arr) {
    geometry_msgs::msg::Quaternion quaternion;
    quaternion.w = arr[0];
    quaternion.x = arr[1];
    quaternion.y = arr[2];
    quaternion.z = arr[3];
    return quaternion;
  }

  static geometry_msgs::msg::Vector3 to_vector3(const std::array<double, 3>& arr) {
    geometry_msgs::msg::Vector3 vector;
    vector.x = arr[0];
    vector.y = arr[1];
    vector.z = arr[2];
    return vector;
  }
};

int
main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto entity = std::make_shared<GazeboEntityState>();

  // TODO(postrantor@gmail.com) Some key information may be reset,
  // so in order to run normally, you need to temporarily comment on this code
  // entity->reset_simulation();

  // Set entity state parameters
  const std::array position = {0.0, 0.0, 0.38};
  const std::array orientation = {1.0, 0.0, 0.0, 0.0};
  const std::array linear = {0.0, 0.0, 0.0};
  const std::array angular = {0.0, 0.0, 0.0};

  entity->set_entity_state("robot_a1", "ground_plane", position, orientation, linear, angular);

  // Get and log entity state
  if (auto state = entity->get_entity_state("robot_a1", "ground_plane")) {
    entity->log_entity_state(*state);
  }

  rclcpp::shutdown();
  return 0;
}
