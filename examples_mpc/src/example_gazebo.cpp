/**
 * @brief 实现四足机器人在实际环境中的控制示例
 *
 * @details 该示例通过结合 `qr_robot.cpp` 和 `qr_robot_a1.cpp` 中提供的 API，
 * 实现对四足机器人的初始化、状态获取和控制逻辑。
 *
 * @author postrantor@gmail.com
 * @date 2025-01-31 12:07:54
 *
 * @copyright MIT License
 */

#include <future>
#include <chrono>
#include <filesystem>

#include "quadruped/robots/qr_robot_a1.hpp"
#include "quadruped/exec/qr_robot_runner.hpp"
#include "quadruped/utils/qr_utils.hpp"

#include "spdlog/spdlog.h"
#include "rclcpp/rclcpp.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_srvs/srv/empty.hpp"
#include "gazebo_msgs/srv/get_entity_state.hpp"
#include "gazebo_msgs/srv/set_entity_state.hpp"

constexpr static uint32_t FUTURE_TIMEOUT{2000};
const static std::string ROBOT_NAME{"robot_a1"};
const static std::string REFERENCE_FRAME{"ground_plane"};
const static std::string GZ_GET_ENTITY_STATE_SERVICE{"/get_entity_state"};
const static std::string GZ_SET_ENTITY_STATE_SERVICE{"/set_entity_state"};
const static std::string CONFIG_DIR{ament_index_cpp::get_package_share_directory("quadruped") + "/../../config/a1_sim/"};
const static std::string CONFIG_A1_FILE{CONFIG_DIR + "/a1_sim.yaml"};

auto static logger{qrLoggerFactory::GetLogger("gazebo::example")};

void
log_entity_state(const gazebo_msgs::msg::EntityState& state) {
  const auto& pos = state.pose.position;
  const auto& orie = state.pose.orientation;
  const auto& linear = state.twist.linear;
  const auto& angular = state.twist.angular;

  logger->info(
      "\n\tpose.position \t- x: {}, y: {}, z: {}"
      "\n\tpose.orie \t- w: {}, x: {}, y: {}, z: {}"
      "\n\tpose.linear \t- x: {}, y: {}, z: {}"
      "\n\tpose.angular \t- x: {}, y: {}, z: {}",
      pos.x, pos.y, pos.z, orie.w, orie.x, orie.y, orie.z, linear.x, linear.y, linear.z, angular.x, angular.y, angular.z);
}

/**
 * @brief 获取机器人在世界坐标系中的位姿
 *
 * @param quadruped 指向机器人实例的指针
 * @param node
 * @param client_base_state 获取链接状态的服务客户端
 */
bool
get_com_position_in_world_frame(
    const std::shared_ptr<Quadruped::qrRobot>& quadruped,
    const std::shared_ptr<rclcpp::Node>& node,
    const std::shared_ptr<rclcpp::Client<gazebo_msgs::srv::GetEntityState>>& client_base_state) {
  // send request
  auto request = std::make_shared<gazebo_msgs::srv::GetEntityState::Request>();
  request->name = ROBOT_NAME;
  request->reference_frame = REFERENCE_FRAME;

  auto response_future = client_base_state->async_send_request(request);
  if (!(rclcpp::spin_until_future_complete(
            node, response_future,  //
            std::chrono::milliseconds(FUTURE_TIMEOUT)) == rclcpp::FutureReturnCode::SUCCESS)) {
    logger->error("[reset_robot] call {} service failed", GZ_GET_ENTITY_STATE_SERVICE);
    return false;
  }

  auto response_result = response_future.get();
  if (!response_result->success) {
    logger->error("[reset_robot] failed to get gazebo link state");
    return false;
  }

  // paser gazebo position and velocity information sent to quadruped object
  Vec3<double> position{
      response_result->state.pose.position.x,  //
      response_result->state.pose.position.y,  //
      response_result->state.pose.position.z};
  Quat<double> orientation{
      response_result->state.pose.orientation.w,  //
      response_result->state.pose.orientation.x,  //
      response_result->state.pose.orientation.y,  //
      response_result->state.pose.orientation.z};
  Vec3<double> twist{
      response_result->state.twist.linear.x,  //
      response_result->state.twist.linear.y,  //
      response_result->state.twist.linear.z};

  // 更新机器人实例的状态信息
  quadruped->base_position = position.cast<float>();
  quadruped->base_orientation = orientation.cast<float>();
  quadruped->base_velocity_in_base_frame = twist.cast<float>();
  // 机器人足端的位置在world frame中的表达
  quadruped->foot_position_in_world_frame = quadruped->GetFootPositionsInWorldFrame(
      true,                    //
      position.cast<float>(),  //
      orientation.cast<float>());

  log_entity_state(response_result->state);

  return true;
}

/**
 * @brief 重置机器人的姿态和关节状态
 *
 * @note 需要显示加载 `gazebo_ros_state` 插件
 *       这里只会重置机器人整体的姿态，并不控制具体的关节
 *       因此，如果之前关节处于站立，则依然会站立，只是base的位姿变化
 *       可以得出，这里的重置是通过设置机器人的位置和姿态来实现的
 * @param node 设置模型状态的服务客户端
 * @param entity_state_client 设置关节状态的服务客户端
 * @return 是否成功重置机器人的状态
 */
bool
reset_robot(const std::shared_ptr<rclcpp::Node>& node, const std::shared_ptr<rclcpp::Client<gazebo_msgs::srv::SetEntityState>>& entity_state_client) {
  // set request
  auto entity_state_request = std::make_shared<gazebo_msgs::srv::SetEntityState::Request>();

  entity_state_request->state.name = ROBOT_NAME;
  entity_state_request->state.reference_frame = REFERENCE_FRAME;

  // 若机器人初始关节位姿差异较大，Z轴设置为 0 或较低值，会是机器人翻滚
  // 因此，此处Z轴设置高度为机器人站立时高度，以重置姿态时发生翻滚
  entity_state_request->state.pose.position.x = .0;
  entity_state_request->state.pose.position.y = .0;
  entity_state_request->state.pose.position.z = .40;

  // It should match the initial value in quadruped.
  entity_state_request->state.pose.orientation.w = 1.0;
  entity_state_request->state.pose.orientation.x = .0;
  entity_state_request->state.pose.orientation.y = .0;
  entity_state_request->state.pose.orientation.z = .0;

  entity_state_request->state.twist.linear.x = .0;
  entity_state_request->state.twist.linear.y = .0;
  entity_state_request->state.twist.linear.z = .0;

  entity_state_request->state.twist.angular.x = .0;
  entity_state_request->state.twist.angular.y = .0;
  entity_state_request->state.twist.angular.z = .0;

  // asynchronously send requests to set entity state and joint state
  auto result_future = entity_state_client->async_send_request(entity_state_request);

  if (!(rclcpp::spin_until_future_complete(
            node->get_node_base_interface(), result_future,  //
            std::chrono::milliseconds(FUTURE_TIMEOUT)) == rclcpp::FutureReturnCode::SUCCESS)) {
    logger->warn("[reset_robot] failed to call service: {}", GZ_SET_ENTITY_STATE_SERVICE);
    return false;
  }

  if (!result_future.get()->success) {
    logger->warn("[reset_robot] failed to set joint state");
    return false;
  }

  return true;
}

/**
 * @brief 初始化并重置机器人
 *
 * @details 初始化ROS节点和服务客户端，重置机器人姿态
 * @param node ROS Node
 * @return Quadruped 实例
 */
std::shared_ptr<Quadruped::qrRobotA1>
initialize_and_reset_robot(const std::shared_ptr<rclcpp::Node>& node) {
  logger->info("[init_reset] initialize and reset robot position ...");

  auto entity_state_client = node->create_client<gazebo_msgs::srv::SetEntityState>(GZ_SET_ENTITY_STATE_SERVICE);

  // wait for service available
  if (!entity_state_client->wait_for_service(std::chrono::milliseconds(FUTURE_TIMEOUT))) {
    logger->error("[init_reset] {} service not available.", GZ_SET_ENTITY_STATE_SERVICE);
  }

  // 重置机器人状态
  if (!reset_robot(node, entity_state_client)) {
    logger->error("[init_reset] reset robot failed.");
  }

  // 重置机器人状态后，应稳定一段时间，便于获取机器人稳定位姿
  // std::this_thread::sleep_for(std::chrono::milliseconds(5000));

  // 构建配置文件路径
  if (!std::filesystem::exists(CONFIG_A1_FILE)) {
    logger->error("[init_reset] couldn`t found config file: \n`{}`.", CONFIG_A1_FILE);
    return nullptr;
  }

  return std::make_shared<Quadruped::qrRobotA1>(node, CONFIG_A1_FILE);
}

/**
 * @brief control loop
 *
 * @details 执行机器人控制逻辑
 * @param quadruped Quadruped 实例
 * @param node ROS节点
 */
void
control_loop(  //
    const std::shared_ptr<Quadruped::qrRobotA1>& quadruped,
    const std::shared_ptr<rclcpp::Node>& node) {
  logger->info("[control_loop] entry control loop ... ...");

  // 1. 初始化机器人并接收初始观测数据
  quadruped->Step(Mat5x12<float>::Zero(), Quadruped::HYBRID_MODE);  // robot will stop
  logger->info("[control_loop] base orientation: \n\t{}", quadruped->GetBaseOrientation());

  // 2. 创建机器人运行实例
  qrRobotRunner robot_runner(quadruped, CONFIG_DIR, node);
  logger->info("[control_loop] robot_runner init finished");
  logger->info("[control_loop] time since reset: {}", quadruped->GetTimeSinceReset());

  // 3. 创建服务客户端，获取初始状态
  auto base_state_client = node->create_client<gazebo_msgs::srv::GetEntityState>(GZ_GET_ENTITY_STATE_SERVICE);
  if (!base_state_client->wait_for_service(std::chrono::seconds(2))) {
    logger->error("[control_loop] {} not available.", GZ_GET_ENTITY_STATE_SERVICE);
  }
  if (!get_com_position_in_world_frame(quadruped, node, base_state_client)) {
    logger->error("[control_loop] get common position in world failed.");
  }

  // 4. 定时器执行
  logger->info("start control loop ... ...");

  float start_time{quadruped->GetTimeSinceReset()};
  float avg_cost{.0f};
  int count{0};

  // 定义定时器回调函数
  std::function<void()> control_timer_callback = [&]() mutable {
    static float start_time_wall = quadruped->GetTimeSinceReset();
    static auto current_time = start_time_wall;

    // 检查时间条件
    current_time = quadruped->GetTimeSinceReset();
    if (current_time - start_time >= 1000.0f) {
      logger->info("[control_loop] timeout reached, exiting...");
      rclcpp::shutdown();
      return;
    }

    // 执行控制逻辑
    start_time_wall = quadruped->GetTimeSinceReset();
    // TODO(postrantor@gmail.com) 会通过调用`Quadruped::Action::StandUp`切换机器人为蹲卧姿态
    robot_runner.Update();  // robot will sit down
    robot_runner.Step();

    // 计算每1000次迭代的平均耗时，需要与controller的计算频率相符
    avg_cost += quadruped->GetTimeSinceReset() - start_time_wall;
    if ((count + 1) % 1000 == 0) {
      logger->info("[control_loop] avg time cost: {}", avg_cost);
      avg_cost = .0f;
    }

    // 检查机器人状态
    if (quadruped->base_position[2] < 0.1f ||                          //
        quadruped->base_position[2] > 0.4f ||                          //
        quadruped->state_data_flow->height_in_control_frame < .05f ||  //
        std::abs(quadruped->base_roll_pitch_yaw[0]) > 0.6f) {
      logger->info("[control_loop] the robot is falling. exiting...");
      logger->info(
          "\n\t- base_position: {}"
          "\n\t- base_rpy: {}"
          "\n\t- height(control_frame): {}",
          quadruped->base_position,        //
          quadruped->base_roll_pitch_yaw,  //
          quadruped->state_data_flow->height_in_control_frame);
    }

    count++;
  };

  // choose the timer interval based on the simulation frequency
  // FIXME(postrantor@gmail.com) if > 666Hz; 0.001s else 0.002s
  auto control_timer = node->create_wall_timer(std::chrono::milliseconds((quadruped->time_step < .0015f) ? 1 : 2), control_timer_callback);

  // 创建多线程执行器
  // FIXME(@postrantor) libquadruped is not thread safe?
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
}

/**
 * @brief 主函数
 * @details 初始化ROS节点，执行主控制循环
 */
int
main(int argc, char** argv) {
  rclcpp::init(argc, argv);

  logger->info("[main] create `robot_sim` node.");
  auto node = rclcpp::Node::make_shared("robot_sim");
  auto robot_sim = initialize_and_reset_robot(node);

  if (!robot_sim) {
    logger->error("[main] create `robot_sim` node failed.");
    return 0;
  }

  try {
    control_loop(robot_sim, node);
  } catch (const std::runtime_error& e) {
    logger->error("[main] control_loop error:\n\t {}", e.what());
    return 1;
  }

  rclcpp::shutdown();

  return 1;
}
