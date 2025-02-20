/**
 * @brief Implementation of a control example for a quadruped robot in a real environment
 *
 * @details This example demonstrates the initialization, state retrieval, and control logic
 * of a quadruped robot by utilizing the APIs provided in `qr_robot.cpp` and `qr_robot_a1.cpp`.
 *
 * @author postrantor@gmail.com
 * @date 2025-02-20 14:39:43
 *
 * @copyright MIT License
 */

#include <chrono>
#include <csignal>
#include <future>
#include <filesystem>

#include "quadruped/exec/qr_robot_runner.hpp"
#include "quadruped/robots/qr_robot_a1.hpp"
#include "quadruped/utils/qr_utils.hpp"

#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "std_srvs/srv/empty.hpp"

const static std::string ROBOT_NAME{"robot_a1"};
const static std::string CONFIG_DIR{ament_index_cpp::get_package_share_directory("quadruped") + "/../../config/a1_real/"};
const static std::string CONFIG_A1_FILE{CONFIG_DIR + "/a1_real.yaml"};

auto static logger{qrLoggerFactory::GetLogger("real::example")};
static std::shared_ptr<Quadruped::qrRobotA1> g_robot;

/**
 * @brief Get the center of mass position of the robot in the world frame
 *
 * @param quadruped Pointer to the robot instance
 * @return bool Returns true if successful, otherwise false
 */
bool
get_com_position_in_world_frame(const std::shared_ptr<Quadruped::qrRobotA1>& quadruped) {
  // Get the base position and orientation of the robot
  Vec3<float> position = quadruped->GetBasePosition();
  Quat<float> orientation = quadruped->GetBaseOrientation();
  Vec3<float> velocity = quadruped->GetBaseVelocityInBaseFrame();

  // Update the state of the robot instance
  quadruped->base_position = position;
  quadruped->base_orientation = orientation;
  quadruped->base_velocity_in_base_frame = velocity;

  // Calculate and get the foot positions in the world frame
  quadruped->foot_position_in_world_frame = quadruped->GetFootPositionsInWorldFrame(
      true,      //
      position,  //
      orientation);

  return true;
}

/**
 * @brief Reset the robot's pose and joint states through the real hardware interface
 *
 * @param quadruped Pointer to the robot instance
 * @return bool Returns true if successful, otherwise false
 */
bool
reset_robot(const std::shared_ptr<Quadruped::qrRobotA1>& quadruped) {
  // apply the initial pose and joint angles, use `sit_down_motor_angles`
  quadruped->Reset();
  Quadruped::Action::SitDown(quadruped, 2.f, 0.001);

  return true;
}

/**
 * @brief Initialize and reset the robot
 *
 * @details Initialize the ROS node and instantiate the robot, then reset the robot's pose
 * @param node ROS Node
 * @return Quadruped::qrRobotA1 Instantiated robot object
 */
std::shared_ptr<Quadruped::qrRobotA1>
initialize_and_reset_robot(const std::shared_ptr<rclcpp::Node>& node) {
  logger->info("[init_reset] initializing and resetting the real robot...");

  auto robot_real = std::make_shared<Quadruped::qrRobotA1>(node, CONFIG_A1_FILE);

  // Reset the robot state
  if (!reset_robot(robot_real)) {
    logger->error("[init_reset] failed to reset the robot.");
    return nullptr;
  }

  g_robot = robot_real;

  logger->info("[init_reset] robot initialization and reset completed.");
  return robot_real;
}

/**
 * @brief Control loop
 *
 * @details Execute the robot control logic, including state retrieval and control commands
 * @param quadruped Robot instance
 * @param node ROS node
 */
void
control_loop(const std::shared_ptr<Quadruped::qrRobotA1>& quadruped, const std::shared_ptr<rclcpp::Node>& node) {
  logger->info("[control_loop] entering control loop...");

  // 1. Initialize the robot and receive initial observation data
  quadruped->Step(Mat5x12<float>::Zero(), Quadruped::HYBRID_MODE);  // The robot will crouch
  logger->info("[control_loop] base orientation: \n\t- {}", quadruped->GetBaseOrientation());

  // 2. Create the robot runner instance
  qrRobotRunner robot_runner(quadruped, CONFIG_DIR, node);
  logger->info("[control_loop] ros module initialization completed.");
  logger->info("[control_loop] time since reset: {}", quadruped->GetTimeSinceReset());

  // 3. Get the initial state
  if (!get_com_position_in_world_frame(quadruped)) {
    logger->error("[control_loop] failed to get the center of mass position in the world frame.");
  }

  // 4. Timer logic replaces the main loop
  logger->info("starting control loop...");

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

    // 计算平均耗时
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

  // create a single-threaded executor
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();
}

bool
add_shutdown_callback(const std::shared_ptr<Quadruped::qrRobotA1>& quadruped) {
  auto context = rclcpp::contexts::get_global_default_context();
  if (!context->is_valid()) {
    logger->error("[main] context is invalid.");
    return false;
  }
  // Send zero command before shutdown
  context->add_pre_shutdown_callback([quadruped]() {
    logger->info("[shutdown_callback] sending zero command to the robot...");
    quadruped->Step(Eigen::Matrix<float, 5, 12>::Zero(), MotorMode::HYBRID_MODE);
  });
  return true;
}

void
signal_handler(int signum) {
  logger->info("Received interrupt signal({}).", signum);
  if (g_robot) {
    logger->info("sending zero command to the robot...");
    g_robot->Step(Mat5x12<float>::Zero(), Quadruped::HYBRID_MODE);
  }
  rclcpp::shutdown();
  exit(signum);
}

/**
 * @brief Main function
 *
 * @details Initialize the ROS node and execute the main control loop
 */
int
main(int argc, char** argv) {
  // Register signal handlers
  signal(SIGINT, signal_handler);
  signal(SIGTERM, signal_handler);

  // Initialize ROS
  rclcpp::init(argc, argv);

  logger->info("[main] creating `robot_real` node.");
  auto node = rclcpp::Node::make_shared("robot_real");
  auto robot_real = initialize_and_reset_robot(node);

  if (!robot_real) {
    logger->error("[main] failed to create `robot_real` node.");
    return 0;
  }

  add_shutdown_callback(robot_real);

  try {
    control_loop(robot_real, node);
  } catch (const std::exception& e) {
    logger->error("[main] control loop error:\n\t {}", e.what());
    robot_real->Step(Mat5x12<float>::Zero(), Quadruped::HYBRID_MODE);
  }

  rclcpp::shutdown();

  return 0;
}
