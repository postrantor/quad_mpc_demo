cmake_minimum_required(VERSION 3.10)
project(example_mpc LANGUAGES CXX)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 11)
  set(CMAKE_C_STANDARD_REQUIRED ON)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror)
  add_compile_options(-Wno-unused-parameter)
  add_compile_options(-Wno-unused-variable)
  add_compile_options(-Wno-overloaded-virtual)
endif()

# set c++ standard and build type
set(CMAKE_BUILD_TYPE "Debug") # ``Debug``, ``Release``, ``RelWithDebInfo`` and ``MinSizeRel``
set(CMAKE_CXX_FLAGS_DEBUG "-g -O0")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")
set(CMAKE_CXX_FLAGS_RELWITHDEBINFO "-O2 -g")
set(CMAKE_CXX_FLAGS_MINSIZEREL "-Os -DNDEBUG")
set(CMAKE_CXX_FLAGS "-DEIGEN_NO_DEBUG -fopenmp")
add_compile_options(-D_cplusplus -D_useros -fPIC)

# find_package
find_package(backward_ros REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(quadruped REQUIRED)
find_package(spdlog REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(unitree_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# dependency
set(DEPENDENCIES
  rclcpp
  ament_index_cpp
  backward_ros
  quadruped
  unitree_msgs
  gazebo_msgs
  std_srvs
  sensor_msgs
  nav_msgs
)

include_directories(include)

add_executable(example_keyboard
  src/example_keyboard.cpp
)
ament_target_dependencies(example_keyboard ${DEPENDENCIES})

# control a entity robot in real
add_executable(example_real
  src/example_real.cpp)
target_link_libraries(example_real
  Eigen3::Eigen
  spdlog::spdlog
)
ament_target_dependencies(example_real ${DEPENDENCIES})

# control a entity robot in gazebo
add_executable(example_gazebo
  src/example_gazebo.cpp)
target_link_libraries(example_gazebo
  Eigen3::Eigen
  spdlog::spdlog
)
ament_target_dependencies(example_gazebo ${DEPENDENCIES})

# control a joint in gazebo
add_executable(example_joint
  src/example_joint.cpp
)
ament_target_dependencies(example_joint ${DEPENDENCIES})

# reset robot position
add_executable(example_set_entity
  src/example_set_entity.cpp
)
ament_target_dependencies(example_set_entity ${DEPENDENCIES})

install(
  TARGETS
  example_joint
  example_gazebo
  example_real
  example_keyboard
  example_set_entity
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  PROGRAMS
  src/example_set_entity_test.py
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
