#!/usr/bin/env python3

"""
ref: [](README/#### 针对 gazebo 与 ros2 互操作的接口使用示例)
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from gazebo_msgs.srv import SetEntityState, GetEntityState
from gazebo_msgs.msg import EntityState
from geometry_msgs.msg import Vector3, Quaternion, Point


class GazeboEntityState(Node):
    def __init__(self):
        super().__init__('gazebo_entity_client')
        self.client = self.create_client(SetEntityState, '/set_entity_state')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.request = SetEntityState.Request()

    def reset_simulation(self):
        """Reset Gazebo simulation using /reset_simulation service"""
        client = self.create_client(Empty, '/reset_simulation')
        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /reset_simulation not available, waiting...')

        request = Empty.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('simulation reset successfully')
            return future.result()
        else:
            self.get_logger().error('failed to reset simulation')
            return None

    def set_entity_state(self, e_name, r_frame,
                         pos, orie,
                         linear, angular):
        self.request.state.name = e_name
        self.request.state.reference_frame = r_frame

        self.request.state.pose.position = Point(x=pos[0], y=pos[1], z=pos[2])
        self.request.state.pose.orientation = Quaternion(w=orie[0], x=orie[1], y=orie[2], z=orie[3])
        self.request.state.twist.linear = Vector3(x=linear[0], y=linear[1], z=linear[2])
        self.request.state.twist.angular = Vector3(x=angular[0], y=angular[1], z=angular[2])

        future = self.client.call_async(self.request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error('failed to set entity state')
            return None

    def get_entity_state(self, e_name, r_frame='world'):
        # Create a client for the '/gazebo/get_entity_state' service
        self.get_entity_state_client = self.create_client(GetEntityState, '/get_entity_state')

        # Wait for the service to be available
        while not self.get_entity_state_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service /get_entity_state not available, waiting again...')

        # Create a request object
        request = GetEntityState.Request()
        request.name = e_name
        request.reference_frame = r_frame

        # Call the service asynchronously
        future = self.get_entity_state_client.call_async(request)

        # Wait for the service call to complete
        rclpy.spin_until_future_complete(self, future)

        # Check if the service call was successful
        if future.result() is not None:
            return future.result()
        else:
            self.get_logger().error('get entity state service call failed')
            return None

    def parse_entity_state(self, response):
        if response.success:
            self.get_logger().info('parse entity state get successfully')

            # Extract state components
            state = response.state
            pos = state.pose.position
            orie = state.pose.orientation
            linear = state.twist.linear
            angular = state.twist.angular

            self.get_logger().info(
                f"\n\tstate.pos.position \t- x: {pos.x:.4f}, y: {pos.y:.4f}, z: {pos.z:.4f}"
                f"\n\tstate.pos.orientation \t- w: {orie.w:.4f}, x: {orie.x:.4f}, y: {orie.y:.4f}, z: {orie.z:.4f}"
                f"\n\tstate.pos.linear \t- x: {linear.x:.4f}, y: {linear.y:.4f}, z: {linear.z:.4f}"
                f"\n\tstate.pos.angular \t- x: {angular.x:.4f}, y: {angular.y:.4f}, z: {angular.z:.4f}"
            )
        else:
            self.get_logger().info('Failed to parse entity state')


def main(args=None):
    rclpy.init(args=args)
    entity = GazeboEntityState()

    # reset simulation, can make the robot be in a squat posture
    entity.reset_simulation()

    # set entity state
    entity_name = 'robot_a1'
    reference_frame = 'ground_plane'
    position = [0.0, 0.0, 0.38,]
    orientation = [1.0, 0.0, 0.0, 0.0,]
    linear = [0.0, 0.0, 0.0,]
    angular = [0.0, 0.0, 0.0,]
    entity.set_entity_state(entity_name, reference_frame, position, orientation, linear, angular)

    entity.get_entity_state(entity_name, reference_frame)

    rclpy.shutdown()


if __name__ == '__main__':
    main()
