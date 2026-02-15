#!/usr/bin/env python3
# encoding: utf-8
"""
Multi-point navigation node for ROS 2 with Nav2.
Migrated from ROS 1 multipoint_nav.py — same logic:
  - Click 'Publish Point' in rviz2 to add waypoints
  - Robot navigates between waypoints in a loop
  - If a goal fails, retry once then move to next
  - Use '2D Pose Estimate' to reset all waypoints

ROS 2 / Nav2 differences:
  - Uses rclpy instead of rospy
  - Uses nav2_msgs NavigateToPose action instead of move_base
  - Uses action_client instead of publishing to /move_base_simple/goal
  - Goal status codes differ (STATUS_SUCCEEDED=4 in ROS2 action)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped


class MultipointNavigation(Node):
    def __init__(self):
        super().__init__('multipoint_navigation')

        self.callback_group = ReentrantCallbackGroup()

        # Target point marker array
        self.marker_array = MarkerArray()
        # point count
        self.count = 0
        # point index
        self.index = 0
        # Allow another attempt to go to the target point that has not been reached
        self.try_again = 1
        # Track if a goal is currently active
        self._goal_handle = None
        self._navigating = False

        # Used to publish target point markers
        self.pub_mark = self.create_publisher(MarkerArray, '/path_point', 100)

        # Subscribe to mark the pressed position in rviz2 (Publish Point tool)
        self.sub_click = self.create_subscription(
            PointStamped, '/clicked_point', self.click_callback, 10,
            callback_group=self.callback_group)

        # Subscribe to the initial pose topic (2D Pose Estimate in rviz2)
        self.sub_initialpose = self.create_subscription(
            PoseWithCovarianceStamped, '/initialpose', self.initialpose_callback, 10,
            callback_group=self.callback_group)

        # Nav2 action client (replaces move_base)
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self.callback_group)

        self.get_logger().info('Waiting for Nav2 navigate_to_pose action server...')
        self.nav_client.wait_for_server()
        self.get_logger().info('Nav2 action server connected!')
        self.get_logger().info('Use "Publish Point" in rviz2 to add waypoints.')
        self.get_logger().info('Use "2D Pose Estimate" in rviz2 to reset all waypoints.')

        # Timer to publish markers at 10Hz (same as ROS1 rate)
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        """Publish marker array at 10Hz."""
        self.pub_mark.publish(self.marker_array)

    def cancel_goal(self):
        """Cancel the current navigation goal."""
        if self._goal_handle is not None:
            self.get_logger().info('Cancelling current goal...')
            self._goal_handle.cancel_goal_async()
            self._goal_handle = None
        self._navigating = False

    def initialpose_callback(self, msg):
        """When user sets initial pose in rviz2, reset all waypoints (same as ROS1)."""
        if not isinstance(msg, PoseWithCovarianceStamped):
            return

        # Cancel current goal
        self.cancel_goal()

        # Clear all markers
        self.marker_array = MarkerArray()
        marker = Marker()
        marker.action = Marker.DELETEALL
        self.marker_array.markers.append(marker)
        self.pub_mark.publish(self.marker_array)

        # Reset state
        self.marker_array = MarkerArray()
        self.count = 0
        self.index = 0
        self.try_again = 1

        self.get_logger().info('Waypoints reset. Add new waypoints with Publish Point.')

    def click_callback(self, msg):
        """When user clicks 'Publish Point' in rviz2, add a waypoint."""
        self.get_logger().info(f'Add a new target point {self.count}.')

        # Create a marker object (same visual config as ROS1)
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 0.6
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.position.x = msg.point.x
        marker.pose.position.y = msg.point.y
        marker.pose.position.z = msg.point.z
        marker.text = str(self.count)

        self.marker_array.markers.append(marker)

        # Set the id of markers
        for idx, m in enumerate(self.marker_array.markers):
            m.id = idx

        # Publish first target point immediately (same as ROS1)
        if self.count == 0:
            self.send_goal(msg.point.x, msg.point.y)
            self.index += 1

        self.count += 1

    def send_goal(self, x, y):
        """Send a navigation goal to Nav2 (replaces PubTargetPoint from ROS1)."""
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.orientation.z = 0.0
        goal_msg.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Navigating to point ({x:.2f}, {y:.2f})')
        self._navigating = True

        send_goal_future = self.nav_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        """Called when the action server accepts/rejects the goal."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning('Goal rejected by Nav2!')
            self._navigating = False
            return

        self._goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def feedback_callback(self, feedback_msg):
        """Optional: log navigation progress."""
        pass

    def goal_result_callback(self, future):
        """
        Called when navigation to a goal is complete.
        Logic is identical to ROS1's goal_result_callback:
          - If reached (status == SUCCEEDED): go to next point, loop when done
          - If failed: retry once, then skip to next point
        """
        result = future.result()
        status = result.status
        self._goal_handle = None
        self._navigating = False

        if self.count == 0:
            return

        self.get_logger().info('Got navigation result!')

        # GoalStatus.STATUS_SUCCEEDED == 4 in ROS2
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.try_again = 1
            # This round of cruise is completed, restart cruise
            if self.index >= self.count:
                self.get_logger().info(f'Reached target point {self.index - 1}. Restarting patrol loop.')
                self.index = 0
                x = self.marker_array.markers[self.index].pose.position.x
                y = self.marker_array.markers[self.index].pose.position.y
                self.send_goal(x, y)
                self.index += 1
            # Cruise to the next point
            elif self.index < self.count:
                self.get_logger().info(f'Reached target point {self.index - 1}.')
                x = self.marker_array.markers[self.index].pose.position.x
                y = self.marker_array.markers[self.index].pose.position.y
                self.send_goal(x, y)
                self.index += 1
        else:
            # Did not reach the target point
            self.get_logger().warning(f'Cannot reach target point {self.index - 1}. (status={status})')

            # Try again once (same as ROS1)
            if self.try_again == 1:
                self.get_logger().warning(f'Trying to reach target point {self.index - 1} again!')
                x = self.marker_array.markers[self.index - 1].pose.position.x
                y = self.marker_array.markers[self.index - 1].pose.position.y
                self.send_goal(x, y)
                self.try_again = 0
            # Skip to next point
            elif self.index < len(self.marker_array.markers):
                self.get_logger().warning(
                    f'Failed to reach target point {self.index - 1}! Moving to next point.')
                if self.index == self.count:
                    self.index = 0
                x = self.marker_array.markers[self.index].pose.position.x
                y = self.marker_array.markers[self.index].pose.position.y
                self.send_goal(x, y)
                self.index += 1
                self.try_again = 1


def main(args=None):
    rclpy.init(args=args)
    node = MultipointNavigation()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down multipoint navigation...')
        node.cancel_goal()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
