import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import random
import math

class AutoMove(Node):

    def __init__(self):
        super().__init__('auto_move')
        self.get_logger().info("🚀 AutoMove Node started")

        # Nav2 action client
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.goal_in_progress = False

        # Map data for free-space checking
        self.map_received = False
        self.occupancy_grid = None
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10
        )

        # Timer to send new goals
        self.timer = self.create_timer(2.0, self.timer_callback)

    def map_callback(self, msg):
        self.occupancy_grid = msg
        self.map_received = True

    def timer_callback(self):
        if not self.goal_in_progress and self.map_received:
            self.send_random_goal()

    def is_free(self, x, y):
        """
        Check if the x,y world coordinate is free in the occupancy grid.
        """
        map_data = self.occupancy_grid
        resolution = map_data.info.resolution
        origin_x = map_data.info.origin.position.x
        origin_y = map_data.info.origin.position.y
        width = map_data.info.width
        height = map_data.info.height

        mx = int((x - origin_x) / resolution)
        my = int((y - origin_y) / resolution)

        if mx < 0 or my < 0 or mx >= width or my >= height:
            return False

        idx = my * width + mx
        if map_data.data[idx] == 0:  # 0 = free
            return True
        return False

    def send_random_goal(self):
        """
        Pick a random free position on the map and send it to Nav2.
        """
        # Wait for Nav2 action server
        self.get_logger().info("Waiting for Nav2 action server...")
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().warn("Nav2 action server not available yet.")
            return

        # Retry until a free cell is found
        for _ in range(50):
            x = random.uniform(-2.0, 2.0)
            y = random.uniform(-2.0, 2.0)
            if self.is_free(x, y):
                break
        else:
            self.get_logger().warn("Could not find free cell, skipping goal.")
            return

        # Random orientation
        yaw = random.uniform(0.0, 2.0 * math.pi)
        quat_z = math.sin(yaw / 2.0)
        quat_w = math.cos(yaw / 2.0)

        # Prepare goal message
        goal_msg = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = quat_z
        pose.pose.orientation.w = quat_w
        goal_msg.pose = pose

        self.get_logger().info(f"📍 Sending goal: x={x:.2f}, y={y:.2f}")
        self.goal_in_progress = True
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        # Optional: process distance remaining
        pass

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn("Goal rejected by Nav2")
            self.goal_in_progress = False
            return
        self.get_logger().info("Goal accepted! Navigating...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info("✅ Navigation succeeded")
        else:
            self.get_logger().warn(f"Navigation failed with status: {status}")
        self.goal_in_progress = False

def main(args=None):
    rclpy.init(args=args)
    node = AutoMove()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

