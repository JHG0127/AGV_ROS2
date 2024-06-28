import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import time

class AGVNavigator(Node):
    def __init__(self):
        super().__init__('agv_navigator')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.waypoints = [
            self.create_pose(3.2413, -1.1711, (0.0, 0.0, 0.95645, 0.29188)),  # 첫
            self.create_pose(-3.4533, 3.5679, (0.0, 0.0, 0.92379, 0.3829))  # 두
        ]
        self.current_waypoint_index = 0
        self.navigate_to_next_pose()

    def create_pose(self, x, y, quaternion):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.x = quaternion[0]
        pose.pose.orientation.y = quaternion[1]
        pose.pose.orientation.z = quaternion[2]
        pose.pose.orientation.w = quaternion[3]
        return pose

    def navigate_to_next_pose(self):
        if self.current_waypoint_index >= len(self.waypoints):
            self.current_waypoint_index = 0

        goal = NavigateToPose.Goal()
        goal.pose = self.waypoints[self.current_waypoint_index]
        self.client.wait_for_server()
        self._send_goal_future = self.client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback: {feedback}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Result: {result}')

        # 3초간 대기
        time.sleep(3)

        self.current_waypoint_index += 1

        # 다음 목적지로 이동
        self.navigate_to_next_pose()

def main(args=None):
    rclpy.init(args=args)
    navigator = AGVNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()