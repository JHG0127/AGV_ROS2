import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float32MultiArray  # Import Float32MultiArray
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
import tf2_ros
import time

class AGVNavigator(Node):
    def __init__(self):
        super().__init__('agv_navigator')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.current_pose = None

        # tf2_ros를 사용하여 현재 위치를 가져오기 위한 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.target_frame = 'map'
        self.base_link_frame = 'base_link'

        # Create a subscriber for /person_camera_distance
        self.distance_sub = self.create_subscription(
            Float32MultiArray, '/person_camera_distance', self.distance_callback, 10)
        # self.val = True

        self.goal_handle = None  # 현재 목표 핸들

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
    
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f'피드백 수신: {feedback}')

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info('목표가 거부되었습니다 :(')
            self.goal_handle = None
            return

        self.get_logger().info('목표가 수락되었습니다 :)')
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'결과: {result}')
        self.goal_handle = None
    
    def update_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, self.base_link_frame, rclpy.time.Time())
            position = transform.transform.translation
            orientation = transform.transform.rotation

            self.current_pose = self.create_pose(position.x, position.y, (orientation.x, orientation.y, orientation.z, orientation.w))
            # self.get_logger().info(f'현재 위치 업데이트: ({self.current_pose.pose.position.x}, {self.current_pose.pose.position.y})')
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().info(f'변환을 얻는 중 오류 발생: {e}')
    
    def quaternion_to_yaw(self, quaternion):
        """
        쿼터니언을 yaw 각도로 변환
        """
        q = quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw
    
    def distance_callback(self, msg):
        b = -msg.data[0] 
        a = msg.data[1]
        self.get_logger().info(f'Received /person_camera_distance: x={b}, z={a}')

        # if self.val:
            # self.val = False

        self.update_current_pose()
    
        # Handle the received Float32MultiArray message and print it to the terminal
        if self.current_pose is None:
            self.get_logger().info('현재 위치를 알 수 없습니다. 앞으로 이동할 수 없습니다.')
            return
        
        else:
            # 현재 방향(쿼터니언)을 yaw 각도로 변환
            orientation = self.current_pose.pose.orientation
            yaw = self.quaternion_to_yaw(orientation)

            # 입력받은 x, y 좌표를 사용하여 새로운 목표 위치 계산
            x = self.current_pose.pose.position.x + (a * math.cos(yaw) - b * math.sin(yaw))*0.9
            y = self.current_pose.pose.position.y + (b * math.cos(yaw) + a * math.sin(yaw))*0.9

            new_pose = self.create_pose(x, y, (
                self.current_pose.pose.orientation.x,
                self.current_pose.pose.orientation.y,
                self.current_pose.pose.orientation.z,
                self.current_pose.pose.orientation.w,
            ))

            goal = NavigateToPose.Goal()
            goal.pose = new_pose
            self.client.wait_for_server()
            self._send_goal_future = self.client.send_goal_async(goal, feedback_callback=self.feedback_callback)
            self._send_goal_future.add_done_callback(self.goal_response_callback)
            self.get_logger().info('일하는중...')

            self.get_logger().info(f'새로운 목표 위치: ({x}, {y})')
            time.sleep(3)

def main(args=None):
    rclpy.init(args=args)
    navigator = AGVNavigator()
    rclpy.spin(navigator)
    navigator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
