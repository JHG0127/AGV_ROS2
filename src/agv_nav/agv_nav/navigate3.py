import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import tf2_ros
import threading
import queue
import math

class AGVNavigator(Node):
    def __init__(self):
        super().__init__('agv_navigator')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.move_point = True
        self.current_pose = None  # 현재 위치 변수
        self.waiting_for_a = False  # x 좌표 입력 대기 상태 변수
        self.waiting_for_b = False  # y 좌표 입력 대기 상태 변수
        self.a = None
        self.b = None
        self.waypoints = [
            self.create_pose(0.7, -1.4, (0.0, 0.0, 0.7, 0.7)),  # 첫 번째 목적지
            self.create_pose(0.6, 1.7, (0.0, 0.0, -0.7, 0.7)),  # 두 번째 목적지
            self.create_pose(-0.40951, 2.0532, (0.0, 0.0, 0.52217, 0.85284))  # 세 번째 목적지
        ]
        self.input_queue = queue.Queue()
        self.input_thread = threading.Thread(target=self.get_user_input)
        self.input_thread.start()
        self.get_logger().info('AGVNavigator 초기화 완료. 명령을 받을 준비가 되었습니다.')

        # tf2_ros를 사용하여 현재 위치를 가져오기 위한 설정
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.target_frame = 'map'
        self.base_link_frame = 'base_link'

        self.goal_handle = None  # 현재 목표 핸들

        # /person_camera_distance 토픽 구독 설정
        self.subscription = self.create_subscription(
            PoseStamped,
            '/person_camera_distance',
            self.person_camera_distance_callback,
            10)
        self.subscription  # prevent unused variable warning

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

    def navigate_to_pose(self, pose):
        goal = NavigateToPose.Goal()
        goal.pose = pose
        self.client.wait_for_server()
        self._send_goal_future = self.client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def cancel_goal(self):
        if self.goal_handle:
            self.get_logger().info('목표를 취소합니다...')
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self.cancel_done_callback)
        else:
            self.get_logger().info('목표가 없습니다. 새로운 목표를 설정합니다.')

    def cancel_done_callback(self, future):
        cancel_result = future.result()
        if cancel_result:
            self.get_logger().info('목표가 성공적으로 취소되었습니다.')
        else:
            self.get_logger().info('목표 취소에 실패했습니다.')

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
        # 결과가 완료된 후 사용자 입력 대기 시작
        self.get_logger().info('다음 목적지로 이동하려면 목적지 번호 (1, 2, 3)를 입력하거나, 특정 지점으로 이동하려면 4를 입력하세요. 목표를 취소하려면 5를 입력하세요:')

    def get_user_input(self):
        while True:
            user_input = input()
            self.input_queue.put(user_input)

    def check_user_input(self):
        while not self.input_queue.empty():
            user_input = self.input_queue.get()

            if self.waiting_for_a:
                try:
                    self.a = float(user_input)
                    self.waiting_for_a = False
                    self.waiting_for_b = True
                    self.get_logger().info('y 좌표를 입력하세요:')
                except ValueError:
                    self.get_logger().info('유효하지 않은 입력입니다. x 좌표를 숫자로 입력하세요.')

            elif self.waiting_for_b:
                try:
                    self.b = float(user_input)
                    self.waiting_for_b = False
                    self.move_point = False
                    self.move_forward(self.a, self.b)
                except ValueError:
                    self.get_logger().info('유효하지 않은 입력입니다. y 좌표를 숫자로 입력하세요.')

            elif self.move_point:
                if user_input.isdigit():
                    index = int(user_input) - 1
                    if index == 3:
                        self.get_logger().info('현재 위치를 요청합니다. 현재 위치를 얻기 위해 기다리는 중...')
                        self.update_current_pose()  # 현재 위치 업데이트
                        self.waiting_for_a = True  # x 좌표 입력 대기 상태 설정
                        self.get_logger().info('x 좌표를 입력하세요:')
                    elif 0 <= index < len(self.waypoints):
                        self.navigate_to_pose(self.waypoints[index])
                    elif index == 4:
                        self.cancel_goal()
                    else:
                        self.get_logger().info('유효하지 않은 입력입니다. 1에서 4 사이의 숫자를 입력하세요.')
                else:
                    self.get_logger().info('유효하지 않은 입력입니다. 1에서 4 사이의 숫자를 입력하세요.')

    def move_forward(self, a, b):
        if self.current_pose is None:
            self.get_logger().info('현재 위치를 알 수 없습니다. 앞으로 이동할 수 없습니다.')
            return

        # 현재 방향(쿼터니언)을 yaw 각도로 변환
        orientation = self.current_pose.pose.orientation
        yaw = self.quaternion_to_yaw(orientation)

        # 입력받은 x, y 좌표를 사용하여 새로운 목표 위치 계산
        x = self.current_pose.pose.position.x + a * math.cos(yaw) - b * math.sin(yaw)
        y = self.current_pose.pose.position.y + b * math.cos(yaw) + a * math.sin(yaw)

        new_pose = self.create_pose(x, y, (
            self.current_pose.pose.orientation.x,
            self.current_pose.pose.orientation.y,
            self.current_pose.pose.orientation.z,
            self.current_pose.pose.orientation.w,
        ))

        self.navigate_to_pose(new_pose)
        self.move_point = True

    def quaternion_to_yaw(self, quaternion):
        """
        쿼터니언을 yaw 각도로 변환
        """
        q = quaternion
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def update_current_pose(self):
        try:
            transform = self.tf_buffer.lookup_transform(self.target_frame, self.base_link_frame, rclpy.time.Time())
            position = transform.transform.translation
            orientation = transform.transform.rotation

            self.current_pose = self.create_pose(position.x, position.y, (orientation.x, orientation.y, orientation.z, orientation.w))
            self.get_logger().info(f'현재 위치 업데이트: ({self.current_pose.pose.position.x}, {self.current_pose.pose.position.y})')
            
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            self.get_logger().info(f'변환을 얻는 중 오류 발생: {e}')

    def person_camera_distance_callback(self, msg):
        self.get_logger().info(f'새로운 목표 수신: x={msg.pose.position.x}, y={msg.pose.position.y}')
        if self.goal_handle:
            self.cancel_goal()
            rclpy.spin_until_future_complete(self, self.goal_handle.cancel_goal_async())
        self.navigate_to_pose(msg)

def main(args=None):
    rclpy.init(args=args)
    navigator = AGVNavigator()
    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(navigator)
    
    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=1)
            navigator.check_user_input()
    finally:
        navigator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
