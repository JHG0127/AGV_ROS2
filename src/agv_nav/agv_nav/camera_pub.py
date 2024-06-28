import rclpy
from rclpy.node import Node
from mycobot_interfaces.srv import DetectionRQ
from vision_msgs.msg import Detection2DArray, Detection2D
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

import cv2
import numpy as np
import yaml
import threading
import sys

from pathlib import Path

class ColorDetectionService(Node):
    def __init__(self, imgshow_flag=False):
        super().__init__('color_detection_service')
        self.srv = self.create_service(DetectionRQ, 'color_detection', self.detect_callback)
        self.imgshow_flag = imgshow_flag
        self._package_path = Path(get_package_share_directory('webcam_white_detector'))
        self.file_path = str(self._package_path) + '/config/example.yaml'
        self.yaml_data = self.read_yaml_file(self.file_path)

        self.bridge = CvBridge()

        self.cap = cv2.VideoCapture(0)
        self.width = 640
        self.height = 480
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)

        if not self.cap.isOpened():
            self.get_logger().error("Failed to open camera")
            sys.exit(1)

        self.armforcesX = self.yaml_data.get('robot_focus').get('x')
        self.armforcesY = self.yaml_data.get('robot_focus').get('y')
        self.detect_data = []

        white_high_h = self.yaml_data.get('white').get('high_h')
        white_low_h = self.yaml_data.get('white').get('low_h')
        white_high_s = self.yaml_data.get('white').get('high_s')
        white_low_s = self.yaml_data.get('white').get('low_s')
        white_high_v = self.yaml_data.get('white').get('high_v')
        white_low_v = self.yaml_data.get('white').get('low_v')

        # 색상 범위 지정
        self.lower_white = np.array([white_low_h, white_low_s, white_low_v])
        self.upper_white = np.array([white_high_h, white_high_s, white_high_v])

        # ROI 설정 (ROI: x, y, width, height)
        self.roi_x = 100
        self.roi_y = 80
        self.roi_width = 440
        self.roi_height = 320

        if self.imgshow_flag:
            self.get_logger().info("Image show flag is enabled")
            _, self.frame = self.cap.read()
            if self.frame is not None:
                self.get_logger().info("Initial frame read successfully")
            else:
                self.get_logger().error("Initial frame read failed")
            imgshow_thread = threading.Thread(target=self.imgShow)
            imgshow_thread.start()

    def detect_callback(self, request: DetectionRQ.Request, response: DetectionRQ.Response):
        if request.trigger:
            response.result = self.detect()
            self.get_logger().info("Service Request")
        return response

    def detect(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().error('Failed to read frame from camera')
            return Detection2DArray()

        ## detecting process
        frame = cv2.flip(frame, 0)
        frame = cv2.flip(frame, 1)

        # ROI 설정
        roi = frame[self.roi_y:self.roi_y + self.roi_height, self.roi_x:self.roi_x + self.roi_width]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        hsv = cv2.GaussianBlur(hsv, (3, 3), 0)

        mask_white = cv2.inRange(hsv, self.lower_white, self.upper_white)

        white_contours, _ = cv2.findContours(mask_white, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        self.detect_data.clear()
        contourarea_threshold = 100

        for contour in white_contours:
            epsilon = 0.05 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            area = cv2.contourArea(approx)
            if area > contourarea_threshold:
                rect = cv2.minAreaRect(contour)
                box = cv2.boxPoints(rect)
                box = np.int64(box)
                (x, y), (box_width, box_height), angle = rect
                center = (int(x) + self.roi_x, int(y) + self.roi_y)
                bb_width = int(box_width)
                bb_height = int(box_height)
                self.detect_data.append(['white', center[0], center[1], bb_width, bb_height, angle])

        detections = Detection2DArray()
        detections.detections = [self.create_detection(color, centerx, centery, width, height, angle)
                                 for color, centerx, centery, width, height, angle in self.detect_data]

        if self.imgshow_flag:
            self.frame = frame

        return detections

    def create_detection(self, color, centerx, centery, width, height, angle):
        detection = Detection2D()
        detection.bbox.center.position.x = float(centerx)
        detection.bbox.center.position.y = float(centery)
        detection.bbox.center.theta = angle
        detection.bbox.size_x = float(width)
        detection.bbox.size_y = float(height)
        detection.id = color

        return detection

    def read_yaml_file(self, file_path):
        with open(file_path, 'r') as yaml_file:
            data = yaml.safe_load(yaml_file)
        return data

    def imgShow(self):
        while True:
            self.get_logger().info("Reading frame in imgShow loop")
            ret, frame = self.cap.read()
            if not ret or frame is None or frame.size == 0:
                self.get_logger().error('Failed to read frame from camera')
                continue

            frame = cv2.flip(frame, 0)
            frame = cv2.flip(frame, 1)

            # ROI 그리기
            cv2.rectangle(frame, (self.roi_x, self.roi_y), (self.roi_x + self.roi_width, self.roi_y + self.roi_height), (255, 0, 0), 2)

            for color, cX, cY, _, _, _ in self.detect_data:
                cv2.circle(frame, (cX, cY), 5, (0, 0, 255), -1)
                transformX = cX - self.armforcesX
                transformY = self.armforcesY - cY
                cv2.putText(frame, f'{color} ({transformX}, {transformY})', (cX - 50, cY - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

            frame = cv2.line(frame, (self.armforcesX, 0), (self.armforcesX, self.height), (0, 255, 0), 1)
            frame = cv2.line(frame, (0, self.armforcesY), (self.width, self.armforcesY), (0, 255, 0), 1)
            frame = cv2.circle(frame, (self.armforcesX, self.armforcesY), 2, (0, 0, 255), -1)

            if frame is not None and frame.size > 0:
                self.get_logger().info("Displaying frame")
                cv2.imshow('Detection', frame)
            else:
                self.get_logger().error('Frame is not valid for display')

            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Quit key pressed")
                break

        self.cap.release()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    imgshow_flag = len(sys.argv) > 1 and sys.argv[1] == 'True'
    color_detection_service = ColorDetectionService(imgshow_flag)
    rclpy.spin(color_detection_service)
    color_detection_service.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
