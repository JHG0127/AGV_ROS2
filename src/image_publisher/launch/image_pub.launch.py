from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='image_publisher',  # 여기서 'your_package_name'을 실제 패키지 이름으로 바꾸세요
            executable='image_publisher',
            name='image_publisher',
            output='screen',
            parameters=[
                # 필요에 따라 여기에 매개변수를 추가할 수 있습니다.
            ]
        )
    ])
