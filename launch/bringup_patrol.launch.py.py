from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, yaml

def generate_launch_description():
    robot_namespaces_arg = DeclareLaunchArgument(
        'robot_namespaces',
        default_value="['/tb3_1','/tb3_2']",
        description="List of robot namespaces"
    )
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='true')
    frame_id_arg = DeclareLaunchArgument('frame_id', default_value='map')

    pkg_share = get_package_share_directory('tb3_server')
    qr_yaml = os.path.join(pkg_share, 'config', 'qr_waypoints.yaml')

    # 코디네이터: QR→Nav2, 교대/복귀 로직 총괄
    coordinator = Node(
        package='tb3_server',
        executable='coordinator',
        output='screen',
        parameters=[{
            'robot_namespaces': eval(LaunchConfiguration('robot_namespaces').perform({})),
            'frame_id': LaunchConfiguration('frame_id'),
            'qr_waypoints_file': qr_yaml,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # 각 로봇별 QR 리더 & 알람(부저+마커) 노드
    nodes = [coordinator]
    for ns in eval("['/tb3_1','/tb3_2']"):
        nodes.append(Node(
            package='tb3_server',
            executable='qr_reader',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'decode_from_camera': False,          # True로 바꾸면 카메라 해석 시도(옵션)
                'image_topic': 'image_raw',
                'qr_event_topic': 'qr_event'
            }]
        ))
        nodes.append(Node(
            package='tb3_server',
            executable='alarm',
            namespace=ns,
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'detection_topic': 'intruder_detected',
                'alarm_topic': 'alarm',
                'marker_topic': 'alarm_marker',
                'marker_frame': 'base_link',
                'hold_time_sec': 5.0,
                'blink_hz': 2.0,
                'stop_on_alarm': True,
                'sound_topic': 'sound',
                'use_aplay_fallback': False,          # SBC 스피커 WAV 쓰려면 True
                'wav_path': '/home/ubuntu/alarm.wav'
            }]
        ))

    return LaunchDescription([
        robot_namespaces_arg, use_sim_time_arg, frame_id_arg,
        *nodes])
