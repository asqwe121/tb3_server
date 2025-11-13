import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from visualization_msgs.msg import Marker
from builtin_interfaces.msg import Duration as RosDuration
from rclpy.duration import Duration as RclpyDuration
import subprocess, time

try:
    from turtlebot3_msgs.msg import Sound as TB3Sound
    HAS_TB3_SOUND = True
except Exception:
    HAS_TB3_SOUND = False

class AlarmNode(Node):
    def __init__(self):
        super().__init__('alarm')

        self.declare_parameter('detection_topic', 'intruder_detected')
        self.declare_parameter('alarm_topic', 'alarm')
        self.declare_parameter('marker_topic', 'alarm_marker')
        self.declare_parameter('marker_frame', 'base_link')
        self.declare_parameter('text', 'ALARM!')
        self.declare_parameter('hold_time_sec', 5.0)
        self.declare_parameter('blink_hz', 2.0)
        self.declare_parameter('stop_on_alarm', True)
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('text_height', 0.6)
        self.declare_parameter('text_size', 0.35)
        self.declare_parameter('sound_topic', 'sound')
        self.declare_parameter('buzzer_value_on', 1)
        self.declare_parameter('buzzer_value_off', 0)
        self.declare_parameter('beep_with_blink', True)
        self.declare_parameter('use_aplay_fallback', False)
        self.declare_parameter('aplay_cmd', 'aplay')
        self.declare_parameter('wav_path', '/home/ubuntu/alarm.wav')

        P = self.get_parameter
        self.det_topic = P('detection_topic').value
        self.alarm_topic = P('alarm_topic').value
        self.marker_topic = P('marker_topic').value
        self.marker_frame = P('marker_frame').value
        self.text = P('text').value
        self.hold = float(P('hold_time_sec').value)
        self.blink_hz = float(P('blink_hz').value)
        self.stop_on_alarm = bool(P('stop_on_alarm').value)
        self.cmd_vel_topic = P('cmd_vel_topic').value
        self.text_height = float(P('text_height').value)
        self.text_size = float(P('text_size').value)
        self.sound_topic = P('sound_topic').value
        self.b_on = int(P('buzzer_value_on').value)
        self.b_off = int(P('buzzer_value_off').value)
        self.beep_with_blink = bool(P('beep_with_blink').value)
        self.use_aplay = bool(P('use_aplay_fallback').value)
        self.aplay_cmd = P('aplay_cmd').value
        self.wav_path = P('wav_path').value

        self.alarm_pub = self.create_publisher(Bool, self.alarm_topic, 10)
        self.marker_pub = self.create_publisher(Marker, self.marker_topic, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        self.create_subscription(Bool, self.det_topic, self._on_detected, 10)

        self.sound_pub = None
        if HAS_TB3_SOUND:
            self.sound_pub = self.create_publisher(TB3Sound, self.sound_topic, 10)

        self.active = False
        self.expire = None
        self.blink_on = True
        period = max(0.05, 1.0 / max(0.1, self.blink_hz))
        self.create_timer(period, self._on_timer)

        self.get_logger().info(
            f"[Alarm] det='{self.det_topic}', alarm='{self.alarm_topic}', marker='{self.marker_topic}', "
            f"frame='{self.marker_frame}', hold={self.hold}s, tb3_sound={'ON' if HAS_TB3_SOUND else 'OFF'}"
        )

    def _on_detected(self, msg: Bool):
        if msg.data:
            self._activate()
        # False로 끄고 싶으면 여기서 _deactivate() 호출 가능

    def _activate(self):
        self.active = True
        self.expire = self.get_clock().now() + RclpyDuration(seconds=self.hold)
        self.blink_on = True
        self._pub_alarm(True)
        if self.stop_on_alarm:
            z = Twist()
            for _ in range(3):
                self.cmd_vel_pub.publish(z)
        self._beep(True)
        self._pub_marker(True)

    def _deactivate(self):
        self.active = False
        self.expire = None
        self._pub_alarm(False)
        self._beep(False)
        self._delete_marker()

    def _pub_alarm(self, state: bool):
        self.alarm_pub.publish(Bool(data=state))

    def _beep(self, on: bool):
        if self.sound_pub is not None:
            msg = TB3Sound()
            msg.value = self.b_on if on else self.b_off
            self.sound_pub.publish(msg)
        if self.use_aplay and on:
            try:
                subprocess.Popen([self.aplay_cmd, "-q", self.wav_path])
            except Exception as e:
                self.get_logger().warn(f"aplay failed: {e}")

    def _on_timer(self):
        if self.active and self.expire and self.get_clock().now() >= self.expire:
            self._deactivate()
            return
        if not self.active:
            return
        self.blink_on = not self.blink_on
        self._pub_marker(self.blink_on)

    def _pub_marker(self, visible: bool):
        m = Marker()
        m.header.frame_id = self.marker_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "alarm"
        m.id = 0
        m.type = Marker.TEXT_VIEW_FACING
        m.action = Marker.ADD
        m.pose.orientation.w = 1.0
        m.pose.position.z = self.text_height
        m.scale.z = self.text_size
        m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0
        m.color.a = 1.0 if visible else 0.15
        m.text = self.text
        m.lifetime = RosDuration(sec=0, nanosec=int(0.8e9))
        self.marker_pub.publish(m)

    def _delete_marker(self):
        m = Marker()
        m.header.frame_id = self.marker_frame
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = "alarm"
        m.id = 0
        m.action = Marker.DELETE
        self.marker_pub.publish(m)

def main():
    rclpy.init()
    node = AlarmNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

