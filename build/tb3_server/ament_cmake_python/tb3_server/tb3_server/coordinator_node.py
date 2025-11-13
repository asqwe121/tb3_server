import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import yaml, math, os
from typing import Dict, List

def yaw_to_quat(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q

class Coordinator(Node):
    """
    - 여러 로봇(ns 목록) 관리
    - /<ns>/qr_event (String) 수신 → qr_waypoints.yaml 매핑으로 Nav2 goal 보냄
    - 알람(/<ns>/alarm=True)인 동안에는 해당 로봇에 새 목표 안 보냄(선택적)
    - 교대/복귀 규칙(기본 예시):
        * 로봇A가 '4' 찍으면, 로봇B를 '4' 위치로 보내고,
          로봇A는 '1' 찍자마자 홈 복귀(또는 홈 직행)
      => 규칙은 _apply_policy()에서 커스터마이즈
    """

    def __init__(self):
        super().__init__('coordinator')

        # ----- 파라미터 -----
        self.declare_parameter('robot_namespaces', ['/tb3_1','/tb3_2'])
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('qr_waypoints_file', '')
        self.declare_parameter('use_sim_time', True)

        self.namespaces: List[str] = self.get_parameter('robot_namespaces').value
        self.frame_id: str = self.get_parameter('frame_id').value
        cfg_path: str = self.get_parameter('qr_waypoints_file').value

        # ----- 설정 로드 -----
        self.qr_waypoints: Dict[str, Dict[str, float]] = {}
        self.homes: Dict[str, Dict[str, float]] = {}
        if cfg_path and os.path.exists(cfg_path):
            with open(cfg_path, 'r') as f:
                data = yaml.safe_load(f) or {}
            self.qr_waypoints = data.get('qr_waypoints', {})
            self.homes = data.get('homes', {})
        else:
            self.get_logger().warn(f"qr_waypoints_file not found: '{cfg_path}'")

        # ----- Nav2 액션 클라이언트 -----
        self.clients: Dict[str, ActionClient] = {}
        for ns in self.namespaces:
            ac = ActionClient(self, NavigateToPose, f"{ns}/navigate_to_pose")
            self.clients[ns] = ac
            self.get_logger().info(f"[INIT] ActionClient for {ns} → {ns}/navigate_to_pose")

        # ----- 구독/퍼블리시 -----
        # QR 이벤트 & 알람 상태
        self.alarm_states: Dict[str, bool] = {ns: False for ns in self.namespaces}
        self.create_timer(1.0, self._on_timer)

        for ns in self.namespaces:
            self.create_subscription(String, f"{ns}/qr_event",
                                     lambda msg, ns=ns: self._on_qr(ns, msg), 10)
            self.create_subscription(Bool, f"{ns}/alarm",
                                     lambda msg, ns=ns: self._on_alarm(ns, msg), 10)

        self.get_logger().info(f"[READY] Coordinator up: ns={self.namespaces}, frame='{self.frame_id}'")

    # ----- 콜백들 -----
    def _on_alarm(self, ns: str, msg: Bool):
        self.alarm_states[ns] = bool(msg.data)

    def _on_qr(self, ns: str, msg: String):
        code = msg.data.strip()
        self.get_logger().info(f"[QR] {ns} → '{code}'")

        # 정책 적용(교대/복귀 등)
        if self._apply_policy(ns, code):
            return

        # 일반 매핑: code → waypoints
        wp = self.qr_waypoints.get(code)
        if not wp:
            self.get_logger().warn(f"[QR] code '{code}' has no waypoint mapping")
            return

        self._go_to(ns, wp)

    # ----- 정책(예시 규칙) -----
    def _apply_policy(self, ns: str, code: str) -> bool:
        """
        True 반환 시: 여기서 목표 전송/복귀 등 처리를 완료해 상위 흐름 스킵
        기본 예시:
         - code == '4'이고 로봇이 2대면, 다른 로봇을 code=4 위치로 보냄
         - code == '1'이면 이 로봇은 홈으로 복귀
        """
        # '1' 찍으면 자기 홈 복귀
        if code == '1':
            self._return_home(ns)
            return True

        # '4' 찍으면 다른 로봇을 4로
        if code == '4' and len(self.namespaces) >= 2:
            other = self._other(ns)
            if other is not None:
                wp = self.qr_waypoints.get('4')
                if wp:
                    self._go_to(other, wp)
                    self.get_logger().info(f"[HANDOFF] {ns} scanned 4 → dispatch {other} to 4")
                    return True

        return False

    def _other(self, ns: str):
        for x in self.namespaces:
            if x != ns:
                return x
        return None

    # ----- Nav2 전송 -----
    def _go_to(self, ns: str, wp: Dict[str, float]):
        if self.alarm_states.get(ns, False):
            self.get_logger().warn(f"[SKIP] {ns} alarm active, skip dispatch")
            return

        client = self.clients.get(ns)
        if not client:
            self.get_logger().error(f"[ERR] no action client for {ns}")
            return

        if not client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn(f"[WAIT] {ns}/navigate_to_pose not ready")
            return

        goal = NavigateToPose.Goal()
        pose = PoseStamped()
        pose.header.frame_id = self.frame_id
        pose.pose.position.x = float(wp.get('x', 0.0))
        pose.pose.position.y = float(wp.get('y', 0.0))
        pose.pose.orientation = yaw_to_quat(float(wp.get('yaw', 0.0)))
        goal.pose = pose

        self.get_logger().info(f"[SEND] {ns} → ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}) yaw={wp.get('yaw',0.0):.2f}")
        send = client.send_goal_async(goal)
        send.add_done_callback(lambda fut, ns=ns: self._on_goal_sent(ns, fut))

    def _on_goal_sent(self, ns, fut):
        try:
            goal_handle = fut.result()
            if not goal_handle.accepted:
                self.get_logger().warn(f"[REJECTED] goal by {ns}")
                return
            self.get_logger().info(f"[ACCEPTED] goal by {ns}")
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(lambda rf, ns=ns: self._on_result(ns, rf))
        except Exception as e:
            self.get_logger().error(f"[EXC goal] {ns}: {e}")

    def _on_result(self, ns, rf):
        try:
            _ = rf.result()
            self.get_logger().info(f"[DONE] {ns} reached goal (or finished)")
        except Exception as e:
            self.get_logger().warn(f"[EXC result] {ns}: {e}")

    def _return_home(self, ns: str):
        wp = self.homes.get(ns)
        if not wp:
            self.get_logger().warn(f"[HOME] no home for {ns}")
            return
        self.get_logger().info(f"[HOME] {ns} → home")
        self._go_to(ns, wp)

    # 주기적으로 Nav2 서버 준비상태를 가볍게 깨워줌(로깅)
    def _on_timer(self):
        pass

def main():
    rclpy.init()
    node = Coordinator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
