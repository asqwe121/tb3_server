import rclpy
from rclpy.node import Node                     # ★ 요게 없어서 난 에러
from rclpy.exceptions import ParameterAlreadyDeclaredException

class Coordinator(Node):
    def __init__(self):
        super().__init__(
            'coordinator',
            automatically_declare_parameters_from_overrides=True
        )

        # 중복 선언 예외 방지용
        def _declare(name, default):
            try:
                self.declare_parameter(name, default)
            except ParameterAlreadyDeclaredException:
                pass

        _declare('use_sim_time', True)
        _declare('frame_id', 'map')
        _declare('port', 8000)
        _declare('robot_namespaces', [''])

        self.use_sim_time = self.get_parameter('use_sim_time').value
        self.frame_id     = self.get_parameter('frame_id').value
        self.port         = int(self.get_parameter('port').value)
        self.namespaces   = list(self.get_parameter('robot_namespaces').value)

        self.get_logger().info(
            f"[INIT] ns={self.namespaces}, frame='{self.frame_id}', "
            f"use_sim_time={self.use_sim_time}, port={self.port}"
        )

def main():
    rclpy.init()
    node = Coordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
