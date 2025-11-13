import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image

class QRReader(Node):
    """
    - 기본: /qr_code (String) → /qr_event (String) 그대로 전달 (가벼운 스켈레톤)
    - 옵션: decode_from_camera=True 이면 /image_raw에서 QR 해석 시도(의존성 필요)
    """
    def __init__(self):
        super().__init__('qr_reader')
        self.declare_parameter('qr_code_topic', 'qr_code')
        self.declare_parameter('qr_event_topic', 'qr_event')
        self.declare_parameter('decode_from_camera', False)
        self.declare_parameter('image_topic', 'image_raw')

        self.qr_code_topic = self.get_parameter('qr_code_topic').value
        self.qr_event_topic = self.get_parameter('qr_event_topic').value
        self.decode_from_camera = bool(self.get_parameter('decode_from_camera').value)
        self.image_topic = self.get_parameter('image_topic').value

        self.event_pub = self.create_publisher(String, self.qr_event_topic, 10)

        if not self.decode_from_camera:
            self.create_subscription(String, self.qr_code_topic, self._on_qr_code, 10)
            self.get_logger().info(f"[QR] passthrough: '{self.qr_code_topic}' → '{self.qr_event_topic}'")
        else:
            # 옵션 경로: 카메라 디코딩
            try:
                from cv_bridge import CvBridge
                import cv2
                from pyzbar import pyzbar
                self.bridge = CvBridge()
                self.cv2 = cv2
                self.pyzbar = pyzbar
                self.create_subscription(Image, self.image_topic, self._on_image, 10)
                self.get_logger().info(f"[QR] camera decode ON: topic='{self.image_topic}' → '{self.qr_event_topic}'")
            except Exception as e:
                self.get_logger().error(f"[QR] camera decode unavailable: {e}")
                self.create_subscription(String, self.qr_code_topic, self._on_qr_code, 10)

    def _on_qr_code(self, msg: String):
        # 그대로 이벤트로 전달
        self.event_pub.publish(String(data=msg.data.strip()))

    def _on_image(self, img: Image):
        try:
            cvimg = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
            gray = self.cv2.cvtColor(cvimg, self.cv2.COLOR_BGR2GRAY)
            codes = self.pyzbar.decode(gray)
            for c in codes:
                data = c.data.decode('utf-8').strip()
                if data:
                    self.event_pub.publish(String(data=data))
        except Exception as e:
            self.get_logger().warn(f"[QR decode] {e}")

def main():
    rclpy.init()
    node = QRReader()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
