import rclpy
from std_msgs.msg import Int8
from force_sample_msg.msg import ColorLightMessage
from force_sample_msg.msg import UltrasonicLightMessage
from force_sample_msg.msg import SpikePrimeMessage

from rclpy.node import Node
from rclpy.qos import QoSProfile

# パブリッシャーノード
class MyPublisherNode(Node):
    # 初期化
    def __init__(self):     # コンストラクタ
        super().__init__("change_mode_node")

        qos_profile = QoSProfile(depth=10, reliability=2)

        # 変数
        self.force_pressed = False
        self.current_force = 0
        self.timer_cnt = 0

        # パブリッシャーの生成
        self.colorA_light_publisher = self.create_publisher(ColorLightMessage, "color_sensorA_light", 10)
        self.ultrasonicB_light_publisher = self.create_publisher(UltrasonicLightMessage, "ultrasonic_sensorB_light", 10)
        # タイマーの生成
        self.timer_callback = self.create_timer(0.1, self.timer_on_tick)
        # サブスクライバーの生成
        self.dev_status_subscription = self.create_subscription(
            SpikePrimeMessage, "spike_status", self.dev_status_on_subscribe, qos_profile)
        
        self.get_logger().info("node init")

    def set_color_light(self, l1, l2, l3):
        colA_light = ColorLightMessage()

        colA_light.light1 = l1
        colA_light.light2 = l2
        colA_light.light3 = l3
        return colA_light
    
    def set_ultra_light(self, l1, l2, l3, l4):
        ultB_light = UltrasonicLightMessage()

        ultB_light.light1 = l1
        ultB_light.light2 = l2
        ultB_light.light3 = l3
        ultB_light.light4 = l4
        return ultB_light

    def timer_on_tick(self):
        colA_light = ColorLightMessage()
        ultB_light = UltrasonicLightMessage()

        if self.current_force < 1 and self.force_pressed:
            #colA_light = self.set_color_light(0, 0, 0)
            ultB_light = self.set_ultra_light(100, 0, 0, 0)
        elif self.current_force >= 1 and self.current_force <= 4:
            #colA_light = self.set_color_light(100, 0, 0)
            ultB_light = self.set_ultra_light(100, 100, 0, 0)
        elif self.current_force >= 5 and self.current_force <= 7:
            #colA_light = self.set_color_light(100, 100, 0)
            ultB_light = self.set_ultra_light(100, 100, 100, 0)
        elif self.current_force >= 7 and self.current_force <= 10:
            #colA_light = self.set_color_light(100, 100, 100)
            ultB_light = self.set_ultra_light(100, 100, 100, 100)
        
        col_brightness = self.current_force * 10
        colA_light = self.set_color_light(col_brightness, col_brightness, col_brightness)
        
        self.colorA_light_publisher.publish(colA_light)
        self.ultrasonicB_light_publisher.publish(ultB_light)

    # サブスクライバー　コールバック
    def dev_status_on_subscribe(self, devise_status):
        self.force_pressed = devise_status.force_e_sensor_pressed
        self.current_force = devise_status.force_e_sensor_force


# メイン
def main(args=None):
    # RCLの初期化
    rclpy.init(args=args)

    # ノードの生成
    node = MyPublisherNode()

    # ノード終了まで待機
    rclpy.spin(node)

    # ノードの破棄
    node.destroy_node()

    # RCLのシャットダウン
    rclpy.shutdown()


if __name__ == "__main__":
    main()
