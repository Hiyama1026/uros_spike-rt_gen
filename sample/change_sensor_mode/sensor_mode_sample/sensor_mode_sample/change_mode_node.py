import rclpy
from std_msgs.msg import Int8
from spike_ros_msg.msg import SpikePrimeMessage

from rclpy.node import Node
from rclpy.qos import QoSProfile

# パブリッシャーノード
class MyPublisherNode(Node):
    # 初期化
    def __init__(self):     # コンストラクタ
        super().__init__("change_mode_node")

        qos_profile = QoSProfile(depth=10, reliability=2)

        # 変数
        self.current_button_status = 0
        self.pre_button_status = 0
        self.current_col_mode = 0
        self.set_col_mode = 0
        self.pre_col_mode = 0
        self.current_ult_mode = 0
        self.set_ult_mode = 0
        self.pre_ult_mode = 0

        self.col_val_1 = 0
        self.col_val_2 = 0
        self.col_val_3 = 0
        self.ult_val = 0

        self.timer_cnt = 0

        # パブリッシャーの生成
        self.colorA_mode_publisher = self.create_publisher(Int8, "color_sensorA_mode", 10)
        self.ultrasonicB_mode_publisher = self.create_publisher(Int8, "ultrasonic_sensorB_mode", 10)
        # タイマーの生成
        self.timer_callback = self.create_timer(0.1, self.timer_on_tick)
        # サブスクライバーの生成
        self.dev_status_subscription = self.create_subscription(
            SpikePrimeMessage, "spike_status", self.dev_status_on_subscribe, qos_profile)
        self.button_status_subscription = self.create_subscription(
            Int8, "spike_button_status", self.button_status_on_subscribe, qos_profile)
        
        self.get_logger().info("node init")


    def display_color_value(self):
        if self.current_col_mode == 0:
            self.get_logger().info("color mode :  non")
        elif self.current_col_mode == 1:
            self.get_logger().info("color mode :  ambient")
        elif self.current_col_mode == 2:
            self.get_logger().info("color mode :  color code")
        elif self.current_col_mode == 3:
            self.get_logger().info("color mode :  reflection")
        elif self.current_col_mode == 4:
            self.get_logger().info("color mode :  RGB")
            self.get_logger().info("  R : " + str(self.col_val_1))
            self.get_logger().info("  G : " + str(self.col_val_2))
            self.get_logger().info("  B : " + str(self.col_val_3))
            return
        elif self.current_col_mode == 5:
            self.get_logger().info("color mode :  HSV")
            self.get_logger().info("  H : " + str(self.col_val_1))
            self.get_logger().info("  S : " + str(self.col_val_2))
            self.get_logger().info("  V : " + str(self.col_val_3))
            return
        else:
            self.get_logger().info("unknown " + str(self.current_col_mode))
        
        self.get_logger().info("  value : " + str(self.col_val_1))
        return

    def display_ultrasinic_value(self):
        if self.current_ult_mode == 0:
            self.get_logger().info("ultrasonic mode :  non")
        elif self.current_ult_mode == 1:
            self.get_logger().info("ultrasonic mode :  distance")
        elif self.current_ult_mode == 2:
            self.get_logger().info("ultrasonic mode :  presence")
        
        self.get_logger().info("  value : " + str(self.ult_val))
        return


    def timer_on_tick(self):
        colA_mode = Int8()
        ultB_mode = Int8()

        if self.pre_button_status == 0 and self.current_button_status == 1:
            self.set_col_mode += 1
            if self.set_col_mode == 6:
                self.set_col_mode = 1
            
            colA_mode.data = self.set_col_mode
            self.colorA_mode_publisher.publish(colA_mode)

        if self.pre_button_status == 0 and self.current_button_status == 2:
            self.set_ult_mode += 1
            if self.set_ult_mode == 3:
                self.set_ult_mode = 1
            
            ultB_mode.data = self.set_ult_mode
            self.ultrasonicB_mode_publisher.publish(ultB_mode)

        # 1s cycle
        if self.timer_cnt == 10:
            self.display_color_value()
            self.display_ultrasinic_value()
            self.get_logger().info("----")
            self.timer_cnt = 0

        self.pre_button_status = self.current_button_status
        self.timer_cnt += 1
        
        

    # サブスクライバー　コールバック
    def dev_status_on_subscribe(self, devise_status):
        self.current_col_mode = devise_status.color_a_mode_id
        self.col_val_1 = devise_status.color_a_sensor_value_1
        self.col_val_2 = devise_status.color_a_sensor_value_2
        self.col_val_3 = devise_status.color_a_sensor_value_3

        self.current_ult_mode = devise_status.ultrasonic_b_mode_id
        self.ult_val = devise_status.ultrasonic_b_sensor

    def button_status_on_subscribe(self, button_status):
        self.current_button_status = button_status.data
        


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
