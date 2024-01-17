import rclpy
from std_msgs.msg import Int8
from std_msgs.msg import Int16
from breakdancer_msg.msg import ColorLightMessage
from breakdancer_msg.msg import MotorStopReset
from breakdancer_msg.msg import SpikePrimeMessage

from rclpy.node import Node
from rclpy.qos import QoSProfile

# パブリッシャーノード
class MyPublisherNode(Node):
    # 初期化
    def __init__(self):     # コンストラクタ
        super().__init__("dance_node")

        qos_profile = QoSProfile(depth=10, reliability=2)

        # 変数等
        self.is_first = True
        self.is_light_on = False
        self.arm_count = 0
        self.pre_leg_count = 0
        self.leg_count = 0
        self.current_button_status = 0
        self.pre_button_status = 0
        self.timer_cnt = 0
        self.arm_run = False

        # ダンスモード
        self.auto_mode = 0
        self.manual_mode = 1
        self.dance_mode = self.auto_mode

        # パブリッシャーの生成
        self.arm_stop_reset_publisher = self.create_publisher(MotorStopReset, "motorC_stop_reset", qos_profile)
        self.arm_speed_publisher = self.create_publisher(Int16, "motorC_speed", qos_profile)
        self.leg_stop_reset_publisher = self.create_publisher(MotorStopReset, "motorD_stop_reset", qos_profile)
        self.leg_speed_publisher = self.create_publisher(Int16, "motorD_speed", qos_profile)
        self.colorB_light_publisher = self.create_publisher(ColorLightMessage, "color_sensorB_light", 10)
        # タイマーの生成
        self.timer_callback = self.create_timer(0.01, self.timer_on_tick)
        # サブスクライバーの生成
        self.dev_status_subscription = self.create_subscription(
            SpikePrimeMessage, "spike_status", self.dev_status_on_subscribe, qos_profile)
        self.button_status_subscription = self.create_subscription(
            Int8, "spike_button_status", self.button_status_on_subscribe, qos_profile)
        
        self.get_logger().info("node init")

    def switch_col_light(self):
        col_light = ColorLightMessage()

        if self.is_light_on:
            col_light.light1 = 0
            col_light.light2 = 0
            col_light.light3 = 0
            self.is_light_on = False
        else:
            col_light.light1 = 60
            col_light.light2 = 60
            col_light.light3 = 60
            self.is_light_on = True

        self.colorB_light_publisher.publish(col_light)

    def auto_dance(self):
        arm_speed = Int16()
        leg_speed = Int16()
        
        leg_speed.data = 400
        self.timer_cnt += 1
        
        if self.timer_cnt == 200:
            if self.arm_run:
                self.arm_run = False
            else:
                self.arm_run = True
            self.timer_cnt = 0
        
        if self.arm_run:
            arm_speed.data = 1000
        else:
            arm_speed.data = 0

        self.arm_speed_publisher.publish(arm_speed)
        self.leg_speed_publisher.publish(leg_speed)

    def manual_dance(self):
        
        if self.current_button_status == 1:     # left
            arm_speed = Int16()
            arm_speed.data = 1000
            self.arm_speed_publisher.publish(arm_speed)
        elif self.current_button_status == 2:     # right
            leg_speed = Int16()
            leg_speed.data = 400
            self.leg_speed_publisher.publish(leg_speed)
        elif self.current_button_status == 3:     # right & left
            arm_speed = Int16()
            leg_speed = Int16()
            arm_speed.data = 1000
            leg_speed.data = 400
            self.arm_speed_publisher.publish(arm_speed)
            self.leg_speed_publisher.publish(leg_speed)
        else:
            arm_speed = Int16()
            leg_speed = Int16()
            arm_speed.data = 0
            leg_speed.data = 0
            self.arm_speed_publisher.publish(arm_speed)
            self.leg_speed_publisher.publish(leg_speed)
        
    def timer_on_tick(self):

        if self.is_first:
            leg_speed = Int16()
            leg_speed.data = 400
            self.leg_speed_publisher.publish(leg_speed)

            if self.leg_count - self.pre_leg_count < 2:
                self.pre_leg_count = self.leg_count
                return
            self.is_first = False

        # button
        if self.pre_button_status == 0 and self.current_button_status == 4:
            self.switch_col_light()
        elif self.pre_button_status == 0 and self.current_button_status == 8:
            leg_speed = Int16()
            arm_speed = Int16()
            leg_speed.data = 0
            arm_speed.data = 0
            self.leg_speed_publisher.publish(leg_speed)
            self.arm_speed_publisher.publish(arm_speed)

            self.dance_mode += 1

            if self.dance_mode == 2:
                self.dance_mode = 0
        self.pre_button_status = self.current_button_status

        # dance
        if self.dance_mode == self.auto_mode:
            self.auto_dance()
        elif self.dance_mode == self.manual_mode:
            self.manual_dance()



    # サブスクライバー　コールバック
    def dev_status_on_subscribe(self, devise_status):
        self.arm_count = devise_status.motor_c_count
        self.leg_count = devise_status.motor_d_count

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