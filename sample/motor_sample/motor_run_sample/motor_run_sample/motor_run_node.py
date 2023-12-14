import rclpy
from std_msgs.msg import Int16
from motor_sample_msg.msg import MotorStopReset
from motor_sample_msg.msg import SpikePrimeMessage

from rclpy.node import Node
from rclpy.qos import QoSProfile

# パブリッシャーノード
class MyPublisherNode(Node):
    # 初期化
    def __init__(self):     # コンストラクタ
        super().__init__("motor_run_node")

        qos_profile = QoSProfile(depth=10, reliability=2)

        # 変数
        self.is_first = True
        self.pre_motorA_count = 0
        self.motorA_count = 0
        self.motorD_count = 0
        self.motorE_count = 0
        self.switch_motor = 0

        # パブリッシャーの生成
        self.motorA_stop_reset_publisher = self.create_publisher(MotorStopReset, "motorA_stop_reset", qos_profile)
        self.motorA_speed_publisher = self.create_publisher(Int16, "motorA_speed", qos_profile)
        self.motorD_stop_reset_publisher = self.create_publisher(MotorStopReset, "motorD_stop_reset", qos_profile)
        self.motorD_speed_publisher = self.create_publisher(Int16, "motorD_speed", qos_profile)
        self.motorE_stop_reset_publisher = self.create_publisher(MotorStopReset, "motorE_stop_reset", qos_profile)
        self.motorE_speed_publisher = self.create_publisher(Int16, "motorE_speed", qos_profile)
        # タイマーの生成
        self.timer_callback = self.create_timer(0.01, self.timer_on_tick)
        # サブスクライバーの生成
        self.dev_status_subscription = self.create_subscription(
            SpikePrimeMessage, "spike_status", self.dev_status_on_subscribe, qos_profile)
        
        self.get_logger().info("node init")

    def run_motor(self):
        if self.switch_motor == 0:
            motorA_st_rs = MotorStopReset()
            motorD_speed = Int16()

            motorA_st_rs.motor_stop_hold = 2    # motor hold
            motorA_st_rs.motor_reset = True
            motorD_speed.data = 400

            self.motorA_stop_reset_publisher.publish(motorA_st_rs)
            self.motorD_speed_publisher.publish(motorD_speed)

        elif self.switch_motor == 1:
            motorD_st_rs = MotorStopReset()
            motorE_speed = Int16()

            motorD_st_rs.motor_stop_hold = 1    # motor stop
            motorD_st_rs.motor_reset = True
            motorE_speed.data = 400
            
            self.motorD_stop_reset_publisher.publish(motorD_st_rs)
            self.motorE_speed_publisher.publish(motorE_speed)

        elif self.switch_motor == 2:
            motorE_st_rs = MotorStopReset()
            motorA_speed = Int16()

            motorE_st_rs.motor_stop_hold = 1    # motor stop
            motorE_st_rs.motor_reset = True
            motorA_speed.data = 40

            self.motorE_stop_reset_publisher.publish(motorE_st_rs)
            self.motorA_speed_publisher.publish(motorA_speed)
        

        self.switch_motor += 1
        if self.switch_motor == 3:
            self.switch_motor = 0


    def check_count(self, m_cnt):
        if m_cnt >= 360:
            self.run_motor()


    def timer_on_tick(self):

        if self.is_first:
            motorA_speed = Int16()
            motorA_speed.data = 40
            self.motorA_speed_publisher.publish(motorA_speed)

            if self.motorA_count - self.pre_motorA_count < 2:
                self.pre_motorA_count = self.motorA_count
                return
            self.is_first = False

        if self.switch_motor == 0:
            self.check_count(self.motorA_count)
        elif self.switch_motor == 1:
            self.check_count(self.motorD_count)
        elif self.switch_motor == 2:
            self.check_count(self.motorE_count)

    # サブスクライバー　コールバック
    def dev_status_on_subscribe(self, devise_status):
        self.motorA_count = devise_status.motor_a_count
        self.motorD_count = devise_status.motor_d_count
        self.motorE_count = devise_status.motor_e_count        


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
