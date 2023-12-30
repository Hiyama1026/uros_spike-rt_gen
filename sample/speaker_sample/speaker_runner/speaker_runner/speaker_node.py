import rclpy
from speaker_sample_msg.msg import SpeakerMessage

from rclpy.node import Node

# パブリッシャーノード
class MyPublisherNode(Node):
    # 初期化
    def __init__(self):     # コンストラクタ
        super().__init__("speaker_node")

        # 変数
        self.c4 = 262
        self.d4 = 294
        self.e4 = 330
        self.f4 = 349
        self.g4 = 392
        self.a4 = 440
        self.b4 = 494
        self.c5 = 523
        self.cnt = 0

        # パブリッシャーの生成
        self.speaker_publisher = self.create_publisher(SpeakerMessage, "speaker_tone", 10)

        # タイマーの生成
        self.timer_callback = self.create_timer(1, self.timer_on_tick)
        
        self.get_logger().info("node init")

    def timer_on_tick(self):
        speaker_data = SpeakerMessage()
        speaker_data.duration = 500

        if self.cnt == 0:
            speaker_data.tone = self.c4
        elif self.cnt == 1:
            speaker_data.tone = self.d4
        elif self.cnt == 2:
            speaker_data.tone = self.e4
        elif self.cnt == 3:
            speaker_data.tone = self.f4
        elif self.cnt == 4:
            speaker_data.tone = self.g4
        elif self.cnt == 5:
            speaker_data.tone = self.a4
        elif self.cnt == 6:
            speaker_data.tone = self.b4
        elif self.cnt == 7:
            speaker_data.tone = self.c5

        self.speaker_publisher.publish(speaker_data)

        self.cnt += 1
        if self.cnt == 8:
            self.cnt = 0

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
