# ROS 2のPython用ライブラをインポート
import rclpy
# rclpy.nodeモジュールからNode,SingleThreadedExecutorクラスをインポート
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor

# トピック通信のメッセージ型をインポート
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose


# turtlesimを制御する速度指令値をトピックcmd_velに出版するノードのクラス
class TwistPubNode(Node):
    # ノード生成時に実行されるコンストラクタ
    def __init__(self):
        # Nodeクラスのコンストラクタを呼び出してノード名をつける
        super().__init__('twist_pub_node')
        # publisherの生成
        # 1番目の引数：トピック通信に使用するメッセージ型
        # 2番目の引数：出版対象のトピック名
        # 3番目の引数：QoS設定におけるキューのサイズ
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)

        # timerの生成
        # 1.00秒ごとにコールバック関数timer_callbackが実行される
        timer_period = 4.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Twistメッセージ型のオブジェクトの生成
        self.vel = Twist()
        # 何回目の呼び出しかをカウントする変数
        self.call_count = 0

    # timerの起動間隔で実行されるコールバック関数
    def timer_callback(self):
        # 現在地のxy座標とtheta値（turtlesimの向き）をログ表示
        # 値はpose_sub_nodeの購読値を対応するクラス変数から取得
        self.get_logger().info("x=%f y=%f theta=%f" %
                               (PoseSubNode.pose.x, PoseSubNode.pose.y, PoseSubNode.pose.theta))
        if self.call_count % 2 == 0:
            self.vel.linear.x = 1.0
            self.vel.angular.z = 0.0
            self.publisher.publish(self.vel)
            self.node.get_logger().info("直進")
        else:
            self.vel.linear.x = 0.0
            self.vel.angular.z = -3.14/8
            self.publisher.publish(self.vel)
            self.node.get_logger().info("左回転")

# turtlesimの位置情報などを含むメッセージをposeから購読するノードのクラス
class PoseSubNode(Node):

    # Poseメッセージ型のオブジェクトの生成
    # twist_pub_nodeからアクセスできるようにクラス変数として用意
    pose = Pose()

    # ノード生成時に実行されるコンストラクタ
    def __init__(self):
        # Nodeクラスのコンストラクタを呼び出してノード名をつける
        super().__init__('pose_sub_node')
        # subscriberの生成
        # 1番目の引数：トピック通信に使用するメッセージ型
        # 2番目の引数：購読対象のトピック名
        # 3番目の引数：メッセージの購読時に実行されるコールバック関数
        # 4番目の引数：QoS設定におけるキューのサイズ
        self.subscription = self.create_subscription(
            Pose, 'turtle1/pose', self.sub_callback, 10)

    # 3番目の引数：メッセージの購読時に実行されるコールバック関数
    def sub_callback(self, Pose):
        # 現在地のxy座標とtheta値（turtlesimの向き）をクラス変数に格納
        self.pose.x = Pose.x
        self.pose.y = Pose.y
        self.pose.theta = Pose.theta


# エントリポイントで実行されるmain関数の実装
def main(args=None):
    # ROS 2環境の初期化：rclpyを実行できるようにする
    rclpy.init(args=args)

    # クラス定義からノードを生成してコンストラクタを実行
    twist_pub_node = TwistPubNode()
    pose_sub_node = PoseSubNode()
    
    # executorと呼ばれるROS 2ノードのスケジューラに相当する機構を生成
    executor = SingleThreadedExecutor()
    # executorにノードを登録
    executor.add_node(twist_pub_node)
    executor.add_node(pose_sub_node)

    # executorを開始してノードの実行を開始
    try:
        executor.spin()
    # Ctrl+Cが入力されたら終了する
    except KeyboardInterrupt:
        pass

    # executorの終了
    executor.shutdown()
    # ノードの削除
    twist_pub_node.destroy_node()
    pose_sub_node.destroy_node()
    # ROS 2環境の終了処理
    rclpy.shutdown()


if __name__ == '__main__':
    main()