#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/int32_multi_array.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/transform_broadcaster.h>

class RpmToOdomNode : public rclcpp::Node {
public:
  RpmToOdomNode() : Node("rpm_to_odom_node") {
    // パラメータ初期化
    wheel_radius_ = 0.0635;        // [m]
    wheel_separation_ = 0.35;      // [m]
    gear_ratio_ = 111.0;
    rpm_to_radps_ = 2.0 * M_PI / 60.0;

    x_ = 0.0; y_ = 0.0; theta_ = 0.0;
    last_time_ = now();

    // 購読
    rpm_sub_ = create_subscription<std_msgs::msg::Int32MultiArray>(
      "/rpm", 10,
      std::bind(&RpmToOdomNode::rpm_callback, this, std::placeholders::_1)
    );

    // odom出力
    odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("/odom", 10);

    // TFブロードキャスト
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);
  }

private:
  void rpm_callback(const std_msgs::msg::Int32MultiArray::SharedPtr msg) {
    auto current_time = now();

    // 回転数（rpm） → 角速度（rad/s）
    double left_rpm = msg->data[0];
    double right_rpm = msg->data[1];
    double left_w = left_rpm * rpm_to_radps_ / gear_ratio_;
    double right_w = right_rpm * rpm_to_radps_ / gear_ratio_;

    // 差動駆動の速度計算
    double v = wheel_radius_ * (right_w + left_w) / 2.0;
    double w = wheel_radius_ * (right_w - left_w) / wheel_separation_;

    // 時間差分
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;

    // 位置更新
    double dx = v * cos(theta_) * dt;
    double dy = v * sin(theta_) * dt;
    double dtheta = w * dt;
    x_ += dx;
    y_ += dy;
    theta_ += dtheta;

    // オドメトリ出力
    geometry_msgs::msg::Quaternion odom_quat;
    odom_quat.z = sin(theta_ / 2.0);
    odom_quat.w = cos(theta_ / 2.0);

    // メッセージ生成
    nav_msgs::msg::Odometry odom;
    odom.header.stamp = current_time;
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_link";
    odom.pose.pose.position.x = x_;
    odom.pose.pose.position.y = y_;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = v;
    odom.twist.twist.angular.z = w;
    odom_pub_->publish(odom);

    // TF送信
    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp = current_time;
    tf.header.frame_id = "odom";
    tf.child_frame_id = "base_link";
    tf.transform.translation.x = x_;
    tf.transform.translation.y = y_;
    tf.transform.rotation = odom_quat;
    tf_broadcaster_->sendTransform(tf);
  }

  // パラメータと変数
  double wheel_radius_, wheel_separation_, gear_ratio_, rpm_to_radps_;
  double x_, y_, theta_;
  rclcpp::Time last_time_;

  // ROSインターフェース
  rclcpp::Subscription<std_msgs::msg::Int32MultiArray>::SharedPtr rpm_sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};
  
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RpmToOdomNode>());
  rclcpp::shutdown();
  return 0;
}

