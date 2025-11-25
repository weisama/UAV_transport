#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

class FastLioToMavros : public rclcpp::Node {
public:
    FastLioToMavros() : Node("fastlio_to_mavros") {
        // 参数
        this->declare_parameter<std::string>("odom_topic", "/Odometry");
        this->declare_parameter<std::string>("vision_pose_topic", "/mavros/vision_pose/pose");
        this->declare_parameter<std::string>("frame_id", "map");  // frame_id 可改为 map

        std::string odom_topic = this->get_parameter("odom_topic").as_string();
        std::string vision_pose_topic = this->get_parameter("vision_pose_topic").as_string();
        frame_id_ = this->get_parameter("frame_id").as_string();

        sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic, 10,
            std::bind(&FastLioToMavros::odomCallback, this, std::placeholders::_1)
        );

        pub_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            vision_pose_topic, 10
        );

        RCLCPP_INFO(this->get_logger(), "Forwarding %s → %s", odom_topic.c_str(), vision_pose_topic.c_str());
    }

private:
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        geometry_msgs::msg::PoseStamped pose_msg;
        pose_msg.header.stamp = msg->header.stamp;
        pose_msg.header.frame_id = frame_id_;
        pose_msg.pose = msg->pose.pose;
        pub_pose_->publish(pose_msg);
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
    std::string frame_id_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FastLioToMavros>());
    rclcpp::shutdown();
    return 0;
}

