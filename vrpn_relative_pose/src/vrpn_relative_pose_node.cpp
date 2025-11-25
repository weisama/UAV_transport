#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace std::chrono_literals;

class VRPNRelativePose : public rclcpp::Node {
public:
    VRPNRelativePose() : Node("vrpn_relative_pose") {
        // 参数声明
        this->declare_parameter<std::string>("this_uav", "t1");
        this_uav_ = this->get_parameter("this_uav").as_string();
        RCLCPP_INFO(this->get_logger(), "Tracking UAV: %s", this_uav_.c_str());

        // 初始化发布者
        pub_self_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/mavros/vision_pose/pose", 10);
        pub_other_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/vrpn/other/pose", 10);
        RCLCPP_INFO(this->get_logger(), "Publishers created");

        // 初始化订阅者
        sub_t1_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vrpn/t1/pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                RCLCPP_DEBUG(this->get_logger(), "Received t1 pose");
                handle_pose("t1", msg);
            });
        
        sub_t2_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/vrpn/t2/pose", 10,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                RCLCPP_DEBUG(this->get_logger(), "Received t2 pose");
                handle_pose("t2", msg);
            });

        RCLCPP_INFO(this->get_logger(), "Subscribers initialized");
    }

private:
    void handle_pose(const std::string& uav_id, const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        // 检查是否已设置该无人机的原点
        bool need_set_origin = false;
        
        if (uav_id == "t1" && !origin_set_t1_) {
            need_set_origin = true;
        } else if (uav_id == "t2" && !origin_set_t2_) {
            need_set_origin = true;
        }

        // 如果需要设置原点
        if (need_set_origin) {
            RCLCPP_INFO(this->get_logger(), "Setting origin for %s", uav_id.c_str());
            set_origin(uav_id, msg);
            return; // 原点设置后不处理这帧数据
        }

        // 计算相对位置
        auto relative_pose = std::make_shared<geometry_msgs::msg::PoseStamped>();
        relative_pose->header = msg->header;
        relative_pose->header.frame_id = "map";  // 明确坐标系为map
        
        if (uav_id == "t1") {
            relative_pose->pose.position.x = msg->pose.position.x - t1_origin_.x;
            relative_pose->pose.position.y = msg->pose.position.y - t1_origin_.y;
            relative_pose->pose.position.z = msg->pose.position.z - t1_origin_.z;
            relative_pose->pose.orientation = msg->pose.orientation;
        } else if (uav_id == "t2") {
            relative_pose->pose.position.x = msg->pose.position.x - t2_origin_.x;
            relative_pose->pose.position.y = msg->pose.position.y - t2_origin_.y;
            relative_pose->pose.position.z = msg->pose.position.z - t2_origin_.z;
            relative_pose->pose.orientation = msg->pose.orientation;
        }

        // 路由发布
        if (uav_id == this_uav_) {
            RCLCPP_DEBUG(this->get_logger(), "Publishing to /mavros/vision_pose/pose");
            pub_self_->publish(*relative_pose);
        } else {
            // 只有当另一架无人机的原点已设置时才发布
            if ((uav_id == "t1" && origin_set_t1_) || (uav_id == "t2" && origin_set_t2_)) {
                RCLCPP_DEBUG(this->get_logger(), "Publishing to /vrpn/other/pose");
                pub_other_->publish(*relative_pose);
            }
        }
    }

    void set_origin(const std::string& uav_id, const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (uav_id == "t1") {
            t1_origin_ = msg->pose.position;
            origin_set_t1_ = true;
            RCLCPP_INFO(this->get_logger(), "Set t1 origin: (%.2f, %.2f, %.2f)", 
                        t1_origin_.x, t1_origin_.y, t1_origin_.z);
        } else if (uav_id == "t2") {
            t2_origin_ = msg->pose.position;
            origin_set_t2_ = true;
            RCLCPP_INFO(this->get_logger(), "Set t2 origin: (%.2f, %.2f, %.2f)", 
                        t2_origin_.x, t2_origin_.y, t2_origin_.z);
        }
        
        RCLCPP_INFO(this->get_logger(), "%s origin ready (map frame)", uav_id.c_str());
    }

    // 订阅者和发布者
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_t1_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_t2_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_self_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_other_;

    // 状态变量
    std::string this_uav_;
    bool origin_set_t1_ = false;
    bool origin_set_t2_ = false;
    geometry_msgs::msg::Point t1_origin_;
    geometry_msgs::msg::Point t2_origin_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VRPNRelativePose>());
    rclcpp::shutdown();
    return 0;
}
