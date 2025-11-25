#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <mavros_msgs/msg/state.hpp>
#include <mavros_msgs/srv/command_bool.hpp>
#include <mavros_msgs/srv/set_mode.hpp>
#include <chrono>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>  // 用于四元数和欧拉角转换

using namespace std::chrono_literals;

class OffboardTestNode : public rclcpp::Node {
public:
    OffboardTestNode() : Node("offboard_test_node") {
        vision_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/mavros/vision_pose/pose", 10,
            std::bind(&OffboardTestNode::visionPoseCallback, this, std::placeholders::_1));

        local_pos_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "/mavros/setpoint_position/local", 10);

        vel_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
            "/mavros/setpoint_velocity/cmd_vel", 10);

        arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(
            "/mavros/cmd/arming");

        set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(
            "/mavros/set_mode");

        // 初始化控制参数
        control_mode_flag = 0; // 默认位置控制
        task = 0;
        delay_counter = 0;

        // 初始化目标位姿和速度
        target_x = 0.0;
        target_y = 0.0;
        target_z = 0.0;
        target_vx = 0.0;
        target_vy = 0.0;
        target_vz = 0.0;

        // 等待服务
        RCLCPP_INFO(this->get_logger(), "Waiting for service servers...");
        arming_client_->wait_for_service();
        set_mode_client_->wait_for_service();

        // 先发送一批位置setpoints保证OFFBOARD能切换
        for (int i = 0; i < 50; ++i) {
            publishPosition(0.0, 0.0, 0.5);
            rclcpp::sleep_for(20ms);
        }
        
        setOffboardMode();
        arm();
        RCLCPP_INFO(this->get_logger(), "Arm");

        timer_ = this->create_wall_timer(20ms, std::bind(&OffboardTestNode::timerCallback, this));
    }

private:
    // 订阅/发布
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vision_pose_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr local_pos_pub_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr vel_pub_;
    rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;
    rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;
    rclcpp::TimerBase::SharedPtr timer_;

    // 状态
    geometry_msgs::msg::PoseStamped current_pose_;

    // 控制变量
    int control_mode_flag; // 0位置控制 1速度控制
    int task;              // 任务状态机
    int delay_counter;     // 计时器计数，用于delay模拟

    // 目标变量
    double target_x, target_y, target_z;
    double target_vx, target_vy, target_vz;

    void visionPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        current_pose_ = *msg;
    }

    void arm() {
        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = true;
        arming_client_->async_send_request(req);
    }

    void disarm() {
        auto req = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
        req->value = false;
        arming_client_->async_send_request(req);
    }

    void setOffboardMode() {
        auto req = std::make_shared<mavros_msgs::srv::SetMode::Request>();
        req->custom_mode = "OFFBOARD";
        set_mode_client_->async_send_request(req);
    }

    // 将yaw转换为四元数
    geometry_msgs::msg::Quaternion yawToQuaternion(double yaw) {
        tf2::Quaternion q;
        q.setRPY(0, 0, yaw);
        return tf2::toMsg(q);
    }
    
    void publishPosition(double x, double y, double z) {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = this->now();
        msg.header.frame_id = "map";
        msg.pose.position.x = x;
        msg.pose.position.y = y;
        msg.pose.position.z = z;

        // 将yaw转换为四元数并设置到msg
        msg.pose.orientation = yawToQuaternion(0.0);

        local_pos_pub_->publish(msg);
    }

    void publishVelocity(double vx, double vy, double vz) {
        geometry_msgs::msg::TwistStamped msg;
        msg.header.stamp = this->now();
        // msg.header.frame_id = "map";
        msg.twist.linear.x = vx;
        msg.twist.linear.y = vy;
        msg.twist.linear.z = vz;

        vel_pub_->publish(msg);
    }

    bool reachPosition(double x, double y, double z, double threshold = 0.1) {
        double dx = current_pose_.pose.position.x - x;
        double dy = current_pose_.pose.position.y - y;
        double dz = current_pose_.pose.position.z - z;
        return (dx * dx + dy * dy + dz * dz) < (threshold * threshold);
    }

    // 模拟延迟函数，单位ms，非阻塞
    // 定时器20ms调用一次，计数到count后返回true
    bool delay_ms(int ms) {
        int count_target = ms / 20;
        if (delay_counter < count_target) {
            delay_counter++;
            return false;
        } else {
            delay_counter = 0;
            return true;
        }
    }

    void timerCallback() {
        // 发布控制命令
        if (control_mode_flag == 0) {
            publishPosition(target_x, target_y, target_z);
        } else {
            publishVelocity(target_vx, target_vy, target_vz);
        }

        // 任务状态机
        switch (task) {
            case 0:  // 起飞至0.5m
                RCLCPP_INFO(this->get_logger(), "Take off");
                control_mode_flag = 0;
                target_x = 0;
                target_y = 0;
                target_z = 0.5;
                if (reachPosition(0,0,0.5)) {
                    task = 1;
                }
                break;
            case 1:  // 悬停2秒
                RCLCPP_INFO(this->get_logger(), "Hover");
                control_mode_flag = 0;
                target_x = 0;
                target_y = 0;
                target_z = 0.5;
                if (delay_ms(5000)) {
                    task = 2;
                }
                break;
            // case 2:  // 飞行到(1,0,0.5)
            //     control_mode_flag = 0;
            //     target_x = 1;
            //     target_y = 0;
            //     target_z = 0.5;
            //     if (reachPosition(target_x, target_y, target_z)) {
            //         task = 3;
            //     }
            //     break;
            // case 3:  // 飞行到(1,1,0.5)
            //     control_mode_flag = 0;
            //     target_x = 1;
            //     target_y = 1;
            //     target_z = 0.5;
            //     if (reachPosition(target_x, target_y, target_z)) {
            //         task = 4;
            //     }
            //     break;
            // case 4:  // 悬停2秒
            //     control_mode_flag = 0;
            //     target_x = 1;
            //     target_y = 1;
            //     target_z = 0.5;
            //     if (delay_ms(2000)) {
            //         task = 5;
            //     }
            //     break;
            // case 5:  // 向后飞行2秒（速度控制，-x方向）
            //     control_mode_flag = 1;
            //     target_vx = -0.5;
            //     target_vy = 0.0;
            //     target_vz = 0.0;
            //     if (delay_ms(2000)) {
            //         task = 6;
            //     }
            //     break;
            // case 6:  // 向左飞行2秒（速度控制，-y方向）
            //     control_mode_flag = 1;
            //     target_vx = 0.0;
            //     target_vy = -0.5;
            //     target_vz = 0.0;
            //     if (delay_ms(2000)) {
            //         task = 7;
            //     }
            //     break;
            case 2:  // 降落
                RCLCPP_INFO(this->get_logger(), "Land");
                control_mode_flag = 0;
                target_x = 0;
                target_y = 0;
                target_z = 0.0;
                if (reachPosition(target_x, target_y, target_z, 0.1)) {
                    disarm();
                    RCLCPP_INFO(this->get_logger(), "任务完成");
                    task = 8;
                }
                break;
            case 8: // 任务结束，不再动作
                break;
            default:
                break;
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OffboardTestNode>());
    rclcpp::shutdown();
    return 0;
}

