#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

class PosePublisher : public rclcpp::Node
{
public:
    PosePublisher() : Node("pose_publisher")
    {
        // 创建发布器
        pose_publisher_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10);

        // 创建定时器，每5秒发布一次目标位姿
        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&PosePublisher::publish_target_pose, this));

        // 预定义几个目标位置
        init_target_poses();
        current_pose_index_ = 0;

        RCLCPP_INFO(this->get_logger(), "位姿发布节点已启动，将循环发布预定义的目标位姿");
    }

private:
    void init_target_poses()
    {
        // 定义几个目标位姿（相对于base_link坐标系）
        geometry_msgs::msg::PoseStamped pose1;
        pose1.header.frame_id = "base_link";
        pose1.pose.position.x = 0.3;
        pose1.pose.position.y = 0.2;
        pose1.pose.position.z = 0.4;
        pose1.pose.orientation.x = 0.0;
        pose1.pose.orientation.y = 0.0;
        pose1.pose.orientation.z = 0.0;
        pose1.pose.orientation.w = 1.0;
        target_poses_.push_back(pose1);

        geometry_msgs::msg::PoseStamped pose2;
        pose2.header.frame_id = "base_link";
        pose2.pose.position.x = 0.4;
        pose2.pose.position.y = -0.1;
        pose2.pose.position.z = 0.5;
        pose2.pose.orientation.x = 0.0;
        pose2.pose.orientation.y = 0.707;
        pose2.pose.orientation.z = 0.0;
        pose2.pose.orientation.w = 0.707;
        target_poses_.push_back(pose2);

        geometry_msgs::msg::PoseStamped pose3;
        pose3.header.frame_id = "base_link";
        pose3.pose.position.x = 0.2;
        pose3.pose.position.y = 0.3;
        pose3.pose.position.z = 0.6;
        pose3.pose.orientation.x = 0.0;
        pose3.pose.orientation.y = 0.0;
        pose3.pose.orientation.z = 0.707;
        pose3.pose.orientation.w = 0.707;
        target_poses_.push_back(pose3);

        // 回到初始位置
        geometry_msgs::msg::PoseStamped pose_home;
        pose_home.header.frame_id = "base_link";
        pose_home.pose.position.x = 0.35;
        pose_home.pose.position.y = 0.0;
        pose_home.pose.position.z = 0.45;
        pose_home.pose.orientation.x = 0.0;
        pose_home.pose.orientation.y = 0.0;
        pose_home.pose.orientation.z = 0.0;
        pose_home.pose.orientation.w = 1.0;
        target_poses_.push_back(pose_home);
    }

    void publish_target_pose()
    {
        if (target_poses_.empty()) return;

        // 获取当前目标位姿
        auto& current_pose = target_poses_[current_pose_index_];
        current_pose.header.stamp = this->get_clock()->now();

        // 发布目标位姿
        pose_publisher_->publish(current_pose);

        RCLCPP_INFO(this->get_logger(), 
            "发布目标位姿 %zu: [%.3f, %.3f, %.3f] [%.3f, %.3f, %.3f, %.3f]",
            current_pose_index_ + 1,
            current_pose.pose.position.x,
            current_pose.pose.position.y,
            current_pose.pose.position.z,
            current_pose.pose.orientation.x,
            current_pose.pose.orientation.y,
            current_pose.pose.orientation.z,
            current_pose.pose.orientation.w);

        // 循环到下一个位姿
        current_pose_index_ = (current_pose_index_ + 1) % target_poses_.size();
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<geometry_msgs::msg::PoseStamped> target_poses_;
    size_t current_pose_index_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PosePublisher>());
    rclcpp::shutdown();
    return 0;
}
