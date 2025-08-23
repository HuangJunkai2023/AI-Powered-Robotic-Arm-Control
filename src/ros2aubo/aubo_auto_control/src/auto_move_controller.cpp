#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include <moveit_msgs/action/move_group.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <cmath>  // 添加数学函数

typedef moveit_msgs::action::MoveGroup MoveGroupAction;

class AutoMoveController : public rclcpp::Node
{
public:
    AutoMoveController() : Node("auto_move_controller")
    {
        // 创建订阅器
        pose_subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "target_pose", 10,
            std::bind(&AutoMoveController::pose_callback, this, std::placeholders::_1));

        // 创建MoveGroup action客户端
        move_group_client_ = rclcpp_action::create_client<MoveGroupAction>(
            this, "move_action");

        is_executing_ = false;

        RCLCPP_INFO(this->get_logger(), "自动移动控制器已启动，等待目标位姿...");
    }

private:
    void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        if (is_executing_) {
            RCLCPP_WARN(this->get_logger(), "机械臂正在执行中，跳过本次目标位姿");
            return;
        }

        RCLCPP_INFO(this->get_logger(), 
            "接收到目标位姿: [%.3f, %.3f, %.3f] [%.3f, %.3f, %.3f, %.3f]",
            msg->pose.position.x, msg->pose.position.y, msg->pose.position.z,
            msg->pose.orientation.x, msg->pose.orientation.y, 
            msg->pose.orientation.z, msg->pose.orientation.w);

        // 执行运动规划和控制
        execute_motion_to_pose(*msg);
    }

    void execute_motion_to_pose(const geometry_msgs::msg::PoseStamped& target_pose)
    {
        is_executing_ = true;

        try {
            // 等待MoveGroup action服务器
            if (!move_group_client_->wait_for_action_server(std::chrono::seconds(10))) {
                RCLCPP_ERROR(this->get_logger(), "MoveGroup action服务器不可用！");
                is_executing_ = false;
                return;
            }

            RCLCPP_INFO(this->get_logger(), "开始运动规划...");

            // 创建MoveGroup目标
            auto move_goal = MoveGroupAction::Goal();
            move_goal.request.group_name = "manipulator_i5";
            move_goal.request.num_planning_attempts = 20;  // 增加规划尝试次数
            move_goal.request.allowed_planning_time = 10.0;  // 增加规划时间
            move_goal.request.max_velocity_scaling_factor = 0.3;  // 降低速度
            move_goal.request.max_acceleration_scaling_factor = 0.3;  // 降低加速度
            move_goal.request.planner_id = "RRTConnect";  // 明确指定规划器

            // 验证目标位姿的四元数
            auto target_quat = target_pose.pose.orientation;
            double quat_norm = sqrt(target_quat.x * target_quat.x + 
                                   target_quat.y * target_quat.y + 
                                   target_quat.z * target_quat.z + 
                                   target_quat.w * target_quat.w);
            
            // 如果四元数为零或接近零，使用默认方向（单位四元数）
            if (quat_norm < 0.1) {
                RCLCPP_WARN(this->get_logger(), "检测到无效的目标方向，使用默认方向");
                target_quat.x = 0.0;
                target_quat.y = 0.0;
                target_quat.z = 0.0;
                target_quat.w = 1.0;
            } else {
                // 归一化四元数
                target_quat.x /= quat_norm;
                target_quat.y /= quat_norm;
                target_quat.z /= quat_norm;
                target_quat.w /= quat_norm;
            }

            // 设置目标位姿
            move_goal.request.goal_constraints.resize(1);
            move_goal.request.goal_constraints[0].position_constraints.resize(1);
            move_goal.request.goal_constraints[0].orientation_constraints.resize(1);

            // 位置约束
            auto& pos_constraint = move_goal.request.goal_constraints[0].position_constraints[0];
            pos_constraint.link_name = "wrist3_Link";  // 末端执行器
            pos_constraint.header.frame_id = target_pose.header.frame_id;
            pos_constraint.constraint_region.primitive_poses.resize(1);
            pos_constraint.constraint_region.primitive_poses[0].position = target_pose.pose.position;
            pos_constraint.constraint_region.primitives.resize(1);
            pos_constraint.constraint_region.primitives[0].type = shape_msgs::msg::SolidPrimitive::SPHERE;
            pos_constraint.constraint_region.primitives[0].dimensions = {0.05};  // 增加位置容差到5cm
            pos_constraint.weight = 1.0;

            // 方向约束
            auto& orient_constraint = move_goal.request.goal_constraints[0].orientation_constraints[0];
            orient_constraint.link_name = "wrist3_Link";
            orient_constraint.header.frame_id = target_pose.header.frame_id;
            orient_constraint.orientation = target_quat;  // 使用验证后的四元数
            orient_constraint.absolute_x_axis_tolerance = 0.3;  // 增加方向容差
            orient_constraint.absolute_y_axis_tolerance = 0.3;
            orient_constraint.absolute_z_axis_tolerance = 0.3;
            orient_constraint.weight = 0.5;  // 降低方向约束权重

            // 发送目标
            auto send_goal_options = rclcpp_action::Client<MoveGroupAction>::SendGoalOptions();
            send_goal_options.result_callback = 
                std::bind(&AutoMoveController::move_group_result_callback, this, std::placeholders::_1);

            auto goal_handle_future = move_group_client_->async_send_goal(move_goal, send_goal_options);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "执行运动时发生异常: %s", e.what());
            is_executing_ = false;
        }
    }

    void move_group_result_callback(const rclcpp_action::ClientGoalHandle<MoveGroupAction>::WrappedResult& result)
    {
        switch (result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "MoveGroup规划和执行成功！");
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "MoveGroup执行被中止");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_WARN(this->get_logger(), "MoveGroup执行被取消");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "MoveGroup执行失败");
                break;
        }
        is_executing_ = false;
    }

    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_subscription_;
    rclcpp_action::Client<MoveGroupAction>::SharedPtr move_group_client_;
    bool is_executing_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    
    auto node = std::make_shared<AutoMoveController>();
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
