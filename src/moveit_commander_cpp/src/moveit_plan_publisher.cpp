#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <string>

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface; 


using namespace std::placeholders;


class Commander{
    public:
        Commander(std::shared_ptr<rclcpp::Node> node){
            node_ = node;
            arm_ = std::make_shared<MoveGroupInterface>(node_, "arm");
            arm_ -> setMaxVelocityScalingFactor(1.0);
            arm_ -> setMaxAccelerationScalingFactor(1.0); 
            ref_traj_pub = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("moveit_trajectory", 10);
            timer_ = node_->create_wall_timer(
                std::chrono::milliseconds(1000),
                std::bind(&Commander::plan, this)
            );
        }

        // point = JointTrajectoryPoint()
        // traj = JointTrajectory()
        // traj.header.stamp = self.get_clock().now().to_msg()
        // traj.joint_names = self.joint_names
        // point.positions = [1.0, -1.0, -2.0, -2.0, 1.0, 0.0]
        // point.velocities = [0.0] * 6
        // # point.effort = [0.0] * 6
        // point.time_from_start.sec = 2
        // traj.points.append(point)

    private:
        std::shared_ptr<rclcpp::Node> node_;
        std::shared_ptr<MoveGroupInterface> arm_;
        rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr ref_traj_pub;
        rclcpp::TimerBase::SharedPtr timer_;
        std::vector<std::string> joint_names = { 
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint"
        };

        void plan()
        {   
            arm_ -> setStartStateToCurrentState();
            MoveGroupInterface::Plan plan;
            bool success = (arm_ -> plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
            trajectory_msgs::msg::JointTrajectory traj;
            trajectory_msgs::msg::JointTrajectoryPoint point;
            traj.header.stamp = this -> node_ -> get_clock() -> now();
            traj.joint_names = this -> joint_names;
            point.positions = {0.0, -1.57, -1.57, 0.0, 0.0, 0.0};
            point.velocities = std::vector<double>(6, 0.0);
            point.time_from_start.sec = 2;
            point.time_from_start.nanosec = 0;

            traj.points.push_back(point);
            
            if (success){
                ref_traj_pub -> publish(traj);
                RCLCPP_INFO(this-> node_-> get_logger(), "The reference trajectory is valid...");
            }
        }
       
};


int main(int argc, char **argv)
{

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("commander");
    auto commander = Commander(node);
    rclcpp::spin(node);        
    rclcpp::shutdown();

    return 0;

}     