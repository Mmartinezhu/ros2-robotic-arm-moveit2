#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

class ArmBridge : public rclcpp::Node {
public:
    ArmBridge() : Node("arm_bridge") {
        // Suscribirse a MoveIt (control)
        traj_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/arm_controller/joint_trajectory", 10,
            std::bind(&ArmBridge::traj_callback, this, std::placeholders::_1));

        // Reenviar estados desde STM32
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&ArmBridge::joint_callback, this, std::placeholders::_1));

        joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/arm/joint_states", 10);
    }

private:
    void traj_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg) {
        // Aquí podrías filtrar o reenviar al STM32
        RCLCPP_INFO(this->get_logger(), "Trajectory received with %ld points", msg->points.size());
    }

    void joint_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
        // Reenvía el estado para RViz
        joint_state_pub_->publish(*msg);
    }

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr traj_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ArmBridge>());
    rclcpp::shutdown();
    return 0;
}

