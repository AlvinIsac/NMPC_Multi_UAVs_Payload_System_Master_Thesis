#include <rclcpp/rclcpp.hpp>
#include "ros2_muavp_interface/msg/input_setpoint.hpp"

using ros2_muavp_interface::msg::InputSetpoint;

class MPCPIDSwitcherX500_1 : public rclcpp::Node {
public:
  MPCPIDSwitcherX500_1()
  : Node("mpc_pid_switcher_x500_1")
  {
    // Subscriptions to MPC and PID topics
    sub_mpc_ = create_subscription<InputSetpoint>(
      "/x500_1/input_setpoint_mpc", 10,
      std::bind(&MPCPIDSwitcherX500_1::mpcCallback, this, std::placeholders::_1));

    sub_pid_ = create_subscription<InputSetpoint>(
      "/x500_1/input_setpoint_pid", 10,
      std::bind(&MPCPIDSwitcherX500_1::pidCallback, this, std::placeholders::_1));

    // Publisher to the final input_setpoint topic
    pub_setpoint_ = create_publisher<InputSetpoint>("/x500_1/input_setpoint", 10);

    RCLCPP_INFO(this->get_logger(), "MPC/PID Switcher Node started for x500_1 (Event-driven)");
  }

private:
  void mpcCallback(const InputSetpoint::SharedPtr msg) {
    last_mpc_ = *msg;
    mpc_received_ = true;
    last_mpc_time_ = this->get_clock()->now();
    
    // Once MPC is received, we always prioritize it
    if (!mpc_active_) {
      mpc_active_ = true;
      RCLCPP_INFO(this->get_logger(), "Switching to MPC control for x500_1");
    }
    
    // Immediately publish MPC data (highest priority)
    pub_setpoint_->publish(*msg);
  }

  void pidCallback(const InputSetpoint::SharedPtr msg) {
    last_pid_ = *msg;
    pid_received_ = true;
    last_pid_time_ = this->get_clock()->now();
    
    // Only publish PID if MPC is not active or MPC data is stale
    auto current_time = this->get_clock()->now();
    bool mpc_fresh = mpc_received_ && 
                     ((current_time - last_mpc_time_).nanoseconds() < 100000000);  // 100ms
    
    if (!mpc_active_ || !mpc_fresh) {
      if (mpc_active_ && !mpc_fresh) {
        RCLCPP_WARN(this->get_logger(), "MPC data stale, using PID for x500_1");
      }
      pub_setpoint_->publish(*msg);
    }
  }

  // ROS entities
  rclcpp::Subscription<InputSetpoint>::SharedPtr sub_mpc_, sub_pid_;
  rclcpp::Publisher<InputSetpoint>::SharedPtr pub_setpoint_;

  // Last messages and timestamps
  InputSetpoint last_mpc_, last_pid_;
  rclcpp::Time last_mpc_time_, last_pid_time_;
  
  // Status flags
  bool mpc_received_ = false;
  bool pid_received_ = false;
  bool mpc_active_ = false;  // Once true, always prioritize MPC
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MPCPIDSwitcherX500_1>());
  rclcpp::shutdown();
  return 0;
}