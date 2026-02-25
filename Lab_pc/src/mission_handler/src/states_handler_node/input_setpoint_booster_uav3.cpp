#include <rclcpp/rclcpp.hpp>
#include "ros2_muavp_interface/msg/input_setpoint.hpp"

using ros2_muavp_interface::msg::InputSetpoint;

class InputSetpointBoosterUAV3 : public rclcpp::Node {
public:
  InputSetpointBoosterUAV3()
  : Node("input_setpoint_booster_uav3")
  {
    // Subscription to UAV3 MPC topic
    sub_uav3_ = create_subscription<InputSetpoint>(
      "/x500_3/input_setpoint_", 10,
      std::bind(&InputSetpointBoosterUAV3::uav3Callback, this, std::placeholders::_1));

    // Publisher to UAV3 MPC topic
    pub_uav3_ = create_publisher<InputSetpoint>("/x500_3/input_setpoint_mpc", 10);

    // Timer at 50Hz to publish last known values
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),  // 50 Hz
      std::bind(&InputSetpointBoosterUAV3::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Setpoint Booster Node started for UAV3 (50 Hz)");
  }

private:
  void uav3Callback(const InputSetpoint::SharedPtr msg) {
    last_uav3_ = *msg;
    uav3_received_ = true;
  }

  void timerCallback() {
    if (uav3_received_) {
      pub_uav3_->publish(last_uav3_);
    }
  }

  // ROS entities
  rclcpp::Subscription<InputSetpoint>::SharedPtr sub_uav3_;
  rclcpp::Publisher<InputSetpoint>::SharedPtr pub_uav3_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Last message
  InputSetpoint last_uav3_;
  bool uav3_received_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InputSetpointBoosterUAV3>());
  rclcpp::shutdown();
  return 0;
}