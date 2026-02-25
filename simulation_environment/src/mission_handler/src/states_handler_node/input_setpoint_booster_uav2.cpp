#include <rclcpp/rclcpp.hpp>
#include "ros2_muavp_interface/msg/input_setpoint.hpp"

using ros2_muavp_interface::msg::InputSetpoint;

class InputSetpointBoosterUAV2 : public rclcpp::Node {
public:
  InputSetpointBoosterUAV2()
  : Node("input_setpoint_booster_uav2")
  {
    // Subscription to UAV2 MPC topic
    sub_uav2_ = create_subscription<InputSetpoint>(
      "/x500_2/input_setpoint_", 10,
      std::bind(&InputSetpointBoosterUAV2::uav2Callback, this, std::placeholders::_1));

    // Publisher to UAV2 MPC topic
    pub_uav2_ = create_publisher<InputSetpoint>("/x500_2/input_setpoint_mpc", 10);

    // Timer at 50Hz to publish last known values
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),  // 50 Hz
      std::bind(&InputSetpointBoosterUAV2::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Setpoint Booster Node started for UAV2 (50 Hz)");
  }

private:
  void uav2Callback(const InputSetpoint::SharedPtr msg) {
    last_uav2_ = *msg;
    uav2_received_ = true;
  }

  void timerCallback() {
    if (uav2_received_) {
      pub_uav2_->publish(last_uav2_);
    }
  }

  // ROS entities
  rclcpp::Subscription<InputSetpoint>::SharedPtr sub_uav2_;
  rclcpp::Publisher<InputSetpoint>::SharedPtr pub_uav2_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Last message
  InputSetpoint last_uav2_;
  bool uav2_received_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InputSetpointBoosterUAV2>());
  rclcpp::shutdown();
  return 0;
}