#include <rclcpp/rclcpp.hpp>
#include "ros2_muavp_interface/msg/input_setpoint.hpp"

using ros2_muavp_interface::msg::InputSetpoint;

class InputSetpointBoosterUAV1 : public rclcpp::Node {
public:
  InputSetpointBoosterUAV1()
  : Node("input_setpoint_booster_uav1")
  {
    // Subscription to UAV1 MPC topic
    sub_uav1_ = create_subscription<InputSetpoint>(
      "/x500_1/input_setpoint_", 10,
      std::bind(&InputSetpointBoosterUAV1::uav1Callback, this, std::placeholders::_1));

    // Publisher to UAV1 MPC topic
    pub_uav1_ = create_publisher<InputSetpoint>("/x500_1/input_setpoint_mpc", 10);

    // Timer at 50Hz to publish last known values
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),  // 50 Hz
      std::bind(&InputSetpointBoosterUAV1::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Setpoint Booster Node started for UAV1 (50 Hz)");
  }

private:
  void uav1Callback(const InputSetpoint::SharedPtr msg) {
    last_uav1_ = *msg;
    uav1_received_ = true;
  }

  void timerCallback() {
    if (uav1_received_) {
      pub_uav1_->publish(last_uav1_);
    }
  }

  // ROS entities
  rclcpp::Subscription<InputSetpoint>::SharedPtr sub_uav1_;
  rclcpp::Publisher<InputSetpoint>::SharedPtr pub_uav1_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Last message
  InputSetpoint last_uav1_;
  bool uav1_received_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InputSetpointBoosterUAV1>());
  rclcpp::shutdown();
  return 0;
}