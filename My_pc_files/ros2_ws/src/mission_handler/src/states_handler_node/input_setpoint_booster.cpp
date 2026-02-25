#include <rclcpp/rclcpp.hpp>
#include "ros2_muavp_interface/msg/input_setpoint.hpp"

using ros2_muavp_interface::msg::InputSetpoint;

class InputSetpointBooster : public rclcpp::Node {
public:
  InputSetpointBooster()
  : Node("input_setpoint_booster")
  {
    // Subscriptions to _ trailing topics
    sub_uav2_ = create_subscription<InputSetpoint>(
      "/x500_2/input_setpoint_", 10,
      std::bind(&InputSetpointBooster::uav2Callback, this, std::placeholders::_1));

    sub_uav3_ = create_subscription<InputSetpoint>(
      "/x500_3/input_setpoint_", 10,
      std::bind(&InputSetpointBooster::uav3Callback, this, std::placeholders::_1));

    // Publishers to real actuator command topics
    pub_uav2_ = create_publisher<InputSetpoint>("/x500_2/input_setpoint", 10);
    pub_uav3_ = create_publisher<InputSetpoint>("/x500_3/input_setpoint", 10);

    // Timer at 50Hz to publish last known values
    timer_ = create_wall_timer(
      std::chrono::milliseconds(20),  // 50 Hz
      std::bind(&InputSetpointBooster::timerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Setpoint Booster Node started (50 Hz for UAV2 and UAV3)");
  }

private:
  void uav2Callback(const InputSetpoint::SharedPtr msg) {
    last_uav2_ = *msg;
    uav2_received_ = true;
  }

  void uav3Callback(const InputSetpoint::SharedPtr msg) {
    last_uav3_ = *msg;
    uav3_received_ = true;
  }

  void timerCallback() {
    if (uav2_received_) {
      pub_uav2_->publish(last_uav2_);
    }
    if (uav3_received_) {
      pub_uav3_->publish(last_uav3_);
    }
  }

  // ROS entities
  rclcpp::Subscription<InputSetpoint>::SharedPtr sub_uav2_, sub_uav3_;
  rclcpp::Publisher<InputSetpoint>::SharedPtr pub_uav2_, pub_uav3_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Last messages
  InputSetpoint last_uav2_, last_uav3_;
  bool uav2_received_ = false;
  bool uav3_received_ = false;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<InputSetpointBooster>());
  rclcpp::shutdown();
  return 0;
}
