#ifndef FAILURESHANDLER_HPP
#define FAILURESHANDLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>

#include "common.hpp"

#include <cstdlib>
#include <ctime>

#include "common.hpp"

// #include <px4_msgs/msg/timesync.hpp>
#include "ros2_muavp_interface/msg/failure_check.hpp"
#include "ros2_muavp_interface/msg/failure_action.hpp"
#include "ros2_muavp_interface/msg/states_info.hpp"


class FailuresHandler : public rclcpp::Node
{
public:

    FailuresHandler(std::string agent_name);
    ~FailuresHandler();

private:

  // rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr _timesync_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::FailureCheck>::SharedPtr _failure_check_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::StatesInfo>::SharedPtr _states_info_sub;
  //
  rclcpp::Publisher<ros2_muavp_interface::msg::FailureCheck>::SharedPtr _failure_check_pub;
  rclcpp::Publisher<ros2_muavp_interface::msg::FailureAction>::SharedPtr _failure_action_pub;


  ros2_muavp_interface::msg::FailureCheck _failure_check_data;
  ros2_muavp_interface::msg::StatesInfo _states_info_data;

  rclcpp::TimerBase::SharedPtr _timer;
  // std::atomic<uint64_t> _timestamp;

	// std::unique_ptr<Command> _actualCommand;

  std::string _agent_name = "";

  void failureCheckCallback(const ros2_muavp_interface::msg::FailureCheck::SharedPtr msg_fc);
  void statesInfoCallback(const ros2_muavp_interface::msg::StatesInfo::SharedPtr msg_si);
  void failuresChecker();

};

#endif // FAILURESHANDLER_HPP
