#include "failures_handler.hpp"
using namespace common;

FailuresHandler::FailuresHandler(std::string agent_name) : Node("failures_handler", "failures_handler", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  // saving the agent name
  _agent_name = agent_name;

  // the ROS2 sub for Failure data
  _failure_check_sub = this->create_subscription<ros2_muavp_interface::msg::FailureCheck>("/agents/failure_check",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::FailureCheck::SharedPtr msg_fc)
          {
            _failure_check_data = *msg_fc;
            failuresChecker();
          });

  // get common timestamp
  // _timesync_sub = this->create_subscription<px4_msgs::msg::Timesync>("/" + agent_name + "/fmu/timesync/out", 1, // /Timesync_PubSubTopic
  //                 [this](const px4_msgs::msg::Timesync::UniquePtr msg_ts)
  //                 {
  //                   _timestamp.store(msg_ts->timestamp);
  //                 });

  _states_info_sub = this->create_subscription<ros2_muavp_interface::msg::StatesInfo>("/" + agent_name + "/states_info",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::StatesInfo::SharedPtr msg_si)
          {
            _states_info_data = *msg_si;
          });


  _failure_check_pub = this->create_publisher<ros2_muavp_interface::msg::FailureCheck>("/agents/failure_check", 1);
  _failure_action_pub = this->create_publisher<ros2_muavp_interface::msg::FailureAction>("/" + agent_name + "/failure_action", 1);

  // initializing message with false values
  _failure_check_data.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  _failure_check_data.uav_1_failed = false;
  _failure_check_data.uav_2_failed = false;
  _failure_check_data.uav_3_failed = false;

  _failure_check_data.uav_1_detached = false;
  _failure_check_data.uav_2_detached = false;
  _failure_check_data.uav_3_detached = false;

  _failure_check_pub->publish(_failure_check_data);

  // lambda that executes the main task
  // auto timercallback = [this]() -> void
  // {
  //   // failuresChecker();
  // };
  // // executing the callback every tot ms, when the timer expires
	// _timer = this->create_wall_timer(std::chrono::milliseconds(10), timercallback);
}

void FailuresHandler::failuresChecker()
{
  // if an agent failed
  bool uav_1_failed = (_failure_check_data.uav_1_failed && _failure_check_data.uav_1_detached);
  bool uav_2_failed = (_failure_check_data.uav_2_failed && _failure_check_data.uav_2_detached);
  bool uav_3_failed = (_failure_check_data.uav_3_failed && _failure_check_data.uav_3_detached);

  if(uav_1_failed || uav_2_failed || uav_3_failed)
  {
    ros2_muavp_interface::msg::FailureAction failure_action_data;
    failure_action_data.agents_alive = {1, 1, 1};

    std::string agent_failed = "";
    // checking which one and signaling it
    if(_failure_check_data.uav_1_failed)
    {
      agent_failed = "x500_1";
      failure_action_data.agents_alive[0] = 0;
    }
    if(_failure_check_data.uav_2_failed)
    {
      agent_failed = "x500_2";
      failure_action_data.agents_alive[1] = 0;
    }
    if(_failure_check_data.uav_3_failed)
    {
      agent_failed = "x500_3";
      failure_action_data.agents_alive[2] = 0;
    }

    // following the routine...
    switch((int) _states_info_data.curr_state)
    {
      case states::PARAMETERS:
        failure_action_data.curr_action = fail_states::KEEP_ON;
        break;

      case states::PRE_TAKE_OFF:
        failure_action_data.curr_action = fail_states::STOP;
        break;

      case states::TAKE_OFF:
        failure_action_data.curr_action = fail_states::STOP;
        // failure_action_data.curr_action = fail_states::RECOVERY;
        break;

      case states::MOVE:
        failure_action_data.curr_action = fail_states::RECOVERY;
        break;

      case states::ROTATE:
        failure_action_data.curr_action = fail_states::RECOVERY;
        break;

      case states::LAND:
        failure_action_data.curr_action = fail_states::STOP;
        break;

      case states::IDLE:
        failure_action_data.curr_action = fail_states::KEEP_ON;
        break;

      default:
        failure_action_data.curr_action = fail_states::GO_TO;
      break;
    }
    // ... but, if the handled agent is the one that failed, assigning a default directive
    if(agent_failed == _agent_name)
    {
      failure_action_data.curr_action = fail_states::STOP;
    }

    failure_action_data.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    // sending multiple messages to avoid data loss
    for(int i = 0; i < 3; i++)
      _failure_action_pub->publish(failure_action_data);
  }
}

FailuresHandler::~FailuresHandler()
{

}
