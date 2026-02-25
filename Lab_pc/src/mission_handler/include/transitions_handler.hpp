#ifndef TRANSITIONSHANDLER_HPP
#define TRANSITIONSHANDLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>

#include <deque>
#include <Eigen/Dense>

#include "px4_msgs/msg/vehicle_odometry.hpp"
// #include <px4_msgs/msg/timesync.hpp>
#include "ros2_muavp_interface/msg/ready_signals.hpp"
#include "ros2_muavp_interface/msg/states_info.hpp"
#include "ros2_muavp_interface/msg/failure_action.hpp"
#include "ros2_muavp_interface/msg/rigid_body_pose.hpp"
#include "ros2_muavp_interface/msg/joint_force_torque.hpp"

#include "ros2_muavp_interface/srv/failure_info.hpp"
#include "ros2_muavp_interface/srv/path_info.hpp"

using Eigen::MatrixXd;
using Eigen::VectorXd;

#define PI 3.14159265

class TransitionsHandler : public rclcpp::Node
{
public:

    TransitionsHandler(std::string agent_name);
    ~TransitionsHandler();

private:

  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odometry_sub;
  // rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr _timesync_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::RigidBodyPose>::SharedPtr _agent_pose_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::ReadySignals>::SharedPtr _ready_takeoff_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::ReadySignals>::SharedPtr _ready_params_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::ReadySignals>::SharedPtr _ready_muavp_ekf_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::ReadySignals>::SharedPtr _ready_move_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::ReadySignals>::SharedPtr _ready_bfr_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::ReadySignals>::SharedPtr _ready_land_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::StatesInfo>::SharedPtr _states_info_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::FailureAction>::SharedPtr _failure_action_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::JointForceTorque>::SharedPtr _force_joint_sub;
  //
  rclcpp::Publisher<ros2_muavp_interface::msg::ReadySignals>::SharedPtr _ready_takeoff_pub;
  rclcpp::Publisher<ros2_muavp_interface::msg::ReadySignals>::SharedPtr _ready_move_pub;
  rclcpp::Publisher<ros2_muavp_interface::msg::StatesInfo>::SharedPtr _states_info_pub;
  rclcpp::Publisher<ros2_muavp_interface::msg::ReadySignals>::SharedPtr _ready_bfr_pub;
  rclcpp::Publisher<ros2_muavp_interface::msg::ReadySignals>::SharedPtr _ready_land_pub;
  rclcpp::Publisher<ros2_muavp_interface::msg::FailureAction>::SharedPtr _failure_action_pub;

  rclcpp::Client<ros2_muavp_interface::srv::PathInfo>::SharedPtr _path_info_clt;
  rclcpp::Client<ros2_muavp_interface::srv::FailureInfo>::SharedPtr _failure_info_clt;

  px4_msgs::msg::VehicleOdometry _drone_pose;
  ros2_muavp_interface::msg::RigidBodyPose _agent_pose_data;
  ros2_muavp_interface::msg::ReadySignals _ready_signal_takeoff_data;
  ros2_muavp_interface::msg::ReadySignals _ready_signal_params_data;
  ros2_muavp_interface::msg::ReadySignals _ready_signal_muavp_ekf_data;
  ros2_muavp_interface::msg::ReadySignals _ready_signal_move_data;
  ros2_muavp_interface::msg::ReadySignals _ready_signal_bfr_data;
  ros2_muavp_interface::msg::ReadySignals _ready_signal_land_data;
  ros2_muavp_interface::msg::StatesInfo _states_info_data;
  ros2_muavp_interface::msg::FailureAction _failure_action_data;
  ros2_muavp_interface::msg::JointForceTorque _joint_data;

  std::deque<ros2_muavp_interface::msg::StatesInfo> _taskListParsed;
  std::deque<ros2_muavp_interface::srv::PathInfo::Request> _movePathListParsed;
  std::deque<ros2_muavp_interface::srv::FailureInfo::Request> _failureListParsed;
  std::map<int, ros2_muavp_interface::msg::StatesInfo> _pfTasksMapParsed;
  ros2_muavp_interface::msg::StatesInfo _currentTask;
  ros2_muavp_interface::msg::StatesInfo _parameters;

  rclcpp::TimerBase::SharedPtr _timer;
  rclcpp::TimerBase::SharedPtr _timer_failure;
  // std::atomic<uint64_t> _timestamp;

  double _euler_angles[3];

  std::string _agent_name = "";
  int _min_iter_takeoff[3];
  int _min_iter_move[3];
  int _min_iter_land[3];
  int _min_iter_bfr[3];
  int _min_iter_stored_param;
  int _min_iter_cond_takeoff;
  int _min_iter_cond_move;
  int _min_iter_cond_land;
  int _min_iter_cond_bfr;
  bool _newTask;
  bool _firstPrint;
  bool _first_pass;
  bool _failure_occured;
  bool _system_recovered;
  bool _failure_info_result;
  bool _path_info_result;
  bool _around_there_tension;
  bool _seq_pre_takeoff;

  struct {
    VectorXd body_tension = VectorXd(3);
    VectorXd inert_tension = VectorXd(3);
    double tension_norm;
  } _tensions;

  void getDroneOdometryCallback(const px4_msgs::msg::VehicleOdometry::SharedPtr msg_odom);
  void parseTaskList();
  void parsePathList();
  void parseFailureList();
  void parsePFTaskList();
  bool sendPathInfoData();
  bool sendFailureInfoData();
  void loadTask();
  bool transitionsChecker();
  bool failureTransitionsChecker();
  void sendReadySignalTakeoff();
  void sendReadySignalMove();
  void sendReadySignalLand();
  void sendReadySignalBackFromRecovery();
  void checkTakeoffReady();
  void checkMoveReady();
  void checkLandReady();
  void checkBackFromRecoveryReady();
  void setList(std::deque<ros2_muavp_interface::msg::StatesInfo> _list);
  void setPathList(std::deque<ros2_muavp_interface::srv::PathInfo::Request> _list);
  void setFailureList(std::deque<ros2_muavp_interface::srv::FailureInfo::Request> _list);
  void setPFTasksMap(std::deque<ros2_muavp_interface::msg::StatesInfo> _list);
  void taskListAdaptation();
  void transitionsController();
  void tensionRodComputation();
  double norm(VectorXd vector);
  void quaternionToEuler(std::array<float, 4> q);

};

#endif // TRANSITIONSHANDLER_HPP
