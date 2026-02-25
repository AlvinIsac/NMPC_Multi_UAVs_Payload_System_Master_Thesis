#ifndef STATESHANDLER_HPP
#define STATESHANDLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/qos.hpp>
#include <algorithm>
#include <math.h>

#include "common.hpp"
#include <Eigen/Dense>
#include <deque>

// #include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/actuator_armed.hpp>
#include "ros2_muavp_interface/msg/states_info.hpp"
#include "ros2_muavp_interface/msg/input_setpoint.hpp"
#include "ros2_muavp_interface/msg/ready_signals.hpp"
#include "ros2_muavp_interface/msg/joint_force_torque.hpp"
#include "ros2_muavp_interface/msg/failure_action.hpp"
#include "ros2_muavp_interface/msg/failure_check.hpp"
#include "ros2_muavp_interface/msg/rigid_body_pose.hpp"
#include "ros2_muavp_interface/msg/exchange_info.hpp"

#include "ros2_muavp_interface/srv/failure_info.hpp"
#include "ros2_muavp_interface/srv/path_info.hpp"

#include <map>
#include <iterator>
#include <random>
#include <fstream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using Eigen::Vector3d;
using namespace common;

#define PI 3.14159265
#define g 9.81

class StatesHandler : public rclcpp::Node
{
public:

    StatesHandler(std::string agent_name);
    ~StatesHandler();


    std::ofstream myfile;
    std::ofstream myfile2;
    std::ofstream myfile3;
    std::ofstream myfile4;
    std::ofstream myfile5;
    std::ofstream myfile6;
    std::ofstream myfile7;
    std::ofstream myfile_T;
    std::ofstream myfile_P;


private:

  // rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr _timesync_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odom_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odom_pa_sub;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odom_na_sub;
  rclcpp::Subscription<px4_msgs::msg::ActuatorArmed>::SharedPtr _act_armed_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::StatesInfo>::SharedPtr _states_info_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::ReadySignals>::SharedPtr _ready_params_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::JointForceTorque>::SharedPtr _force_joint_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::ReadySignals>::SharedPtr _ready_move_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::ReadySignals>::SharedPtr _ready_bfr_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::ReadySignals>::SharedPtr _ready_land_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::FailureAction>::SharedPtr _failure_action_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::InputSetpoint>::SharedPtr _setpoint_sub;
  //
  rclcpp::Subscription<ros2_muavp_interface::msg::RigidBodyPose>::SharedPtr _agent_pose_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::RigidBodyPose>::SharedPtr _prev_agent_pose_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::RigidBodyPose>::SharedPtr _next_agent_pose_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::RigidBodyPose>::SharedPtr _payload_pose_sub;

  rclcpp::Subscription<ros2_muavp_interface::msg::ExchangeInfo>::SharedPtr _ei_prev_agent_sub;
  rclcpp::Subscription<ros2_muavp_interface::msg::ExchangeInfo>::SharedPtr _ei_next_agent_sub;

  rclcpp::Service<ros2_muavp_interface::srv::PathInfo>::SharedPtr _path_info_srv;
  rclcpp::Service<ros2_muavp_interface::srv::FailureInfo>::SharedPtr _failure_info_srv;

  //
  rclcpp::Publisher<ros2_muavp_interface::msg::InputSetpoint>::SharedPtr _setpoint_pub;
  rclcpp::Publisher<ros2_muavp_interface::msg::ReadySignals>::SharedPtr _ready_params_pub;
  rclcpp::Publisher<ros2_muavp_interface::msg::ExchangeInfo>::SharedPtr _ei_agent_pub;
  rclcpp::Publisher<ros2_muavp_interface::msg::ReadySignals>::SharedPtr _ready_land_pub;
  rclcpp::Publisher<ros2_muavp_interface::msg::FailureCheck>::SharedPtr _failure_pub;

  
  rclcpp::Subscription<ros2_muavp_interface::msg::FailureCheck>::SharedPtr failure_sub_; //changes here

  ros2_muavp_interface::msg::StatesInfo _parameters;
  ros2_muavp_interface::msg::StatesInfo _states_info_data;
  ros2_muavp_interface::msg::JointForceTorque _joint_data;
  ros2_muavp_interface::msg::ReadySignals _ready_signal_params_data;
  ros2_muavp_interface::msg::ReadySignals _ready_signal_move_data;
  ros2_muavp_interface::msg::ReadySignals _ready_signal_bfr_data;
  ros2_muavp_interface::msg::ReadySignals _ready_signal_land_data;
  ros2_muavp_interface::msg::FailureAction _failure_action_data;
  ros2_muavp_interface::msg::InputSetpoint _setpoint_msg;

  //
  ros2_muavp_interface::msg::RigidBodyPose _agent_pose_data;
  ros2_muavp_interface::msg::RigidBodyPose _agent_pose_home;
  ros2_muavp_interface::msg::RigidBodyPose _agents_sum_pose_data;
  ros2_muavp_interface::msg::RigidBodyPose _prev_agent_pose_data;
  ros2_muavp_interface::msg::RigidBodyPose _next_agent_pose_data;
  ros2_muavp_interface::msg::RigidBodyPose _payload_pose_data;

  ros2_muavp_interface::msg::ExchangeInfo _ei_prev_agent_data;
  ros2_muavp_interface::msg::ExchangeInfo _ei_next_agent_data;

  ros2_muavp_interface::msg::InputSetpoint _msg_stop;
  ros2_muavp_interface::msg::InputSetpoint _msg_smc;
  ros2_muavp_interface::msg::InputSetpoint _msg_idle;
  ros2_muavp_interface::msg::InputSetpoint _msg_land;

  px4_msgs::msg::VehicleOdometry _odom_data;
  px4_msgs::msg::VehicleOdometry _pa_odom_data;
  px4_msgs::msg::VehicleOdometry _na_odom_data;
  px4_msgs::msg::ActuatorArmed _actuator_armed_data;

  // std::map<int, ros2_muavp_interface::msg::StatesInfo> _failure_info_data;

  std::map<int, ros2_muavp_interface::srv::PathInfo::Request> _path_info_data;
  std::map<int, ros2_muavp_interface::srv::FailureInfo::Request> _failure_info_data;

  double _euler_angles[3];
  double _na_euler_angles[3];
  double _pa_euler_angles[3];

  struct {
    VectorXd r_uav = VectorXd(3);
    VectorXd r_l1 = VectorXd(3);
    VectorXd r_l2 = VectorXd(3);
    VectorXd r_l3 = VectorXd(3);
    VectorXd uav_anchor_point = VectorXd(3);
    VectorXd surv_uav_anchor_point = VectorXd(3);
    VectorXd load_anchor_point_1 = VectorXd(3);
    VectorXd load_anchor_point_2 = VectorXd(3);
    VectorXd load_anchor_point_3 = VectorXd(3);
    VectorXd uav_payload_exact_dist = VectorXd(3);
    double dist_norm_exact;
  } _ancPointsData;

  struct {
    VectorXd surv_position = VectorXd(3);
    VectorXd surv_lin_vel = VectorXd(3);
    VectorXd surv_orientation = VectorXd(3);
    int me[3] = {0, 0, 0};
    int surv_agent[3] = {0, 0, 0};
    bool prev_agent_alive = false;
    bool next_agent_alive = false;
  } _detachUAVData;

  struct {
    VectorXd body_tension = VectorXd(3);
    VectorXd inert_tension = VectorXd(3);
    double tension_norm;
  } _tensions;

  struct {
    double e_x = 0.0;
    double e_vx = 0.0;
    double e_y = 0.0;
    double e_vy = 0.0;
    double e_phi = 0.0;
    double e_phi_dot = 0.0;
    double e_theta = 0.0;
    double e_theta_dot = 0.0;
  } _smc_integral;

  struct {
    double alpha;
    double f_ex;
    double T_max;
    double n_alpha;
    double i_alpha;
    double inc_alpha;
    double dist_g_me;
    double dist_g_srvd;
    VectorXd v_1 = VectorXd(3);
    VectorXd v_2 = VectorXd(3);
    VectorXd p_m_correction = VectorXd(3);
    int n;
    int nf;
    int i;
    bool first_gen = true;
    bool des_vel_mean = false;
  } _fc_sp_params;

  double _integral_dist_x = 0.0;
  double _exact_integral_dist_x = 0.0;
  double _integral_dist_y = 0.0;
  double _exact_integral_dist_y = 0.0;
  double _integral_dist_z = 0.0;
  double _exact_integral_dist_z = 0.0;
  double _exact_integral_tension = 0.0;
  double _integral_dist_phi = 0.0;
  double _integral_dist_theta = 0.0;
  double _integral_dist_psi = 0.0;
  double _a;
  double _integralTension = 0.0;
  double _prevTensionError = 0.0;

  double _x_des;
  double _y_des;
  double _z_des;
  double _yaw_des;
  double _thrust;
  double _dist_threshold = 0.0;
  double _takeoff_setpoint_z = 0.0;

  int _counter_Jacobian;
  int _counter_Dynamics;
  int _m = 800;
  int _m2 = 50;
  int _land_counter = 0;

  VectorXd _u_eq = VectorXd(2);

  VectorXd _eq_pt = VectorXd(2);

  VectorXd _des_dist_me_prev = VectorXd(2);

  VectorXd _des_dist_me_next = VectorXd(2);

  VectorXd _des_dist_me_srvd = VectorXd(2);

  VectorXd _desired_velocity = VectorXd(3);

  VectorXd _desired_velocity_fc = VectorXd(3);

  VectorXd _torques_ctrl = VectorXd(3);

  VectorXd _euler_rates = VectorXd(3);

  MatrixXd _I_uav = MatrixXd(3,3);

  VectorXd _versor_fc_objective = VectorXd(3);


  rclcpp::TimerBase::SharedPtr _timer;
  // std::atomic<uint64_t> _timestamp;

  rclcpp::Time _prev;
  rclcpp::Time _curr;
  rclcpp::Clock _clocksis;

  std::string _agent_name = "";
  std::string _prev_agent_name = "";
  std::string _next_agent_name = "";

  bool _first_pre;
  bool _first_take;
  bool _first_move;
  bool _first_move_fc;
  bool _first_idle;
  bool _first_land;
  bool _first_msg;
  bool _first_des_pos;
  bool _failure_occured;
  bool _system_recovered;
  bool _recoverying_already;
  bool _recovery_arrived;
  bool _changed_pid_params;
  bool _first_agent;
  bool _first_stop;
  bool _first_recovery;
  bool _first_failure;
  bool _first_failure_traj;
  bool _first_smc;
  bool _first_smc_pos;
  bool _first_bearing;
  bool _arrived;

  bool _recovery_stop;



  double YawAdjuster(VectorXd formation_center);

  void fixingOdometry(px4_msgs::msg::VehicleOdometry &odom_data, std::string agent_name);

  void storeParameters();
  void storeControlEquilibriumData();
  void handlePreTakeOff();
  void handleTakeOff();
  void handleMove();
  void handleIdle();
  void handleLand();
  void failuresRunner();
  void statesRunner();
  void checkParamsStored();

  // ----------------- Author: Barbara RATTO --------------------------
  double PretakeoffTensionPid(double delta);

  void anchorPointsStructFiller();
  void PIDGeneratorPosition(double delta, bool basic_cond_int, double* pid_control, double pid_sp[3], double z_dot_des);
  void PIDGeneratorOrientation(double delta, bool basic_cond_int, double* angle_sp, double* pid_control);
  void adjustConsensusBasedControl(double *r31_z);
  void stopAndKeepPosition();
  void detachmentControl();
  void torquesControlGenerator(double delta, Vector3d T_I, double u);
  void torquesBodyTransform();
  double bearingAngleComputation();
  VectorXd tensionInertialToBody(double T_x, double T_y, double T_z);
  VectorXd slidingModeController(double theta_disturbance, double y_disturbance, double phi_disturbance, double x_disturbance, double integral_cond, double delta);
  VectorXd positionGeneratorSMC();
  // VectorXd RecoveryFormationRotation(VectorXd uav_pos_me, VectorXd uav_pos_srvd, VectorXd goal_pos);
  void survivedUAVcheck();
  void survivedAgentPoseAssignment();
  void printStatesOnFIle(VectorXd des_state = VectorXd::Zero(12));
  void openFilesForDataStoring();
  VectorXd anchorPointsAndVelComputation();

  void storeDataTensionIdentificationHandler();
  void storeDataTensionIdentification(double residual, VectorXd load_anchor_point_1, VectorXd load_anchor_point_2, VectorXd r_1, VectorXd r_2);
  void tensionRodComputation();
  Vector3d tensionEstimation();
  void storeTensions(double T_x, double T_y, double T_z);

  double norm(VectorXd vector);
  void quaternionToEuler(std::array<float, 4> q, int pos);
};

#endif // STATESHANDLER_HPP
