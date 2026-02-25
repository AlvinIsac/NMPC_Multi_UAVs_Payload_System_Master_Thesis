#include <rclcpp/rclcpp.hpp>
#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <map>
#include <iostream>
#include <iomanip>
#include <chrono>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "ros2_muavp_interface/msg/rigid_body_pose.hpp"
#include "ros2_muavp_interface/msg/input_setpoint.hpp"
#include "ros2_muavp_interface/msg/failure_check.hpp"
#include "ros2_muavp_interface/msg/joint_force_torque.hpp"

using ros2_muavp_interface::msg::RigidBodyPose;
using ros2_muavp_interface::msg::InputSetpoint;
using ros2_muavp_interface::msg::FailureCheck;
using ros2_muavp_interface::msg::JointForceTorque;

constexpr double G = 9.80665;

class NMPCControllerWithTension : public rclcpp::Node {
public:
  NMPCControllerWithTension() : Node("NMPC_controller_with_tension") {
    declare_parameters();
    load_parameters();
    build_mpc_solver();

    // Subscriptions
    sub_failure_ = create_subscription<FailureCheck>(
        "/agents/failure_check", 10,
        std::bind(&NMPCControllerWithTension::failureCheckCb, this, std::placeholders::_1));
    
    sub_uav_pose_3 = create_subscription<RigidBodyPose>(
        "/x500_3/pose", rclcpp::SensorDataQoS(),
        std::bind(&NMPCControllerWithTension::uavPoseCb3, this, std::placeholders::_1));

    sub_uav_pose_2 = create_subscription<RigidBodyPose>(
        "/x500_2/pose", rclcpp::SensorDataQoS(),
        std::bind(&NMPCControllerWithTension::uavPoseCb2, this, std::placeholders::_1));

    sub_payload_ = create_subscription<RigidBodyPose>(
        "/payload/pose", 10,
        std::bind(&NMPCControllerWithTension::payloadCb, this, std::placeholders::_1));

    // Tension force subscriptions - from mpc_main.cpp
    sub_uav3_tension_ = create_subscription<JointForceTorque>(
        "/x500_3/rod_force_torque", 10,
        std::bind(&NMPCControllerWithTension::uav3TensionCb, this, std::placeholders::_1));

    sub_uav2_tension_ = create_subscription<JointForceTorque>(
        "/x500_2/rod_force_torque", 10,
        std::bind(&NMPCControllerWithTension::uav2TensionCb, this, std::placeholders::_1));

    // Publishers
    pub_setpoint_3 = create_publisher<InputSetpoint>("/x500_3/input_setpoint_", 10);
    pub_setpoint_2 = create_publisher<InputSetpoint>("/x500_2/input_setpoint_", 10);

    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),  // 50 Hz (1000 ms / 50 = 20 ms)
      std::bind(&NMPCControllerWithTension::controlLoop, this));

    RCLCPP_INFO(get_logger(),
                "NMPC controller with tension topics for uav2 & uav3 BOOM BOOM!! - dt = %.3f s, horizon = %u",
                dt_, N_);
  }

private:
  // Parameters
  double m_, mP_, Ixx_, Iyy_, Izz_;
  double L_, k_, b_;  // Tether parameters
  double T_MIN_, T_MAX_, TAU_PITCH_MIN_, TAU_PITCH_MAX_, TAU_ROLL_MIN_, TAU_ROLL_MAX_, TAU_YAW_MIN_, TAU_YAW_MAX_;
  double dt_;
  unsigned int N_;
  double dis_between_uav_;

  // State weights 
  double w_payload_vel_, w_payload_center_, w_payload_accel_;
  double w_uav_coordination_, w_formation_constraint_;
  double w_terminal_vel_, w_terminal_center_, w_terminal_uav_vel_;
  double PHI_MAX_, THETA_MAX_, PSI_MAX_;

  // ROS entities
  rclcpp::Subscription<JointForceTorque>::SharedPtr sub_uav3_tension_;
  rclcpp::Subscription<JointForceTorque>::SharedPtr sub_uav2_tension_;
  rclcpp::Subscription<RigidBodyPose>::SharedPtr sub_uav_pose_3;
  rclcpp::Subscription<RigidBodyPose>::SharedPtr sub_uav_pose_2;
  rclcpp::Subscription<FailureCheck>::SharedPtr  sub_failure_;
  rclcpp::Publisher<InputSetpoint>::SharedPtr    pub_setpoint_3;
  rclcpp::Publisher<InputSetpoint>::SharedPtr    pub_setpoint_2;
  rclcpp::Subscription<RigidBodyPose>::SharedPtr sub_payload_;
  rclcpp::TimerBase::SharedPtr                   control_timer_;

  bool   active_{false};
  bool   state_ready_uav2{false};
  bool   state_ready_uav3{false};
  bool   state_ready_payload{false};
  bool   state_ready_uav2_tension{false};
  bool   state_ready_uav3_tension{false};

  double pN_uav2{0.0}, vN_uav2{0.0};
  double pE_uav2{0.0}, vE_uav2{0.0};
  double pD_uav2{0.0}, vD_uav2{0.0};

  double pN_uav3{0.0}, vN_uav3{0.0};
  double pE_uav3{0.0}, vE_uav3{0.0};
  double pD_uav3{0.0}, vD_uav3{0.0};

  double phi_uav2{0.0},  phi_dot_uav2{0.0};
  double theta_uav2{0.0},theta_dot_uav2{0.0};
  double psi_uav2{0.0},  psi_dot_uav2{0.0};

  double phi_uav3{0.0},  phi_dot_uav3{0.0};
  double theta_uav3{0.0},theta_dot_uav3{0.0};
  double psi_uav3{0.0},  psi_dot_uav3{0.0};

  double pN_payload{0.0}, pE_payload{0.0}, pD_payload{0.0};
  double vN_payload{0.0}, vE_payload{0.0}, vD_payload{0.0};

  // Tension forces from topics - from mpc_main.cpp
  double pN_tension_uav2{0.0}, pE_tension_uav2{0.0}, pD_tension_uav2{0.0};
  double pN_tension_uav3{0.0}, pE_tension_uav3{0.0}, pD_tension_uav3{0.0};

  Eigen::Vector4d u_prev_uav2, u_prev_uav3;

  // Debug counter and timing
  int debug_counter_{0};
  std::chrono::high_resolution_clock::time_point last_cycle_time_;
  
  // Pose topic timestamps for debugging
  uint64_t uav2_pose_timestamp_{0};
  uint64_t uav3_pose_timestamp_{0};
  uint64_t payload_pose_timestamp_{0};
  
  // Previous timestamps for difference calculation
  uint64_t prev_uav2_pose_timestamp_{0};
  uint64_t prev_uav3_pose_timestamp_{0};
  uint64_t prev_payload_pose_timestamp_{0};

  // CasADi solver
  casadi::Function solver_;
  casadi::DM lbx_, ubx_, lbg_, ubg_;
  
  // --------------------------------------------------------------------------
  // Parameter handling
  void load_parameters() {
    m_   = get_parameter("m").as_double();
    mP_  = get_parameter("mP").as_double();
    Ixx_ = get_parameter("Ixx").as_double();
    Iyy_ = get_parameter("Iyy").as_double();
    Izz_ = get_parameter("Izz").as_double();

    L_ = get_parameter("L").as_double();
    k_ = get_parameter("k").as_double();
    b_ = get_parameter("b").as_double();

    T_MIN_ = get_parameter("T_MIN").as_double();
    T_MAX_ = get_parameter("T_MAX").as_double();
    TAU_PITCH_MIN_ = get_parameter("TAU_PITCH_MIN").as_double();
    TAU_PITCH_MAX_ = get_parameter("TAU_PITCH_MAX").as_double();
    TAU_ROLL_MIN_ = get_parameter("TAU_ROLL_MIN").as_double();
    TAU_ROLL_MAX_ = get_parameter("TAU_ROLL_MAX").as_double();
    TAU_YAW_MIN_  = get_parameter("TAU_YAW_MIN").as_double();
    TAU_YAW_MAX_  = get_parameter("TAU_YAW_MAX").as_double();

    dt_ = get_parameter("dt").as_double();
    N_  = get_parameter("N").as_int();

    dis_between_uav_ = get_parameter("dis_between_uav").as_double();

    w_payload_vel_ = get_parameter("w_payload_vel").as_double();
    w_payload_center_ = get_parameter("w_payload_center").as_double();
    w_payload_accel_ = get_parameter("w_payload_accel").as_double();
    w_uav_coordination_ = get_parameter("w_uav_coordination").as_double();
    w_formation_constraint_ = get_parameter("w_formation_constraint").as_double();
    w_terminal_vel_ = get_parameter("w_terminal_vel").as_double();
    w_terminal_center_ = get_parameter("w_terminal_center").as_double();
    w_terminal_uav_vel_ = get_parameter("w_terminal_uav_vel").as_double();

    PHI_MAX_ = get_parameter("PHI_MAX").as_double();
    THETA_MAX_ = get_parameter("THETA_MAX").as_double();
    PSI_MAX_ = get_parameter("PSI_MAX").as_double();
  }

  void declare_parameters() {
    // Physical parameters
    declare_parameter("m", 2.0);        // UAV mass
    declare_parameter("mP", 1.0);       // Payload mass
    declare_parameter("Ixx", 0.022);     
    declare_parameter("Iyy", 0.022);     
    declare_parameter("Izz", 0.04);      

    // Tether parameters
    declare_parameter("L", 1.6);         // Tether length
    declare_parameter("k", 80.0);        // Spring constant
    declare_parameter("b", 2.0);         // Damping constant 

    declare_parameter("dt", 0.02);       // MATLAB: dt = 0.01
    declare_parameter("N",  20);         // MATLAB: N = 30

    // Formation parameter - MATLAB: dis_between_uav = 1.2
    declare_parameter("dis_between_uav", 0.8); 

    declare_parameter("T_MIN", 20.0);
    declare_parameter("T_MAX", 33.0);
    declare_parameter("TAU_ROLL_MIN",  -0.3);
    declare_parameter("TAU_ROLL_MAX",   0.3);
    declare_parameter("TAU_PITCH_MIN", -0.3);
    declare_parameter("TAU_PITCH_MAX",  0.3);
    declare_parameter("TAU_YAW_MIN",   -0.1);
    declare_parameter("TAU_YAW_MAX",    0.1);

    declare_parameter("PHI_MAX", M_PI/4);    // 45 degrees
    declare_parameter("THETA_MAX", M_PI/4);  // 45 degrees
    declare_parameter("PSI_MAX", M_PI);       

    // Cost function weights
    declare_parameter("w_payload_vel", 50.0);
    declare_parameter("w_payload_center", 15.0);
    declare_parameter("w_payload_accel", 5.0);
    declare_parameter("w_uav_coordination", 2.0);
    declare_parameter("w_formation_constraint", 1.0);
    declare_parameter("w_terminal_vel", 100.0);
    declare_parameter("w_terminal_center", 30.0);
    declare_parameter("w_terminal_uav_vel", 35.0);
  }

  // --------------------------------------------------------------------------
  // Callback functions for UAV poses
  void uavPoseCb2(const RigidBodyPose::SharedPtr msg) {
    // Extract timestamp for debugging
    prev_uav2_pose_timestamp_ = uav2_pose_timestamp_;
    uav2_pose_timestamp_ = msg->timestamp;
    
    // Position (NED)
    pN_uav2 = msg->position[0];
    pE_uav2 = msg->position[1];
    pD_uav2 = msg->position[2]; 

    vN_uav2 = msg->lin_velocity[0];
    vE_uav2 = msg->lin_velocity[1];
    vD_uav2 = msg->lin_velocity[2];

    phi_uav2   = msg->orientation[0];
    theta_uav2 = msg->orientation[1];
    psi_uav2   = msg->orientation[2];

    phi_dot_uav2   = msg->ang_velocity[0]; // p
    theta_dot_uav2 = msg->ang_velocity[1]; // q
    psi_dot_uav2   = msg->ang_velocity[2]; // r

    state_ready_uav2 = true;
  }

  void uavPoseCb3(const RigidBodyPose::SharedPtr msg) {
    // Extract timestamp for debugging
    prev_uav3_pose_timestamp_ = uav3_pose_timestamp_;
    uav3_pose_timestamp_ = msg->timestamp;
    
    // Position (NED)
    pN_uav3 = msg->position[0];
    pE_uav3 = msg->position[1];
    pD_uav3 = msg->position[2];

    vN_uav3 = msg->lin_velocity[0];
    vE_uav3 = msg->lin_velocity[1];
    vD_uav3 = msg->lin_velocity[2];

    phi_uav3   = msg->orientation[0];
    theta_uav3 = msg->orientation[1];
    psi_uav3   = msg->orientation[2];

    phi_dot_uav3   = msg->ang_velocity[0]; // p
    theta_dot_uav3 = msg->ang_velocity[1]; // q
    psi_dot_uav3   = msg->ang_velocity[2]; // r

    state_ready_uav3 = true;
  }

  void payloadCb(const RigidBodyPose::SharedPtr msg) {
    // Extract timestamp for debugging
    prev_payload_pose_timestamp_ = payload_pose_timestamp_;
    payload_pose_timestamp_ = msg->timestamp;
    
    // Position (NED)
    pN_payload = msg->position[0];
    pE_payload = msg->position[1];
    pD_payload = msg->position[2];

    vN_payload = msg->lin_velocity[0];
    vE_payload = msg->lin_velocity[1];
    vD_payload = msg->lin_velocity[2];

    state_ready_payload = true;
  }

  // --------------------------------------------------------------------------
  // Tension callback functions - from mpc_main.cpp
  void uav2TensionCb(const JointForceTorque::SharedPtr msg) {
    pN_tension_uav2 = msg->force_joint_1[0];
    pE_tension_uav2 = msg->force_joint_1[1];
    pD_tension_uav2 = msg->force_joint_1[2];

    state_ready_uav2_tension = true;
  }

  void uav3TensionCb(const JointForceTorque::SharedPtr msg) {
    pN_tension_uav3 = msg->force_joint_1[0];
    pE_tension_uav3 = msg->force_joint_1[1];
    pD_tension_uav3 = msg->force_joint_1[2];

    state_ready_uav3_tension = true;
  }

  // --------------------------------------------------------------------------
  void failureCheckCb(const FailureCheck::SharedPtr msg) {
    if (!active_ && (msg->uav_1_failed || msg->uav_2_failed || msg->uav_3_failed)) {
      active_ = true;
      RCLCPP_WARN(get_logger(),
                  "UAV-1 failure detected - Activating MPC controller for payload stabilization");
    }
  }

  // --------------------------------------------------------------------------
  // Control loop
  void controlLoop() {
    if (!active_) return;
    if (!state_ready_uav2 || !state_ready_uav3 || !state_ready_payload ||
        !state_ready_uav2_tension || !state_ready_uav3_tension) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for initial state and tension data...");
      return;
    }

    // Pack current state vector [30x1] matching MATLAB
    casadi::DM P = casadi::DM::zeros(36, 1); // Extended to include tension forces

    // UAV1 states (indices 0-11) - using UAV2 data
    P(0)  = pN_uav2;      P(1)  = vN_uav2;
    P(2)  = pE_uav2;      P(3)  = vE_uav2;
    P(4)  = pD_uav2;      P(5)  = vD_uav2;
    P(6)  = phi_uav2;     P(7)  = phi_dot_uav2;
    P(8)  = theta_uav2;   P(9)  = theta_dot_uav2;
    P(10) = psi_uav2;     P(11) = psi_dot_uav2;

    // UAV2 states (indices 12-23) - using UAV3 data
    P(12) = pN_uav3;      P(13) = vN_uav3;
    P(14) = pE_uav3;      P(15) = vE_uav3;
    P(16) = pD_uav3;      P(17) = vD_uav3;
    P(18) = phi_uav3;     P(19) = phi_dot_uav3;
    P(20) = theta_uav3;   P(21) = theta_dot_uav3;
    P(22) = psi_uav3;     P(23) = psi_dot_uav3;

    // Payload states (indices 24-29)
    P(24) = pN_payload;   P(25) = vN_payload;
    P(26) = pE_payload;   P(27) = vE_payload;
    P(28) = pD_payload;   P(29) = vD_payload;

    // Tension forces from topics (indices 30-35)
    P(30) = pN_tension_uav2; P(31) = pE_tension_uav2; P(32) = pD_tension_uav2;
    P(33) = pN_tension_uav3; P(34) = pE_tension_uav3; P(35) = pD_tension_uav3;

    // =====================================================================================
    
    // debug_counter_++;
    // auto cycle_start_time = std::chrono::high_resolution_clock::now();
    
    // // Calculate and display time difference from last cycle
    // if (debug_counter_ > 1) {
    //   auto time_diff = std::chrono::duration_cast<std::chrono::microseconds>(cycle_start_time - last_cycle_time_);
    //   double cycle_diff_ms = time_diff.count() / 1000.0;
    //   std::cout << "MPC CYCLE " << debug_counter_ << " [time diff: " << std::fixed << std::setprecision(2) << cycle_diff_ms << " ms]" << std::endl;
    // } else {
    //   std::cout << "MPC CYCLE " << debug_counter_ << " [FIRST CYCLE]" << std::endl;
    // }
    
    // // Display pose timestamps and differences for debugging data freshness
    // std::cout << "Pose Timestamps - UAV2: " << uav2_pose_timestamp_ 
    //           << ", UAV3: " << uav3_pose_timestamp_ 
    //           << ", Payload: " << payload_pose_timestamp_ << std::endl;
    
    // // Calculate and display timestamp differences (in microseconds)
    // if (debug_counter_ > 1) {
    //   int64_t uav2_ts_diff = static_cast<int64_t>(uav2_pose_timestamp_) - static_cast<int64_t>(prev_uav2_pose_timestamp_);
    //   int64_t uav3_ts_diff = static_cast<int64_t>(uav3_pose_timestamp_) - static_cast<int64_t>(prev_uav3_pose_timestamp_);
    //   int64_t payload_ts_diff = static_cast<int64_t>(payload_pose_timestamp_) - static_cast<int64_t>(prev_payload_pose_timestamp_);
      
    //   std::cout << "Timestamp Diffs (μs) - UAV2: " << uav2_ts_diff 
    //             << ", UAV3: " << uav3_ts_diff 
    //             << ", Payload: " << payload_ts_diff << std::endl;
      
    //   // Convert to milliseconds for easier reading
    //   double uav2_ms_diff = uav2_ts_diff / 1000.0;
    //   double uav3_ms_diff = uav3_ts_diff / 1000.0;
    //   double payload_ms_diff = payload_ts_diff / 1000.0;
      
    //   std::cout << "Timestamp Diffs (ms) - UAV2: " << std::fixed << std::setprecision(2) << uav2_ms_diff 
    //             << ", UAV3: " << uav3_ms_diff 
    //             << ", Payload: " << payload_ms_diff << std::endl << std::endl;
    // }
    
    // last_cycle_time_ = cycle_start_time;
    // std::cout << "UAV2 (UAV1 in solver): Pos=[" << std::fixed << std::setprecision(3) 
    //           << pN_uav2 << ", " << pE_uav2 << ", " << pD_uav2 << "] Vel=[" 
    //           << vN_uav2 << ", " << vE_uav2 << ", " << vD_uav2 << "]" << std::endl;
    // std::cout << "UAV2 Angles: phi=" << phi_uav2 << " theta=" << theta_uav2 << " psi=" << psi_uav2;
    // std::cout << " Rates: p=" << phi_dot_uav2 << " q=" << theta_dot_uav2 << " r=" << psi_dot_uav2 << std::endl;
    // std::cout << "UAV2 Tension: [" << pN_tension_uav2 << ", " << pE_tension_uav2 << ", " << pD_tension_uav2 << "]" << std::endl;

    // std::cout << std::endl;
    
    // std::cout << "UAV3 (UAV2 in solver): Pos=[" << pN_uav3 << ", " << pE_uav3 << ", " << pD_uav3 
    //           << "] Vel=[" << vN_uav3 << ", " << vE_uav3 << ", " << vD_uav3 << "]" << std::endl;
    // std::cout << "UAV3 Angles: phi=" << phi_uav3 << " theta=" << theta_uav3 << " psi=" << psi_uav3 ;
    // std::cout << " Rates: p=" << phi_dot_uav3 << " q=" << theta_dot_uav3 << " r=" << psi_dot_uav3 << std::endl;
    // std::cout << "UAV3 Tension: [" << pN_tension_uav3 << ", " << pE_tension_uav3 << ", " << pD_tension_uav3 << "]" << std::endl;

    // std::cout << std::endl;
    
    // std::cout << "Payload: Pos=[" << pN_payload << ", " << pE_payload << ", " << pD_payload 
    //           << "] Vel=[" << vN_payload << ", " << vE_payload << ", " << vD_payload << "]" << std::endl;

    // =====================================================================================

    // Warm start vector - INITIALIZE WITH CURRENT STATES INSTEAD OF ZEROS
    static casadi::DM w0;
    if (w0.is_empty()) {
      w0 = casadi::DM::zeros(30*(N_+1) + 8*N_);
      
      // Initialize with CURRENT states repeated over horizon
      for (size_t k = 0; k <= N_; ++k) {
        size_t offset = 30 * k;
        // Copy current UAV1 states (UAV2 data)
        w0(offset + 0) = pN_uav2;    w0(offset + 1) = vN_uav2;
        w0(offset + 2) = pE_uav2;    w0(offset + 3) = vE_uav2;
        w0(offset + 4) = pD_uav2;    w0(offset + 5) = vD_uav2;
        w0(offset + 6) = phi_uav2;   w0(offset + 7) = phi_dot_uav2;
        w0(offset + 8) = theta_uav2; w0(offset + 9) = theta_dot_uav2;
        w0(offset + 10) = psi_uav2;  w0(offset + 11) = psi_dot_uav2;
        
        // Copy current UAV2 states (UAV3 data)
        w0(offset + 12) = pN_uav3;   w0(offset + 13) = vN_uav3;
        w0(offset + 14) = pE_uav3;   w0(offset + 15) = vE_uav3;
        w0(offset + 16) = pD_uav3;   w0(offset + 17) = vD_uav3;
        w0(offset + 18) = phi_uav3;  w0(offset + 19) = phi_dot_uav3;
        w0(offset + 20) = theta_uav3; w0(offset + 21) = theta_dot_uav3;
        w0(offset + 22) = psi_uav3;  w0(offset + 23) = psi_dot_uav3;
        
        // Copy current payload states
        w0(offset + 24) = pN_payload; w0(offset + 25) = vN_payload;
        w0(offset + 26) = pE_payload; w0(offset + 27) = vE_payload;
        w0(offset + 28) = pD_payload; w0(offset + 29) = vD_payload;
      }
      
      // Initialize control with hover thrust
      double hover_thrust = (mP_*0.5 + m_) * G;
      size_t control_offset = 30 * (N_ + 1);
      for (size_t k = 0; k < N_; ++k) {
        w0(control_offset + 8*k + 0) = hover_thrust;  // T1
        w0(control_offset + 8*k + 4) = hover_thrust;  // T2
        // Torques remain 0
      }
    }

    std::map<std::string, casadi::DM> arg, res;
    arg["p"]   = P;
    arg["x0"]  = w0;
    arg["lbx"] = lbx_;
    arg["ubx"] = ubx_;
    arg["lbg"] = lbg_;
    arg["ubg"] = ubg_;

    try {
      res = solver_(arg);
      auto stats = solver_.stats();

      casadi::DM w_opt = res.at("x");
      w0 = w_opt;

      // ==================================================

      // std::cout << "=== MPC SOLVER STATUS: " << stats.at("return_status").to_string() << " ===" << std::endl;
      // casadi::DM f_opt = res.at("f");
      // std::cout << "MPC Optimal Cost: " << std::fixed << std::setprecision(6) 
      //           << static_cast<double>(f_opt.scalar()) << std::endl;
      // std::cout << "Solver Iterations: " << static_cast<int>(stats.at("iter_count").to_int()) << std::endl;

      // ==================================================


      size_t state_vars = 30 * (N_ + 1); 
      // UAV1 control (UAV2 in ROS)
      size_t uav1_offset = state_vars + 0; 
      double thrust_uav2     = static_cast<double>(w_opt(uav1_offset + 0).scalar());
      double tau_roll_uav2   = static_cast<double>(w_opt(uav1_offset + 1).scalar());
      double tau_pitch_uav2  = static_cast<double>(w_opt(uav1_offset + 2).scalar());
      double tau_yaw_uav2    = static_cast<double>(w_opt(uav1_offset + 3).scalar());
        
      // UAV2 control (UAV3 in ROS)
      size_t uav2_offset = state_vars + 4; 
      double thrust_uav3     = static_cast<double>(w_opt(uav2_offset + 0).scalar());
      double tau_roll_uav3   = static_cast<double>(w_opt(uav2_offset + 1).scalar());
      double tau_pitch_uav3  = static_cast<double>(w_opt(uav2_offset + 2).scalar());
      double tau_yaw_uav3    = static_cast<double>(w_opt(uav2_offset + 3).scalar());

      // =====================================================================================

      // std::cout << "=== MPC DEBUG: OPTIMAL CONTROL OUTPUTS ===" << std::endl;
      // std::cout << "UAV2 Controls: T=" << std::fixed << std::setprecision(3) << -thrust_uav2 
      //           << " tau_roll=" << tau_roll_uav2 << " tau_pitch=" << tau_pitch_uav2 
      //           << " tau_yaw=" << tau_yaw_uav2 << std::endl;
      // std::cout << "UAV3 Controls: T=" << -thrust_uav3 << " tau_roll=" << tau_roll_uav3 
      //           << " tau_pitch=" << tau_pitch_uav3 << " tau_yaw=" << tau_yaw_uav3 << std::endl;

      // // Calculate and display cycle timing
      // auto cycle_end_time = std::chrono::high_resolution_clock::now();
      // auto cycle_duration = std::chrono::duration_cast<std::chrono::microseconds>(cycle_end_time - cycle_start_time);
      // // std::cout << "=== CYCLE TIMING: " << cycle_duration.count() << " microseconds (" 
      // //           << std::fixed << std::setprecision(2) << cycle_duration.count() / 1000.0 << " ms) ===" << std::endl;

      // std::cout << "============================================" << std::endl;

      //====================================================================================

      InputSetpoint cmd_uav2;
      cmd_uav2.timestamp = 1;
      cmd_uav2.setpoint[0] = static_cast<float>(tau_roll_uav2); // when i try to apply roll the pitch moves + control to negative rotation
      cmd_uav2.setpoint[1] = static_cast<float>(tau_pitch_uav2);
      cmd_uav2.setpoint[2] = static_cast<float>(-thrust_uav2);
      cmd_uav2.setpoint[3] = static_cast<float>(tau_yaw_uav2);
      cmd_uav2.setpoint_type = 2;

      InputSetpoint cmd_uav3;
      cmd_uav3.timestamp = 1;
      cmd_uav3.setpoint[0] = static_cast<float>(tau_roll_uav3);
      cmd_uav3.setpoint[1] = static_cast<float>(tau_pitch_uav3);
      cmd_uav3.setpoint[2] = static_cast<float>(-thrust_uav3);
      cmd_uav3.setpoint[3] = static_cast<float>(tau_yaw_uav3);
      cmd_uav3.setpoint_type = 2;
      
      pub_setpoint_2->publish(cmd_uav2);
      pub_setpoint_3->publish(cmd_uav3);


    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "MPC solver failed: %s", e.what());
    }
  }

  // --------------------------------------------------------------------------
  // Build MPC solver (Modified to use tension forces from topics instead of calculating them)
  void build_mpc_solver() {
    using namespace casadi;

    // Decision variables - 30 states (UAV1 + UAV2 + Payload), 8 controls (4 per UAV)
    SX X = SX::sym("X", 30, N_ + 1);     // State trajectory
    SX U = SX::sym("U",  8, N_);         // Control trajectory
    SX P = SX::sym("P", 36);             // Parameters (current state + tension forces)

    // Objective & constraints
    SX cost = 0;
    std::vector<SX> g;

    // Initial state constraint
    g.push_back(X(Slice(), 0) - P(Slice(0, 30))); // Only use first 30 elements for state

    for (unsigned int k = 0; k < N_; ++k) {
      SX xk = X(Slice(), k);
      SX uk = U(Slice(), k);

      // Extract UAV1 states (indices 0-11)
      SX N1 = xk(0), vN1 = xk(1), E1 = xk(2), vE1 = xk(3), D1 = xk(4), vD1 = xk(5);
      SX phi1 = xk(6), p1 = xk(7), th1 = xk(8), q1 = xk(9), psi1 = xk(10), r1 = xk(11);
      
      // Extract UAV2 states (indices 12-23)
      SX N2 = xk(12), vN2 = xk(13), E2 = xk(14), vE2 = xk(15), D2 = xk(16), vD2 = xk(17);
      SX phi2 = xk(18), p2 = xk(19), th2 = xk(20), q2 = xk(21), psi2 = xk(22), r2 = xk(23);
      
      // Extract Payload states 
      SX Np = xk(24), vNp = xk(25), Ep = xk(26), vEp = xk(27), Dp = xk(28), vDp = xk(29);

      SX T1 = uk(0), tau_phi1 = uk(1), tau_theta1 = uk(2), tau_psi1 = uk(3);
      SX T2 = uk(4), tau_phi2 = uk(5), tau_theta2 = uk(6), tau_psi2 = uk(7);

      // Use tension forces from topics (indices 30-35 in P)
      SX F1[3] = {P(30), P(31), P(32)};  // UAV2 tension forces
      SX F2[3] = {P(33), P(34), P(35)};  // UAV3 tension forces

      // UAV1 translational accelerations (NED, negative thrust in Z)
      SX aN1 = -(T1/m_)*(sin(phi1)*sin(psi1) + cos(phi1)*sin(th1)*cos(psi1)) + F1[0]/m_;
      SX aE1 = -(T1/m_)*(cos(phi1)*sin(th1)*sin(psi1) - sin(phi1)*cos(psi1)) + F1[1]/m_;
      SX aD1 = -(T1/m_)*(cos(phi1)*cos(th1)) + G + F1[2]/m_;

      // UAV2 translational accelerations
      SX aN2 = -(T2/m_)*(sin(phi2)*sin(psi2) + cos(phi2)*sin(th2)*cos(psi2)) + F2[0]/m_;
      SX aE2 = -(T2/m_)*(cos(phi2)*sin(th2)*sin(psi2) - sin(phi2)*cos(psi2)) + F2[1]/m_;
      SX aD2 = -(T2/m_)*(cos(phi2)*cos(th2)) + G + F2[2]/m_;

      // UAV rotational dynamics
      SX p1_dot = (tau_phi1 - (Iyy_ - Izz_)*q1*r1)/Ixx_;
      SX q1_dot = (tau_theta1 - (Izz_ - Ixx_)*p1*r1)/Iyy_;
      SX r1_dot = (tau_psi1 - (Ixx_ - Iyy_)*p1*q1)/Izz_;

      SX p2_dot = (tau_phi2 - (Iyy_ - Izz_)*q2*r2)/Ixx_;
      SX q2_dot = (tau_theta2 - (Izz_ - Ixx_)*p2*r2)/Iyy_;
      SX r2_dot = (tau_psi2 - (Ixx_ - Iyy_)*p2*q2)/Izz_;

      // Integrate body rates
      SX p1_new = p1 + p1_dot*dt_;
      SX q1_new = q1 + q1_dot*dt_;
      SX r1_new = r1 + r1_dot*dt_;
      SX p2_new = p2 + p2_dot*dt_;
      SX q2_new = q2 + q2_dot*dt_;
      SX r2_new = r2 + r2_dot*dt_;

      // Convert body rates to Euler angle rates
      SX cth1 = cos(th1);
      SX cth1_safe = fmax(fabs(cth1), 1e-6) * sign(cth1); 
      SX tth1_safe = sin(th1) / cth1_safe;  // Safe tangent calculation
      SX eul1_dot1 = p1_new + sin(phi1)*tth1_safe*q1_new + cos(phi1)*tth1_safe*r1_new;
      SX eul1_dot2 = cos(phi1)*q1_new - sin(phi1)*r1_new;
      SX eul1_dot3 = sin(phi1)/cth1_safe*q1_new + cos(phi1)/cth1_safe*r1_new;

      SX cth2 = cos(th2);
      SX cth2_safe = fmax(fabs(cth2), 1e-6) * sign(cth2);
      SX tth2_safe = sin(th2) / cth2_safe;  // Safe tangent calculation
      SX eul2_dot1 = p2_new + sin(phi2)*tth2_safe*q2_new + cos(phi2)*tth2_safe*r2_new;
      SX eul2_dot2 = cos(phi2)*q2_new - sin(phi2)*r2_new;
      SX eul2_dot3 = sin(phi2)/cth2_safe*q2_new + cos(phi2)/cth2_safe*r2_new;

      // Integrate Euler angles
      SX phi1_new = phi1 + eul1_dot1*dt_;
      SX th1_new = th1 + eul1_dot2*dt_;
      SX psi1_new = psi1 + eul1_dot3*dt_;
      SX phi2_new = phi2 + eul2_dot1*dt_;
      SX th2_new = th2 + eul2_dot2*dt_;
      SX psi2_new = psi2 + eul2_dot3*dt_;

      // Integrate UAV translations
      SX vN1_new = vN1 + aN1*dt_;
      SX N1_new = N1 + vN1_new*dt_;
      SX vE1_new = vE1 + aE1*dt_;
      SX E1_new = E1 + vE1_new*dt_;
      SX vD1_new = vD1 + aD1*dt_;
      SX D1_new = D1 + vD1_new*dt_;

      SX vN2_new = vN2 + aN2*dt_;
      SX N2_new = N2 + vN2_new*dt_;
      SX vE2_new = vE2 + aE2*dt_;
      SX E2_new = E2 + vE2_new*dt_;
      SX vD2_new = vD2 + aD2*dt_;
      SX D2_new = D2 + vD2_new*dt_;

      // Payload dynamics
      SX aNp = -(F1[0] + F2[0])/mP_;
      SX aEp = -(F1[1] + F2[1])/mP_;
      SX aDp = -(F1[2] + F2[2])/mP_ + G;

      // Integrate payload states
      SX vNp_new = vNp + aNp*dt_;
      SX Np_new = Np + vNp_new*dt_;
      SX vEp_new = vEp + aEp*dt_;
      SX Ep_new = Ep + vEp_new*dt_;
      SX vDp_new = vDp + aDp*dt_;
      SX Dp_new = Dp + vDp_new*dt_;

      // Pack next state
      SX x_next = SX::vertcat({
        N1_new, vN1_new, E1_new, vE1_new, D1_new, vD1_new,
        phi1_new, p1_new, th1_new, q1_new, psi1_new, r1_new,
        N2_new, vN2_new, E2_new, vE2_new, D2_new, vD2_new,
        phi2_new, p2_new, th2_new, q2_new, psi2_new, r2_new,
        Np_new, vNp_new, Ep_new, vEp_new, Dp_new, vDp_new
      });

      g.push_back(X(Slice(), k+1) - x_next); // X(:, k+1) = x_next

      // ------------- Cost function - MATLAB EXACT MATCHING -------------
      // Extract states for cost calculation (MATLAB indexing style)
      SX N1_cost = xk(0), vN1_cost = xk(1), E1_cost = xk(2), vE1_cost = xk(3), vD1_cost = xk(5);
      SX N2_cost = xk(12), vN2_cost = xk(13), E2_cost = xk(14), vE2_cost = xk(15), vD2_cost = xk(17);
      SX Np_cost = xk(24), vNp_cost = xk(25), Ep_cost = xk(26), vEp_cost = xk(27), vDp_cost = xk(29);
      
      SX u1_cost[4] = {uk(0), uk(1), uk(2), uk(3)};  // UAV1 controls
      SX u2_cost[4] = {uk(4), uk(5), uk(6), uk(7)};  // UAV2 controls
      
      // 1. Payload oscillation damping
      SX payload_vel_cost = w_payload_vel_ * (vNp_cost*vNp_cost + vEp_cost*vEp_cost + vDp_cost*vDp_cost);
      
      // 2. Payload centering
      SX payload_center_N = (N1_cost + N2_cost) / 2.0;
      SX payload_center_E = (E1_cost + E2_cost) / 2.0;
      SX payload_center_cost = w_payload_center_ * ((Np_cost - payload_center_N)*(Np_cost - payload_center_N) + 
                                                    (Ep_cost - payload_center_E)*(Ep_cost - payload_center_E));
      
      // 3. Payload acceleration penalty
      SX payload_accel_cost = 0;
      if (k > 0) {
        SX xk_prev = X(Slice(), k-1);
        SX vNp_prev = xk_prev(25), vEp_prev = xk_prev(27), vDp_prev = xk_prev(29);
        SX aNp_calc = (vNp_cost - vNp_prev) / dt_;
        SX aEp_calc = (vEp_cost - vEp_prev) / dt_;
        SX aDp_calc = (vDp_cost - vDp_prev) / dt_;
        payload_accel_cost = w_payload_accel_ * (aNp_calc*aNp_calc + aEp_calc*aEp_calc + aDp_calc*aDp_calc);
      }
      
      // 4. UAV velocity coordination
      SX uav_coord_cost = w_uav_coordination_ * ((vN1_cost - vN2_cost)*(vN1_cost - vN2_cost) + 
                                                 (vE1_cost - vE2_cost)*(vE1_cost - vE2_cost) + 
                                                 (vD1_cost - vD2_cost)*(vD1_cost - vD2_cost));
      
      // 5. UAV formation constraints
      SX dist_N_diff = (N1_cost - N2_cost)*(N1_cost - N2_cost) - dis_between_uav_*dis_between_uav_;
      SX dist_E_diff = (E1_cost - E2_cost)*(E1_cost - E2_cost) - dis_between_uav_*dis_between_uav_;
      SX formation_cost = w_formation_constraint_ * (dist_N_diff*dist_N_diff + dist_E_diff*dist_E_diff);
      
      // 6. Control effort
      SX control_cost = (0.03*u1_cost[0]*u1_cost[0] + 0.015*u1_cost[1]*u1_cost[1] + 0.015*u1_cost[2]*u1_cost[2] + 0.015*u1_cost[3]*u1_cost[3] +
                        0.03*u2_cost[0]*u2_cost[0] + 0.015*u2_cost[1]*u2_cost[1] + 0.015*u2_cost[2]*u2_cost[2] + 0.015*u2_cost[3]*u2_cost[3]);

      cost += payload_vel_cost + payload_center_cost + payload_accel_cost + 
              uav_coord_cost + formation_cost + control_cost;
    }

    // Terminal cost - MATLAB EXACT MATCHING
    SX xT = X(Slice(), N_);
    
    // Payload velocity terminal cost
    SX vNpT = xT(25), vEpT = xT(27), vDpT = xT(29);
    SX payload_vel_final_sq = vNpT*vNpT + vEpT*vEpT + vDpT*vDpT;
    cost += w_terminal_vel_ * payload_vel_final_sq;

    // Payload centering terminal cost
    SX N1T = xT(0), N2T = xT(12), E1T = xT(2), E2T = xT(14);
    SX NpT = xT(24), EpT = xT(26);
    SX payload_center_NT = (N1T + N2T) / 2.0;
    SX payload_center_ET = (E1T + E2T) / 2.0;
    cost += w_terminal_center_ * ((NpT - payload_center_NT)*(NpT - payload_center_NT) + 
                                  (EpT - payload_center_ET)*(EpT - payload_center_ET));

    // UAV velocity terminal cost
    SX vN1T = xT(1), vE1T = xT(3), vD1T = xT(5);
    SX vN2T = xT(13), vE2T = xT(15), vD2T = xT(17);
    cost += w_terminal_uav_vel_ * (vN1T*vN1T + vE1T*vE1T + vD1T*vD1T + 
                                   vN2T*vN2T + vE2T*vE2T + vD2T*vD2T);

    // Flatten decision vector
    SX w = SX::vertcat({SX::reshape(X, 30*(N_+1), 1),  
                        SX::reshape(U,  8*N_,      1)});
    SX g_concat = SX::vertcat(g);

    // NLP problem
    SXDict nlp = {{"x", w}, {"f", cost}, {"g", g_concat}, {"p", P}};

    // Solver options - FAST AND RELIABLE FOR 50Hz
    Dict opts;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = false;                  
    opts["ipopt.sb"] = "yes";                    
    opts["ipopt.max_iter"] = 50;
    opts["ipopt.tol"] = 1e-2;
    opts["ipopt.dual_inf_tol"] = 1e-2;           
    opts["ipopt.constr_viol_tol"] = 1e-1;        
    opts["ipopt.compl_inf_tol"] = 1e-1;          
    
    // Basic warm start
    opts["ipopt.warm_start_init_point"] = "yes"; 
    opts["ipopt.warm_start_bound_push"] = 1e-3;  
    opts["ipopt.warm_start_mult_bound_push"] = 1e-3;
    
    // Basic settings
    opts["ipopt.mu_strategy"] = "adaptive";      
    
    // MULTI-CORE LINEAR SOLVER
    opts["ipopt.linear_solver"] = "mumps";
    opts["ipopt.mumps_mem_percent"] = 1000;

    solver_ = nlpsol("solver", "ipopt", nlp, opts);

    // Variable bounds
    size_t total_vars = 30*(N_+1) + 8*N_;
    lbx_ = DM::zeros(total_vars);
    ubx_ = DM::zeros(total_vars);

    // Default: unbounded states
    for (size_t i = 0; i < total_vars; ++i) {
      lbx_(i) = -DM::inf();
      ubx_(i) =  DM::inf();
    }

    // Angle constraints for both UAVs
    for (size_t k = 0; k <= N_; ++k) {
      // UAV1 angle limits (indices 6, 8, 10)
      lbx_(6  + 30*k) = -PHI_MAX_;   ubx_(6  + 30*k) = PHI_MAX_;    
      lbx_(8  + 30*k) = -THETA_MAX_; ubx_(8  + 30*k) = THETA_MAX_;  
      
      // UAV2 angle limits (indices 18, 20, 22)
      lbx_(18 + 30*k) = -PHI_MAX_;   ubx_(18 + 30*k) = PHI_MAX_;    
      lbx_(20 + 30*k) = -THETA_MAX_; ubx_(20 + 30*k) = THETA_MAX_;  
    }

    // Control bounds
    for (size_t k = 0; k < N_; ++k) {
      size_t idx = 30*(N_+1) + 8*k; 
      
      // UAV1 controls
      lbx_(idx + 0) = T_MIN_;          ubx_(idx + 0) = T_MAX_;          
      lbx_(idx + 1) = TAU_ROLL_MIN_;   ubx_(idx + 1) = TAU_ROLL_MAX_;   
      lbx_(idx + 2) = TAU_PITCH_MIN_;  ubx_(idx + 2) = TAU_PITCH_MAX_;  
      lbx_(idx + 3) = TAU_YAW_MIN_;    ubx_(idx + 3) = TAU_YAW_MAX_;    
      
      // UAV2 controls
      lbx_(idx + 4) = T_MIN_;          ubx_(idx + 4) = T_MAX_;          
      lbx_(idx + 5) = TAU_ROLL_MIN_;   ubx_(idx + 5) = TAU_ROLL_MAX_;   
      lbx_(idx + 6) = TAU_PITCH_MIN_;  ubx_(idx + 6) = TAU_PITCH_MAX_;  
      lbx_(idx + 7) = TAU_YAW_MIN_;    ubx_(idx + 7) = TAU_YAW_MAX_;    
    }

    lbg_ = DM::zeros(30*(N_+1), 1); 
    ubg_ = DM::zeros(30*(N_+1), 1);
  }
};

// ============================================================================
// Main
// ============================================================================
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<NMPCControllerWithTension>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}