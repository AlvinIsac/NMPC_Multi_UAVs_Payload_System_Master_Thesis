#include <rclcpp/rclcpp.hpp>
#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <map>
#include <iostream>

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

class MpcPositionController : public rclcpp::Node {
public:
  MpcPositionController() : Node("uav_position_mpc_controller") {
    declare_parameters();
    load_parameters();
    init_prev_u();
    build_mpc_solver();

    // Subscriptions
    sub_failure_ = create_subscription<FailureCheck>(
        "/agents/failure_check", 10,
        std::bind(&MpcPositionController::failureCheckCb, this, std::placeholders::_1));
    
    sub_uav_pose_3 = create_subscription<RigidBodyPose>(
        "/x500_3/pose", rclcpp::SensorDataQoS(),
        std::bind(&MpcPositionController::uavPoseCb3, this, std::placeholders::_1));

    sub_uav_pose_2 = create_subscription<RigidBodyPose>(
        "/x500_2/pose", rclcpp::SensorDataQoS(),
        std::bind(&MpcPositionController::uavPoseCb2, this, std::placeholders::_1));

    sub_payload_ = create_subscription<RigidBodyPose>(
        "/payload/pose", 10,
        std::bind(&MpcPositionController::payloadCb, this, std::placeholders::_1));

    sub_uav3_tension_ = create_subscription<JointForceTorque>(
        "/x500_3/rod_force_torque", 10,
        std::bind(&MpcPositionController::uav3TensionCb, this, std::placeholders::_1));

    sub_uav2_tension_ = create_subscription<JointForceTorque>(
        "/x500_2/rod_force_torque", 10,
        std::bind(&MpcPositionController::uav2TensionCb, this, std::placeholders::_1));

    // Publishers
    pub_setpoint_3 = create_publisher<InputSetpoint>("/x500_3/input_setpoint_", 10);
    pub_setpoint_2 = create_publisher<InputSetpoint>("/x500_2/input_setpoint_", 10);

    control_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&MpcPositionController::controlLoop, this));

    RCLCPP_INFO(get_logger(),
                "MPC controller for uav2 & uav3 BOOM BOOM!! - dt = %.3f s, horizon = %u",
                dt_, N_);
  }

private:
  // Parameters
  double M_, Ixx_, Iyy_, Izz_;
  double T_MIN_, T_MAX_, TAU_PITCH_MIN_, TAU_PITCH_MAX_, TAU_ROLL_MIN_, TAU_ROLL_MAX_, TAU_YAW_MIN_, TAU_YAW_MAX_;
  double dt_;
  unsigned int N_;

  // State weights
  double w_pn_, w_vn_, w_pe_, w_ve_, w_pd_, w_vd_;
  double w_phi_, w_phi_dot_, w_theta_, w_theta_dot_, w_psi_, w_psi_dot_;
  double w_u_t_, w_u_tau_pitch_, w_u_tau_roll_, w_u_tau_yaw_;
  double w_du_t_, w_du_tau_pitch_, w_du_tau_roll_, w_du_tau_yaw_;
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

  double pN_des_uav2{0.0}, pE_des_uav2{0.0}, pD_des_uav2{0.0};
  double pN_des_uav3{0.0}, pE_des_uav3{0.0}, pD_des_uav3{0.0};

  double pN_payload{0.0}, pE_payload{0.0}, pD_payload{0.0};
  double vN_payload{0.0}, vE_payload{0.0}, vD_payload{0.0};

  double pN_tension_uav2{0.0}, pE_tension_uav2{0.0}, pD_tension_uav2{0.0};
  double pN_tension_uav3{0.0}, pE_tension_uav3{0.0}, pD_tension_uav3{0.0};

  double pN_uav2_des_{0.0}, pE_uav2_des_{0.0}, pD_uav2_des_{0.0};
  double pN_uav3_des_{0.0}, pE_uav3_des_{0.0}, pD_uav3_des_{0.0};

  double velocity_threshold_{0.5};  
  double position_threshold_{0.8};  
  double custom_vel_e_{0.0};        

  double vel_e_target = 0.0;

  Eigen::Vector4d u_prev_uav2, u_prev_uav3;

  // CasADi solver
  casadi::Function solver_;
  casadi::DM lbx_, ubx_, lbg_, ubg_;
  // --------------------------------------------------------------------------
  // Parameter handling
    void load_parameters() {
    M_   = get_parameter("M").as_double();
    Ixx_ = get_parameter("Ixx").as_double();
    Iyy_ = get_parameter("Iyy").as_double();
    Izz_ = get_parameter("Izz").as_double();

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

    w_pn_ = get_parameter("w_pn").as_double();
    w_vn_ = get_parameter("w_vn").as_double();
    w_pe_ = get_parameter("w_pe").as_double();
    w_ve_ = get_parameter("w_ve").as_double();
    w_pd_ = get_parameter("w_pd").as_double();
    w_vd_ = get_parameter("w_vd").as_double();

    w_phi_       = get_parameter("w_phi").as_double();
    w_phi_dot_   = get_parameter("w_phi_dot").as_double();
    w_theta_     = get_parameter("w_theta").as_double();
    w_theta_dot_ = get_parameter("w_theta_dot").as_double();
    w_psi_       = get_parameter("w_psi").as_double();
    w_psi_dot_   = get_parameter("w_psi_dot").as_double();

    w_u_t_         = get_parameter("w_u_t").as_double();
    w_u_tau_pitch_ = get_parameter("w_u_tau_pitch").as_double();
    w_u_tau_roll_  = get_parameter("w_u_tau_roll").as_double();
    w_u_tau_yaw_   = get_parameter("w_u_tau_yaw").as_double();

    w_du_t_         = get_parameter("w_du_t").as_double();
    w_du_tau_pitch_ = get_parameter("w_du_tau_pitch").as_double();
    w_du_tau_roll_  = get_parameter("w_du_tau_roll").as_double();
    w_du_tau_yaw_   = get_parameter("w_du_tau_yaw").as_double();

    PHI_MAX_ = get_parameter("PHI_MAX").as_double();
    THETA_MAX_ = get_parameter("THETA_MAX").as_double();
    PSI_MAX_ = get_parameter("PSI_MAX").as_double();
  }

  void declare_parameters() {
    // Rigid body parameters
    declare_parameter("M", 2.0);        
    declare_parameter("Ixx", 0.022);     
    declare_parameter("Iyy", 0.022);     
    declare_parameter("Izz", 0.04);      

        // MPC grid
    declare_parameter("dt", 0.02);       // 50 Hz
    declare_parameter("N",  10);         // horizon

    // Control limits
    declare_parameter("T_MIN", -34.0);
    declare_parameter("T_MAX", 34.0);
    declare_parameter("TAU_ROLL_MIN",  -0.15);
    declare_parameter("TAU_ROLL_MAX",   0.15);
    declare_parameter("TAU_PITCH_MIN", -0.15);
    declare_parameter("TAU_PITCH_MAX",  0.15);
    declare_parameter("TAU_YAW_MIN",   -0.015);
    declare_parameter("TAU_YAW_MAX",    0.015);

    declare_parameter("PHI_MAX", M_PI/4);    
    declare_parameter("THETA_MAX", M_PI/4);  
    declare_parameter("PSI_MAX", M_PI);       

declare_parameter("w_pn", 3.0);    
declare_parameter("w_vn", 0.0);

declare_parameter("w_pe", 0.0);    
declare_parameter("w_ve", 0.0);

declare_parameter("w_pd", 3.0);    
declare_parameter("w_vd", 0.0);    

declare_parameter("w_phi",       6.0);   
declare_parameter("w_phi_dot",   1.5);   

declare_parameter("w_theta",     6.0);   
declare_parameter("w_theta_dot", 1.5);   

declare_parameter("w_psi",       0.2);    
declare_parameter("w_psi_dot",   0.1);    

// Control weights
declare_parameter("w_u_t",          0.2);   
declare_parameter("w_u_tau_roll",   0.1);  
declare_parameter("w_u_tau_pitch",  0.1);  
declare_parameter("w_u_tau_yaw",    0.1);   

// Control-rate changes
declare_parameter("w_du_t",          0.0);
declare_parameter("w_du_tau_roll",   0.0); 
declare_parameter("w_du_tau_pitch",  0.0);
declare_parameter("w_du_tau_yaw",    0.0);

  }

  void init_prev_u() {
    u_prev_uav2 << -25.0, 0.0, 0.0, 0.0;
    u_prev_uav3 << -25.0, 0.0, 0.0, 0.0;
  }

  // --------------------------------------------------------------------------
  void uavPoseCb2(const RigidBodyPose::SharedPtr msg) {
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
    // Position (NED)
    pN_payload = msg->position[0];
    pE_payload = msg->position[1];
    pD_payload = msg->position[2];

    vN_payload = msg->lin_velocity[0];
    vE_payload = msg->lin_velocity[1];
    vD_payload = msg->lin_velocity[2];

    state_ready_payload = true;
  }
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
      //uav 2 position at the time of failure
      pN_uav2_des_ = pN_uav2;
      pE_uav2_des_ = pE_uav2;
      pD_uav2_des_ = pD_uav2;

      //uav 3 position at the time of failure
      pN_uav3_des_ = pN_uav3;
      pE_uav3_des_ = pE_uav3;
      pD_uav3_des_ = pD_uav3;
      RCLCPP_WARN(get_logger(),
                  "UAV-1 failure detected - Activating MPC controller  = (%.2f, %.2f, %.2f, %.2f, %.2f, %.2f) for UAV-2 and UAV-3",
                  pN_uav2_des_, pE_uav2_des_, pD_uav2_des_, pN_uav3_des_, pE_uav3_des_, pD_uav3_des_);
    }
  }

  // --------------------------------------------------------------------------
  // Control loop
  void controlLoop() {
    if (!active_) return;
    if (!state_ready_uav2 || !state_ready_uav3 || !state_ready_payload ||
        !state_ready_uav2_tension || !state_ready_uav3_tension) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for initial state...");
      return;
    }

    double dist_e_uav2_payload = std::abs(pE_uav2 - pE_payload);
    double dist_e_uav3_payload = std::abs(pE_uav3 - pE_payload);

    if (dist_e_uav2_payload > position_threshold_ || dist_e_uav3_payload > position_threshold_) {
            vel_e_target = vE_payload;
    } else {
      vel_e_target = vE_payload;
    }
    double yaw_error_uav2 = psi_uav2 - (-1.57);
    double yaw_error_uav3 = psi_uav3 - (-1.57);
    double yaw_cost = w_psi_ * (pow(yaw_error_uav2, 2) + pow(yaw_error_uav3, 2));
    
    double vel_n_cost = w_vn_ * (pow(vN_uav2, 2) + pow(vN_uav3, 2));
    double vel_e_cost = w_ve_ * (pow(vE_uav2 - vel_e_target, 2) + pow(vE_uav3 - vel_e_target, 2));
    double vel_d_cost = w_vd_ * (pow(vD_uav2, 2) + pow(vD_uav3, 2));
    
    double pos_n_cost = w_pn_ * (pow(pN_uav2 - pN_uav2_des_, 2) + pow(pN_uav3 - pN_uav3_des_, 2));
    double pos_d_cost = w_pd_ * (pow(pD_uav2 - pD_uav2_des_, 2) + pow(pD_uav3 - pD_uav3_des_, 2));

    double u_t_cost = w_u_t_ * (pow(u_prev_uav2(0), 2) + pow(u_prev_uav3(0), 2));
    double u_roll_cost = w_u_tau_roll_ * (pow(u_prev_uav2(1), 2) + pow(u_prev_uav3(1), 2));
    double u_pitch_cost = w_u_tau_pitch_ * (pow(u_prev_uav2(2), 2) + pow(u_prev_uav3(2), 2));
    double u_yaw_cost = w_u_tau_yaw_ * (pow(u_prev_uav2(3), 2) + pow(u_prev_uav3(3), 2));
    double control_cost = u_t_cost + u_roll_cost + u_pitch_cost + u_yaw_cost;
    
    double initial_cost = yaw_cost + vel_n_cost + vel_e_cost + vel_d_cost + pos_n_cost + pos_d_cost + control_cost;
    
    std::cout << "cost: " << std::fixed << std::setprecision(4) << initial_cost << std::endl;
    std::cout << ""
          << "Yaw=" << std::fixed << std::setprecision(4) << yaw_cost 
          << ", VelN=" << vel_n_cost
          << ", VelE=" << vel_e_cost
          << ", VelD=" << vel_d_cost
          << ", PosN=" << pos_n_cost
          << ", PosD=" << pos_d_cost
          << ", Control=" << control_cost
          << std::endl;

    casadi::DM P = casadi::DM::zeros(44, 1);  // 44 elements total

    // Current states for UAV-2
    P(0)  = pN_uav2;      P(1)  = vN_uav2;
    P(2)  = pE_uav2;      P(3)  = vE_uav2;
    P(4)  = pD_uav2;      P(5)  = vD_uav2;
    P(6)  = phi_uav2;     P(7)  = phi_dot_uav2;
    P(8)  = theta_uav2;   P(9)  = theta_dot_uav2;
    P(10) = psi_uav2;     P(11) = psi_dot_uav2;

    // Current states for UAV-3
    P(12) = pN_uav3;      P(13) = vN_uav3;
    P(14) = pE_uav3;      P(15) = vE_uav3;
    P(16) = pD_uav3;      P(17) = vD_uav3;
    P(18) = phi_uav3;     P(19) = phi_dot_uav3;
    P(20) = theta_uav3;   P(21) = theta_dot_uav3;
    P(22) = psi_uav3;     P(23) = psi_dot_uav3;

    P(24) = pN_uav2_des_; P(25) = pD_uav2_des_; 
    P(26) = pN_uav3_des_; P(27) = pD_uav3_des_; 
    P(28) = vel_e_target; P(29) = vel_e_target;

    P(30) = pN_tension_uav2; P(31) = pE_tension_uav2; P(32) = pD_tension_uav2;
    P(33) = pN_tension_uav3; P(34) = pE_tension_uav3; P(35) = pD_tension_uav3;
    
    // Previous control for UAV-2 
    P(36) = u_prev_uav2(0);
    P(37) = u_prev_uav2(1);
    P(38) = u_prev_uav2(2);
    P(39) = u_prev_uav2(3);

    // Previous control for UAV-3 
    P(40) = u_prev_uav3(0);
    P(41) = u_prev_uav3(1);
    P(42) = u_prev_uav3(2);
    P(43) = u_prev_uav3(3);

    // Warm start vector
    static casadi::DM w0;
    if (w0.is_empty()) {
      w0 = casadi::DM::zeros(24*(N_+1) + 8*N_);
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
        // print the status for the debug
      auto stats = solver_.stats();
      std::cout << stats.at("return_status").to_string() << std::endl;

      casadi::DM w_opt = res.at("x");
      w0 = w_opt;

      size_t state_vars = 24 * (N_ + 1); 

      // UAV2 control
      size_t uav2_offset = state_vars + 0; 
      double thrust_uav2     = static_cast<double>(w_opt(uav2_offset + 0).scalar());
      double tau_roll_uav2   = static_cast<double>(w_opt(uav2_offset + 1).scalar());
      double tau_pitch_uav2  = static_cast<double>(w_opt(uav2_offset + 2).scalar());
      double tau_yaw_uav2    = static_cast<double>(w_opt(uav2_offset + 3).scalar());
        
      // UAV3 control
      size_t uav3_offset = state_vars + 4; 
      double thrust_uav3     = static_cast<double>(w_opt(uav3_offset + 0).scalar());
      double tau_roll_uav3   = static_cast<double>(w_opt(uav3_offset + 1).scalar());
      double tau_pitch_uav3  = static_cast<double>(w_opt(uav3_offset + 2).scalar());
      double tau_yaw_uav3    = static_cast<double>(w_opt(uav3_offset + 3).scalar());

      InputSetpoint cmd_uav2;
      cmd_uav2.timestamp = 1;
      cmd_uav2.setpoint[0] = static_cast<float>(tau_roll_uav2);
      cmd_uav2.setpoint[1] = static_cast<float>(tau_pitch_uav2);
      cmd_uav2.setpoint[2] = static_cast<float>(thrust_uav2);
      cmd_uav2.setpoint[3] = static_cast<float>(tau_yaw_uav2);
      cmd_uav2.setpoint_type = 2;

      InputSetpoint cmd_uav3;
      cmd_uav3.timestamp = 1;
      cmd_uav3.setpoint[0] = static_cast<float>(tau_roll_uav3);
      cmd_uav3.setpoint[1] = static_cast<float>(tau_pitch_uav3);
      cmd_uav3.setpoint[2] = static_cast<float>(thrust_uav3);
      cmd_uav3.setpoint[3] = static_cast<float>(tau_yaw_uav3);
      cmd_uav3.setpoint_type = 2;
      
      pub_setpoint_3->publish(cmd_uav3);
      pub_setpoint_2->publish(cmd_uav2);

      // Update previous control for next Δu cost
      u_prev_uav2 << thrust_uav2, tau_roll_uav2, tau_pitch_uav2, tau_yaw_uav2;
      u_prev_uav3 << thrust_uav3, tau_roll_uav3, tau_pitch_uav3, tau_yaw_uav3;

      // Debug output
      // std::cout << "[MPC UAV2] T = " << thrust_uav2 << " | τφ = " << tau_roll_uav2 
      //           << " | τθ = " << tau_pitch_uav2 << " | τψ = " << tau_yaw_uav2 << std::endl;
      // std::cout << "[MPC UAV3] T = " << thrust_uav3 << " | τφ = " << tau_roll_uav3
      //           << " | τθ = " << tau_pitch_uav3 << " | τψ = " << tau_yaw_uav3 << std::endl;

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "MPC solver failed: %s", e.what());
    }
  }

  // --------------------------------------------------------------------------
  // Build MPC solver (NED dynamics with yaw coupling + Δu cost)
  void build_mpc_solver() {
    using namespace casadi;

    // Decision variables
    SX X = SX::sym("X", 24, N_ + 1);     // State trajectory
    SX U = SX::sym("U",  8, N_);         // Control trajectory
    SX P = SX::sym("P", 44);           

    // Objective & constraints
    SX cost = 0;
    std::vector<SX> g;

    // Initial state constraint
    g.push_back(X(Slice(), 0) - P(Slice(0, 24)));

    for (unsigned int k = 0; k < N_; ++k) {
      SX xk = X(Slice(), k);
      SX uk = U(Slice(), k);

      // States for uav2
      SX pN_uav2 = xk(0), vN_uav2 = xk(1);
      SX pE_uav2 = xk(2), vE_uav2 = xk(3);
      SX pD_uav2 = xk(4), vD_uav2 = xk(5);

      SX phi_uav2 = xk(6),  phi_dot_uav2 = xk(7);
      SX th_uav2  = xk(8),  th_dot_uav2  = xk(9);
      SX psi_uav2 = xk(10), psi_dot_uav2 = xk(11);
      
      // States for uav3
      SX pN_uav3 = xk(12), vN_uav3 = xk(13);
      SX pE_uav3 = xk(14), vE_uav3 = xk(15);
      SX pD_uav3 = xk(16), vD_uav3 = xk(17);

      SX phi_uav3 = xk(18),  phi_dot_uav3 = xk(19);
      SX th_uav3  = xk(20),  th_dot_uav3  = xk(21);
      SX psi_uav3 = xk(22), psi_dot_uav3 = xk(23);

      // Controls uav2
      SX T_uav2      = uk(0);
      SX tau_ph_uav2 = uk(1);
      SX tau_th_uav2 = uk(2);
      SX tau_ps_uav2 = uk(3);

      // Controls uav3
      SX T_uav3      = uk(4);
      SX tau_ph_uav3 = uk(5);
      SX tau_th_uav3 = uk(6);
      SX tau_ps_uav3 = uk(7);

      SX tension_N_uav2 = P(30);
      SX tension_E_uav2 = P(31);
      SX tension_D_uav2 = P(32);
      SX tension_N_uav3 = P(33);
      SX tension_E_uav3 = P(34);
      SX tension_D_uav3 = P(35);

      SX aN_uav2 = (T_uav2 / M_) * ( sin(phi_uav2) * sin(psi_uav2) + cos(phi_uav2) * sin(th_uav2) * cos(psi_uav2) ) + tension_N_uav2/M_;
      SX aE_uav2 = (T_uav2 / M_) * ( cos(phi_uav2) * sin(th_uav2) * sin(psi_uav2) - sin(phi_uav2) * cos(psi_uav2) ) + tension_E_uav2/M_;
      SX aD_uav2 = - (T_uav2 / M_) * ( cos(phi_uav2) * cos(th_uav2) ) + G + tension_D_uav2/M_;

      SX phi_ddot_uav2 = tau_ph_uav2/ Ixx_;
      SX th_ddot_uav2  = tau_th_uav2 / Iyy_;
      SX psi_ddot_uav2 = tau_ps_uav2 / Izz_;

      SX aN_uav3 = (T_uav3 / M_) * ( sin(phi_uav3) * sin(psi_uav3) + cos(phi_uav3) * sin(th_uav3) * cos(psi_uav3) ) + tension_N_uav3/M_;
      SX aE_uav3 = (T_uav3 / M_) * ( cos(phi_uav3) * sin(th_uav3) * sin(psi_uav3) - sin(phi_uav3) * cos(psi_uav3) ) + tension_E_uav3/M_;
      SX aD_uav3 = - (T_uav3 / M_) * ( cos(phi_uav3) * cos(th_uav3) ) + G + tension_D_uav3/M_;

      SX phi_ddot_uav3 = tau_ph_uav3 / Ixx_;
      SX th_ddot_uav3  = tau_th_uav3 / Iyy_;
      SX psi_ddot_uav3 = tau_ps_uav3 / Izz_;

      // State derivatives
      SX xdot = SX::vertcat({
        vN_uav2,        aN_uav2,
        vE_uav2,        aE_uav2,
        vD_uav2,        aD_uav2,
        phi_dot_uav2,  phi_ddot_uav2,
        th_dot_uav2,   th_ddot_uav2,
        psi_dot_uav2,  psi_ddot_uav2,
        
        vN_uav3,       aN_uav3,
        vE_uav3,       aE_uav3,
        vD_uav3,       aD_uav3,
        phi_dot_uav3,  phi_ddot_uav3,
        th_dot_uav3,   th_ddot_uav3,
        psi_dot_uav3,  psi_ddot_uav3
      });

      // Forward Euler integration
      SX x_next = xk + dt_ * xdot;
      g.push_back(X(Slice(), k+1) - x_next);

      // ------------- Cost terms -------------
      SX pos_cost =
          w_pn_ * pow(pN_uav2 - P(24), 2) + 
          w_pd_ * pow(pD_uav2 - P(25), 2) +
          w_pn_ * pow(pN_uav3 - P(26), 2) +
          w_pd_ * pow(pD_uav3 - P(27), 2);

      // Velocity damping
      SX vel_cost =
          w_vn_ * pow(vN_uav2, 2) +
          w_ve_ * pow(vE_uav2 - P(28), 2) +
          w_vd_ * pow(vD_uav2, 2) +

          w_vn_ * pow(vN_uav3, 2) +
          w_ve_ * pow(vE_uav3 - P(29), 2) +  
          w_vd_ * pow(vD_uav3, 2);

      // Attitude stabilization (to zero)
      SX att_cost_uav2 =
          // w_phi_       * pow(phi_uav2, 2)      + w_phi_dot_   * pow(phi_dot_uav2, 2) +
          // w_theta_     * pow(th_uav2,  2)      + w_theta_dot_ * pow(th_dot_uav2,  2) +
          w_psi_       * pow(psi_uav2 - (-1.57), 2)      + w_psi_dot_   * pow(psi_dot_uav2, 2);

      SX att_cost_uav3 =
          // w_phi_       * pow(phi_uav3, 2)      + w_phi_dot_   * pow(phi_dot_uav3, 2) +
          // w_theta_     * pow(th_uav3,  2)      + w_theta_dot_ * pow(th_dot_uav3,  2) +
          w_psi_       * pow(psi_uav3 - (-1.57), 2)      + w_psi_dot_   * pow(psi_dot_uav3, 2);

      SX att_cost = att_cost_uav2 + att_cost_uav3;

      // Control effort
      SX u_cost_uav2 =
          w_u_t_         * pow(T_uav2,      2) +
          w_u_tau_roll_  * pow(tau_ph_uav2, 2) +
          w_u_tau_pitch_ * pow(tau_th_uav2, 2) +
          w_u_tau_yaw_   * pow(tau_ps_uav2, 2);

      SX u_cost_uav3 =
          w_u_t_         * pow(T_uav3,      2) +
          w_u_tau_roll_  * pow(tau_ph_uav3, 2) +
          w_u_tau_pitch_ * pow(tau_th_uav3, 2) +
          w_u_tau_yaw_   * pow(tau_ps_uav3, 2);

      SX u_cost = u_cost_uav2 + u_cost_uav3;

      // UAV2
      // SX du_uav2_0, du_uav2_1, du_uav2_2, du_uav2_3;
      // if (k == 0) {
      //     du_uav2_0 = uk(0) - P(36);  // Correct indices
      //     du_uav2_1 = uk(1) - P(37);
      //     du_uav2_2 = uk(2) - P(38);
      //     du_uav2_3 = uk(3) - P(39);
      // } else {
      //     du_uav2_0 = uk(0) - U(0, k-1);
      //     du_uav2_1 = uk(1) - U(1, k-1);
      //     du_uav2_2 = uk(2) - U(2, k-1);
      //     du_uav2_3 = uk(3) - U(3, k-1);
      // }

      // UAV3
      // SX du_uav3_0, du_uav3_1, du_uav3_2, du_uav3_3;
      // if (k == 0) {
      //     du_uav3_0 = uk(4) - P(40);  // Correct indices
      //     du_uav3_1 = uk(5) - P(41);
      //     du_uav3_2 = uk(6) - P(42);
      //     du_uav3_3 = uk(7) - P(43);
      // } else {
      //     du_uav3_0 = uk(4) - U(4, k-1);
      //     du_uav3_1 = uk(5) - U(5, k-1);
      //     du_uav3_2 = uk(6) - U(6, k-1);
      //     du_uav3_3 = uk(7) - U(7, k-1);
      // }

      // SX du_cost_uav2 =
      //     w_du_t_         * pow(du_uav2_0, 2) +
      //     w_du_tau_roll_  * pow(du_uav2_1, 2) +
      //     w_du_tau_pitch_ * pow(du_uav2_2, 2) +
      //     w_du_tau_yaw_   * pow(du_uav2_3, 2);

      // SX du_cost_uav3 =
      //     w_du_t_         * pow(du_uav3_0, 2) +
      //     w_du_tau_roll_  * pow(du_uav3_1, 2) +
      //     w_du_tau_pitch_ * pow(du_uav3_2, 2) +
      //     w_du_tau_yaw_   * pow(du_uav3_3, 2);

      // SX du_cost = du_cost_uav2 + du_cost_uav3;

      cost += pos_cost + vel_cost + att_cost;  //+ u_cost + du_cost;
    }

    // Flatten decision vector
    SX w = SX::vertcat({SX::reshape(X, 24*(N_+1), 1),  
                        SX::reshape(U,  8*N_,      1)});
    SX g_concat = SX::vertcat(g);

    // NLP problem
    SXDict nlp = {{"x", w}, {"f", cost}, {"g", g_concat}, {"p", P}};

    // Solver options
    Dict opts;
    opts["ipopt.print_level"] = 0;  // turn of this if dont want log
    opts["print_time"] = 0;         // Enable timing information
    opts["ipopt.sb"] = "yes";
    opts["ipopt.max_iter"] = 50;
    // opts["ipopt.file_print_level"] = 12;
    // opts["ipopt.output_file"] = "ipopt_log.txt"; // comment if you dont want to save file

    solver_ = nlpsol("solver", "ipopt", nlp, opts);

    // Variable bounds
    size_t total_vars = 24*(N_+1) + 8*N_;
    lbx_ = DM::zeros(total_vars);
    ubx_ = DM::zeros(total_vars);

    // Default: unbounded states
    for (size_t i = 0; i < total_vars; ++i) {
      lbx_(i) = -DM::inf();
      ubx_(i) =  DM::inf();
    }

    for (size_t k = 0; k <= N_; ++k) {
      // UAV2
      lbx_(6  + 24*k) = -PHI_MAX_;   ubx_(6  + 24*k) = PHI_MAX_;    
      lbx_(8  + 24*k) = -THETA_MAX_; ubx_(8  + 24*k) = THETA_MAX_;  
      lbx_(10 + 24*k) = -PSI_MAX_;   ubx_(10 + 24*k) = PSI_MAX_;    
      
      // UAV3
      lbx_(18 + 24*k) = -PHI_MAX_;   ubx_(18 + 24*k) = PHI_MAX_;    
      lbx_(20 + 24*k) = -THETA_MAX_; ubx_(20 + 24*k) = THETA_MAX_;  
      lbx_(22 + 24*k) = -PSI_MAX_;   ubx_(22 + 24*k) = PSI_MAX_;    
    }

    // Control bounds
    for (size_t k = 0; k < N_; ++k) {
      // UAV2 controls
      size_t idx = 24*(N_+1) + 8*k; 
      lbx_(idx + 0) = T_MIN_;          ubx_(idx + 0) = T_MAX_;          
      lbx_(idx + 1) = TAU_ROLL_MIN_;   ubx_(idx + 1) = TAU_ROLL_MAX_;   
      lbx_(idx + 2) = TAU_PITCH_MIN_;  ubx_(idx + 2) = TAU_PITCH_MAX_;  
      lbx_(idx + 3) = TAU_YAW_MIN_;    ubx_(idx + 3) = TAU_YAW_MAX_;    
      
      // UAV3 controls
      lbx_(idx + 4) = T_MIN_;          ubx_(idx + 4) = T_MAX_;          
      lbx_(idx + 5) = TAU_ROLL_MIN_;   ubx_(idx + 5) = TAU_ROLL_MAX_;   
      lbx_(idx + 6) = TAU_PITCH_MIN_;  ubx_(idx + 6) = TAU_PITCH_MAX_;  
      lbx_(idx + 7) = TAU_YAW_MIN_;    ubx_(idx + 7) = TAU_YAW_MAX_;    
    }

    lbg_ = DM::zeros(24*(N_+1), 1); 
    ubg_ = DM::zeros(24*(N_+1), 1);
  }
};

// ============================================================================
// Main
// ============================================================================
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MpcPositionController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}