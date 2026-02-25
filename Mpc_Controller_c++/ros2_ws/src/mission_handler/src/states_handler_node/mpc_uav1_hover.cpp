#include <rclcpp/rclcpp.hpp>
#include <casadi/casadi.hpp>
#include <Eigen/Dense>
#include <cmath>
#include <map>
#include <iostream>
#include <iomanip>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include "ros2_muavp_interface/msg/rigid_body_pose.hpp"
#include "ros2_muavp_interface/msg/input_setpoint.hpp"
#include "ros2_muavp_interface/msg/failure_check.hpp"

using ros2_muavp_interface::msg::RigidBodyPose;
using ros2_muavp_interface::msg::InputSetpoint;
using ros2_muavp_interface::msg::FailureCheck;

constexpr double G = 9.80665;

class UAV1HoverController : public rclcpp::Node {
public:
  UAV1HoverController() : Node("UAV1_hover_controller") {
    declare_parameters();
    load_parameters();
    build_mpc_solver();

    // Subscriptions
    sub_failure_ = create_subscription<FailureCheck>(
        "/agents/failure_check", 10,
        std::bind(&UAV1HoverController::failureCheckCb, this, std::placeholders::_1));
    
    sub_uav_pose_1 = create_subscription<RigidBodyPose>(
        "/x500_1/pose", rclcpp::SensorDataQoS(),
        std::bind(&UAV1HoverController::uavPoseCb1, this, std::placeholders::_1));

    // Publisher
    pub_setpoint_1 = create_publisher<InputSetpoint>("/x500_1/input_setpoint_", 10);

    control_timer_ = create_wall_timer(
      std::chrono::milliseconds(5),  // E.g. 100 Hz (1000 ms / 100 = 10 ms) put here 1000/(what hz you want)
      std::bind(&UAV1HoverController::controlLoop, this));

    RCLCPP_INFO(get_logger(),
                "UAV1 MPC controller initialized - dt = %.3f s, horizon = %u",
                dt_, N_);
  }

private:
  // Parameters
  double m_, Ixx_, Iyy_, Izz_;
  double T_MIN_, T_MAX_, TAU_PITCH_MIN_, TAU_PITCH_MAX_, TAU_ROLL_MIN_, TAU_ROLL_MAX_, TAU_YAW_MIN_, TAU_YAW_MAX_;
  double dt_;
  unsigned int N_;

  // Cost function weights 
  double w_position_, w_velocity_, w_attitude_, w_angular_vel_, w_control_effort_;
  double w_terminal_position_, w_terminal_velocity_, w_terminal_attitude_;
  double PHI_MAX_, THETA_MAX_, PSI_MAX_;

  // Hover target (set when failure is detected)
  double target_N_, target_E_, target_D_;
  double target_psi_;  // Maintain yaw orientation
  bool hover_target_set_{false};

  // ROS entities
  rclcpp::Subscription<RigidBodyPose>::SharedPtr sub_uav_pose_1;
  rclcpp::Subscription<FailureCheck>::SharedPtr  sub_failure_;
  rclcpp::Publisher<InputSetpoint>::SharedPtr    pub_setpoint_1;
  rclcpp::TimerBase::SharedPtr                   control_timer_;

  bool   active_{false};
  bool   state_ready_uav1{false};

  // UAV1 states
  double pN_uav1{0.0}, vN_uav1{0.0};
  double pE_uav1{0.0}, vE_uav1{0.0};
  double pD_uav1{0.0}, vD_uav1{0.0};

  double phi_uav1{0.0},  phi_dot_uav1{0.0};
  double theta_uav1{0.0},theta_dot_uav1{0.0};
  double psi_uav1{0.0},  psi_dot_uav1{0.0};

  // Debug counter
  int debug_counter_{0};

  // CasADi solver
  casadi::Function solver_;
  casadi::DM lbx_, ubx_, lbg_, ubg_;

  // --------------------------------------------------------------------------
  // Parameter handling
  void load_parameters() {
    m_   = get_parameter("m").as_double();
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

    w_position_ = get_parameter("w_position").as_double();
    w_velocity_ = get_parameter("w_velocity").as_double();
    w_attitude_ = get_parameter("w_attitude").as_double();
    w_angular_vel_ = get_parameter("w_angular_vel").as_double();
    w_control_effort_ = get_parameter("w_control_effort").as_double();
    w_terminal_position_ = get_parameter("w_terminal_position").as_double();
    w_terminal_velocity_ = get_parameter("w_terminal_velocity").as_double();
    w_terminal_attitude_ = get_parameter("w_terminal_attitude").as_double();

    PHI_MAX_ = get_parameter("PHI_MAX").as_double();
    THETA_MAX_ = get_parameter("THETA_MAX").as_double();
    PSI_MAX_ = get_parameter("PSI_MAX").as_double();
  }

  void declare_parameters() {
    // Physical parameters
    declare_parameter("m", 2.0);        // UAV mass
    declare_parameter("Ixx", 0.022);     
    declare_parameter("Iyy", 0.022);     
    declare_parameter("Izz", 0.04);      

    declare_parameter("dt", 0.02);       // Control timestep
    declare_parameter("N",  15);         // Shorter horizon for hover (faster computation)

    // Control limits
    declare_parameter("T_MIN", 15.0);
    declare_parameter("T_MAX", 33.0);
    declare_parameter("TAU_ROLL_MIN",  -0.1);
    declare_parameter("TAU_ROLL_MAX",   0.1);
    declare_parameter("TAU_PITCH_MIN", -0.1);
    declare_parameter("TAU_PITCH_MAX",  0.1);
    declare_parameter("TAU_YAW_MIN",   -0.1);
    declare_parameter("TAU_YAW_MAX",    0.1);

    declare_parameter("PHI_MAX", M_PI/4);    // 45 degrees for stable hover
    declare_parameter("THETA_MAX", M_PI/4);  // 45 degrees for stable hover
    declare_parameter("PSI_MAX", M_PI);       

    // Cost function weights for STABLE hovering (no position tracking)
    declare_parameter("w_position", 0.0);          // NO position tracking - let it drift
    declare_parameter("w_velocity", 5.0);          // Light velocity penalty - allow some drift
    declare_parameter("w_attitude", 100.0);        // CRITICAL: heavy penalty on roll/pitch to prevent flipping
    declare_parameter("w_angular_vel", 50.0);      // CRITICAL: heavy penalty on angular rates to prevent tumbling
    declare_parameter("w_control_effort", 0.5);    // Moderate control effort for smooth control
    declare_parameter("w_terminal_position", 0.0); // NO terminal position constraint
    declare_parameter("w_terminal_velocity", 80.0); // Strong terminal velocity constraint (especially vertical)
    declare_parameter("w_terminal_attitude", 150.0); // CRITICAL: very strong terminal attitude constraint
  }

  // --------------------------------------------------------------------------
  void uavPoseCb1(const RigidBodyPose::SharedPtr msg) {
    // Position (NED)
    pN_uav1 = msg->position[0];
    pE_uav1 = msg->position[1];
    pD_uav1 = msg->position[2]; 

    vN_uav1 = msg->lin_velocity[0];
    vE_uav1 = msg->lin_velocity[1];
    vD_uav1 = msg->lin_velocity[2];

    phi_uav1   = msg->orientation[0];
    theta_uav1 = msg->orientation[1];
    psi_uav1   = msg->orientation[2];

    phi_dot_uav1   = msg->ang_velocity[0]; // p (body frame angular velocity)
    theta_dot_uav1 = msg->ang_velocity[1]; // q
    psi_dot_uav1   = msg->ang_velocity[2]; // r

    state_ready_uav1 = true;
  }

  // --------------------------------------------------------------------------
  void failureCheckCb(const FailureCheck::SharedPtr msg) {
    if (!active_ && (msg->uav_1_failed || msg->uav_2_failed || msg->uav_3_failed)) {
      active_ = true;
      
      // Set hover target at current position when failure is detected
      if (!hover_target_set_ && state_ready_uav1) {
        target_N_ = pN_uav1;
        target_E_ = pE_uav1;
        target_D_ = pD_uav1;
        target_psi_ = psi_uav1;  // Maintain current yaw
        hover_target_set_ = true;
        
        RCLCPP_WARN(get_logger(),
                    "Failure detected - UAV1 hover controller activated at position [%.2f, %.2f, %.2f]",
                    target_N_, target_E_, target_D_);
      }
    }
  }

  // --------------------------------------------------------------------------
  // Control loop
  void controlLoop() {
    if (!active_ || !hover_target_set_) return;
    if (!state_ready_uav1) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for UAV1 state...");
      return;
    }

    // Pack current state vector [12x1] for single UAV
    casadi::DM P = casadi::DM::zeros(16, 1);  // 12 states + 4 target states

    // UAV1 states (indices 0-11)
    P(0)  = pN_uav1;      P(1)  = vN_uav1;
    P(2)  = pE_uav1;      P(3)  = vE_uav1;
    P(4)  = pD_uav1;      P(5)  = vD_uav1;
    P(6)  = phi_uav1;     P(7)  = phi_dot_uav1;
    P(8)  = theta_uav1;   P(9)  = theta_dot_uav1;
    P(10) = psi_uav1;     P(11) = psi_dot_uav1;

    // Target hover position and yaw (indices 12-15)
    P(12) = target_N_;
    P(13) = target_E_;
    P(14) = target_D_;
    P(15) = target_psi_;

    // Increment debug counter
    debug_counter_++;
    
    // DEBUG: Print UAV1 state =================================================

    // std::cout << "=== UAV1 HOVER MPC DEBUG CYCLE " << debug_counter_ << " ===" << std::endl;
    // std::cout << "UAV1 Current: Pos=[" << std::fixed << std::setprecision(3) 
    //           << pN_uav1 << ", " << pE_uav1 << ", " << pD_uav1 << "] Vel=[" 
    //           << vN_uav1 << ", " << vE_uav1 << ", " << vD_uav1 << "]" << std::endl;
    // std::cout << "UAV1 Angles: phi=" << phi_uav1 << " theta=" << theta_uav1 << " psi=" << psi_uav1;
    // std::cout << " Rates: p=" << phi_dot_uav1 << " q=" << theta_dot_uav1 << " r=" << psi_dot_uav1 << std::endl;
    // std::cout << "Target: Pos=[" << target_N_ << ", " << target_E_ << ", " << target_D_ 
    //           << "] Yaw=" << target_psi_ << std::endl;
    
    // // Position errors
    // double pos_error = sqrt((pN_uav1-target_N_)*(pN_uav1-target_N_) + 
    //                        (pE_uav1-target_E_)*(pE_uav1-target_E_) + 
    //                        (pD_uav1-target_D_)*(pD_uav1-target_D_));
    // double vel_magnitude = sqrt(vN_uav1*vN_uav1 + vE_uav1*vE_uav1 + vD_uav1*vD_uav1);
    
    // std::cout << "Position Error: " << pos_error << "m, Velocity: " << vel_magnitude << "m/s" << std::endl;

    // ==============================================================================

    // Warm start vector
    static casadi::DM w0;
    if (w0.is_empty()) {
      w0 = casadi::DM::zeros(12*(N_+1) + 4*N_);
      
      // Initialize with current states repeated over horizon
      for (size_t k = 0; k <= N_; ++k) {
        size_t offset = 12 * k;
        w0(offset + 0) = pN_uav1;    w0(offset + 1) = vN_uav1;
        w0(offset + 2) = pE_uav1;    w0(offset + 3) = vE_uav1;
        w0(offset + 4) = pD_uav1;    w0(offset + 5) = vD_uav1;
        w0(offset + 6) = phi_uav1;   w0(offset + 7) = phi_dot_uav1;
        w0(offset + 8) = theta_uav1; w0(offset + 9) = theta_dot_uav1;
        w0(offset + 10) = psi_uav1;  w0(offset + 11) = psi_dot_uav1;
      }
      
      // Initialize control with hover thrust
      double hover_thrust = m_ * G;
      size_t control_offset = 12 * (N_ + 1);
      for (size_t k = 0; k < N_; ++k) {
        w0(control_offset + 4*k + 0) = hover_thrust;  // T
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

      //  ======================

      auto stats = solver_.stats();
      std::cout << "=== HOVER MPC SOLVER STATUS: " << stats.at("return_status").to_string() << " ===" << std::endl;
      
      //  ======================

      casadi::DM w_opt = res.at("x");
      w0 = w_opt;

      // DEBUG: Print solver statistics and cost =====================

      // casadi::DM f_opt = res.at("f");
      // std::cout << "Hover MPC Optimal Cost: " << std::fixed << std::setprecision(6) 
      //           << static_cast<double>(f_opt.scalar()) << std::endl;
      // std::cout << "Solver Iterations: " << static_cast<int>(stats.at("iter_count").to_int()) << std::endl;

      // =======================================

      size_t state_vars = 12 * (N_ + 1); 

      // UAV1 control
      double thrust_uav1     = static_cast<double>(w_opt(state_vars + 0).scalar());
      double tau_roll_uav1   = static_cast<double>(w_opt(state_vars + 1).scalar());
      double tau_pitch_uav1  = static_cast<double>(w_opt(state_vars + 2).scalar());
      double tau_yaw_uav1    = static_cast<double>(w_opt(state_vars + 3).scalar());

      // DEBUG: Print optimal control outputs ===============

      // std::cout << "UAV1 Hover Controls: T=" << std::fixed << std::setprecision(3) << -thrust_uav1 
      //           << " tau_roll=" << tau_roll_uav1 << " tau_pitch=" << tau_pitch_uav1 
      //           << " tau_yaw=" << tau_yaw_uav1 << std::endl;

      // ===========================

      InputSetpoint cmd_uav1;
      cmd_uav1.timestamp = 1;
      cmd_uav1.setpoint[0] = static_cast<float>(tau_roll_uav1);
      cmd_uav1.setpoint[1] = static_cast<float>(tau_pitch_uav1);
      cmd_uav1.setpoint[2] = static_cast<float>(-thrust_uav1); // PX4 needs negative thrust
      cmd_uav1.setpoint[3] = static_cast<float>(tau_yaw_uav1);
      cmd_uav1.setpoint_type = 2;
      
      pub_setpoint_1->publish(cmd_uav1);

      // =========================

      // std::cout << "=== END UAV1 HOVER DEBUG CYCLE ===" << std::endl << std::endl;

      // ==========================

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "UAV1 Hover MPC solver failed: %s", e.what());
    }
  }

  // --------------------------------------------------------------------------
  // Build MPC solver for UAV1 hover dynamics
  void build_mpc_solver() {
    using namespace casadi;

    // Decision variables - 12 states (single UAV), 4 controls
    SX X = SX::sym("X", 12, N_ + 1);     // State trajectory
    SX U = SX::sym("U",  4, N_);         // Control trajectory  
    SX P = SX::sym("P", 16);             // Parameters (current state + target)

    // Objective & constraints
    SX cost = 0;
    std::vector<SX> g;

    // Initial state constraint
    g.push_back(X(Slice(0, 12), 0) - P(Slice(0, 12))); // X(:,0) = P(0:11)

    for (unsigned int k = 0; k < N_; ++k) {
      SX xk = X(Slice(), k);
      SX uk = U(Slice(), k);

      // Extract UAV1 states (indices 0-11)
      SX N1 = xk(0), vN1 = xk(1), E1 = xk(2), vE1 = xk(3), D1 = xk(4), vD1 = xk(5);
      SX phi1 = xk(6), p1 = xk(7), th1 = xk(8), q1 = xk(9), psi1 = xk(10), r1 = xk(11);
      
      // Extract controls
      SX T1 = uk(0), tau_phi1 = uk(1), tau_theta1 = uk(2), tau_psi1 = uk(3);

      // Extract target position and yaw from parameters
      SX target_N = P(12), target_E = P(13), target_D = P(14), target_psi = P(15);

      // UAV1 translational dynamics (simple quadrotor dynamics without tether forces)
      SX aN1 = -(T1/m_)*(sin(phi1)*sin(psi1) + cos(phi1)*sin(th1)*cos(psi1));
      SX aE1 = -(T1/m_)*(cos(phi1)*sin(th1)*sin(psi1) - sin(phi1)*cos(psi1));
      SX aD1 = -(T1/m_)*(cos(phi1)*cos(th1)) + G;

      // UAV rotational dynamics
      SX p1_dot = (tau_phi1 - (Iyy_ - Izz_)*q1*r1)/Ixx_;
      SX q1_dot = (tau_theta1 - (Izz_ - Ixx_)*p1*r1)/Iyy_;
      SX r1_dot = (tau_psi1 - (Ixx_ - Iyy_)*p1*q1)/Izz_;

      // Integrate body rates
      SX p1_new = p1 + p1_dot*dt_;
      SX q1_new = q1 + q1_dot*dt_;
      SX r1_new = r1 + r1_dot*dt_;

      // Convert body rates to Euler angle rates
      SX cth1 = cos(th1);
      SX cth1_safe = fmax(fabs(cth1), 1e-6) * sign(cth1); 
      SX tth1_safe = sin(th1) / cth1_safe;  // Safe tangent calculation
      SX eul1_dot1 = p1_new + sin(phi1)*tth1_safe*q1_new + cos(phi1)*tth1_safe*r1_new;
      SX eul1_dot2 = cos(phi1)*q1_new - sin(phi1)*r1_new;
      SX eul1_dot3 = sin(phi1)/cth1_safe*q1_new + cos(phi1)/cth1_safe*r1_new;

      // Integrate Euler angles
      SX phi1_new = phi1 + eul1_dot1*dt_;
      SX th1_new = th1 + eul1_dot2*dt_;
      SX psi1_new = psi1 + eul1_dot3*dt_;

      // Integrate UAV translations
      SX vN1_new = vN1 + aN1*dt_;
      SX N1_new = N1 + vN1_new*dt_;
      SX vE1_new = vE1 + aE1*dt_;
      SX E1_new = E1 + vE1_new*dt_;
      SX vD1_new = vD1 + aD1*dt_;
      SX D1_new = D1 + vD1_new*dt_;

      // Pack next state
      SX x_next = SX::vertcat({
        N1_new, vN1_new, E1_new, vE1_new, D1_new, vD1_new,
        phi1_new, p1_new, th1_new, q1_new, psi1_new, r1_new
      });

      g.push_back(X(Slice(), k+1) - x_next); // X(:, k+1) = x_next

      // ------------- STABLE HOVER COST FUNCTION (NO POSITION TRACKING) -------------
      
      // 1. Attitude stabilization - CRITICAL: prevent flipping by heavily penalizing roll/pitch
      SX attitude_cost = w_attitude_ * (phi1*phi1 + th1*th1);
      
      // 2. Angular velocity damping - CRITICAL: prevent tumbling and oscillations
      SX angular_vel_cost = w_angular_vel_ * (p1*p1 + q1*q1 + r1*r1);
      
      // 3. Velocity stabilization - prevent excessive drift but allow some movement
      SX velocity_cost = w_velocity_ * (vN1*vN1 + vE1*vE1 + vD1*vD1);
      
      // 4. Vertical velocity control - prevent free fall (most important for safety)
      SX vertical_vel_cost = 10.0 * w_velocity_ * (vD1*vD1);  // Extra penalty for vertical motion
      
      // 5. Thrust stabilization - maintain reasonable thrust around hover value
      SX hover_thrust = m_ * G;
      SX thrust_deviation = T1 - hover_thrust;
      SX thrust_cost = w_control_effort_ * 2.0 * (thrust_deviation*thrust_deviation);
      
      // 6. Torque minimization - smooth control
      SX torque_cost = w_control_effort_ * (tau_phi1*tau_phi1 + tau_theta1*tau_theta1 + tau_psi1*tau_psi1);

      cost += attitude_cost + angular_vel_cost + velocity_cost + vertical_vel_cost + thrust_cost + torque_cost;
    }

    // Terminal cost - focus on stability, NOT position
    SX xT = X(Slice(), N_);
    
    // Terminal attitude stability - MOST IMPORTANT: prevent terminal flipping
    SX phiT = xT(6), thT = xT(8);
    SX terminal_attitude_cost = w_terminal_attitude_ * (phiT*phiT + thT*thT);
    
    // Terminal angular velocity - ensure smooth terminal state
    SX pT = xT(7), qT = xT(9), rT = xT(11);
    SX terminal_angular_vel_cost = w_terminal_velocity_ * (pT*pT + qT*qT + rT*rT);
    
    // Terminal vertical velocity - prevent terminal free fall
    SX vDT = xT(5);
    SX terminal_vertical_vel_cost = w_terminal_velocity_ * 5.0 * (vDT*vDT);

    cost += terminal_attitude_cost + terminal_angular_vel_cost + terminal_vertical_vel_cost;

    // Flatten decision vector
    SX w = SX::vertcat({SX::reshape(X, 12*(N_+1), 1),  
                        SX::reshape(U,  4*N_,      1)});
    SX g_concat = SX::vertcat(g);

    // NLP problem
    SXDict nlp = {{"x", w}, {"f", cost}, {"g", g_concat}, {"p", P}};

    // Solver options - optimized for real-time hover control
    Dict opts;
    opts["ipopt.print_level"] = 0;               
    opts["print_time"] = false;                  
    opts["ipopt.sb"] = "yes";                    
    opts["ipopt.max_iter"] = 50;                 // Reduced for faster solve
    opts["ipopt.tol"] = 1e-2;                    
    opts["ipopt.dual_inf_tol"] = 1e-2;           
    opts["ipopt.constr_viol_tol"] = 1e-1;        
    opts["ipopt.compl_inf_tol"] = 1e-1;          
    
    // Warm start
    opts["ipopt.warm_start_init_point"] = "yes"; 
    opts["ipopt.warm_start_bound_push"] = 1e-3;  
    opts["ipopt.warm_start_mult_bound_push"] = 1e-3;
    
    opts["ipopt.mu_strategy"] = "adaptive";      
    opts["ipopt.linear_solver"] = "mumps";       

    solver_ = nlpsol("solver", "ipopt", nlp, opts);

    // Variable bounds
    size_t total_vars = 12*(N_+1) + 4*N_;
    lbx_ = DM::zeros(total_vars);
    ubx_ = DM::zeros(total_vars);

    // Default: unbounded states
    for (size_t i = 0; i < total_vars; ++i) {
      lbx_(i) = -DM::inf();
      ubx_(i) =  DM::inf();
    }

    // Angle constraints
    for (size_t k = 0; k <= N_; ++k) {
      // UAV1 angle limits (indices 6, 8, 10)
      lbx_(6  + 12*k) = -PHI_MAX_;   ubx_(6  + 12*k) = PHI_MAX_;    
      lbx_(8  + 12*k) = -THETA_MAX_; ubx_(8  + 12*k) = THETA_MAX_;  
      // No strict yaw limits for hover
    }

    // Control bounds
    for (size_t k = 0; k < N_; ++k) {
      size_t idx = 12*(N_+1) + 4*k; 
      
      // UAV1 controls
      lbx_(idx + 0) = T_MIN_;          ubx_(idx + 0) = T_MAX_;          
      lbx_(idx + 1) = TAU_ROLL_MIN_;   ubx_(idx + 1) = TAU_ROLL_MAX_;   
      lbx_(idx + 2) = TAU_PITCH_MIN_;  ubx_(idx + 2) = TAU_PITCH_MAX_;  
      lbx_(idx + 3) = TAU_YAW_MIN_;    ubx_(idx + 3) = TAU_YAW_MAX_;    
    }

    lbg_ = DM::zeros(12*(N_+1), 1); 
    ubg_ = DM::zeros(12*(N_+1), 1);
  }
};

// ============================================================================
// Main
// ============================================================================
int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<UAV1HoverController>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}