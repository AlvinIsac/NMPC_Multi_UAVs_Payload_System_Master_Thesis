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

using ros2_muavp_interface::msg::RigidBodyPose;
using ros2_muavp_interface::msg::InputSetpoint;
using ros2_muavp_interface::msg::FailureCheck;

// Gravity constant (m/s^2), acts along +D in NED
constexpr double G = 9.80665;

// ============================================================================
// World frame: NED (North, East, Down). Pose topic already gives NED.
// States (12):
//   x = [pN vN pE vE pD vD φ φ̇ θ θ̇ ψ ψ̇]^T

class MpcPositionController : public rclcpp::Node {
public:
  MpcPositionController() : Node("uav_position_mpc_controller") {
    declare_parameters();
    load_parameters();
    init_prev_u();
    build_mpc_solver();

    // Subscriptions
    sub_uav_pose_ = create_subscription<RigidBodyPose>(
        "/x500_3/pose", rclcpp::SensorDataQoS(),
        std::bind(&MpcPositionController::uavPoseCb, this, std::placeholders::_1));

    sub_failure_ = create_subscription<FailureCheck>(
        "/agents/failure_check", 10,
        std::bind(&MpcPositionController::failureCheckCb, this, std::placeholders::_1));

    // Publishers
    pub_setpoint_3 = create_publisher<InputSetpoint>("/x500_3/input_setpoint", 10);

    control_timer_ = create_wall_timer(
        std::chrono::milliseconds(20),
        std::bind(&MpcPositionController::controlLoop, this));

    RCLCPP_INFO(get_logger(),
                "MPC controller initialized uav3 - dt = %.3f s, horizon = %u",
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
  rclcpp::Subscription<RigidBodyPose>::SharedPtr sub_uav_pose_;
  rclcpp::Subscription<FailureCheck>::SharedPtr  sub_failure_;
  rclcpp::Publisher<InputSetpoint>::SharedPtr    pub_setpoint_3;
  rclcpp::TimerBase::SharedPtr                   control_timer_;

  bool   active_{false};
  bool   state_ready_{false};

  double pN_{0.0}, vN_{0.0};
  double pE_{0.0}, vE_{0.0};
  double pD_{0.0}, vD_{0.0};

  double phi_{0.0},  phi_dot_{0.0};
  double theta_{0.0},theta_dot_{0.0};
  double psi_{0.0},  psi_dot_{0.0};

  // Desired setpoint (detachment position) in NED
  double pN_des_{0.0}, pE_des_{0.0}, pD_des_{0.0};

  // Previous control (for Δu cost and warm start) : [T, τθ, τφ, τψ]
  Eigen::Vector4d u_prev_;

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
    declare_parameter("M", 2.5);        
    declare_parameter("Ixx", 0.022);     
    declare_parameter("Iyy", 0.022);     
    declare_parameter("Izz", 0.04);      

        // MPC grid
    declare_parameter("dt", 0.02);       // 50 Hz
    declare_parameter("N",  15);         // horizon

    // Control limits
    declare_parameter("T_MIN", -33.0);
    declare_parameter("T_MAX", -20.0);
    declare_parameter("TAU_ROLL_MIN",  -0.13);
    declare_parameter("TAU_ROLL_MAX",   0.13);
    declare_parameter("TAU_PITCH_MIN", -0.1);
    declare_parameter("TAU_PITCH_MAX",  0.1);
    declare_parameter("TAU_YAW_MIN",   -0.001);
    declare_parameter("TAU_YAW_MAX",    0.001);

    declare_parameter("PHI_MAX", M_PI/12);    
    declare_parameter("THETA_MAX", M_PI/12);  
    declare_parameter("PSI_MAX", M_PI);       

declare_parameter("w_pn", 0.0);    
declare_parameter("w_vn", 0.2);

declare_parameter("w_pe", 0.0);    
declare_parameter("w_ve", 0.2);

declare_parameter("w_pd", 10.0);    
declare_parameter("w_vd", 8.0);    

// Attitude weights – HIGH (for stable hover)
declare_parameter("w_phi",       6.0);   
declare_parameter("w_phi_dot",   1.5);   

declare_parameter("w_theta",     6.0);   
declare_parameter("w_theta_dot", 1.5);   

declare_parameter("w_psi",       0.2);    
declare_parameter("w_psi_dot",   0.1);    

// Control effort weights – MEDIUM
declare_parameter("w_u_t",          0.8);   
declare_parameter("w_u_tau_roll",   0.4);  
declare_parameter("w_u_tau_pitch",  0.4);  
declare_parameter("w_u_tau_yaw",    0.1);   

// Control-rate (Δu) weights – HIGH (to keep outputs smooth)
declare_parameter("w_du_t",          15.0);
declare_parameter("w_du_tau_roll",   10.0); 
declare_parameter("w_du_tau_pitch",  10.0);
declare_parameter("w_du_tau_yaw",    5.0);

  }

  void init_prev_u() {
    // Hover thrust and zero torques (good for Δu at first step)
    u_prev_ << -M_*G, 0.0, 0.0, 0.0;
  }

  // --------------------------------------------------------------------------
  void uavPoseCb(const RigidBodyPose::SharedPtr msg) {
    // Position (NED)
    pN_ = msg->position[0];
    pE_ = msg->position[1];
    pD_ = msg->position[2]; 

    vN_ = msg->lin_velocity[0];
    vE_ = msg->lin_velocity[1];
    vD_ = msg->lin_velocity[2];

    phi_   = msg->orientation[0];
    theta_ = msg->orientation[1];
    psi_   = msg->orientation[2];

    phi_dot_   = msg->ang_velocity[0];
    theta_dot_ = msg->ang_velocity[1];
    psi_dot_   = msg->ang_velocity[2];

    state_ready_ = true;
  }
  // --------------------------------------------------------------------------
  void failureCheckCb(const FailureCheck::SharedPtr msg) {
    if (!active_ && (msg->uav_1_failed || msg->uav_2_failed || msg->uav_3_failed)) {
      active_ = true;
      pN_des_ = pN_;
      pE_des_ = pE_;
      pD_des_ = pD_;
      RCLCPP_WARN(get_logger(),
                  "UAV failure detected - Activating MPC controller = (%.2f, %.2f, %.2f)",
                  pN_des_, pE_des_, pD_des_);
    }
  }

    static inline Eigen::Matrix3d R_world_from_body(double phi, double th, double psi) {
    const double cphi = std::cos(phi),  sphi = std::sin(phi);
    const double cth  = std::cos(th),   sth  = std::sin(th);
    const double cpsi = std::cos(psi),  spsi = std::sin(psi);

    Eigen::Matrix3d Rz; Rz << cpsi, -spsi, 0,
                               spsi,  cpsi, 0,
                                  0,     0, 1;
    Eigen::Matrix3d Ry; Ry <<  cth, 0, sth,
                                 0, 1,   0,
                               -sth, 0, cth;
    Eigen::Matrix3d Rx; Rx << 1,   0,    0,
                              0, cphi, -sphi,
                              0, sphi,  cphi;
    return Rz * Ry * Rx; // world_from_body
  }

  // --------------------------------------------------------------------------
  // Control loop
  void controlLoop() {
    if (!active_) return;
    if (!state_ready_) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000, "Waiting for initial state...");
      return;
    }


    // P packs: current state (12) + desired pos (3) + previous control (4) = 19
    casadi::DM P = casadi::DM::zeros(19, 1);
    P(0)  = pN_;      P(1)  = vN_;
    P(2)  = pE_;      P(3)  = vE_;
    P(4)  = pD_;      P(5)  = vD_;
    P(6)  = phi_;     P(7)  = phi_dot_;
    P(8)  = theta_;   P(9)  = theta_dot_;
    P(10) = psi_;     P(11) = psi_dot_;
    // Desired positions
    P(12) = pN_des_;
    P(13) = pE_des_;
    P(14) = pD_des_;
    // Previous control u_{-1}
    P(15) = u_prev_(0);
    P(16) = u_prev_(1);
    P(17) = u_prev_(2);
    P(18) = u_prev_(3);

    // Warm start vector
    static casadi::DM w0;
    if (w0.is_empty()) {
      w0 = casadi::DM::zeros(12*(N_+1) + 4*N_);
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
      casadi::DM w_opt = res.at("x");
      w0 = w_opt;

      // Extract first control action
      size_t offset = 12 * (N_ + 1);
      double thrust     = static_cast<double>(w_opt(offset + 0).scalar());
      double tau_roll   = static_cast<double>(w_opt(offset + 1).scalar());
      double tau_pitch  = static_cast<double>(w_opt(offset + 2).scalar());
      double tau_yaw    = static_cast<double>(w_opt(offset + 3).scalar());

// --- after extracting thrust, tau_roll, tau_pitch, tau_yaw ---

// Convert torque vector from WORLD to BODY
Eigen::Vector3d tau_world(tau_roll, tau_pitch, tau_yaw);
Eigen::Matrix3d Rwb = R_world_from_body(phi_, theta_, psi_);
Eigen::Vector3d tau_body = Rwb.transpose() * tau_world; // body_from_world = R^T

// (optional) clamp to parameter limits in body frame
tau_body.x() = std::clamp(tau_body.x(), TAU_ROLL_MIN_,  TAU_ROLL_MAX_);
tau_body.y() = std::clamp(tau_body.y(), TAU_PITCH_MIN_, TAU_PITCH_MAX_);
tau_body.z() = std::clamp(tau_body.z(), TAU_YAW_MIN_,   TAU_YAW_MAX_);

// Publish BODY-frame command: [τ_roll_b, τ_pitch_b, thrust, τ_yaw_b]
InputSetpoint cmd;
cmd.timestamp = 1;
cmd.setpoint[0] = static_cast<float>(tau_body.x()); // roll (about body X)
cmd.setpoint[1] = static_cast<float>(tau_body.y()); // pitch (about body Y)
cmd.setpoint[2] = static_cast<float>(thrust);       // thrust (along body Z)
cmd.setpoint[3] = static_cast<float>(tau_body.z()); // yaw (about body Z)
cmd.setpoint_type = 2;
pub_setpoint_3->publish(cmd);

// Store for next cycle (still keep what you actually sent)
u_prev_ << thrust, tau_body.x(), tau_body.y(), tau_body.z();


      std::cout << "[MPC] T = " << thrust << " | τφ = " << tau_roll << " | τθ = " << tau_pitch << " | τψ = " << tau_yaw << std::endl;

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "MPC solver failed: %s", e.what());
    }
  }

  // --------------------------------------------------------------------------
  // Build MPC solver (NED dynamics with yaw coupling + Δu cost)
  void build_mpc_solver() {
    using namespace casadi;

    // Decision variables
    SX X = SX::sym("X", 12, N_ + 1);     // State trajectory
    SX U = SX::sym("U",  4, N_);         // Control trajectory
    SX P = SX::sym("P", 19);             // Parameters: x0(12) + p_des(3) + u_prev(4)

    // Objective & constraints
    SX cost = 0;
    std::vector<SX> g;

    // Initial state constraint
    g.push_back(X(Slice(), 0) - P(Slice(0, 12)));

    for (unsigned int k = 0; k < N_; ++k) {
      SX xk = X(Slice(), k);
      SX uk = U(Slice(), k);

      // States
      SX pN = xk(0), vN = xk(1);
      SX pE = xk(2), vE = xk(3);
      SX pD = xk(4), vD = xk(5);

      SX phi = xk(6),  phi_dot = xk(7);
      SX th  = xk(8),  th_dot  = xk(9);
      SX psi = xk(10), psi_dot = xk(11);

      // Controls
      SX T      = uk(0);
      SX tau_ph = uk(1);
      SX tau_th = uk(2);
      SX tau_ps = uk(3);


      SX aN = (T / M_) * ( sin(phi) * sin(psi) + cos(phi) * sin(th) * cos(psi) );
      SX aE = (T / M_) * ( cos(phi) * sin(th) * sin(psi) - sin(phi) * cos(psi) );
      SX aD = (T / M_) * ( cos(phi) * cos(th) ) + G;

      SX phi_ddot = tau_ph / Ixx_;
      SX th_ddot  = tau_th / Iyy_;
      SX psi_ddot = tau_ps / Izz_;

      // State derivatives
      SX xdot = SX::vertcat({
        vN, aN,
        vE, aE,
        vD, aD,
        phi_dot,  phi_ddot,
        th_dot,   th_ddot,
        psi_dot,  psi_ddot
      });

      // Forward Euler integration
      SX x_next = xk + dt_ * xdot;
      g.push_back(X(Slice(), k+1) - x_next);

      // ------------- Cost terms -------------
      // Position tracking to detachment point
      SX pos_cost =
          w_pn_ * pow(pN - P(12), 2) +
          w_pe_ * pow(pE - P(13), 2) +
          w_pd_ * pow(pD - P(14), 2);

      // Velocity damping
      SX vel_cost =
          w_vn_ * pow(vN, 2) +
          w_ve_ * pow(vE, 2) +
          w_vd_ * pow(vD, 2);

      // Attitude stabilization (to zero)
      SX att_cost =
          w_phi_       * pow(phi, 2)      + w_phi_dot_   * pow(phi_dot, 2) +
          w_theta_     * pow(th,  2)      + w_theta_dot_ * pow(th_dot,  2) +
          w_psi_       * pow(psi, 2)      + w_psi_dot_   * pow(psi_dot, 2);

      // Control effort
      SX u_cost =
          w_u_t_         * pow(T,      2) +
          w_u_tau_roll_  * pow(tau_ph, 2) +
          w_u_tau_pitch_ * pow(tau_th, 2) +
          w_u_tau_yaw_   * pow(tau_ps, 2);

      // Control rate (Δu) with u_{-1} from P(15:18)
      SX du0, du1, du2, du3;
      if (k == 0) {
        du0 = uk(0) - P(15);
        du1 = uk(1) - P(16);
        du2 = uk(2) - P(17);
        du3 = uk(3) - P(18);
      } else {
        du0 = uk(0) - U(0, k-1);
        du1 = uk(1) - U(1, k-1);
        du2 = uk(2) - U(2, k-1);
        du3 = uk(3) - U(3, k-1);
      }
      SX du_cost =
          w_du_t_         * pow(du0, 2) +
          w_du_tau_roll_  * pow(du1, 2) +
          w_du_tau_pitch_ * pow(du2, 2) +
          w_du_tau_yaw_   * pow(du3, 2);

      cost += pos_cost + vel_cost + att_cost + u_cost + du_cost;
    }

    // Flatten decision vector
    SX w = SX::vertcat({SX::reshape(X, 12*(N_+1), 1),
                        SX::reshape(U,  4*N_,      1)});
    SX g_concat = SX::vertcat(g);

    // NLP problem
    SXDict nlp = {{"x", w}, {"f", cost}, {"g", g_concat}, {"p", P}};

    // Solver options
    Dict opts;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = 0;
    opts["ipopt.sb"] = "yes";
    opts["ipopt.max_iter"] = 50;

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

    // Attitude bounds
    for (size_t k = 0; k <= N_; ++k) {
      lbx_(6  + 12*k) = -PHI_MAX_;   ubx_(6  + 12*k) = PHI_MAX_;    // φ
      lbx_(8  + 12*k) = -THETA_MAX_; ubx_(8  + 12*k) = THETA_MAX_;  // θ
      lbx_(10 + 12*k) = -PSI_MAX_;   ubx_(10 + 12*k) = PSI_MAX_;    // ψ
    }

    // Control bounds
    for (size_t k = 0; k < N_; ++k) {
      size_t idx = 12*(N_+1) + 4*k;
      lbx_(idx + 0) = T_MIN_;          ubx_(idx + 0) = T_MAX_;          // Thrust
      lbx_(idx + 1) = TAU_ROLL_MIN_;   ubx_(idx + 1) = TAU_ROLL_MAX_;   // Roll torque
      lbx_(idx + 2) = TAU_PITCH_MIN_;  ubx_(idx + 2) = TAU_PITCH_MAX_;  // Pitch torque
      lbx_(idx + 3) = TAU_YAW_MIN_;    ubx_(idx + 3) = TAU_YAW_MAX_;    // Yaw torque
    }

    // Equality constraints (g=0): initial + N dynamics steps = 12*(N+1)
    lbg_ = DM::zeros(12*(N_+1), 1);
    ubg_ = DM::zeros(12*(N_+1), 1);
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