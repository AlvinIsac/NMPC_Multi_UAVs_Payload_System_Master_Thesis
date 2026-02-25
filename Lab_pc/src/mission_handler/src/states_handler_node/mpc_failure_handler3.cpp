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

using std::placeholders::_1;
using ros2_muavp_interface::msg::RigidBodyPose;
using ros2_muavp_interface::msg::InputSetpoint;
using ros2_muavp_interface::msg::FailureCheck;
using ros2_muavp_interface::msg::JointForceTorque;

class MpcUavNode : public rclcpp::Node {
public:
  MpcUavNode() : Node("mpc_uav_node") {
    // ---------------- Parameters ----------------
    declare_parameter("m", 2.0);
    declare_parameter("g", 9.80665);
    declare_parameter("Ixx", 0.022);
    declare_parameter("Iyy", 0.022);
    declare_parameter("Izz", 0.04);
    declare_parameter("dt", 0.05);            // 20 Hz
    declare_parameter("N", 20);               // horizon
    declare_parameter("target_N", -5.0);      // NED targets
    declare_parameter("target_E", -7.0);
    declare_parameter("target_D", -10.0);
    declare_parameter("target_psi", 0.0);

    // Weights
    declare_parameter("w_pN", 5.0);
    declare_parameter("w_vN", 0.2);
    declare_parameter("w_pE", 5.0);
    declare_parameter("w_vE", 0.2);
    declare_parameter("w_pD", 6.0);
    declare_parameter("w_vD", 0.3);
    declare_parameter("w_psi", 0.2);
    declare_parameter("w_r", 0.05);
    declare_parameter("w_u_T", 0.02);
    declare_parameter("w_u_tau", 0.01);
    declare_parameter("w_term", 20.0);

    // Limits
    declare_parameter("phi_max_deg", 45.0);
    declare_parameter("theta_max_deg", 45.0);
    declare_parameter("psi_max_deg", 180.0);
    declare_parameter("T_min", 0.0);
    declare_parameter("T_max", 25.0);
    declare_parameter("tau_roll_min", -0.15);
    declare_parameter("tau_roll_max",  0.15);
    declare_parameter("tau_pitch_min", -0.15);
    declare_parameter("tau_pitch_max",  0.15);
    declare_parameter("tau_yaw_min",   -0.02);
    declare_parameter("tau_yaw_max",    0.02);

    // Fetch params
    m_ = get_parameter("m").as_double();
    g_ = get_parameter("g").as_double();
    Ixx_= get_parameter("Ixx").as_double();
    Iyy_= get_parameter("Iyy").as_double();
    Izz_= get_parameter("Izz").as_double();
    dt_ = get_parameter("dt").as_double();
    N_  = get_parameter("N").as_int();

    Ndes_   = get_parameter("target_N").as_double();
    Edes_   = get_parameter("target_E").as_double();
    Ddes_   = get_parameter("target_D").as_double();
    psi_des_= get_parameter("target_psi").as_double();

    w_pN_=get_parameter("w_pN").as_double();
    w_vN_=get_parameter("w_vN").as_double();
    w_pE_=get_parameter("w_pE").as_double();
    w_vE_=get_parameter("w_vE").as_double();
    w_pD_=get_parameter("w_pD").as_double();
    w_vD_=get_parameter("w_vD").as_double();
    w_psi_=get_parameter("w_psi").as_double();
    w_r_  =get_parameter("w_r").as_double();
    w_u_T_=get_parameter("w_u_T").as_double();
    w_u_tau_=get_parameter("w_u_tau").as_double();
    w_term_=get_parameter("w_term").as_double();

    double phi_max = get_parameter("phi_max_deg").as_double()*M_PI/180.0;
    double theta_max = get_parameter("theta_max_deg").as_double()*M_PI/180.0;
    double psi_max = get_parameter("psi_max_deg").as_double()*M_PI/180.0;

    T_min_=get_parameter("T_min").as_double();
    T_max_=get_parameter("T_max").as_double();
    tau_roll_min_=get_parameter("tau_roll_min").as_double();
    tau_roll_max_=get_parameter("tau_roll_max").as_double();
    tau_pitch_min_=get_parameter("tau_pitch_min").as_double();
    tau_pitch_max_=get_parameter("tau_pitch_max").as_double();
    tau_yaw_min_  =get_parameter("tau_yaw_min").as_double();
    tau_yaw_max_  =get_parameter("tau_yaw_max").as_double();

    // ---------------- Build solver ----------------
    build_solver(phi_max, theta_max, psi_max);

    // ---------------- ROS I/O ----------------
    sub_odom_ = create_subscription<RigidBodyPose>(
        "/x500_3/pose", rclcpp::SensorDataQoS(),
        std::bind(&MpcUavNode::odom_cb, this, std::placeholders::_1));

    pub_uav = create_publisher<InputSetpoint>("/x500_3/input_setpoint_", 10);

    timer_ = create_wall_timer(std::chrono::milliseconds(int(dt_*1000.0)),
      std::bind(&MpcUavNode::control_step, this));

    RCLCPP_INFO(get_logger(), "MPC node ready. dt=%.3f, N=%d", dt_, N_);
  }

private:
  // ---------------- Internal state ----------------
  bool have_state_{false};
  // State: [N vN E vE D vD phi p theta q psi r]
  Eigen::Matrix<double,12,1> x_;

  // Params
  double m_, g_, Ixx_, Iyy_, Izz_, dt_;
  int N_;
  double Ndes_, Edes_, Ddes_, psi_des_;
  double w_pN_, w_vN_, w_pE_, w_vE_, w_pD_, w_vD_, w_psi_, w_r_, w_u_T_, w_u_tau_, w_term_;
  double T_min_, T_max_, tau_roll_min_, tau_roll_max_, tau_pitch_min_, tau_pitch_max_, tau_yaw_min_, tau_yaw_max_;

  // CasADi
  casadi::Function solver_;
  casadi::DM lbx_, ubx_, lbg_, ubg_;
  casadi::DM lbU_, ubU_;
  casadi::DM warm_; // warm start

  // ROS
  rclcpp::Subscription<RigidBodyPose>::SharedPtr sub_odom_;
  rclcpp::Publisher<InputSetpoint>::SharedPtr pub_uav;
  rclcpp::TimerBase::SharedPtr timer_;

  // ---------------- Callbacks ----------------
  void odom_cb(const RigidBodyPose::SharedPtr msg) {
  // Positions [N,E,D] in NED (already world-frame NED) -> no sign changes
  x_(0) = msg->position[0];  // N
  x_(2) = msg->position[1];  // E
  x_(4) = msg->position[2];  

  // Linear velocities [vN, vE, vD] in NED
  x_(1) = msg->lin_velocity[0];  // vN
  x_(3) = msg->lin_velocity[1];  // vE
  x_(5) = msg->lin_velocity[2];  // vD

  // Euler angles [phi, theta, psi] in radians (already given)
  x_(6)  = msg->orientation[0];  // phi (roll)
  x_(8)  = msg->orientation[1];  // theta (pitch)
  x_(10) = msg->orientation[2];  // psi (yaw)

  // Angular rates [p, q, r] in rad/s (body rates)
  x_(7)  = msg->ang_velocity[0]; // p
  x_(9)  = msg->ang_velocity[1]; // q
  x_(11) = msg->ang_velocity[2]; // r

  have_state_ = true;
}

  void control_step() {
    if (!have_state_) return;

    // Build parameter vector P:
    // P = [x0(12); Ndes; Edes; Ddes; psi_des; weights as needed]
    casadi::DM P = casadi::DM::zeros(12 + 4, 1);
    for (int i=0;i<12;++i) P(i) = x_(i);
    P(12) = Ndes_;
    P(13) = Edes_;
    P(14) = Ddes_;
    P(15) = psi_des_;

    // Warm start
    if (warm_.is_empty()) {
      warm_ = casadi::DM::zeros( (12*(N_+1) + 4*N_) , 1);
      // initialize controls to hover-ish:
      for (int k=0;k<N_;++k) warm_(12*(N_+1) + 4*k + 0) = m_*g_;
    }

    std::map<std::string, casadi::DM> arg, res;
    arg["p"]   = P;
    arg["x0"]  = warm_;
    arg["lbx"] = lbx_;
    arg["ubx"] = ubx_;
    arg["lbg"] = lbg_;
    arg["ubg"] = ubg_;

    try {
      res = solver_(arg);
      warm_ = res.at("x");

      // Extract first control (u0)
      int nx = 12;
      int nu = 4;
      int state_blk = nx*(N_+1);
      double T      = static_cast<double>(warm_(state_blk + 0).scalar());
      double tau_ph = static_cast<double>(warm_(state_blk + 1).scalar());
      double tau_th = static_cast<double>(warm_(state_blk + 2).scalar());
      double tau_ps = static_cast<double>(warm_(state_blk + 3).scalar());

      // Publish Wrench (body frame command)
      InputSetpoint cmd;
      cmd.timestamp = 1;
      cmd.setpoint[0] = tau_ph;
      cmd.setpoint[1] = tau_th;
      cmd.setpoint[2] = T;
      cmd.setpoint[3] = tau_ps;
      cmd.setpoint_type = 2; 
      pub_uav->publish(cmd);

    } catch (const std::exception& e) {
      RCLCPP_ERROR(get_logger(), "MPC solve failed: %s", e.what());
    }
  }

  // ---------------- Build CasADi NLP ----------------
  void build_solver(double phi_max, double theta_max, double psi_max) {
    using namespace casadi;

    int nx = 12, nu = 4;
    SX X = SX::sym("X", nx, N_+1);
    SX U = SX::sym("U", nu, N_);
    SX P = SX::sym("P", 16); // [x0(12); Ndes; Edes; Ddes; psi_des]

    // Helper lambdas
    auto row = [](const SX& M, int r){ return M(casadi::Slice(r,r+1), casadi::Slice()); };

    // Cost & constraints
    SX cost = 0;
    std::vector<SX> g;

    // Initial condition
    g.push_back( X(casadi::Slice(),0) - P(casadi::Slice(0,12)) );

    // Weights
    double w_uT = w_u_T_;
    double w_uTau = w_u_tau_;

    for (int k=0;k<N_;++k) {
      SX xk = X(casadi::Slice(), k);
      SX uk = U(casadi::Slice(), k);

      SX Np=xk(0), vN=xk(1), Ep=xk(2), vE=xk(3), Dp=xk(4), vD=xk(5);
      SX phi=xk(6), p=xk(7), theta=xk(8), q=xk(9), psi=xk(10), r=xk(11);

      SX T  = uk(0);
      SX tau_ph = uk(1);
      SX tau_th = uk(2);
      SX tau_ps = uk(3);

      // Translational accelerations (NED) from body thrust + attitude
      SX aN = (T/m_) * ( sin(phi)*sin(psi) + cos(phi)*sin(theta)*cos(psi) );
      SX aE = (T/m_) * ( cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi) );
      SX aD = g_     - (T/m_) * ( cos(phi)*cos(theta) );

      // Attitude rates
      SX p_dot = tau_ph / Ixx_;
      SX q_dot = tau_th / Iyy_;
      SX r_dot = tau_ps / Izz_;

      // xdot
      SX xdot = SX::vertcat({
        vN, aN,
        vE, aE,
        vD, aD,
        p,  p_dot,
        q,  q_dot,
        r,  r_dot
      });

      // Euler step
      SX xnext = xk + dt_*xdot;
      g.push_back( X(casadi::Slice(),k+1) - xnext );

      // Running cost
      SX pos_cost =
        w_pN_*pow(Np - P(12),2) + w_pE_*pow(Ep - P(13),2) + w_pD_*pow(Dp - P(14),2);
      SX vel_cost =
        w_vN_*pow(vN,2) + w_vE_*pow(vE,2) + w_vD_*pow(vD,2);
      SX yaw_cost = w_psi_*pow(psi - P(15),2) + w_r_*pow(r,2);
      SX u_cost = w_uT*pow(T,2) + w_uTau*(pow(tau_ph,2)+pow(tau_th,2)+pow(tau_ps,2));
      SX soft_att = 0.5*( pow(phi/(M_PI/180.0*50.0),2) + pow(theta/(M_PI/180.0*50.0),2) );

      cost += pos_cost + vel_cost + yaw_cost + u_cost + soft_att;
    }

    // Terminal position penalty
    SX xT = X(casadi::Slice(), N_);
    cost += w_term_*( pow(xT(0)-P(12),2) + pow(xT(2)-P(13),2) + pow(xT(4)-P(14),2) );

    // Decision vector
    SX w = SX::vertcat({ SX::reshape(X, nx*(N_+1), 1), SX::reshape(U, nu*N_, 1) });
    SX gcat = SX::vertcat(g);

    // NLP dict
    SXDict nlp { {"x", w}, {"f", cost}, {"g", gcat}, {"p", P} };

    // Solver opts
    casadi::Dict opts;
    opts["ipopt.print_level"] = 0;
    opts["print_time"] = 0;
    opts["ipopt.sb"] = "yes";
    opts["ipopt.max_iter"] = 80;
    solver_ = nlpsol("solver", "ipopt", nlp, opts);

    // Bounds for w
    int nvars = nx*(N_+1) + nu*N_;
    lbx_ = casadi::DM::zeros(nvars,1);
    ubx_ = casadi::DM::zeros(nvars,1);
    for (int i=0;i<nvars;++i){ lbx_(i) = -casadi::DM::inf(); ubx_(i) = casadi::DM::inf(); }

    // Angle bounds on states across horizon
    for (int k=0;k<=N_;++k) {
      int base = nx*k;
      lbx_(base + 6)  = -phi_max;   ubx_(base + 6)  =  phi_max;    // phi
      lbx_(base + 8)  = -theta_max; ubx_(base + 8)  =  theta_max;  // theta
      lbx_(base + 10) = -psi_max;   ubx_(base + 10) =  psi_max;    // psi
    }

    // Control bounds
    for (int k=0;k<N_;++k) {
      int base = nx*(N_+1) + nu*k;
      lbx_(base+0) = T_min_;       ubx_(base+0) = T_max_;
      lbx_(base+1) = tau_roll_min_;ubx_(base+1) = tau_roll_max_;
      lbx_(base+2) = tau_pitch_min_;ubx_(base+2)= tau_pitch_max_;
      lbx_(base+3) = tau_yaw_min_; ubx_(base+3) = tau_yaw_max_;
    }

    // Equality constraints (dynamics + init): size nx*(N_+1)
    lbg_ = casadi::DM::zeros(nx*(N_+1),1);
    ubg_ = casadi::DM::zeros(nx*(N_+1),1);
  }
};

int main(int argc, char** argv){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MpcUavNode>());
  rclcpp::shutdown();
  return 0;
}