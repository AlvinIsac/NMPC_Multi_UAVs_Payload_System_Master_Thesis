#include "states_handler.hpp"

// %%%--------------------------%%%
// --------- TORQUES CONTROL ----------
void StatesHandler::torquesControlGenerator(double delta, Vector3d T_I, double u)
{
  double phi = _euler_angles[0];
  double theta = _euler_angles[1];
  double psi = _euler_angles[2];

  double i_xx = _parameters.ctrl_params[0];
  double i_yy = _parameters.ctrl_params[1];
  double i_zz = _parameters.ctrl_params[2];


  double phi_dot = _euler_rates[0];
  double theta_dot = _euler_rates[1];
  double psi_dot = _euler_rates[2];


  VectorXd T_B = VectorXd(3);

  T_B = tensionInertialToBody(T_I(0), T_I(1), T_I(2));

  double T_x = T_B(0);
  double T_y = T_B(1);
  double T_z = T_B(2);

  if(_agent_name == "x500_2")
  {
    // std::cout << _agent_name << " T_x_b = " << T_x << std::endl;
    // std::cout << _agent_name << " T_y_b = " << T_y << std::endl;
    // std::cout << _agent_name << " T_z_b = " << T_z << std::endl;
  }

  // Coriolis PHI-CONFIG

  double Coriolis_phi = (-cos(theta)*(pow(sin(phi),2)*(1 - i_zz/i_xx)*(1 + pow(tan(theta),2)) + pow(cos(phi),2)*(i_zz/i_xx - 1) - 1) + tan(theta)*sin(theta))*theta_dot*psi_dot - tan(theta)*(pow(sin(phi),2)*(i_zz/i_yy - 1) - 1)*phi_dot*theta_dot -
                        sin(theta)*sin(phi)*cos(phi)*(1 - i_zz/i_yy)*phi_dot*psi_dot - cos(phi)*sin(phi)*(1 - i_zz/i_xx)*pow(theta_dot,2) - sin(phi)*cos(phi)*(i_zz/i_xx - 1)*pow(psi_dot,2);

  double Tensions_phi = (_ancPointsData.r_uav[1]*T_z - _ancPointsData.r_uav[2]*T_y)/i_xx + tan(theta)*sin(phi)*(_ancPointsData.r_uav[2]*T_x - _ancPointsData.r_uav[0]*T_z)/i_yy + tan(theta)*cos(phi)*(_ancPointsData.r_uav[0]*T_y - _ancPointsData.r_uav[1]*T_x)/i_zz;

  double Coriolis_theta = -cos(theta)*(pow(cos(phi),2)*(1 - i_zz/i_yy) + 1)*phi_dot*psi_dot - sin(phi)*sin(theta)*cos(phi)*(1 - i_zz/i_yy)*theta_dot*psi_dot - sin(phi)*cos(phi)*(i_zz/i_yy - 1)*phi_dot*theta_dot -
                          cos(theta)*sin(theta)*pow(cos(phi),2)*(i_zz/i_yy - 1)*pow(psi_dot,2);

  double Tensions_theta = cos(phi)*(_ancPointsData.r_uav[2]*T_x - _ancPointsData.r_uav[0]*T_z)/i_yy - sin(phi)*(_ancPointsData.r_uav[0]*T_y - _ancPointsData.r_uav[1]*T_x)/i_zz;

  double Coriolis_psi = -tan(theta)*(pow(sin(phi),2)*(1 - i_zz/i_yy) - 1)*theta_dot*psi_dot - (pow(sin(phi),2)*(i_zz/i_yy - 1) - 1)*phi_dot*theta_dot/cos(theta) -
                        sin(phi)*cos(phi)*(1 - i_zz/i_yy)*phi_dot*psi_dot - sin(phi)*sin(theta)*cos(phi)*(i_zz/i_yy - 1)*pow(psi_dot,2);

  double Tensions_psi = (sin(phi)/cos(theta))*(_ancPointsData.r_uav[2]*T_x - _ancPointsData.r_uav[0]*T_z)/i_yy + (cos(phi)/cos(theta))*(_ancPointsData.r_uav[0]*T_y - _ancPointsData.r_uav[1]*T_x)/i_zz;


  if(_counter_Dynamics < _m2)
  {
    // std::cout << _agent_name << " - _counter_Dynamics: " << _counter_Dynamics << std::endl;
    myfile4 << std::to_string(Coriolis_phi) + "," + std::to_string(Tensions_phi) + "," + std::to_string(Coriolis_theta) + "," + std::to_string(Tensions_theta) + "," + std::to_string(Coriolis_psi) + "," + std::to_string(Tensions_psi) << std::endl;
    _counter_Dynamics++;
  }
  else
  {
    myfile4.close();
  }

  // getting the bearing angle between the two remaining agents (just one time).
  // the resulting angle is then adjusted to become the desired yaw.
  if(_failure_occured && _first_bearing)
  {
    _yaw_des = bearingAngleComputation();
    _first_bearing = false;
  }

  // TODO: CONSIDER TO SIMPLIFY PID TO CONTROL YAW ONLY, IF ROLL AND PITCH
  // ARE CONTROLLED BY SMC

  // composing the desired values vector
  double pid_sp[3] = {_eq_pt[6], _eq_pt[7], _yaw_des};
  double pid_control[3];
  // calling the control to get the input variables
  PIDGeneratorOrientation(delta, true, pid_sp, pid_control);

  double psi_disturbance = Coriolis_psi + Tensions_psi;

  pid_control[2] -= psi_disturbance;

  // computing the terms deriving from tensions and coriolis terms
  // underlying them as "disturbances", to summarize their effect
  double theta_disturbance = Coriolis_theta + Tensions_theta;
  // theta_disturbance = 0.0;
  // double theta_disturbance = Coriolis_theta;
  double y_disturbance = ((cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi))*u + T_I(1))/_parameters.params[0];
  double phi_disturbance = Coriolis_phi + Tensions_phi;
  // phi_disturbance = 0.0;
  // double phi_disturbance = Coriolis_phi;
  double x_disturbance = ((sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi))*u + T_I(0))/_parameters.params[0];

  VectorXd roll_pitch_ctrl = VectorXd(2);
  // sliding mode control
  roll_pitch_ctrl = slidingModeController(theta_disturbance, y_disturbance, phi_disturbance, x_disturbance, true, delta);

  // assigning them to the control variable that will be transformed back
  // NOTE: The transformation is [r, q, p]' = W_eta*[psi, theta, phi]', hence
  // the torques on phi and psi need to be switched
  _torques_ctrl << roll_pitch_ctrl[0], roll_pitch_ctrl[1], pid_control[2];

  if(_agent_name == "x500_2")
  {
    // std::cout << _agent_name << " tau_phi = " << roll_pitch_ctrl[0] << std::endl;
    // std::cout << _agent_name << " tau_theta = " << roll_pitch_ctrl[1] << std::endl;
    // std::cout << _agent_name << " tau_psi = " << pid_control[2] << std::endl;
  }
}

// %%%--------------------------%%%
// --------- TRANSFORMING THE CONTROL BACK IN BODY FRAME ----------

void StatesHandler::torquesBodyTransform()
{
  // constructing the transformation matrix that links inertial torques to body ones
  // this is a multiplication between the inertia matrix of the UAV and the
  // transformation matrix linking body rates to euler angles rates
  MatrixXd I_uav_W_eta = MatrixXd(3,3);

  I_uav_W_eta <<_I_uav(0,0), 0, -_I_uav(0,0)*sin(_euler_angles[1]),
                0, _I_uav(1,1)*cos(_euler_angles[0]), _I_uav(1,1)*cos(_euler_angles[1])*sin(_euler_angles[0]),
                0, -_I_uav(2,2)*sin(_euler_angles[0]), _I_uav(2,2)*cos(_euler_angles[1])*cos(_euler_angles[0]);

  // tranforming them back
  _torques_ctrl = I_uav_W_eta*_torques_ctrl;
}





// %%%--------------------------%%%
// --------- POSITION GENERATOR FOR THE SLIDING MODE CONTROLLER ----------

VectorXd StatesHandler::positionGeneratorSMC()
{

  VectorXd uav_des_pos = VectorXd(3);

  return uav_des_pos;

}
