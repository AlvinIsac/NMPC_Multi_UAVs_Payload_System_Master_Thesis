#include "states_handler.hpp"

// %%%--------------------------%%%
// --------- PID ON POSITION ----------

void StatesHandler::PIDGeneratorPosition(double delta, bool basic_cond_int, double* pid_control, double pid_sp[3], double z_dot_des)
{
  // TODO: Remove this and adapt to new convention
  if((int)_failure_action_data.curr_action == fail_states::LAND_F)
  {
    pid_sp[2] = _msg_land.setpoint[2];
  }


  if(basic_cond_int)
  {
    // _integral_dist_x += delta*(_states_info_data.x - _odom_data.position[0]);
    // _exact_integral_dist_x += delta*(pid_sp[0] - _agent_pose_data.position[0]);
    // _integral_dist_y += delta*(_states_info_data.y - _odom_data.position[1]);
    // _exact_integral_dist_y += delta*(pid_sp[1] - _agent_pose_data.position[1]);
    // _integral_dist_z += delta*(_states_info_data.z - _odom_data.position[2]);
    _exact_integral_dist_z += delta*(pid_sp[2] - _agent_pose_data.position[2]);
  }
  else
  {
    // _integral_dist_x = 0;
    // _exact_integral_dist_x = 0;
    // _integral_dist_y = 0;
    // _exact_integral_dist_y = 0;
    // _integral_dist_z = 0;
    _exact_integral_dist_z = 0;
  }

  // and clamping it between two values to prevent too much accumulation
  // _integral_dist_x = std::clamp(_integral_dist_z, -_states_info_data.ctrl_params[4], _states_info_data.ctrl_params[4]);
  // _exact_integral_dist_x = std::clamp(_exact_integral_dist_x, -_states_info_data.ctrl_params[4], _states_info_data.ctrl_params[4]);
  // _integral_dist_y = std::clamp(_integral_dist_z, -_states_info_data.ctrl_params[9], _states_info_data.ctrl_params[9]);
  // _exact_integral_dist_y = std::clamp(_exact_integral_dist_y, -_states_info_data.ctrl_params[9], _states_info_data.ctrl_params[9]);
  // _integral_dist_z = std::clamp(_integral_dist_z, -_states_info_data.ctrl_params[4], _states_info_data.ctrl_params[4]);

  if(_failure_occured && !_system_recovered)
    _exact_integral_dist_z = std::clamp(_exact_integral_dist_z, -_failure_info_data[(int)_failure_action_data.curr_action].ctrl_params[4], _failure_info_data[(int)_failure_action_data.curr_action].ctrl_params[4]);
  else
    _exact_integral_dist_z = std::clamp(_exact_integral_dist_z, -_states_info_data.ctrl_params[4], _states_info_data.ctrl_params[4]);

  // double proportional_dist_x = std::clamp((_states_info_data.x - _odom_data.position[0]), -_states_info_data.ctrl_params[3], _states_info_data.ctrl_params[3]);
  // double exact_proportional_dist_x = std::clamp((pid_sp[0] - _agent_pose_data.position[0]), -_states_info_data.ctrl_params[3], _states_info_data.ctrl_params[3]);
  // double proportional_dist_y = std::clamp((_states_info_data.y - _odom_data.position[1]), -_states_info_data.ctrl_params[8], _states_info_data.ctrl_params[8]);
  // double exact_proportional_dist_y = std::clamp((pid_sp[1] - _agent_pose_data.position[1]), -_states_info_data.ctrl_params[8], _states_info_data.ctrl_params[8]);
  // double proportional_dist_z = std::clamp((_states_info_data.z - _odom_data.position[2]), -_states_info_data.ctrl_params[3], _states_info_data.ctrl_params[3]);

  double exact_proportional_dist_z(0.0);

  if(_failure_occured && !_system_recovered)
    exact_proportional_dist_z = std::clamp((pid_sp[2] - _agent_pose_data.position[2]), -_failure_info_data[(int)_failure_action_data.curr_action].ctrl_params[3], _failure_info_data[(int)_failure_action_data.curr_action].ctrl_params[3]);
  else
    exact_proportional_dist_z = std::clamp((pid_sp[2] - _agent_pose_data.position[2]), -_states_info_data.ctrl_params[3], _states_info_data.ctrl_params[3]);

  // defining the three PID parts
  // double r1_p_x = - _states_info_data.ctrl_params[0]*proportional_dist_x;
  // double r1_i_x = - _states_info_data.ctrl_params[1]*_integral_dist_x;
  // double r1_d_x = - _states_info_data.ctrl_params[2]*_odom_data.velocity[0];

  // double r1_ep_x = - _states_info_data.ctrl_params[0]*exact_proportional_dist_x;
  // double r1_ei_x = - _states_info_data.ctrl_params[1]*_exact_integral_dist_x;
  // double r1_ed_x = - _states_info_data.ctrl_params[2]*(0 - _agent_pose_data.lin_velocity[0]); // x_dot_des - x_dot

  // double r1_p_y = - _states_info_data.ctrl_params[5]*proportional_dist_y;
  // double r1_i_y = - _states_info_data.ctrl_params[6]*_integral_dist_y;
  // double r1_d_y = - _states_info_data.ctrl_params[7]*_odom_data.velocity[1];

  // double r1_ep_y = - _states_info_data.ctrl_params[5]*exact_proportional_dist_y;
  // double r1_ei_y = - _states_info_data.ctrl_params[6]*_exact_integral_dist_y;
  // double r1_ed_y = - _states_info_data.ctrl_params[7]*(0 - _agent_pose_data.lin_velocity[1]); // y_dot_des - y_dot

  // double r1_p_z = - _states_info_data.ctrl_params[0]*proportional_dist_z;
  // double r1_i_z = - _states_info_data.ctrl_params[1]*_integral_dist_z;
  // double r1_d_z = - _states_info_data.ctrl_params[2]*_odom_data.velocity[2];
  double r1_ep_z(0.0), r1_ei_z(0.0), r1_ed_z(0.0);
  double k_p_z(0.0), k_i_z(0.0), k_d_z(0.0);
  if(_failure_occured && !_system_recovered)
  {
    int fail_state = (int)_failure_action_data.curr_action;
    // std::cout << fail_state << std::endl;

    // designing an adaptive PID control to give more strenght with big errors
    // if(adaptive)
    // {
    //   // k = k_f + (k_i - k_f)*exp(-alpha*(e_z - e_z_ref)), e_z_ref used to shift exponential so at max dist is = 1
    //   // k_p_z = _failure_info_data[fail_state].params[3] + (_failure_info_data[fail_state].ctrl_params[0] - _failure_info_data[fail_state].params[3])*exp(-alpha*(exact_proportional_dist_z + _failure_info_data[fail_state].ctrl_params[3]));
    //   // k_i_z = _failure_info_data[fail_state].params[4] + (_failure_info_data[fail_state].ctrl_params[1] - _failure_info_data[fail_state].params[4])*exp(-alpha*(exact_proportional_dist_z + _failure_info_data[fail_state].ctrl_params[4]));
    //   // k_d_z = ismc_eps + (_failure_info_data[fail_state].ctrl_params[2] - ismc_eps)*exp(-alpha*(exact_proportional_dist_z + _failure_info_data[fail_state].ctrl_params[3]));
    //
    //   k_p_z = _failure_info_data[fail_state].ctrl_params[0];
    //   k_i_z = _failure_info_data[fail_state].ctrl_params[1];
    //   k_d_z = _failure_info_data[fail_state].ctrl_params[2];
    //
    //   // std::cout << _agent_name << "k_p: " << k_p_z << std::endl;
    //   // std::cout << _agent_name << "k_i: " << k_i_z << std::endl;
    //   // std::cout << _agent_name << "k_d: " << k_d_z << std::endl;
    // }
    // else
    // {
    k_p_z = _failure_info_data[fail_state].ctrl_params[0];
    k_i_z = _failure_info_data[fail_state].ctrl_params[1];
    k_d_z = _failure_info_data[fail_state].ctrl_params[2];
    // }
    r1_ep_z = - k_p_z*exact_proportional_dist_z;
    r1_ei_z = - k_i_z*_exact_integral_dist_z;
    r1_ed_z = - k_d_z*(z_dot_des - _agent_pose_data.lin_velocity[2]);

    // Added by THEA
    // if(_states_info_data.curr_state==MOVE)
    // {
    //
    //   double clamp_derivative_term = 1.5;
    //
    //   r1_ed_z= - _states_info_data.ctrl_params[2]*( pid_sp[2] - _agent_pose_data.position[2] - prevError)/delta;
    //
    //   r1_ed_z = std::clamp(r1_ed_z, -clamp_derivative_term, clamp_derivative_term);
    //   prevError = pid_sp[2] - _agent_pose_data.position[2];
    // }
  }
  else
  {
    r1_ep_z = - _states_info_data.ctrl_params[0]*exact_proportional_dist_z;
    r1_ei_z = - _states_info_data.ctrl_params[1]*_exact_integral_dist_z;
    r1_ed_z = - _states_info_data.ctrl_params[2]*(z_dot_des - _agent_pose_data.lin_velocity[2]);
  }

  // std::cout << _agent_name << "r1_ep_z: " << r1_ep_z << std::endl;
  // std::cout << _agent_name << "r1_ei_z: " << r1_ei_z << std::endl;
  // std::cout << _agent_name << "r1_ed_z: " << r1_ed_z << std::endl;

  // and composing the final controller
  // double r1_x = r1_d_x + r1_p_x + r1_i_x;
  // double r11_x = r1_ed_x + r1_ep_x + r1_ei_x;

  // double r1_y = r1_d_y + r1_p_y + r1_i_y;
  // double r11_y = r1_ed_y + r1_ep_y + r1_ei_y;

  // double r1_z = r1_d_z + r1_p_z + r1_i_z;
  double r11_z = r1_ed_z + r1_ep_z + r1_ei_z;

  // pid_control[0] = r11_x;
  // pid_control[1] = r11_y;
  pid_control[2] = r11_z;
}


// %%%--------------------------%%%
// --------- PID ON ORIENTATION (Principally for Recovery Fail State)----------

void StatesHandler::PIDGeneratorOrientation(double delta, bool basic_cond_int, double* angle_sp, double* pid_control)
{
  // rememeber that, given the formation of W_eta, the rates correspondence is reversed
  // double phi_dot = _euler_rates[2];
  // double theta_dot = _euler_rates[1];
  double psi_dot = _euler_rates[2];

  // double phi_error = angle_sp[0] - _euler_angles[0];
  // double theta_error = angle_sp[1] - _euler_angles[1];

  // psi being in the range [0, 2*PI], when computing the error I need to be careful
  // because if the desired angle and the actual one are on the over the limits,
  // the error, that would be small, will result big (taking the error inside
  // the interval). Circular error arithmetic is then used: this formula handles
  // angles that wrap around the maximum value to the minimum value.
  // fmod computes the reminder of the division between the two inputs.
  // the error then is wrapped in the interval [-PI, PI], where yaw belongs to.
  double psi_error = (fmod(fmod(angle_sp[2] - _euler_angles[2], 2*PI) + 3*PI, 2*PI) - PI);

  if (_agent_name == "x500_2")
  {
    // std::cout << _agent_name << " - e_phi: " << phi_error << std::endl;
    // std::cout << _agent_name << " - e_theta: " << theta_error << std::endl;
    // std::cout << _agent_name << " - e_psi: " << psi_error << std::endl;
  }
  if(basic_cond_int)
  {
    // _integral_dist_phi += delta*(phi_error);
    // _integral_dist_theta += delta*(theta_error);
    _integral_dist_psi += delta*(psi_error);
  }
  else
  {
    // _integral_dist_phi = 0;
    // _integral_dist_theta = 0;
    _integral_dist_psi = 0;
  }

  double clamp_p_psi = 0.0;
  double clamp_i_psi = 0.0;
  double gain_p_psi = 0.0;
  double gain_i_psi = 0.0;
  double gain_d_psi = 0.0;


  if(_failure_occured)
  {
    // saving the fail state to cast not everytime
    int fail_state = (int)_failure_action_data.curr_action;
    // assigning the variables
    clamp_p_psi = _failure_info_data[fail_state].ctrl_params[20];
    clamp_i_psi = _failure_info_data[fail_state].ctrl_params[21];

    gain_p_psi = _failure_info_data[fail_state].ctrl_params[17];
    gain_i_psi = _failure_info_data[fail_state].ctrl_params[18];
    gain_d_psi = _failure_info_data[fail_state].ctrl_params[19];
  }
  else
  {
    clamp_p_psi = _states_info_data.ctrl_params[20];
    clamp_i_psi = _states_info_data.ctrl_params[21];

    gain_p_psi = _states_info_data.ctrl_params[17];
    gain_i_psi = _states_info_data.ctrl_params[18];
    gain_d_psi = _states_info_data.ctrl_params[19];
  }

  // _integral_dist_phi = std::clamp(_integral_dist_phi, -ismc_k5, ismc_k5);
  // _integral_dist_theta = std::clamp(_integral_dist_theta, -ismc_lambda_2, ismc_lambda_2);
  _integral_dist_psi = std::clamp(_integral_dist_psi, -clamp_i_psi, clamp_i_psi);

  // double proportional_dist_phi = std::clamp(phi_error, -ismc_k4, ismc_k4);
  // double proportional_dist_theta = std::clamp(theta_error, -ismc_lambda_1, ismc_lambda_1);
  double proportional_dist_psi = std::clamp(psi_error, -clamp_p_psi, clamp_p_psi);

  // double r_p_phi = ismc_k1*proportional_dist_phi;
  // double r_i_phi = ismc_k2*_integral_dist_phi;
  // double r_d_phi = ismc_k3*(0 - phi_dot);
  //
  // double r_p_theta = ismc_k6*proportional_dist_theta;
  // double r_i_theta = ismc_k7*_integral_dist_theta;
  // double r_d_theta = ismc_k8*(0 - theta_dot);

  double r_p_psi = gain_p_psi*proportional_dist_psi;
  double r_i_psi = gain_i_psi*_integral_dist_psi;
  double r_d_psi = gain_d_psi*(0 - psi_dot);

  // and composing the final controller
  // double r_phi = r_p_phi + r_i_phi + r_d_phi;
  // double r_theta = r_p_theta + r_i_theta + r_d_theta;
  double r_psi = r_p_psi + r_i_psi + r_d_psi;

  // pid_control[0] = r_phi;
  // pid_control[1] = r_theta;
  pid_control[2] = r_psi;
  // std::cout << _agent_name << "r_psi: " << r_psi << std::endl;
}



// ----------------- Author: Barbara RATTO --------------------------
double StatesHandler::PretakeoffTensionPid(double delta)
{
    // _states_info_data.params[17] = 3.926852;


    // errore tensione
    double tensionError = _states_info_data.params[17] - _tensions.tension_norm;

    // -----------------------------------------------------------------------------------------

    // parte proporzionale tensione
    double tensionPropOutput = _states_info_data.ctrl_params[30] * tensionError;

    // parte integrativa tensione
    _integralTension += tensionError * delta;
    double tensionIntegrOutput = _states_info_data.ctrl_params[31] * _integralTension;

    // parte derivativa tensione
    // double derivativeTension = ( tensionError - _prevTensionError ) / delta;
    // double tensionDerivOutput = _states_info_data.ctrl_params[32] * derivativeTension;

    _prevTensionError = tensionError;

    // totale PID tensione
    double totalTensionOutputPID = tensionPropOutput + tensionIntegrOutput; // + tensionDerivOutput

    // -----------------------------------------------------------------------------------------

    // DEBUG
    if(_agent_name == "x500_2")
    {
      std::cout << _agent_name << " tensionError: " << tensionError << std::endl;
      // std::cout << _agent_name << " _agent_pose_data.position[2]: " << _agent_pose_data.position[2] << std::endl;
      // std::cout << _agent_name << " _tensions.tension_norm: " << _tensions.tension_norm << std::endl;
      // std::cout << _agent_name << " totalOutputPID: " << totalTensionOutputPID << std::endl;
    }

    return totalTensionOutputPID;


}


// %%%--------------------------%%%
// --------- CONSENSUS BASED ----------

void StatesHandler::adjustConsensusBasedControl(double *r31_z)
{
  if(_failure_occured && !_system_recovered)
  {
    // depending on who is alive, the parameter gain changes
    if(_detachUAVData.next_agent_alive)
      *r31_z = - (_failure_info_data[(int)_failure_action_data.curr_action].ctrl_params[5]*(_next_agent_pose_data.position[2] - _agent_pose_data.position[2]));
    else if(_detachUAVData.prev_agent_alive)
      *r31_z = - (_failure_info_data[(int)_failure_action_data.curr_action].ctrl_params[6]*(_prev_agent_pose_data.position[2] - _agent_pose_data.position[2]));
  }
  else
    *r31_z = - (_states_info_data.ctrl_params[5]*(_next_agent_pose_data.position[2] - _agent_pose_data.position[2]) + _states_info_data.ctrl_params[6]*(_prev_agent_pose_data.position[2] - _agent_pose_data.position[2]));
}


// %%%--------------------------%%%
// --------- SLIDING MODE CONTROL ON UNDERACTUATED DYNAMICS ----------
// --------- (ROLL, Y) (PITCH, X) ----------

VectorXd StatesHandler::slidingModeController(double theta_disturbance, double y_disturbance, double phi_disturbance, double x_disturbance, double integral_cond, double delta)
{

  if(_failure_occured && _first_smc)
  {
    _x_des = _agent_pose_data.position[0];
    _y_des = _agent_pose_data.position[1];
    // _msg_smc.setpoint[2] = _odom_data.velocity[0];
    // _msg_smc.setpoint[3] = _odom_data.velocity[1];
    // _msg_smc.setpoint[2] = _desired_velocity[0];
    // _msg_smc.setpoint[3] = _desired_velocity[1];

    // STOP RECOVERY
    _desired_velocity[0] = 0.0;
    _desired_velocity[1] = 0.0;


    // _msg_smc.setpoint[2] = _agent_pose_data.lin_velocity[0];
    // _msg_smc.setpoint[3] = _agent_pose_data.lin_velocity[1];
    // _msg_smc.setpoint[2] = 0;
    // _msg_smc.setpoint[3] = 0;


    _first_smc = false;
  }

  double phi = _euler_angles[0];
  double theta = _euler_angles[1];
  double psi = _euler_angles[2];

  MatrixXd R = MatrixXd(3,3);
  MatrixXd R_inv = MatrixXd(3,3);

  R << cos(psi)*cos(theta), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi),
      cos(theta)*sin(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi),
      -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta);

  // orthogonal matrix, hence the inverse is its transpose
  R_inv = R.transpose();

  // double e_vy = std::clamp(_desired_velocity[1] - _agent_pose_data.lin_velocity[1], -0.25, 0.25); // _odom_data.velocity[1]
  //
  // double e_vx = std::clamp(_desired_velocity[0] - _agent_pose_data.lin_velocity[0], -0.25, 0.25); // _odom_data.velocity[0]

  double e_vy = _desired_velocity[1] - _agent_pose_data.lin_velocity[1]; // _odom_data.velocity[1]

  double e_vx = _desired_velocity[0] - _agent_pose_data.lin_velocity[0]; // _odom_data.velocity[0]

  VectorXd vel_error_i = VectorXd(3);
  VectorXd vel_error_b = VectorXd(3);

  vel_error_i << e_vx, e_vy, 0;
  vel_error_b = R_inv*vel_error_i;




  VectorXd roll_pitch_ctrl = VectorXd(2);
  // this vector contains the values of the desired states for the couple (roll,y), in order:
  // y_dot_dot_des, y_dot_des, y_des, roll_dot_dot_des, roll_dot_des, roll_des
  VectorXd roll_des_states = VectorXd(6);
  roll_des_states << 0, _desired_velocity[1], _y_des, 0, 0, 0;
  // this one instead for the couple (pitch, x), in order:
  // x_dot_dot_des, x_dot_des, x_des, pitch_dot_dot_des, pitch_dot_des, pitch_des
  VectorXd pitch_des_states = VectorXd(6);;
  pitch_des_states << 0, _desired_velocity[0], _x_des, 0, 0,_eq_pt(7);

  VectorXd des_states = VectorXd(12);

  // TODO: SET Z_DES TO PRINT ON FILE
  des_states << pitch_des_states(2), pitch_des_states(1), roll_des_states(2), roll_des_states(1), _z_des, _desired_velocity(2), roll_des_states(5), roll_des_states(4), pitch_des_states(5), pitch_des_states(4), _yaw_des, 0.0;
  printStatesOnFIle(des_states);

  // defining the errors and clamping them
  // double e_ay = std::clamp(roll_des_states(0) - y_disturbance, -5.5, 5.5);
  // double e_y = std::clamp(roll_des_states(2) - _agent_pose_data.position[1], -1.5, 1.5); //_odom_data.position[1]
  // double e_phi_dot_dot = std::clamp(roll_des_states(3) - phi_disturbance, -5.5, 5.5);
  // double e_phi_dot = std::clamp(roll_des_states(4) - _euler_rates(0), -30.8, 30.8);
  // double e_phi = std::clamp(roll_des_states(5) - _euler_angles[0], -10.5, 10.5);
  //
  // double e_ax = std::clamp(pitch_des_states(0) - x_disturbance, -5.5, 5.5);
  // double e_x = std::clamp(pitch_des_states(2) - _agent_pose_data.position[0], -1.5, 1.5); //_odom_data.position[0]
  // double e_theta_dot_dot = std::clamp(pitch_des_states(3) - theta_disturbance, -5.5, 5.5);
  // double e_theta_dot = std::clamp(pitch_des_states(4) - _euler_rates(1), -30.8, 30.8);
  // double e_theta = std::clamp(pitch_des_states(5) - _euler_angles[1], -10.5, 10.5);

  double e_ay = roll_des_states(0) - y_disturbance;
  double e_y = roll_des_states(2) - _agent_pose_data.position[1]; //_odom_data.position[1]
  double e_phi_dot_dot = roll_des_states(3) - phi_disturbance;
  double e_phi_dot = roll_des_states(4) - _euler_rates(0);
  double e_phi = roll_des_states(5) - _euler_angles[0];

  double e_ax = pitch_des_states(0) - x_disturbance;
  double e_x = pitch_des_states(2) - _agent_pose_data.position[0]; //_odom_data.position[0]
  double e_theta_dot_dot = pitch_des_states(3) - theta_disturbance;
  double e_theta_dot = pitch_des_states(4) - _euler_rates(1);
  double e_theta = pitch_des_states(5) - _euler_angles[1];

  // i =  inertial, b = body (frame)
  VectorXd pos_error_i = VectorXd(3);
  VectorXd pos_error_b = VectorXd(3);

  VectorXd acc_error_i = VectorXd(3);
  VectorXd acc_error_b = VectorXd(3);

  // needing them in body frame cause the translational equations are expressed
  // in NED, while the rotational ones are extrinsically expressed in NED by the
  // rate transformation matrix, but they are in Body frame.

  pos_error_i << e_x, e_y, 0;
  acc_error_i << e_ax, e_ay, 0;

  pos_error_b = R_inv*pos_error_i;
  acc_error_b = R_inv*acc_error_i;

  // switching sign cause the rotation is counterclockwise
  pos_error_b(0) = - pos_error_b(0);
  acc_error_b(0) = - acc_error_b(0);
  vel_error_b(0) = - vel_error_b(0);

  if(_failure_occured)
  {
    vel_error_b(0) = std::clamp(vel_error_b(0), -0.3, 0.3);
    vel_error_b(1) = std::clamp(vel_error_b(1), -0.3, 0.3);
    // pos_error_b(0) = std::clamp(pos_error_b(0), -0.3, 0.3);
    // pos_error_b(1) = std::clamp(pos_error_b(1), -0.3, 0.3);
  }

  // computing the integral parts
  if(integral_cond)
  {
    _smc_integral.e_x += delta*pos_error_b(0);
    _smc_integral.e_vx += delta*vel_error_b(0);
    _smc_integral.e_y += delta*pos_error_b(1);
    _smc_integral.e_vy += delta*vel_error_b(1);
    _smc_integral.e_phi += delta*e_phi;
    _smc_integral.e_phi_dot += delta*e_phi_dot;
    _smc_integral.e_theta += delta*e_theta;
    _smc_integral.e_theta_dot += delta*e_theta_dot;
    // _smc_integral.e_x = std::clamp(_smc_integral.e_x + delta*pos_error_b(0), -0.3, 0.3);
    // _smc_integral.e_vx = std::clamp(_smc_integral.e_vx + delta*vel_error_b(0), -0.5, 0.5);
    // _smc_integral.e_y = std::clamp(_smc_integral.e_y + delta*pos_error_b(1), -0.3, 0.3);
    // _smc_integral.e_vy = std::clamp(_smc_integral.e_vy + delta*vel_error_b(0), -0.5, 0.5);
    // _smc_integral.e_phi = std::clamp(_smc_integral.e_phi + delta*e_phi, -0.2, 0.2);
    // _smc_integral.e_phi_dot = std::clamp(_smc_integral.e_phi_dot + delta*e_phi_dot, -0.2, 0.2);
    // _smc_integral.e_theta = std::clamp(_smc_integral.e_theta + delta*e_theta, -0.2, 0.2);
    // _smc_integral.e_theta_dot = std::clamp(_smc_integral.e_theta_dot + delta*e_theta_dot, -0.2, 0.2);
  }
  else
  {
    _smc_integral.e_x = 0.0;
    _smc_integral.e_vx = 0.0;
    _smc_integral.e_y = 0.0;
    _smc_integral.e_vy = 0.0;
    _smc_integral.e_phi = 0.0;
    _smc_integral.e_phi_dot = 0.0;
    _smc_integral.e_theta = 0.0;
    _smc_integral.e_theta_dot = 0.0;
  }


  // defining the gains for the ISMC
  double ismc_k1 = 0.0;
  double ismc_k2 = 0.0;
  double ismc_k3 = 0.0;
  double ismc_k4 = 0.0;
  double ismc_k5 = 0.0;
  double ismc_k6 = 0.0;
  double ismc_k7 = 0.0;
  double ismc_k8 = 0.0;
  double ismc_ki_1 = 0.0;
  double ismc_ki_2 = 0.0;
  double ismc_ki_3 = 0.0;
  double ismc_ki_4 = 0.0;
  double ismc_ki_5 = 0.0;
  double ismc_ki_6 = 0.0;
  double ismc_ki_7 = 0.0;
  double ismc_ki_8 = 0.0;

  double ismc_eps = 0.0;
  double ismc_rho = 0.0;
  double ismc_lambda_1 = 0.0;
  double ismc_lambda_2 = 0.0;

  if(_failure_occured)
  {
    int fail_state = 0;

    if(_recovery_stop)
    {
      fail_state = (int)fail_states::STOP;

      _smc_integral.e_x = 0.0;
      _smc_integral.e_vx = 0.0;
      _smc_integral.e_y = 0.0;
      _smc_integral.e_vy = 0.0;
      _smc_integral.e_phi = 0.0;
      _smc_integral.e_phi_dot = 0.0;
      _smc_integral.e_theta = 0.0;
      _smc_integral.e_theta_dot = 0.0;
    }
    else
      fail_state = (int)_failure_action_data.curr_action;

    ismc_k1 = _failure_info_data[fail_state].ctrl_params[7];
    ismc_k2 = _failure_info_data[fail_state].ctrl_params[8];
    ismc_k3 = _failure_info_data[fail_state].ctrl_params[9];
    ismc_k4 = _failure_info_data[fail_state].ctrl_params[10];
    ismc_k5 = _failure_info_data[fail_state].ctrl_params[11];
    ismc_k6 = _failure_info_data[fail_state].ctrl_params[12];
    ismc_k7 = _failure_info_data[fail_state].ctrl_params[13];
    ismc_k8 = _failure_info_data[fail_state].ctrl_params[14];

    ismc_ki_1 = _failure_info_data[fail_state].ctrl_params[22];
    ismc_ki_2 = _failure_info_data[fail_state].ctrl_params[23];
    ismc_ki_3 = _failure_info_data[fail_state].ctrl_params[24];
    ismc_ki_4 = _failure_info_data[fail_state].ctrl_params[25];
    ismc_ki_5 = _failure_info_data[fail_state].ctrl_params[26];
    ismc_ki_6 = _failure_info_data[fail_state].ctrl_params[27];
    ismc_ki_7 = _failure_info_data[fail_state].ctrl_params[28];
    ismc_ki_8 = _failure_info_data[fail_state].ctrl_params[29];

    ismc_eps = _failure_info_data[fail_state].params[5];
    ismc_rho = _failure_info_data[fail_state].params[6];
    ismc_lambda_1 = _failure_info_data[fail_state].ctrl_params[15];
    ismc_lambda_2 = _failure_info_data[fail_state].ctrl_params[16];
  }
  else
  {
    ismc_k1 = _states_info_data.ctrl_params[7];
    ismc_k2 = _states_info_data.ctrl_params[8];
    ismc_k3 = _states_info_data.ctrl_params[9];
    ismc_k4 = _states_info_data.ctrl_params[10];
    ismc_k5 = _states_info_data.ctrl_params[11];
    ismc_k6 = _states_info_data.ctrl_params[12];
    ismc_k7 = _states_info_data.ctrl_params[13];
    ismc_k8 = _states_info_data.ctrl_params[14];

    ismc_ki_1 = _states_info_data.ctrl_params[22];
    ismc_ki_2 = _states_info_data.ctrl_params[23];
    ismc_ki_3 = _states_info_data.ctrl_params[24];
    ismc_ki_4 = _states_info_data.ctrl_params[25];
    ismc_ki_5 = _states_info_data.ctrl_params[26];
    ismc_ki_6 = _states_info_data.ctrl_params[27];
    ismc_ki_7 = _states_info_data.ctrl_params[28];
    ismc_ki_8 = _states_info_data.ctrl_params[29];

    ismc_eps = _states_info_data.params[15];
    ismc_rho = _states_info_data.params[16];
    ismc_lambda_1 = _states_info_data.ctrl_params[15];
    ismc_lambda_2 = _states_info_data.ctrl_params[16];

  }



  // creating the sliding surface for phi control
  double sliding_phi = ismc_k1*vel_error_b(1) + ismc_k2*pos_error_b(1) +
                      ismc_k3*e_phi_dot + ismc_k4*e_phi;

  // creating the sliding surface for theta control
  double sliding_theta = ismc_k5*vel_error_b(0) + ismc_k6*pos_error_b(0) +
                      ismc_k7*e_theta_dot + ismc_k8*e_theta;

  // and the integral parts
  double sliding_phi_integral = ismc_ki_1*_smc_integral.e_vy + ismc_ki_2*_smc_integral.e_y +
                      ismc_ki_3*_smc_integral.e_phi_dot + ismc_ki_4*_smc_integral.e_phi;

  double sliding_theta_integral = ismc_ki_5*_smc_integral.e_vx + ismc_ki_6*_smc_integral.e_x +
                      ismc_ki_7*_smc_integral.e_theta_dot + ismc_ki_8*_smc_integral.e_theta;


  // discontinuous control component for phi control
  // double phi_disc = 1.0*(sliding_phi/(std::abs(sliding_phi) + ismc_lambda_1)) + ismc_lambda_2*sliding_phi;
  //
  // // discontinuous control component for theta control
  // double theta_disc = 1.0*(sliding_theta/(std::abs(sliding_theta) + ismc_lambda_1)) + ismc_lambda_2*sliding_theta;

  // discontinuous control component for phi control
  double phi_disc = ismc_lambda_1*(pow(sliding_phi,3)/(std::abs(pow(sliding_phi,3)) + ismc_eps*exp(-ismc_rho*std::abs(sliding_phi)))) + ismc_lambda_2*sliding_phi;

  // discontinuous control component for theta control
  double theta_disc = ismc_lambda_1*(pow(sliding_theta,3)/(std::abs(pow(sliding_theta,3)) + ismc_eps*exp(-ismc_rho*std::abs(sliding_theta)))) + ismc_lambda_2*sliding_theta;
  //
  // // discontinuous integral
  // double phi_disc_integral = 1.0*(sliding_phi_integral/(std::abs(sliding_phi_integral) + ismc_lambda_1)) + ismc_lambda_2*sliding_phi_integral;
  //
  // double theta_disc_integral = 1.0*(sliding_theta_integral/(std::abs(sliding_theta_integral) + ismc_lambda_1)) + ismc_lambda_2*sliding_theta_integral;


  // discontinuous integral
  double phi_disc_integral = ismc_lambda_1*(pow(sliding_phi_integral,3)/(std::abs(pow(sliding_phi_integral,3)) + ismc_eps*exp(-ismc_rho*std::abs(sliding_phi_integral)))) + ismc_lambda_2*sliding_phi_integral;

  double theta_disc_integral = ismc_lambda_1*(pow(sliding_theta_integral,3)/(std::abs(pow(sliding_theta_integral,3)) + ismc_eps*exp(-ismc_rho*std::abs(sliding_theta_integral)))) + ismc_lambda_2*sliding_theta_integral;

  // creating the contribution for the equivalent part containing the derivative of the rotation matrix
  VectorXd body_rates = VectorXd(3);
  body_rates << _odom_data.angular_velocity[0], _odom_data.angular_velocity[1], _odom_data.angular_velocity[2];


  MatrixXd S = MatrixXd(3,3);
  S << 0, -_odom_data.angular_velocity[2], _odom_data.angular_velocity[1],
      _odom_data.angular_velocity[2], 0, -_odom_data.angular_velocity[0],
       -_odom_data.angular_velocity[1], _odom_data.angular_velocity[0], 0;


  MatrixXd R_dot = MatrixXd(3,3);
  R_dot = S*R;


  VectorXd pos_error_b_R_dot = VectorXd(3);
  VectorXd vel_error_b_R_dot = VectorXd(3);

  pos_error_b_R_dot = R_dot.transpose()*pos_error_i;
  vel_error_b_R_dot = R_dot.transpose()*vel_error_i;

  // for phi
  double phi_eqv_R_dot = vel_error_b_R_dot(1)*ismc_k1/ismc_k3 +
                    pos_error_b_R_dot(1)*ismc_k2/ismc_k3;

  // and for theta
  double theta_eqv_R_dot  = vel_error_b_R_dot(0)*ismc_k5/ismc_k7 +
                     pos_error_b_R_dot(0)*ismc_k6/ismc_k7;



  // generating now the equivalent control component for phi
  double phi_eqv = acc_error_b(1)*ismc_k1/ismc_k3 + vel_error_b(1)*ismc_k2/ismc_k3 +
                    e_phi_dot_dot + e_phi_dot*ismc_k4/ismc_k3;

  // and finally the equivalent control component for theta
  double theta_eqv = acc_error_b(0)*ismc_k5/ismc_k7 + vel_error_b(0)*ismc_k6/ismc_k7 +
                      e_theta_dot_dot + e_theta_dot*ismc_k8/ismc_k7;

  // and their integral part
  double phi_eqv_integral = vel_error_b(1)*ismc_ki_1/ismc_k3 + pos_error_b(1)*ismc_ki_2/ismc_k3 +
                            e_phi_dot*ismc_ki_3/ismc_k3 + e_phi*ismc_ki_4/ismc_k3;

  double theta_eqv_integral = vel_error_b(0)*ismc_ki_5/ismc_k7 + vel_error_b(0)*ismc_ki_6/ismc_k7 +
                              e_theta_dot*ismc_ki_7/ismc_k7 + e_theta_dot*ismc_ki_8/ismc_k7;


  // std::cout << _agent_name << " e_vx_b: " << vel_error_b(0) << std::endl;
  // std::cout << _agent_name << " e_vy_b: " << vel_error_b(1) << std::endl;
  // std::cout << _agent_name << " e_x_b: " << pos_error_b(0) << std::endl;
  // std::cout << _agent_name << " e_y_b: " << pos_error_b(1) << std::endl;

  // std::cout << _agent_name << " e_x_b: " << pos_error_b(0) << std::endl;
  // std::cout << _agent_name << " e_y_b: " << pos_error_b(1) << std::endl;

  if(true) // _agent_name == "x500_2"
  {
    std::cout << _agent_name << " e_x_b: " << pos_error_b(0) << std::endl;
    std::cout << _agent_name << " e_y_b: " << pos_error_b(1) << std::endl;
    // std::cout << _agent_name << " phi_eqv_R_dot: " << phi_eqv_R_dot << std::endl;
    // std::cout << _agent_name << " theta_eqv_R_dot: " << theta_eqv_R_dot << std::endl;
    // std::cout << _agent_name << " smc_sign_phi: " << 1.0*(sliding_phi/(std::abs(sliding_phi) + ismc_lambda_1)) + 1.0*(sliding_phi_integral/(std::abs(sliding_phi_integral) + ismc_lambda_1)) << std::endl;
    // std::cout << _agent_name << " smc_sign_theta: " << 1.0*(sliding_theta/(std::abs(sliding_theta) + ismc_lambda_1)) + 1.0*(sliding_theta_integral/(std::abs(sliding_theta_integral) + ismc_lambda_1)) << std::endl;
    // std::cout << _agent_name << " smc_cubic_phi: " << 1.0*(pow(sliding_phi,3)/(std::abs(pow(sliding_phi,3)) + 0.1*exp(-30*std::abs(sliding_phi)))) + 1.0*(pow(sliding_phi_integral,3)/(std::abs(pow(sliding_phi_integral,3)) + 0.1*exp(-30*std::abs(sliding_phi_integral)))) << std::endl;
    // std::cout << _agent_name << " smc_cubic_theta: " << 1.0*(pow(sliding_theta,3)/(std::abs(pow(sliding_theta,3)) + 0.1*exp(-30*std::abs(sliding_theta)))) + 1.0*(pow(sliding_theta_integral,3)/(std::abs(pow(sliding_theta_integral,3)) + 0.0*exp(-30*std::abs(sliding_theta_integral)))) << std::endl;
    std::cout << _agent_name << " e_vx_b: " << vel_error_b(0) << std::endl;
    std::cout << _agent_name << " e_vy_b: " << vel_error_b(1) << std::endl;
    // std::cout << _agent_name << " e_ax_b: " << acc_error_b(0) << std::endl;
    // std::cout << _agent_name << " e_ay_b: " << acc_error_b(1) << std::endl;

    // std::cout << _agent_name << " phi_disc_integral: " << phi_disc_integral << std::endl;
    // std::cout << _agent_name << " theta_disc_integral: " << theta_disc_integral << std::endl;
    // std::cout << _agent_name << " phi_eqv_integral: " << phi_eqv_integral << std::endl;
    // std::cout << _agent_name << " theta_eqv_integral: " << theta_eqv_integral << std::endl;

    // std::cout << _agent_name << " y_disturbance: " << y_disturbance << std::endl;
    // std::cout << _agent_name << " x_disturbance: " << x_disturbance << std::endl;
    // std::cout << _agent_name << " phi_disc: " << phi_disc + phi_disc_integral << std::endl;
    // std::cout << _agent_name << " phi_eqv: " << phi_eqv << std::endl;
    // std::cout << _agent_name << " theta_disc: " << theta_disc + theta_disc_integral << std::endl;
    // std::cout << _agent_name << " theta_eqv: " << theta_eqv << std::endl;
    // std::cout << _agent_name << " e_theta: " << e_theta*360.0/PI << std::endl;
    // std::cout << _agent_name << " e_phi: " << e_phi*360.0/PI << std::endl;
    // std::cout << _agent_name << " theta: " << _euler_angles[1]*360.0/PI << std::endl;
    // std::cout << _agent_name << " phi: " << _euler_angles[0]*360.0/PI << std::endl;
    // std::cout << _agent_name << " e_phi_dot: " << e_phi_dot*360.0/PI << std::endl;
    // std::cout << _agent_name << " e_theta_dot: " << e_theta_dot*360.0/PI << std::endl;
    // std::cout << _agent_name << "e_y_acc: " << (roll_des_states(0) - y_disturbance)*ismc_k1/ismc_k3 << std::endl;
    // std::cout << _agent_name << "e_y_vel: " << (roll_des_states(1) - _odom_data.velocity[1])*ismc_k2/ismc_k3 << std::endl;
    // std::cout << _agent_name << "e_phi_acc: " << roll_des_states(3) - phi_disturbance << std::endl;
    // std::cout << _agent_name << "e_phi_vel: " << (roll_des_states(4) - _euler_rates(2))*ismc_k4/ismc_k3 << std::endl;

    // std::cout << _agent_name << "e_theta_acc: " << pitch_des_states(3) - theta_disturbance << std::endl;
    // std::cout << _agent_name << "e_theta_vel: " << (pitch_des_states(4) - _euler_rates(1))*ismc_k8/ismc_k7 << std::endl;
    // std::cout << _agent_name << "e_x_vel: " << (pitch_des_states(1) - _odom_data.velocity[0])*ismc_k6/ismc_k7 << std::endl;
  }
  // filling the control vector
  roll_pitch_ctrl << (phi_eqv + phi_disc) + (phi_eqv_integral + phi_disc_integral) + phi_eqv_R_dot, (theta_eqv + theta_disc) + (theta_eqv_integral + theta_disc_integral) + theta_eqv_R_dot;
  // roll_pitch_ctrl << (phi_eqv + phi_disc),  (theta_eqv + theta_disc);

  return roll_pitch_ctrl;
}
