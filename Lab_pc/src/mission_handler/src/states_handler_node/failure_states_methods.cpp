#include "states_handler.hpp"

// %%%--------------------------%%%
// --------- FAILURE STOP STATE ----------

void StatesHandler::stopAndKeepPosition()
{
  // saving the position at the time of the first entrance
  if(_first_stop)
  {
    _msg_stop.setpoint[0] = _agent_pose_data.position[0] + 2.0*cos(_euler_angles[2]);
    _msg_stop.setpoint[1] = _agent_pose_data.position[1] + 2.0*sin(_euler_angles[2]);
    // _msg_stop.setpoint[2] = _agent_pose_data.position[2];
    _msg_stop.setpoint[2] = 0.0; // killing it -- CHANGE THIS! SIMILAR TO LAND_F
    _msg_stop.setpoint[3] = _euler_angles[2]; // I can enter here only if I am the failed one.
    // std::cout << _agent_name << "x: " << _msg_stop.setpoint[0] << std::endl;
    // std::cout << _agent_name << "y: " << _msg_stop.setpoint[1] << std::endl;
    _first_stop = false;
  }

  // fulfilling the message to be sent to Offboard-RTPS node, with the same
  // desired position at each iteration
  ros2_muavp_interface::msg::InputSetpoint msg;

  msg.setpoint[0] = _msg_stop.setpoint[0];
  msg.setpoint[1] = _msg_stop.setpoint[1];
  msg.setpoint[2] = _msg_stop.setpoint[2];
  msg.setpoint[3] = _msg_stop.setpoint[3];

  if(_agent_pose_data.position[2] < _failure_info_data[fail_states::LAND_F].params[0])
    msg.setpoint_type = 1;
  else
    msg.setpoint_type = 4;

  // _setpoint_pub->publish(msg);
}


// %%%--------------------------%%%
// --------- FAILURE RECOVERY STATE ----------

void StatesHandler::detachmentControl()
{

    // auto one_clock = _clocksis.now();

    // computing the delta between two iterations
    _prev = _curr;
    _curr = _clocksis.now();
    auto delta = (_curr.nanoseconds() - _prev.nanoseconds())*0.0000000001;


    double pid_control[3];
    double pid_sp[3] = {0.0, 0.0, 0.0};
    pid_sp[2] = _states_info_data.z;
    _z_des = _states_info_data.z;


    PIDGeneratorPosition(delta, true, pid_control, pid_sp, 0.0);

    double r31_z = 0;
    adjustConsensusBasedControl(&r31_z);

    // Estimating the tension acting on the cables in the Inertial Frame
    Vector3d tension_inert = Vector3d(3);
    tension_inert = tensionEstimation();

    storeTensions(tension_inert(0), tension_inert(1), tension_inert(2));

    // TRIAL: Tensions can be measured, using the measure in the control.
    // tension_inert(0) = _tensions.inert_tension[0];
    // tension_inert(1) = _tensions.inert_tension[1];
    // tension_inert(2) = _tensions.inert_tension[2];

    double u_over_m = (-g - pid_control[2] - r31_z); //  // *_parameters.params[0] - round(tension_z*10.0/_parameters.params[0])/10.0

    if(!_recovery_arrived)
      u_over_m -= (tension_inert(2)/_parameters.params[0]);
    else
    {
      u_over_m -= round(tension_inert(2)/_parameters.params[0])/1.5;

      if(!_changed_pid_params)
      {
        _failure_info_data[(int)_failure_action_data.curr_action].ctrl_params[0] += 3.0;
        _failure_info_data[(int)_failure_action_data.curr_action].ctrl_params[1] += 3.0;
        _failure_info_data[(int)_failure_action_data.curr_action].ctrl_params[2] += 3.0;
        _changed_pid_params = true;
      }
    }
      // u_over_m -= round(tension_inert(2)/_parameters.params[0]);

    u_over_m /= (cos(_euler_angles[1])*cos(_euler_angles[0]));

    // filling then the torques controls
    torquesControlGenerator(delta, tension_inert, std::clamp(u_over_m*_parameters.params[0], -_parameters.ctrl_params[3], 0.0));

    if(_agent_name == "x500_2")
    {
      // std::cout << _agent_name << " - u_over_m: "<< u_over_m << std::endl;
      // std::cout << _agent_name << " - u: "<< -u_over_m*_parameters.params[0] << std::endl;
      // std::cout << _agent_name << " - pid: "<< -pid_control[2]*_parameters.params[0]/(cos(_euler_angles[1])*cos(_euler_angles[0])) << std::endl;
      // std::cout << _agent_name << " - cb: "<< -(r31_z/_parameters.params[0])/(cos(_euler_angles[1])*cos(_euler_angles[0])) << std::endl;
      // std::cout << _agent_name << " - e_z: "<< _states_info_data.z - _agent_pose_data.position[2] << std::endl;
      // std::cout << _agent_name << " - tau_p_b: "<< _torques_ctrl[0] << std::endl;
      // std::cout << _agent_name << " - tau_q: "<< _torques_ctrl[1] << std::endl;
      // std::cout << _agent_name << " - tau_r: "<< _torques_ctrl[2] << std::endl;
      // std::cout << " ---------------------------------------------- "<< std::endl;
      // std::cout << _agent_name << " - _a: "<< _a << std::endl;
      // std::cout << _agent_name << " - _states_info_data.ctrl_params[10]: "<< _states_info_data.ctrl_params[10] << std::endl;

      // std::cout << _agent_name << " - cb: "<< (r31_z/_parameters.params[0])*_parameters.params[0]/(cos(_euler_angles[1]*_euler_angles[0])) << std::endl;
      // std::cout << _agent_name << " - tension_u: "<< - tension_z/(cos(_euler_angles[1])*cos(_euler_angles[0])) << std::endl;
      // std::cout << _agent_name << " - tension_joint: "<< - _tensions.inert_tension[2]/(cos(_euler_angles[1])*cos(_euler_angles[0])) << std::endl;
      // std::cout << _agent_name << " - tension_x: "<< tension_x << std::endl;
      // std::cout << _agent_name << " - tension_y: "<< tension_y << std::endl;
    }

    torquesBodyTransform();

    ros2_muavp_interface::msg::InputSetpoint msg;

    if(_first_stop)
    {
      _msg_stop.setpoint[0] = _agent_pose_data.position[0] + 2.0*cos(_euler_angles[2]);
      _msg_stop.setpoint[1] = _agent_pose_data.position[1] + 2.0*sin(_euler_angles[2]);
      _msg_stop.setpoint[2] = _agent_pose_data.position[2];
      _first_stop = false;
    }

    // msg.setpoint[0] = _msg_stop.setpoint[0]; //position
    // msg.setpoint[1] = _msg_stop.setpoint[1]; //position
    // msg.setpoint[2] = _msg_stop.setpoint[2];

    msg.setpoint[0] = _torques_ctrl[0]; // actuators
    msg.setpoint[1] = _torques_ctrl[1]; // actuators

    // msg.setpoint[0] = _desired_velocity(0); //velocity
    // msg.setpoint[1] = _desired_velocity(1); // velocity

    // msg.setpoint[2] = u_over_m;
    // just for print purposes
    _thrust = -u_over_m*_parameters.params[0];
    msg.setpoint[2] = u_over_m*_parameters.params[0]; // actuators
    // msg.setpoint[2] = u_over_m; // position/velocity
    // msg.setpoint[3] = _yaw_des; // position/velocity
    msg.setpoint[3] = _torques_ctrl[2]; // actuators


    if(_agent_name == "x500_2")
    {
      // std::cout << _agent_name << " - u_over_m: "<< u_over_m << std::endl;
      // std::cout << _agent_name << " - u: "<< -u_over_m << std::endl;
      // std::cout << _agent_name << " - pid: "<< -(pid_control[2]/_parameters.params[0])*_parameters.params[0]/(cos(_euler_angles[1])*cos(_euler_angles[0])) << std::endl;
      // std::cout << _agent_name << " - cb: "<< -(r31_z/_parameters.params[0])/(cos(_euler_angles[1])*cos(_euler_angles[0])) << std::endl;
      // std::cout << _agent_name << " - e_z: "<< _states_info_data.z - _agent_pose_data.position[2] << std::endl;
      // std::cout << _agent_name << " - tau_p: "<< _torques_ctrl[2] << std::endl;
      // std::cout << _agent_name << " - tau_q: "<< _torques_ctrl[1] << std::endl;
      // std::cout << _agent_name << " - tau_r: "<< _torques_ctrl[0] << std::endl;
      std::cout << " ---------------------------------------------- "<< std::endl;
      // std::cout << _agent_name << " - _a: "<< _a << std::endl;
      // std::cout << _agent_name << " - _states_info_data.ctrl_params[10]: "<< _states_info_data.ctrl_params[10] << std::endl;

      // std::cout << _agent_name << " - cb: "<< (r31_z/_parameters.params[0])*_parameters.params[0]/(cos(_euler_angles[1]*_euler_angles[0])) << std::endl;
      // std::cout << _agent_name << " - tension_u: "<< - tension_z/(cos(_euler_angles[1])*cos(_euler_angles[0])) << std::endl;
      // std::cout << _agent_name << " - tension_joint: "<< - _tensions.inert_tension[2]/(cos(_euler_angles[1])*cos(_euler_angles[0])) << std::endl;
      // std::cout << _agent_name << " - tension_x: "<< tension_x << std::endl;
      // std::cout << _agent_name << " - tension_y: "<< tension_y << std::endl;
    }

    // msg.setpoint_type = 3; // velocity
    // msg.setpoint_type = 0; // position
    msg.setpoint_type = 2; // actuators
    // msg.setpoint_type = 1;

    // _setpoint_pub->publish(msg);

    // checking the execution time of this function
    // auto two_clock = _clocksis.now();
    // auto delta_clock = (two_clock.nanoseconds() - one_clock.nanoseconds())*0.0000000001;
    // std::cout << _agent_name << " - delta_exec: "<< delta_clock << std::endl;
}


// %%%--------------------------%%%
// --------- ASSIGNING THE POSITION OF THE SURVIVED AGENT ----------

void StatesHandler::survivedAgentPoseAssignment()
{
  VectorXd surv_uav_position = VectorXd(2);
  // depending on the survived agent, the assigned position changes
  if(_detachUAVData.prev_agent_alive)
  {
    _detachUAVData.surv_position << _prev_agent_pose_data.position[0], _prev_agent_pose_data.position[1], _prev_agent_pose_data.position[2];
    _detachUAVData.surv_lin_vel << _prev_agent_pose_data.lin_velocity[0], _prev_agent_pose_data.lin_velocity[1], _prev_agent_pose_data.lin_velocity[2];

    _detachUAVData.surv_orientation << _pa_euler_angles[0], _pa_euler_angles[1], _pa_euler_angles[2];
  }
  else if(_detachUAVData.next_agent_alive)
  {
    _detachUAVData.surv_position << _next_agent_pose_data.position[0], _next_agent_pose_data.position[1], _next_agent_pose_data.position[2];
    _detachUAVData.surv_lin_vel << _next_agent_pose_data.lin_velocity[0], _next_agent_pose_data.lin_velocity[1], _next_agent_pose_data.lin_velocity[2];

    _detachUAVData.surv_orientation << _na_euler_angles[0], _na_euler_angles[1], _na_euler_angles[2];
  }

  // if(_detachUAVData.prev_agent_alive)
  //   _detachUAVData.surv_position <<  _pa_odom_data.x,  _pa_odom_data.y,  _pa_odom_data.z;
  // else if(_detachUAVData.next_agent_alive)
  //   _detachUAVData.surv_position << _na_odom_data.x, _na_odom_data.y, _na_odom_data.z;

}
