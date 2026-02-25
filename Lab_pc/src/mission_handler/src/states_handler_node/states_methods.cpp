#include "states_handler.hpp"

// unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
// std::default_random_engine generator (seed);
// const double mean = 0.0;
// const double stddev = 3.65;
// std::normal_distribution<double> dist(mean, stddev);

// %%%--------------------------%%%
// --------- PRE-TAKEOFF STATE ----------

void StatesHandler::handlePreTakeOff()
{
  ros2_muavp_interface::msg::InputSetpoint msg;
  // msg.setpoint[2] = _states_info_data.z;

  // computing the delta between two iterations
  _prev = _curr;
  _curr = _clocksis.now();
  auto delta = (_curr - _prev).seconds();
  // std::cout << "delta: " << delta << std::endl;
  // saving this condition, so that the integral part of the pid controler gets
  // accumulated only when the uav has performed the takeoff
  bool basic_cond_takeoff = _actuator_armed_data.armed && _agent_pose_data.position[2] < -1.3;



  // ---------------------------------------------------------------------------

  // Estimating the tension acting on the cables in the Inertial Frame
  Vector3d tension_inert = Vector3d(3);
  tension_inert = tensionEstimation();

  storeTensions(tension_inert(0), tension_inert(1), tension_inert(2));

  // TRIAL: Tensions can be measured, using the measure in the control.
  tensionRodComputation();
  // tension_inert(0) = _tensions.inert_tension[0];
  // tension_inert(1) = _tensions.inert_tension[1];
  // tension_inert(2) = _tensions.inert_tension[2];
  // tension_inert(0) = 0;
  // tension_inert(1) = 0;
  // tension_inert(2) = 0;

  // ---------------------------------------------------------------------------




  double pid_control[3];
  double pid_sp[3] = {0.0, 0.0, 0.0};
  pid_sp[2] = _states_info_data.z;

  PIDGeneratorPosition(delta, basic_cond_takeoff, pid_control, pid_sp, 0.0);

  double pid_tension = 0.0;
  // making sure that the PI controller on tension enters into action only after
  // the quadrotor has reached a certain altitude
  if(abs(pid_sp[2] - _agent_pose_data.position[2]) <= _states_info_data.params[11])
    pid_tension = PretakeoffTensionPid(delta);

  // pid_tension = 0.0;

  double u_over_m = (-g - _tensions.inert_tension[2]/_parameters.params[0] - pid_control[2] - pid_tension)/(cos(_euler_angles[1])*cos(_euler_angles[0])); // *_parameters.params[0] - round(tension_z*10.0/_parameters.params[0])/10.0

  // double a = u_z;

  // --------------- Author: Barbara RATTO ----------------
  // double resultCarrotPoint[2];

  // CarrotPointXY(resultCarrotPoint);
  //
  // _x_des = resultCarrotPoint[0];
  // _y_des = resultCarrotPoint[1];
  // end

  if(_agent_name == "x500_1")
  {
    // msg.setpoint[0] = _states_info_data.x*cos(_states_info_data.params[12]);
    // msg.setpoint[1] = _states_info_data.x*sin(_states_info_data.params[12]);
    // msg.setpoint[3] = _states_info_data.params[12]; // [-PI:PI]

    _x_des = _states_info_data.x*cos(_states_info_data.params[12]) + _parameters.params[5];
    _y_des = _states_info_data.x*sin(_states_info_data.params[12]) + _parameters.params[6];
    _yaw_des = _states_info_data.params[12];
  }
  else if(_agent_name == "x500_2")
  {
    // msg.setpoint[0] = _states_info_data.x*cos(_states_info_data.params[13]);
    // msg.setpoint[1] = _states_info_data.x*sin(_states_info_data.params[13]);
    // msg.setpoint[3] = _states_info_data.params[13]; // [-PI:PI]

    _x_des = _states_info_data.x*cos(_states_info_data.params[13]) + _parameters.params[7];
    _y_des = _states_info_data.x*sin(_states_info_data.params[13]) + _parameters.params[8];
   
    // changes here to force the yaw to -90 degree
    // _yaw_des = _states_info_data.params[13];
        _yaw_des = -1.57; // FORCING THE YAW TO -90 DEGREE, TO AVOID COLLISIONS
  }
  else if(_agent_name == "x500_3")
  {
    // msg.setpoint[0] = _states_info_data.x*cos(_states_info_data.params[14]);
    // msg.setpoint[1] = _states_info_data.x*sin(_states_info_data.params[14]);
    // msg.setpoint[3] = _states_info_data.params[14]; // [-PI:PI]

    _x_des = _states_info_data.x*cos(_states_info_data.params[14]) + _parameters.params[9];
    _y_des = _states_info_data.x*sin(_states_info_data.params[14]) + _parameters.params[10];
        // changes here to force the yaw to -90 degree
    // _yaw_des = _states_info_data.params[13];
      _yaw_des = -1.57; // FORCING THE YAW TO -90 DEGREE, TO AVOID COLLISIONS
  }

  // filling then the torques controls
  torquesControlGenerator(delta, tension_inert, std::clamp(u_over_m*_parameters.params[0], -_parameters.ctrl_params[3], 0.0));
  torquesBodyTransform();

  if(_agent_name == "x500_2")
  {
    std::cout << "-----------------------------------------" << std::endl;
    // std::cout << _states_info_data.ctrl_params[0] << std::endl;
    // std::cout << _states_info_data.z << std::endl;
    // std::cout << _agent_name << " odom z: " << _odom_data.position[2] << std::endl;
    std::cout << _agent_name << " exact z: " << _agent_pose_data.position[2] << std::endl;
    // std::cout << _agent_name << " odom vz: " << _odom_data.vz << std::endl;
    // std::cout << _agent_name << " payload z: " << _payload_pose_data.position[2] << std::endl;
    // std::cout << _agent_name << " pid_x: " << pid_control[0] << std::endl;
    // std::cout << _agent_name << " pid_y: " << pid_control[1] << std::endl;
    // std::cout << _agent_name << " pid_z: " << pid_control[2] << std::endl;
    // std::cout << _agent_name << " odom x: " << _odom_data.position[0] << std::endl;
    // std::cout << _agent_name << " odom y: " << _odom_data.position[1] << std::endl;
    // std::cout << _agent_name << " exact x: " << _agent_pose_data.position[0] << std::endl;
    // std::cout << _agent_name << " exact y: " << _agent_pose_data.position[1] << std::endl;
    // std::cout << _agent_name << " y_des: " << y_des << std::endl;
    // std::cout << _agent_name << " rod_orientation: " << _joint_data.orientation[0]*180.0/3.14 << std::endl;
    // std::cout << _agent_name << " u_z: " << u_over_m << std::endl;
    // std::cout << _agent_name << " a: " << a << std::endl;
  }

  // adapting it to the NED frame
  // msg.setpoint[2] = -a;


  msg.setpoint[0] = _torques_ctrl[0]; // actuators
  msg.setpoint[1] = _torques_ctrl[1]; // actuators

  msg.setpoint[2] = u_over_m*_parameters.params[0]; // actuators

  msg.setpoint[3] = _torques_ctrl[2]; // actuators

  msg.setpoint_type = 2; // actuators
  // msg.setpoint_type = 0;

  // just for MoCap, listening at myself and checking if other nodes
  // modified this flag
  // msg.out_of_mocap = _setpoint_msg.out_of_mocap;

  _setpoint_pub->publish(msg);
}


// %%%--------------------------%%%
// --------- TAKEOFF STATE ----------

void StatesHandler::handleTakeOff()
{
  ros2_muavp_interface::msg::InputSetpoint msg;
  // msg.setpoint[2] = _states_info_data.z;

  // saving this condition, so that the integral part of the pid controler gets
  // accumulated only when the uav has performed the takeoff
  bool basic_cond_takeoff = _actuator_armed_data.armed && _agent_pose_data.position[2] < -0.3;

  // computing the delta between two iterations
  _prev = _curr;
  _curr = _clocksis.now();
  auto delta = (_curr - _prev).seconds();

  // std::cout << "delta: " << delta << std::endl;



  // Estimating the tension acting on the cables in the Inertial Frame
  Vector3d tension_inert = Vector3d(3);
  tension_inert = tensionEstimation();

  storeTensions(tension_inert(0), tension_inert(1), tension_inert(2));

  // TRIAL: Tensions can be measured, using the measure in the control.
  tensionRodComputation();

  // ---------------------------------------------------------------------------




  double pid_control[3];
  double pid_sp[3] = {0.0, 0.0, 0.0};
  pid_sp[2] = _states_info_data.z;

  PIDGeneratorPosition(delta, basic_cond_takeoff, pid_control, pid_sp, 0.0);

  // generating consensus based control
  double r31_z = 0;
  adjustConsensusBasedControl(&r31_z);

  double u_over_m = (-g - pid_control[2] - r31_z)/(cos(_euler_angles[1])*cos(_euler_angles[0])); // - tension_inert(2)/(_parameters.params[0])





  if(_agent_name == "x500_1")
  {

    _x_des = _states_info_data.x*cos(_states_info_data.params[12]) + _parameters.params[5];
    _y_des = _states_info_data.x*sin(_states_info_data.params[12]) + _parameters.params[6];
    _yaw_des = _states_info_data.params[12];
  }
  else if(_agent_name == "x500_2")
  {

    _x_des = _states_info_data.x*cos(_states_info_data.params[13]) + _parameters.params[7];
    _y_des = _states_info_data.x*sin(_states_info_data.params[13]) + _parameters.params[8];
        // changes here to force the yaw to -90 degree
    // _yaw_des = _states_info_data.params[13];
    _yaw_des = -1.57; // FORCING THE YAW TO -90 DEGREE, TO AVOID COLLISIONS
  }
  else if(_agent_name == "x500_3")
  {

    _x_des = _states_info_data.x*cos(_states_info_data.params[14]) + _parameters.params[9];
    _y_des = _states_info_data.x*sin(_states_info_data.params[14]) + _parameters.params[10];
        // changes here to force the yaw to -90 degree
    // _yaw_des = _states_info_data.params[13];
    _yaw_des = -1.57; // FORCING THE YAW TO -90 DEGREE, TO AVOID COLLISIONS
  }

  // filling then the torques controls
  torquesControlGenerator(delta, tension_inert, std::clamp(u_over_m*_parameters.params[0], -_parameters.ctrl_params[3], 0.0));
  torquesBodyTransform();

  // adapting it to the NED frame
  // msg.setpoint[2] = -a;

  if(_agent_name == "x500_2")
  {
    std::cout << "-----------------------------------------" << std::endl;
    // std::cout << _agent_name << " odom x: " << _odom_data.position[0] << std::endl;
    // std::cout << _agent_name << " odom y: " << _odom_data.position[1] << std::endl;
    // std::cout << _agent_name << " tension_inert(0): " << tension_inert(0) << std::endl;
    // std::cout << _agent_name << " tension_inert(1): " << tension_inert(1) << std::endl;
    // std::cout << _agent_name << " tension_est: " << tension_inert << std::endl;
    // std::cout << _agent_name << " tension_measure: " << _tensions.inert_tension << std::endl;
    // std::cout << _agent_name << " tension_inert_ctrl(2): " << floor(tension_inert(2)/(_parameters.params[0])) << std::endl;
    // std::cout << _agent_name << " r31_z: " << r31_z << std::endl;
    // std::cout << _agent_name << " pid_control[2]: " << pid_control[2] << std::endl;
    // std::cout << _agent_name << " phi: " << _euler_angles[0] << std::endl;
    // std::cout << _agent_name << " theta: " << _euler_angles[1] << std::endl;
    // std::cout << _agent_name << " odom z: " << _odom_data.position[2] << std::endl;
    std::cout << _agent_name << " exact z: " << _agent_pose_data.position[2] << std::endl;
    std::cout << _agent_name << " u_z: " << u_over_m*_parameters.params[0] << std::endl;
  }


  msg.setpoint[0] = _torques_ctrl[0]; // actuators
  msg.setpoint[1] = _torques_ctrl[1]; // actuators

  msg.setpoint[2] = u_over_m*_parameters.params[0]; // actuators

  msg.setpoint[3] = _torques_ctrl[2]; // actuators

  msg.setpoint_type = 2; // actuators

  _setpoint_pub->publish(msg);
}


// %%%--------------------------%%%
// --------- MOVE STATE ----------

void StatesHandler::handleMove()
{

}

// %%%--------------------------%%%
// --------- IDLE STATE ----------

void StatesHandler::handleIdle()
{

}



void StatesHandler::handleLand()
{
  ros2_muavp_interface::msg::InputSetpoint msg;

  // computing the delta between two iterations
  _prev = _curr;
  _curr = _clocksis.now();
  auto delta = (_curr - _prev).seconds();

  // std::cout << "delta: " << delta << std::endl;


  // Estimating the tension acting on the cables in the Inertial Frame
  Vector3d tension_inert = Vector3d(3);
  tension_inert = tensionEstimation();

  storeTensions(tension_inert(0), tension_inert(1), tension_inert(2));

  // TRIAL: Tensions can be measured, using the measure in the control.
  tensionRodComputation();
  // tension_inert(0) = _tensions.inert_tension[0];
  // tension_inert(1) = _tensions.inert_tension[1];
  // tension_inert(2) = _tensions.inert_tension[2];
  // tension_inert(0) = 0;
  // tension_inert(1) = 0;
  // tension_inert(2) = 0;

  // ---------------------------------------------------------------------------

  // depending on wether I am in a failed state or not, I get the corresponding
  // value to be compared in the next condition
  double compare_param = 0.0;
  double compare_param_tens = 0.0;
  if(_failure_occured)
  {
    compare_param = _failure_info_data[(int)_failure_action_data.curr_action].params[9];
    compare_param_tens = _failure_info_data[(int)_failure_action_data.curr_action].params[10];
  }
  else
  {
    compare_param = _states_info_data.params[9];
    compare_param_tens = _states_info_data.params[10];
  }

  // decreasing the setpoint
  if(_agent_pose_data.position[2] < compare_param)
  {
    // tensionRodComputation();
    // if I am above a certain altitude, I decrease normally.
    if(_agent_pose_data.position[2] < compare_param - 2.5)
      _msg_land.setpoint[2] += 0.005;
    else if(_tensions.tension_norm > compare_param_tens) // if I pass that altitude, means I am approaching land.
      _msg_land.setpoint[2] += 0.001; // thus I decrease the desired sp slowly, until I feel a certain tension on the cable.
    else // means the payload is landed! approaching ground a bit faster.
      _msg_land.setpoint[2] += 0.01;


    msg.setpoint_type = 2; // actuators
  }
  else
    msg.setpoint_type = 4; // letting the autopilot handle it

  // if I am landing after a failure, setting the desired position as the one
  // passed with the message.
  if(_failure_occured)
  {
    _x_des = _msg_land.setpoint[0];
    _y_des = _msg_land.setpoint[1];
    _yaw_des = _msg_land.setpoint[3];
  }

  double pid_control[3];
  double pid_sp[3] = {0.0, 0.0, 0.0};
  pid_sp[2] = _msg_land.setpoint[2];

  PIDGeneratorPosition(delta, true, pid_control, pid_sp, 0.0);

  // generating consensus based control
  double r31_z = 0;
  adjustConsensusBasedControl(&r31_z);

  double u_over_m = (-g - pid_control[2] - r31_z)/(cos(_euler_angles[1])*cos(_euler_angles[0])); //  - _tensions.inert_tension[2]/_parameters.params[0]


  // filling then the torques controls
  torquesControlGenerator(delta, tension_inert, std::clamp(u_over_m*_parameters.params[0], -_parameters.ctrl_params[3], 0.0));
  torquesBodyTransform();


  if(_agent_name == "x500_2")
  {
    std::cout << "---------------------LAND--------------------" << std::endl;
    // std::cout << _agent_name << " odom x: " << _odom_data.position[0] << std::endl;
    // std::cout << _agent_name << " odom y: " << _odom_data.position[1] << std::endl;
    // std::cout << _agent_name << " phi: " << _euler_angles[0] << std::endl;
    std::cout << _agent_name << " tension_norm: " << _tensions.tension_norm << std::endl;
    // std::cout << _agent_name << " odom z: " << _odom_data.position[2] << std::endl;
    std::cout << _agent_name << " - z_sp: " << _msg_land.setpoint[2] << std::endl;
    std::cout << _agent_name << " - z: " << _agent_pose_data.position[2] << std::endl;
  }


  msg.setpoint[0] = _torques_ctrl[0]; // actuators
  msg.setpoint[1] = _torques_ctrl[1]; // actuators

  msg.setpoint[2] = u_over_m*_parameters.params[0]; // actuators

  msg.setpoint[3] = _torques_ctrl[2]; // actuators

  // msg.setpoint[0] = _msg_land.setpoint[0];
  // msg.setpoint[1] = _msg_land.setpoint[1];
  // msg.setpoint[2] = u_over_m;
  // msg.setpoint[3] = _msg_land.setpoint[3];
  //
  // msg.setpoint_type = 0;

  _setpoint_pub->publish(msg);
}
