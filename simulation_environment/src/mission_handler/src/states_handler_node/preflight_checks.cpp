#include "states_handler.hpp"

// %%%--------------------------%%%
// --------- STORE PARAMETERS ---------

void StatesHandler::storeParameters()
{
  bool pass_cond;

  if(_agent_name == "x500_1")
    pass_cond = _ready_signal_params_data.uav_1_ready;
  else if(_agent_name == "x500_2")
    pass_cond = _ready_signal_params_data.uav_2_ready;
  else if(_agent_name == "x500_3")
    pass_cond = _ready_signal_params_data.uav_3_ready;

  // avoiding to store more times if I need to wait for the others
  // this state can be entered more than one time if I am still waiting for data
  if(!pass_cond && _states_info_data.path != "")
  {
    _parameters = _states_info_data;

    _ancPointsData.r_uav << _parameters.params[2], _parameters.params[3], _parameters.params[4];
    _ancPointsData.r_l1 << _parameters.params[12], _parameters.params[13], _parameters.params[14];
    _ancPointsData.r_l2 << _parameters.params[15], _parameters.params[16], _parameters.params[17];
    _ancPointsData.r_l3 << _parameters.params[18], _parameters.params[19], _parameters.params[20];

    _I_uav << _parameters.ctrl_params[0], 0, 0,
              0, _parameters.ctrl_params[1], 0,
              0, 0, _parameters.ctrl_params[2];

    // if I specified a path where to store the files for data, I open them
    // BUT this condition ensured the opening function will be executed one time,
    // only after the parameters will be stored by previous instructions
    // if(_parameters.path != "")
    openFilesForDataStoring();

    std::cout << "Parameters stored for Agent: " << _agent_name << std::endl;
  }
}


// %%%--------------------------%%%
// --------- OPENING FILES FOR DATA STORING ----------

void StatesHandler::openFilesForDataStoring()
{
  // myfile.open(_parameters.path + _agent_name + "_matlab_data_sim.txt");
  // myfile << "actual_k,actual_lt,residuals,distance_norm,timestamp" << std::endl;
  //
  // myfile2.open(_parameters.path + _agent_name + "_payload_data_matlab_data_sim.txt");
  // myfile2 << "payload_anc_1_x,payload_anc_1_y,payload_anc_1_z,payload_anc_2_x,payload_anc_2_y,payload_anc_2_z,payload_anc_1_vel_x,payload_anc_1_vel_y,payload_anc_1_vel_z,payload_anc_2_vel_x,payload_anc_2_vel_y,payload_anc_2_vel_z,timestamp" << std::endl;

  // myfile3.open(_parameters.path + _agent_name + "_uav_data_matlab_data_sim.txt");
  // myfile3 << "uav_anc_x,uav_anc_y,uav_anc_z,uav_anc_vel_x,uav_anc_vel_y,uav_anc_vel_z,timestamp" << std::endl;

  // myfile4.open(_parameters.path + _agent_name + "_coriolis_tension_data.txt");
  // myfile4 << "C_phi, T_phi, C_theta, T_theta, C_psi, T_psi" << std::endl;

  myfile5.open(_parameters.path + _agent_name + "_state.txt");
  myfile5 << "x, x_dot, y, y_dot, z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot, timestamp" << std::endl; //, tension

  myfile6.open(_parameters.path + _agent_name + "_payload_state.txt");
  myfile6 << "x, x_dot, y, y_dot, z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot, timestamp" << std::endl;

  myfile7.open(_parameters.path + _agent_name + "_desired_state.txt");
  myfile7 << "x, x_dot, y, y_dot, z, z_dot, phi, phi_dot, theta, theta_dot, psi, psi_dot" << std::endl;

  // myfile_T.open(_parameters.path + _agent_name + "_tensions.txt");
  // myfile_T << "T_body_x, T_body_y, T_body_z, T_cable_x, T_cable_y, T_cable_z" << std::endl;

  // myfile_T << "T_sensor_x, T_sensor_y, T_sensor_z, T_computed_x, T_computed_y, T_computed_z" << std::endl;

  // myfile_P.open(_parameters.path + _agent_name + "_path_info.txt");
  // myfile_P << "type, radius, center_x, center_y, start_x, start_y, start_z, end_x, end_y, end_z, length, A" << std::endl;

  std::cout << "Files Opened for Agent: " << _agent_name << std::endl;
}


// %%%--------------------------%%%
// --------- STORE EQ POINT ----------

void StatesHandler::storeControlEquilibriumData()
{
  // opening the file containing the control vector at the equilibrium point
  std::ifstream in_u_eq("src/mission_handler/conf/u_eq.txt", std::ios::in);
  // taking the first number in the file, or the numbers of columns in u_eq
  int numColsUeq(0);
  in_u_eq >> numColsUeq;
  // resizing the store variable
  _u_eq.resize(numColsUeq);
  // and iterating over the file to store each value
  for(int i = 0; i < numColsUeq; i++)
    in_u_eq >> _u_eq(i);

  // std::cout << "Equilibrium Control Vector parsed for Agent: " << _agent_name << std::endl;
  // closing the file finally
  in_u_eq.close();

  // opening the file containing the equilibrium point itself
  std::ifstream in_eq_pt("src/mission_handler/conf/eq_pt.txt", std::ios::in);
  // and doing the same as with u_eq
  int numColsEqPt(0);
  in_eq_pt >> numColsEqPt;
  // resizing the store variable
  _eq_pt.resize(numColsEqPt);
  // iterating over the file to store each value
  for(int j = 0; j < numColsEqPt; j++)
    in_eq_pt >> _eq_pt(j);

  // std::cout << "Equilibrium State Vector stored for Agent: " << _agent_name << std::endl;
  // closing the file finally
  in_eq_pt.close();
}


// %%%--------------------------%%%
// --------- CHECK ON PARAMS ----------

void StatesHandler::checkParamsStored()
{
  if(_parameters.params[0] != 0.0 && _parameters.params[1] != 0.0 && !_failure_info_data.empty() && !_path_info_data.empty())
  {
    _ready_signal_params_data.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    // filling the right variable, depening on the agent accessing this function
    if(_agent_name == "x500_1" && !_ready_signal_params_data.uav_1_ready)
      _ready_signal_params_data.uav_1_ready = true;
    else if(_agent_name == "x500_2" && !_ready_signal_params_data.uav_2_ready)
      _ready_signal_params_data.uav_2_ready = true;
    else if(_agent_name == "x500_3" && !_ready_signal_params_data.uav_3_ready)
      _ready_signal_params_data.uav_3_ready = true;
    _ready_params_pub->publish(_ready_signal_params_data);
  }
  // else
    // std::cout << "necessary conditions unverified for checkParamsStored(), please check" << std::endl;
}
