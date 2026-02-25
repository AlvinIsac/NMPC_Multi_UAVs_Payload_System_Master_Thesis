#include "Parser.h"
#define PI 3.141592653589



Parser::Parser()
{

}

// -----------------------------------------------------------------------------
// function loading the file
bool Parser::loadFile(std::string file)
{
  try
  {
    cfg.readFile(file.c_str());
  }
  catch(const FileIOException &fioex)
  {
    std::cerr << "I/O error while reading file." << std::endl;
    return(EXIT_FAILURE);
  }
  catch(const ParseException &pex)
  {
    std::cerr << "Parse error at " << pex.getFile() << ":" << pex.getLine()
              << " - " << pex.getError() << std::endl;
    return(EXIT_FAILURE);
  }

  return true;
}

// -----------------------------------------------------------------------------
// function parsing the tasks file
bool Parser::parse()
{

    // Get the store name.
    try
    {
      std::string name = cfg.lookup("name");
      std::cout << "Store name: " << name << std::endl;
    }
    catch(const SettingNotFoundException &nfex)
    {
      std::cerr << "No 'name' setting in configuration file." << std::endl;
    }

    const Setting& root = cfg.getRoot();

    // iterating all over the tasks to check if they are present
    // parameters
    try
    {
      const Setting &pre_flight_parameters = root["tasks"]["parameters"];
      // and if they do, parse the task
      parseParameters(pre_flight_parameters[0]);
    }
    catch(const SettingNotFoundException &nfex)
    {
      std::cerr << "Parameters not found, please fix" << std::endl;
      return(EXIT_FAILURE);
    }

    // pre takeoff
    try
    {
      const Setting &tasks_pre_takeoff = root["tasks"]["pre_takeoff"];
      // and if they do, parse the task
      parsePreTakeOff(tasks_pre_takeoff[0]);
    }
    catch(const SettingNotFoundException &nfex)
    {
      std::cerr << "Pre Takeoff task not found, please fix" << std::endl;
      return(EXIT_FAILURE);
    }

    // takeoff
    try
    {
      const Setting &tasks_takeoff = root["tasks"]["takeoff"];
      parseTakeOff(tasks_takeoff[0]);
    }
    catch(const SettingNotFoundException &nfex)
    {
      std::cerr << "Takeoff task not found, please fix" << std::endl;
      return(EXIT_FAILURE);
    }

    // rotate
    // try
    // {
    //   const Setting &tasks_rotate = root["tasks"]["rotate"];
    //   parseRotate(tasks_rotate[0]);
    // }
    // catch(const SettingNotFoundException &nfex)
    // {
    //   std::cerr << "Rotate task not found, please fix" << std::endl;
    //   return(EXIT_FAILURE);
    // }

    // move
    try
    {
      const Setting &tasks_move = root["tasks"]["move"];
      parseMove(tasks_move[0]);
    }
    catch(const SettingNotFoundException &nfex)
    {
      std::cerr << "Move task not found, please fix" << std::endl;
      return(EXIT_FAILURE);
    }

    // land
    try
    {
      const Setting &tasks_land = root["tasks"]["land"];
      parseLand(tasks_land[0]);
    }
    catch(const SettingNotFoundException &nfex)
    {
      std::cerr << "Land task not found, please fix" << std::endl;
      return(EXIT_FAILURE);
    }

    return true;
}

// -----------------------------------------------------------------------------
// function parsing the parameters for the path in move task
bool Parser::parsePath()
{

    // Get the store name.
    try
    {
      std::string name = cfg.lookup("name");
      std::cout << "Store name: " << name << std::endl;
    }
    catch(const SettingNotFoundException &nfex)
    {
      std::cerr << "No 'name' setting in configuration file." << std::endl;
    }

    const Setting& root = cfg.getRoot();

    // iterating all over the tasks to check if they are present
    // parameters
    try
    {
      const Setting &path_parameters = root["paths"]["move"];
      // and if they do, parse the task
      parsePathMove(path_parameters[0]);
    }
    catch(const SettingNotFoundException &nfex)
    {
      std::cerr << "Trajetory details for Move Task not found, please fix" << std::endl;
      return(EXIT_FAILURE);
    }


    return true;
}

// -----------------------------------------------------------------------------
// function parsing the failure parameters file
bool Parser::parseFailure()
{

    // Get the store name.
    try
    {
      std::string name = cfg.lookup("name");
      std::cout << "Store name: " << name << std::endl;
    }
    catch(const SettingNotFoundException &nfex)
    {
      std::cerr << "No 'name' setting in configuration file." << std::endl;
    }

    const Setting& root = cfg.getRoot();

    // iterating all over the tasks to check if they are present
    // parameters
    try
    {
      const Setting &recovery_parameters = root["failure"]["recovery"];
      // and if they do, parse the task
      parseRecovery(recovery_parameters[0]);
    }
    catch(const SettingNotFoundException &nfex)
    {
      std::cerr << "Recovery not found, please fix" << std::endl;
      return(EXIT_FAILURE);
    }

    try
    {
      const Setting &stop_parameters = root["failure"]["stop"];
      // and if they do, parse the task
      parseStop(stop_parameters[0]);
    }
    catch(const SettingNotFoundException &nfex)
    {
      std::cerr << "Stop not found, please fix" << std::endl;
      return(EXIT_FAILURE);
    }

    // land
    try
    {
      const Setting &tasks_land = root["failure"]["land"];
      parseLandFailure(tasks_land[0]);
    }
    catch(const SettingNotFoundException &nfex)
    {
      std::cerr << "Land-Failure task not found, please fix" << std::endl;
      return(EXIT_FAILURE);
    }


    return true;
}

// -----------------------------------------------------------------------------
// function parsing the parameters for post-failure tasks
bool Parser::parsePostFailure()
{
  // Get the store name.
  try
  {
    std::string name = cfg.lookup("name");
    std::cout << "Store name: " << name << std::endl;
  }
  catch(const SettingNotFoundException &nfex)
  {
    std::cerr << "No 'name' setting in configuration file." << std::endl;
  }

  const Setting& root = cfg.getRoot();

  // idle
  try
  {
    const Setting &tasks_idle = root["tasks_pf"]["idle"];
    parseIdlePF(tasks_idle[0]);
  }
  catch(const SettingNotFoundException &nfex)
  {
    std::cerr << "Idle task after failure not found, please fix" << std::endl;
    return(EXIT_FAILURE);
  }

  return true;

}

// -----------------------------------------------------------------------------
// getting the list of parsed tasks
std::deque<ros2_muavp_interface::msg::StatesInfo> Parser::getTaskListParsed()
{
  return _taskListParsed;
}

// -----------------------------------------------------------------------------
// getting the list of parsed path for move task
std::deque<ros2_muavp_interface::srv::PathInfo::Request> Parser::getPathListParsed()
{
  return _movePathListParsed;
}

// -----------------------------------------------------------------------------
// getting the list of parsed failure tasks
std::deque<ros2_muavp_interface::srv::FailureInfo::Request> Parser::getFailureListParsed()
{
  return _failureListParsed;
}

// -----------------------------------------------------------------------------
// getting the list of parsed post-failure tasks
std::deque<ros2_muavp_interface::msg::StatesInfo> Parser::getPostFailureListParsed()
{
  return _postFailureListParsed;
}



// -----------------------------------------------------------------------------
// parsing recovery task
bool Parser::parseRecovery(const Setting &recovery_parameters)
{
  std::cout << "Parsing Recovery Failure State" << std::endl;
  double k_1(0.0), l_t_1(0.0), b_1(0.0);
  double k_2(0.0), l_t_2(0.0), b_2(0.0);
  double k_3(0.0), l_t_3(0.0), b_3(0.0);
  double dist_thld(0.0), increment_gain(0.0);
  double pid_p_z(0.0), pid_i_z(0.0), pid_d_z(0.0);
  // double pid_p_z_f(0.0), pid_i_z_f(0.0), pid_d_z_f(0.0);
  double pid_p_clamp(0.0), pid_i_clamp(0.0);
  double cb_gain_prev(0.0), cb_gain_next(0.0);

  // double pid_p_phi(0.0), pid_i_phi(0.0), pid_d_phi(0.0);
  // double pid_p_phi_clamp(0.0), pid_i_phi_clamp(0.0);
  //
  // double pid_p_theta(0.0), pid_i_theta(0.0), pid_d_theta(0.0);
  // double pid_p_theta_clamp(0.0), pid_i_theta_clamp(0.0);

  double pid_p_psi(0.0), pid_i_psi(0.0), pid_d_psi(0.0);
  double pid_p_psi_clamp(0.0), pid_i_psi_clamp(0.0);

  double smc_k1(0.0), smc_k2(0.0), smc_k3(0.0), smc_k4(0.0);
  double smc_k5(0.0), smc_k6(0.0), smc_k7(0.0), smc_k8(0.0);
  double smc_k2_f(0.0), smc_k6_f(0.0), epsilon(0.0), rho(0.0);
  double smc_i_k1(0.0), smc_i_k2(0.0), smc_i_k3(0.0), smc_i_k4(0.0);
  double smc_i_k5(0.0), smc_i_k6(0.0), smc_i_k7(0.0), smc_i_k8(0.0);
  double smc_ksign(0.0), smc_kdisc(0.0);

  if(!recovery_parameters.lookupValue("k_1", k_1))
  {
    std::cerr << "Rod's elastic coefficient not found for Recovery Failure State, please add it by k_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("l_t_1", l_t_1))
  {
    std::cerr << "Rod's min length not found for Recovery Failure State, please add it by l_t_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("b_1", b_1))
  {
    std::cerr << "Rod's damping coefficient not found for Recovery Failure State, please add it by b_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("k_2", k_2))
  {
    std::cerr << "Rod's elastic coefficient not found for Recovery Failure State, please add it by k_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("l_t_2", l_t_2))
  {
    std::cerr << "Rod's min length not found for Recovery Failure State, please add it by l_t_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("b_2", b_2))
  {
    std::cerr << "Rod's damping coefficient not found for Recovery Failure State, please add it by b_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }
  if(!recovery_parameters.lookupValue("k_3", k_3))
  {
    std::cerr << "Rod's elastic coefficient not found for Recovery Failure State, please add it by k_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("l_t_3", l_t_3))
  {
    std::cerr << "Rod's min length not found for Recovery Failure State, please add it by l_t_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("b_3", b_3))
  {
    std::cerr << "Rod's damping coefficient not found for Recovery Failure State, please add it by b_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("pid_p_z", pid_p_z))
  {
    std::cerr << "Quadrotor's PID proportional gain not found for Recovery Failure State, please add it by pid_p_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("pid_i_z", pid_i_z))
  {
    std::cerr << "Quadrotor's PID integral gain not found for Recovery Failure State, please add it by pid_i_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("pid_d_z", pid_d_z))
  {
    std::cerr << "Quadrotor's PID derivative gain not found for Recovery Failure State, please add it by pid_d_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  // if(!recovery_parameters.lookupValue("pid_p_z_f", pid_p_z_f))
  // {
  //   std::cerr << "Quadrotor's PID proportional gain not found for Recovery Failure State, please add it by pid_p_z_f = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!recovery_parameters.lookupValue("pid_i_z_f", pid_i_z_f))
  // {
  //   std::cerr << "Quadrotor's PID integral gain not found for Recovery Failure State, please add it by pid_i_z_f = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!recovery_parameters.lookupValue("pid_d_z_f", pid_d_z_f))
  // {
  //   std::cerr << "Quadrotor's PID derivative gain not found for Recovery Failure State, please add it by pid_d_z_f = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!recovery_parameters.lookupValue("alpha", alpha))
  // {
  //   std::cerr << "Quadrotor's Adaptive PID decay rate not found for Recovery Failure State, please add it by alpha = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }

  if(!recovery_parameters.lookupValue("epsilon", epsilon))
  {
    std::cerr << "Quadrotor's Exponential gain for sign function (SMC) not found for Recovery Failure State, please add it by epsilon = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("rho", rho))
  {
    std::cerr << "Quadrotor's Exponential decay rate for sign function (SMC) not found for Recovery Failure State, please add it by rho = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("dist_thld", dist_thld))
  {
    std::cerr << "Quadrotor's Threshold Distance not found for Recovery Failure State, please add it by move_dist_thld = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("increment_gain", increment_gain))
  {
    std::cerr << "Quadrotor's Increment Gain not found for Recovery Failure State, please add it by increment_gain = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("pid_p_clamp", pid_p_clamp))
  {
    std::cerr << "Quadrotor's PID proportional clamp limit not found for Recovery Failure State, please add it by pid_p_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("pid_i_clamp", pid_i_clamp))
  {
    std::cerr << "Quadrotor's PID integrative clamp limit not found for Recovery Failure State, please add it by pid_i_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("cb_gain_prev", cb_gain_prev))
  {
    std::cerr << "Quadrotor's Consensus Based gain for previous UAV info not found for Recovery Failure State, please add it by cb_gain_prev = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("cb_gain_next", cb_gain_next))
  {
    std::cerr << "Quadrotor's Consensus Based gain for previous UAV info not found for Recovery Failure State, please add it by cb_gain_next = value" << std::endl;
    return(EXIT_FAILURE);
  }

  // if(!recovery_parameters.lookupValue("pid_p_phi", pid_p_phi))
  // {
  //   std::cerr << "Quadrotor's PID proportional gain not found for Recovery Failure State, please add it by pid_p_phi = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!recovery_parameters.lookupValue("pid_i_phi", pid_i_phi))
  // {
  //   std::cerr << "Quadrotor's PID integral gain not found for Recovery Failure State, please add it by pid_i_phi = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!recovery_parameters.lookupValue("pid_d_phi", pid_d_phi))
  // {
  //   std::cerr << "Quadrotor's PID derivative gain not found for Recovery Failure State, please add it by pid_d_phi = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!recovery_parameters.lookupValue("pid_p_phi_clamp", pid_p_phi_clamp))
  // {
  //   std::cerr << "Quadrotor's PID proportional clamp limit not found for Recovery Failure State, please add it by pid_p_phi_clamp = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!recovery_parameters.lookupValue("pid_i_phi_clamp", pid_i_phi_clamp))
  // {
  //   std::cerr << "Quadrotor's PID integrative clamp limit not found for Recovery Failure State, please add it by pid_i_phi_clamp = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!recovery_parameters.lookupValue("pid_p_theta", pid_p_theta))
  // {
  //   std::cerr << "Quadrotor's PID proportional gain not found for Recovery Failure State, please add it by pid_p_theta = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!recovery_parameters.lookupValue("pid_i_theta", pid_i_theta))
  // {
  //   std::cerr << "Quadrotor's PID integral gain not found for Recovery Failure State, please add it by pid_i_theta = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!recovery_parameters.lookupValue("pid_d_theta", pid_d_theta))
  // {
  //   std::cerr << "Quadrotor's PID derivative gain not found for Recovery Failure State, please add it by pid_d_theta = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!recovery_parameters.lookupValue("pid_p_theta_clamp", pid_p_theta_clamp))
  // {
  //   std::cerr << "Quadrotor's PID proportional clamp limit not found for Recovery Failure State, please add it by pid_p_theta_clamp = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!recovery_parameters.lookupValue("pid_i_theta_clamp", pid_i_theta_clamp))
  // {
  //   std::cerr << "Quadrotor's PID integrative clamp limit not found for Recovery Failure State, please add it by pid_i_theta_clamp = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }

  if(!recovery_parameters.lookupValue("pid_p_psi", pid_p_psi))
  {
    std::cerr << "Quadrotor's PID proportional gain not found for Recovery Failure State, please add it by pid_p_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("pid_i_psi", pid_i_psi))
  {
    std::cerr << "Quadrotor's PID integral gain not found for Recovery Failure State, please add it by pid_i_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("pid_d_psi", pid_d_psi))
  {
    std::cerr << "Quadrotor's PID derivative gain not found for Recovery Failure State, please add it by pid_d_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("pid_p_psi_clamp", pid_p_psi_clamp))
  {
    std::cerr << "Quadrotor's PID proportional clamp limit not found for Recovery Failure State, please add it by pid_p_psi_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("pid_i_psi_clamp", pid_i_psi_clamp))
  {
    std::cerr << "Quadrotor's PID integrative clamp limit not found for Recovery Failure State, please add it by pid_i_psi_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("smc_k1", smc_k1))
  {
    std::cerr << "Quadrotor's SMC k1 Gain not found for Recovery Failure State, please add it by smc_k1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("smc_k2", smc_k2))
  {
    std::cerr << "Quadrotor's SMC k2 Gain not found for Recovery Failure State, please add it by smc_k2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("smc_k2_f", smc_k2_f))
  {
    std::cerr << "Quadrotor's SMC k2 Final Gain not found for Recovery Failure State, please add it by smc_k2_f = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("smc_k3", smc_k3))
  {
    std::cerr << "Quadrotor's SMC k3 Gain not found for Recovery Failure State, please add it by smc_k3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("smc_k4", smc_k4))
  {
    std::cerr << "Quadrotor's SMC k4 Gain not found for Recovery Failure State, please add it by smc_k4 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("smc_k5", smc_k5))
  {
    std::cerr << "Quadrotor's SMC k5 Gain not found for Recovery Failure State, please add it by smc_k5 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("smc_k6", smc_k6))
  {
    std::cerr << "Quadrotor's SMC k6 Gain not found for Recovery Failure State, please add it by smc_k6 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("smc_k6_f", smc_k6_f))
  {
    std::cerr << "Quadrotor's SMC k6 Final Gain not found for Recovery Failure State, please add it by smc_k6_f = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("smc_k7", smc_k7))
  {
    std::cerr << "Quadrotor's SMC k7 Gain not found for Recovery Failure State, please add it by smc_k7 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("smc_k8", smc_k8))
  {
    std::cerr << "Quadrotor's SMC k8 Gain not found for Recovery Failure State, please add it by smc_k8 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("smc_i_k1", smc_i_k1))
  {
    std::cerr << "Quadrotor's SMC k1 Integral Gain not found for Recovery Failure State, please add it by smc_i_k1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("smc_i_k2", smc_i_k2))
  {
    std::cerr << "Quadrotor's SMC k2 Integral Gain not found for Recovery Failure State, please add it by smc_i_k2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("smc_i_k3", smc_i_k3))
  {
    std::cerr << "Quadrotor's SMC k3 Integral Gain not found for Recovery Failure State, please add it by smc_i_k3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("smc_i_k4", smc_i_k4))
  {
    std::cerr << "Quadrotor's SMC k4 integral Gain not found for Recovery Failure State, please add it by smc_i_k4 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("smc_i_k5", smc_i_k5))
  {
    std::cerr << "Quadrotor's SMC k5 Integral Gain not found for Recovery Failure State, please add it by smc_i_k5 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("smc_i_k6", smc_i_k6))
  {
    std::cerr << "Quadrotor's SMC k6 Integral Gain not found for Recovery Failure State, please add it by smc_i_k6 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("smc_i_k7", smc_i_k7))
  {
    std::cerr << "Quadrotor's SMC k7 Integral Gain not found for Recovery Failure State, please add it by smc_i_k7 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("smc_i_k8", smc_i_k8))
  {
    std::cerr << "Quadrotor's SMC k8 Integral Gain not found for Recovery Failure State, please add it by smc_i_k8 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  // if(!recovery_parameters.lookupValue("smc_delta", smc_delta))
  // {
  //   std::cerr << "Quadrotor's SMC delta Gain not found for Recovery Failure State, please add it by smc_delta = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }

  if(!recovery_parameters.lookupValue("smc_ksign", smc_ksign))
  {
    std::cerr << "Quadrotor's SMC Gain for sign function not found for Recovery Failure State, please add it by smc_ksign = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!recovery_parameters.lookupValue("smc_kdisc", smc_kdisc))
  {
    std::cerr << "Quadrotor's SMC Discontinuous Gain not found for Recovery Failure State, please add it by smc_kdisc = value" << std::endl;
    return(EXIT_FAILURE);
  }

  std::cout << "Recovery Failure Task parsed successfully" << std::endl;

  ros2_muavp_interface::srv::FailureInfo::Request task;

  task.curr_state = fail_states::RECOVERY;
  task.params[0] = k_1;
  task.params[1] = l_t_1;
  task.params[2] = b_1;
  // task.params[3] = pid_p_z_f;
  // task.params[4] = pid_i_z_f;
  // task.params[5] = pid_d_z_f;
  task.params[3] = smc_k2_f;
  task.params[4] = smc_k6_f;
  task.params[5] = epsilon;
  // task.params[6] = alpha;
  task.params[6] = rho;
  task.params[7] = dist_thld;
  task.params[8] = increment_gain;
  task.params[9] = k_2;
  task.params[10] = l_t_2;
  task.params[11] = b_2;
  task.params[12] = k_3;
  task.params[13] = l_t_3;
  task.params[14] = b_3;
  task.ctrl_params[0] = pid_p_z;
  task.ctrl_params[1] = pid_i_z;
  task.ctrl_params[2] = pid_d_z;
  task.ctrl_params[3] = pid_p_clamp;
  task.ctrl_params[4] = pid_i_clamp;
  task.ctrl_params[5] = cb_gain_prev;
  task.ctrl_params[6] = cb_gain_next;
  task.ctrl_params[7] = smc_k1;
  task.ctrl_params[8] = smc_k2;
  task.ctrl_params[9] = smc_k3;
  task.ctrl_params[10] = smc_k4;
  task.ctrl_params[11] = smc_k5;
  task.ctrl_params[12] = smc_k6;
  task.ctrl_params[13] = smc_k7;
  task.ctrl_params[14] = smc_k8;
  // task.ctrl_params[15] = smc_delta;
  task.ctrl_params[15] = smc_ksign;
  task.ctrl_params[16] = smc_kdisc;
  task.ctrl_params[17] = pid_p_psi;
  task.ctrl_params[18] = pid_i_psi;
  task.ctrl_params[19] = pid_d_psi;
  task.ctrl_params[20] = pid_p_psi_clamp;
  task.ctrl_params[21] = pid_i_psi_clamp;
  task.ctrl_params[22] = smc_i_k1;
  task.ctrl_params[23] = smc_i_k2;
  task.ctrl_params[24] = smc_i_k3;
  task.ctrl_params[25] = smc_i_k4;
  task.ctrl_params[26] = smc_i_k5;
  task.ctrl_params[27] = smc_i_k6;
  task.ctrl_params[28] = smc_i_k7;
  task.ctrl_params[29] = smc_i_k8;


  _failureListParsed.push_back(task);

  return true;

}

// -----------------------------------------------------------------------------
// parsing stop task
bool Parser::parseStop(const Setting &stop_parameters)
{
  std::cout << "Parsing Stop Failure State" << std::endl;
  double pid_p_z(0.0), pid_i_z(0.0), pid_d_z(0.0);
  double pid_p_z_f(0.0), pid_i_z_f(0.0), pid_d_z_f(0.0);
  double pid_p_clamp(0.0), pid_i_clamp(0.0);
  double cb_gain_prev(0.0), cb_gain_next(0.0);

  double pid_p_psi(0.0), pid_i_psi(0.0), pid_d_psi(0.0);
  double pid_p_psi_clamp(0.0), pid_i_psi_clamp(0.0);

  double smc_k1(0.0), smc_k2(0.0), smc_k3(0.0), smc_k4(0.0);
  double smc_k5(0.0), smc_k6(0.0), smc_k7(0.0), smc_k8(0.0);
  double smc_i_k1(0.0), smc_i_k2(0.0), smc_i_k3(0.0), smc_i_k4(0.0);
  double smc_i_k5(0.0), smc_i_k6(0.0), smc_i_k7(0.0), smc_i_k8(0.0);
  double smc_delta(0.0), smc_kdisc(0.0);

  double k_1(0.0), l_t_1(0.0), b_1(0.0);
  double k_2(0.0), l_t_2(0.0), b_2(0.0);
  double k_3(0.0), l_t_3(0.0), b_3(0.0);

  double dist_thld(0.0), increment_gain(0.0);
  double smc_k2_f(0.0), smc_k6_f(0.0), epsilon(0.0), rho(0.0);

  if(!stop_parameters.lookupValue("k_1", k_1))
  {
    std::cerr << "Rod's elastic coefficient not found for Stop Failure State, please add it by k_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("l_t_1", l_t_1))
  {
    std::cerr << "Rod's min length not found for Stop Failure State, please add it by l_t_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("b_1", b_1))
  {
    std::cerr << "Rod's damping coefficient not found for Stop Failure State, please add it by b_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("k_2", k_2))
  {
    std::cerr << "Rod's elastic coefficient not found for Stop Failure State, please add it by k_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("l_t_2", l_t_2))
  {
    std::cerr << "Rod's min length not found for Stop Failure State, please add it by l_t_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("b_2", b_2))
  {
    std::cerr << "Rod's damping coefficient not found for Stop Failure State, please add it by b_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }
  if(!stop_parameters.lookupValue("k_3", k_3))
  {
    std::cerr << "Rod's elastic coefficient not found for Stop Failure State, please add it by k_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("l_t_3", l_t_3))
  {
    std::cerr << "Rod's min length not found for Stop Failure State, please add it by l_t_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("b_3", b_3))
  {
    std::cerr << "Rod's damping coefficient not found for Stop Failure State, please add it by b_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("epsilon", epsilon))
  {
    std::cerr << "Quadrotor's Exponential gain for sign function (SMC) not found for Stop Failure State, please add it by epsilon = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("rho", rho))
  {
    std::cerr << "Quadrotor's Exponential decay rate for sign function (SMC) not found for Stop Failure State, please add it by rho = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("dist_thld", dist_thld))
  {
    std::cerr << "Quadrotor's Threshold Distance not found for Stop Failure State, please add it by move_dist_thld = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("increment_gain", increment_gain))
  {
    std::cerr << "Quadrotor's Increment Gain not found for Stop Failure State, please add it by increment_gain = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("pid_p_z", pid_p_z))
  {
    std::cerr << "Quadrotor's PID proportional gain not found for Stop Failure State, please add it by pid_p_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("pid_i_z", pid_i_z))
  {
    std::cerr << "Quadrotor's PID integral gain not found for Stop Failure State, please add it by pid_i_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("pid_d_z", pid_d_z))
  {
    std::cerr << "Quadrotor's PID derivative gain not found for Stop Failure State, please add it by pid_d_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("pid_p_z_f", pid_p_z_f))
  {
    std::cerr << "Quadrotor's PID proportional gain not found for Stop Failure State, please add it by pid_p_z_f = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("pid_i_z_f", pid_i_z_f))
  {
    std::cerr << "Quadrotor's PID integral gain not found for Stop Failure State, please add it by pid_i_z_f = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("pid_d_z_f", pid_d_z_f))
  {
    std::cerr << "Quadrotor's PID derivative gain not found for Stop Failure State, please add it by pid_d_z_f = value" << std::endl;
    return(EXIT_FAILURE);
  }

  // if(!stop_parameters.lookupValue("alpha", alpha))
  // {
  //   std::cerr << "Quadrotor's Adaptive PID decay rate not found for Stop Failure State, please add it by alpha = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }

  if(!stop_parameters.lookupValue("pid_p_clamp", pid_p_clamp))
  {
    std::cerr << "Quadrotor's PID proportional clamp limit not found for Recovery Stop State, please add it by pid_p_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("pid_i_clamp", pid_i_clamp))
  {
    std::cerr << "Quadrotor's PID integrative clamp limit not found for Recovery Stop State, please add it by pid_i_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("cb_gain_prev", cb_gain_prev))
  {
    std::cerr << "Quadrotor's Consensus Based gain for previous UAV info not found for Stop Failure State, please add it by cb_gain_prev = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("cb_gain_next", cb_gain_next))
  {
    std::cerr << "Quadrotor's Consensus Based gain for previous UAV info not found for Stop Failure State, please add it by cb_gain_next = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("pid_p_psi", pid_p_psi))
  {
    std::cerr << "Quadrotor's PID proportional gain not found for Stop Failure State, please add it by pid_p_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("pid_i_psi", pid_i_psi))
  {
    std::cerr << "Quadrotor's PID integral gain not found for Recovery Stop State, please add it by pid_i_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("pid_d_psi", pid_d_psi))
  {
    std::cerr << "Quadrotor's PID derivative gain not found for Recovery Stop State, please add it by pid_d_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("pid_p_psi_clamp", pid_p_psi_clamp))
  {
    std::cerr << "Quadrotor's PID proportional clamp limit not found for Stop Failure State, please add it by pid_p_psi_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("pid_i_psi_clamp", pid_i_psi_clamp))
  {
    std::cerr << "Quadrotor's PID integrative clamp limit not found for Stop Failure State, please add it by pid_i_psi_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("smc_k1", smc_k1))
  {
    std::cerr << "Quadrotor's SMC k1 Gain not found for Recovery Stop State, please add it by smc_k1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("smc_k2", smc_k2))
  {
    std::cerr << "Quadrotor's SMC k2 Gain not found for Recovery Stop State, please add it by smc_k2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("smc_k2_f", smc_k2_f))
  {
    std::cerr << "Quadrotor's SMC k2 Final Gain not found for Stop Failure State, please add it by smc_k2_f = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("smc_k3", smc_k3))
  {
    std::cerr << "Quadrotor's SMC k3 Gain not found for Recovery Stop State, please add it by smc_k3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("smc_k4", smc_k4))
  {
    std::cerr << "Quadrotor's SMC k4 Gain not found for Recovery Stop State, please add it by smc_k4 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("smc_k5", smc_k5))
  {
    std::cerr << "Quadrotor's SMC k5 Gain not found for Recovery Stop State, please add it by smc_k5 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("smc_k6", smc_k6))
  {
    std::cerr << "Quadrotor's SMC k6 Gain not found for Recovery Stop State, please add it by smc_k6 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("smc_k6_f", smc_k6_f))
  {
    std::cerr << "Quadrotor's SMC k6 Final Gain not found for Stop Failure State, please add it by smc_k6_f = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("smc_k7", smc_k7))
  {
    std::cerr << "Quadrotor's SMC k7 Gain not found for Recovery Stop State, please add it by smc_k7 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("smc_k8", smc_k8))
  {
    std::cerr << "Quadrotor's SMC k8 Gain not found for Recovery Stop State, please add it by smc_k8 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("smc_i_k1", smc_i_k1))
  {
    std::cerr << "Quadrotor's SMC k1 Integral Gain not found for Stop Failure State, please add it by smc_i_k1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("smc_i_k2", smc_i_k2))
  {
    std::cerr << "Quadrotor's SMC k2 Integral Gain not found for Stop Failure State, please add it by smc_i_k2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("smc_i_k3", smc_i_k3))
  {
    std::cerr << "Quadrotor's SMC k3 Integral Gain not found for Stop Failure State, please add it by smc_i_k3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("smc_i_k4", smc_i_k4))
  {
    std::cerr << "Quadrotor's SMC k4 integral Gain not found for Stop Failure State, please add it by smc_i_k4 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("smc_i_k5", smc_i_k5))
  {
    std::cerr << "Quadrotor's SMC k5 Integral Gain not found for Stop Failure State, please add it by smc_i_k5 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("smc_i_k6", smc_i_k6))
  {
    std::cerr << "Quadrotor's SMC k6 Integral Gain not found for Stop Failure State, please add it by smc_i_k6 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("smc_i_k7", smc_i_k7))
  {
    std::cerr << "Quadrotor's SMC k7 Integral Gain not found for Stop Failure State, please add it by smc_i_k7 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("smc_i_k8", smc_i_k8))
  {
    std::cerr << "Quadrotor's SMC k8 Integral Gain not found for Stop Failure State, please add it by smc_i_k8 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("smc_delta", smc_delta))
  {
    std::cerr << "Quadrotor's SMC delta Gain not found for Recovery Stop State, please add it by smc_delta = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!stop_parameters.lookupValue("smc_kdisc", smc_kdisc))
  {
    std::cerr << "Quadrotor's SMC Discontinuous Gain not found for Stop Failure State, please add it by smc_kdisc = value" << std::endl;
    return(EXIT_FAILURE);
  }

  std::cout << "Stop Failure Task parsed successfully" << std::endl;

  ros2_muavp_interface::srv::FailureInfo::Request task;

  task.curr_state = fail_states::STOP;
  task.params[0] = k_1;
  task.params[1] = l_t_1;
  task.params[2] = b_1;
  task.params[3] = smc_k2_f;
  task.params[4] = smc_k6_f;
  task.params[5] = epsilon;
  task.params[6] = rho;
  task.params[7] = dist_thld;
  task.params[8] = increment_gain;
  task.params[9] = k_2;
  task.params[10] = l_t_2;
  task.params[11] = b_2;
  task.params[12] = k_3;
  task.params[13] = l_t_3;
  task.params[14] = b_3;
  task.ctrl_params[0] = pid_p_z;
  task.ctrl_params[1] = pid_i_z;
  task.ctrl_params[2] = pid_d_z;
  task.ctrl_params[3] = pid_p_clamp;
  task.ctrl_params[4] = pid_i_clamp;
  task.ctrl_params[5] = cb_gain_prev;
  task.ctrl_params[6] = cb_gain_next;
  task.ctrl_params[7] = smc_k1;
  task.ctrl_params[8] = smc_k2;
  task.ctrl_params[9] = smc_k3;
  task.ctrl_params[10] = smc_k4;
  task.ctrl_params[11] = smc_k5;
  task.ctrl_params[12] = smc_k6;
  task.ctrl_params[13] = smc_k7;
  task.ctrl_params[14] = smc_k8;
  task.ctrl_params[15] = smc_delta;
  task.ctrl_params[16] = smc_kdisc;
  task.ctrl_params[17] = pid_p_psi;
  task.ctrl_params[18] = pid_i_psi;
  task.ctrl_params[19] = pid_d_psi;
  task.ctrl_params[20] = pid_p_psi_clamp;
  task.ctrl_params[21] = pid_i_psi_clamp;
  task.ctrl_params[22] = smc_i_k1;
  task.ctrl_params[23] = smc_i_k2;
  task.ctrl_params[24] = smc_i_k3;
  task.ctrl_params[25] = smc_i_k4;
  task.ctrl_params[26] = smc_i_k5;
  task.ctrl_params[27] = smc_i_k6;
  task.ctrl_params[28] = smc_i_k7;
  task.ctrl_params[29] = smc_i_k8;

  _failureListParsed.push_back(task);

  return true;

}

// -----------------------------------------------------------------------------
// parsing land-failure task
bool Parser::parseLandFailure(const Setting &tasks_land)
{
  std::cout << "Parsing Land Failure Task" << std::endl;

  double z_min(0.0);
  double k_1(0.0), l_t_1(0.0), b_1(0.0);
  double k_2(0.0), l_t_2(0.0), b_2(0.0);
  double k_3(0.0), l_t_3(0.0), b_3(0.0);
  double pid_p_z(0.0), pid_i_z(0.0), pid_d_z(0.0);

  double pid_p_clamp(0.0), pid_i_clamp(0.0);
  double smc_k1(0.0), smc_k2(0.0), smc_k3(0.0), smc_k4(0.0);
  double smc_k5(0.0), smc_k6(0.0), smc_k7(0.0), smc_k8(0.0);
  double epsilon(0.0), rho(0.0);
  double smc_i_k1(0.0), smc_i_k2(0.0), smc_i_k3(0.0), smc_i_k4(0.0);
  double smc_i_k5(0.0), smc_i_k6(0.0), smc_i_k7(0.0), smc_i_k8(0.0);
  double smc_ksign(0.0), smc_kdisc(0.0);

  double pid_p_psi(0.0), pid_i_psi(0.0), pid_d_psi(0.0);
  double pid_p_psi_clamp(0.0), pid_i_psi_clamp(0.0);

  // double x(0.0), y(0.0), z(0.0), thld_x(0.0), thld_y(0.0), thld_z(0.0);
  // double yaw_des_uav_1(0.0), yaw_des_uav_2(0.0), yaw_des_uav_3(0.0);
  double cb_gain_prev(0.0), cb_gain_next(0.0);
  double tension_comp(0.0);



  if(!tasks_land.lookupValue("z_min", z_min))
  {
    std::cerr << "Desired Position not found for Land task, please add it by z_min = value" << std::endl;
    return(EXIT_FAILURE);
  }

  // if(!tasks_land.lookupValue("x", x))
  // {
  //   std::cerr << "Desired Position not found for Land task, please add it by x = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!tasks_land.lookupValue("y", y))
  // {
  //   std::cerr << "Desired Position not found for Land task, please add it by y = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!tasks_land.lookupValue("z", z))
  // {
  //   std::cerr << "Desired Position not found for Land task, please add it by z = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }

  // if(!tasks_land.lookupValue("threshold_x", thld_x))
  // {
  //   std::cerr << "Desired Threshold not found for Land task, please add it by threshold_x = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!tasks_land.lookupValue("threshold_y", thld_y))
  // {
  //   std::cerr << "Desired Threshold not found for Land task, please add it by threshold_y = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!tasks_land.lookupValue("threshold_z", thld_z))
  // {
  //   std::cerr << "Desired Threshold not found for Land task, please add it by threshold_z = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }

  // if(!tasks_land.lookupValue("yaw_des_uav_1", yaw_des_uav_1))
  // {
  //   std::cerr << "Desired Yaw for first UAV not found for Land task, please add it by yaw_des_uav_1 = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!tasks_land.lookupValue("yaw_des_uav_2", yaw_des_uav_2))
  // {
  //   std::cerr << "Desired Yaw for second UAV not found for Land task, please add it by yaw_des_uav_2 = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!tasks_land.lookupValue("yaw_des_uav_3", yaw_des_uav_3))
  // {
  //   std::cerr << "Desired Yaw for third UAV not found for Land task, please add it by yaw_des_uav_3 = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }

  if(!tasks_land.lookupValue("pid_p_z", pid_p_z))
  {
    std::cerr << "Proportional Gain not found for Land Failure task, please add it by pid_p_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("pid_i_z", pid_i_z))
  {
    std::cerr << "Integral Gain not found for Land Failure task, please add it by pid_i_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("pid_d_z", pid_d_z))
  {
    std::cerr << "Derivative Gain not found for Land Failure task, please add it by pid_d_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("pid_p_clamp", pid_p_clamp))
  {
    std::cerr << "Proportional Threshold not found for Land Failure task, please add it by pid_p_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("pid_i_clamp", pid_i_clamp))
  {
    std::cerr << "Integrative Threshold not found for Land Failure task, please add it by pid_i_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("k_1", k_1))
  {
    std::cerr << "Rod's elastic coefficient not found for Land Failure task, please add it by k_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("l_t_1", l_t_1))
  {
    std::cerr << "Rod's min length not found for Land Failure task, please add it by l_t_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("b_1", b_1))
  {
    std::cerr << "Rod's damping coefficient not found for Land Failure task, please add it by b_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("k_2", k_2))
  {
    std::cerr << "Rod's elastic coefficient not found for Land Failure task, please add it by k_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("l_t_2", l_t_2))
  {
    std::cerr << "Rod's min length not found for Land Failure task, please add it by l_t_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("b_2", b_2))
  {
    std::cerr << "Rod's damping coefficient not found for Land Failure task, please add it by b_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }
  if(!tasks_land.lookupValue("k_3", k_3))
  {
    std::cerr << "Rod's elastic coefficient not found for Land Failure task, please add it by k_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("l_t_3", l_t_3))
  {
    std::cerr << "Rod's min length not found for Land Failure task, please add it by l_t_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("b_3", b_3))
  {
    std::cerr << "Rod's damping coefficient not found for Land Failure task, please add it by b_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }
  if(!tasks_land.lookupValue("epsilon", epsilon))
  {
    std::cerr << "Quadrotor's Exponential gain for sign function (SMC) not found for Land Failure task, please add it by epsilon = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("rho", rho))
  {
    std::cerr << "Quadrotor's Exponential decay rate for sign function (SMC) not found for Land Failure task, please add it by rho = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("pid_p_psi", pid_p_psi))
  {
    std::cerr << "Quadrotor's PID proportional gain not found for Land Failure task, please add it by pid_p_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("pid_i_psi", pid_i_psi))
  {
    std::cerr << "Quadrotor's PID integral gain not found for Land Failure task, please add it by pid_i_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("pid_d_psi", pid_d_psi))
  {
    std::cerr << "Quadrotor's PID derivative gain not found for Land Failure task, please add it by pid_d_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("pid_p_psi_clamp", pid_p_psi_clamp))
  {
    std::cerr << "Quadrotor's PID proportional clamp limit not found for Land Failure task, please add it by pid_p_psi_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("pid_i_psi_clamp", pid_i_psi_clamp))
  {
    std::cerr << "Quadrotor's PID integrative clamp limit not found for Land Failure task, please add it by pid_i_psi_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_k1", smc_k1))
  {
    std::cerr << "Quadrotor's SMC k1 Gain not found for Land Failure task, please add it by smc_k1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_k2", smc_k2))
  {
    std::cerr << "Quadrotor's SMC k2 Gain not found for Land Failure task, please add it by smc_k2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_k3", smc_k3))
  {
    std::cerr << "Quadrotor's SMC k3 Gain not found for Land Failure task, please add it by smc_k3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_k4", smc_k4))
  {
    std::cerr << "Quadrotor's SMC k4 Gain not found for Land Failure task, please add it by smc_k4 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_k5", smc_k5))
  {
    std::cerr << "Quadrotor's SMC k5 Gain not found for Land Failure task, please add it by smc_k5 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_k6", smc_k6))
  {
    std::cerr << "Quadrotor's SMC k6 Gain not found for Land Failure task, please add it by smc_k6 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_k7", smc_k7))
  {
    std::cerr << "Quadrotor's SMC k7 Gain not found for Land Failure task, please add it by smc_k7 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_k8", smc_k8))
  {
    std::cerr << "Quadrotor's SMC k8 Gain not found for Land Failure task, please add it by smc_k8 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_i_k1", smc_i_k1))
  {
    std::cerr << "Quadrotor's SMC k1 Integral Gain not found for Land Failure task, please add it by smc_i_k1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_i_k2", smc_i_k2))
  {
    std::cerr << "Quadrotor's SMC k2 Integral Gain not found for Land Failure task, please add it by smc_i_k2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_i_k3", smc_i_k3))
  {
    std::cerr << "Quadrotor's SMC k3 Integral Gain not found for Land Failure task, please add it by smc_i_k3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_i_k4", smc_i_k4))
  {
    std::cerr << "Quadrotor's SMC k4 integral Gain not found for Land Failure task, please add it by smc_i_k4 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_i_k5", smc_i_k5))
  {
    std::cerr << "Quadrotor's SMC k5 Integral Gain not found for Land Failure task, please add it by smc_i_k5 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_i_k6", smc_i_k6))
  {
    std::cerr << "Quadrotor's SMC k6 Integral Gain not found for Land Failure task, please add it by smc_i_k6 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_i_k7", smc_i_k7))
  {
    std::cerr << "Quadrotor's SMC k7 Integral Gain not found for Land Failure task, please add it by smc_i_k7 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_i_k8", smc_i_k8))
  {
    std::cerr << "Quadrotor's SMC k8 Integral Gain not found for Land Failure task, please add it by smc_i_k8 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_ksign", smc_ksign))
  {
    std::cerr << "Quadrotor's SMC Gain for sign function not found for Land Failure task, please add it by smc_ksign = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_kdisc", smc_kdisc))
  {
    std::cerr << "Quadrotor's SMC Discontinuous Gain not found for Land Failure task, please add it by smc_kdisc = value" << std::endl;
    return(EXIT_FAILURE);
  }


  if(!tasks_land.lookupValue("cb_gain_prev", cb_gain_prev))
  {
    std::cerr << "Quadrotor's Consensus Based gain for previous UAV info not found for Land Failure task, please add it by cb_gain_prev = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("cb_gain_next", cb_gain_next))
  {
    std::cerr << "Quadrotor's Consensus Based gain for previous UAV info not found for Land Failure task, please add it by cb_gain_next = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("tension_comp", tension_comp))
  {
    std::cerr << "Quadrotor's Comparing Tension not found for Land Failure task, please add it by tension_comp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  std::cout << "Land Failure Task parsed successfully" << std::endl;

  ros2_muavp_interface::srv::FailureInfo::Request task;

  task.curr_state = fail_states::LAND_F;

  // task.x = x;
  // task.y = y;
  // task.z = z;

  task.params[0] = k_1;
  task.params[1] = l_t_1;
  task.params[2] = b_1;
  task.params[3] = k_2;
  task.params[4] = l_t_2;
  task.params[5] = b_2;
  task.params[6] = k_3;
  task.params[7] = l_t_3;
  task.params[8] = b_3;
  task.params[9] = z_min;
  task.params[10] = tension_comp;
  // task.params[9] = thld_x;
  // task.params[10] = thld_y;
  // task.params[11] = thld_z;
  // task.params[12] = yaw_des_uav_1;
  // task.params[13] = yaw_des_uav_2;
  // task.params[14] = yaw_des_uav_3;
  // task.params[3] = smc_k2_f;
  // task.params[4] = smc_k6_f;
  task.params[15] = epsilon;
  task.params[16] = rho;
  // task.params[7] = dist_thld;
  // task.params[8] = increment_gain;


  task.ctrl_params[0] = pid_p_z;
  task.ctrl_params[1] = pid_i_z;
  task.ctrl_params[2] = pid_d_z;
  task.ctrl_params[3] = pid_p_clamp;
  task.ctrl_params[4] = pid_i_clamp;
  task.ctrl_params[5] = cb_gain_prev;
  task.ctrl_params[6] = cb_gain_next;
  task.ctrl_params[7] = smc_k1;
  task.ctrl_params[8] = smc_k2;
  task.ctrl_params[9] = smc_k3;
  task.ctrl_params[10] = smc_k4;
  task.ctrl_params[11] = smc_k5;
  task.ctrl_params[12] = smc_k6;
  task.ctrl_params[13] = smc_k7;
  task.ctrl_params[14] = smc_k8;
  task.ctrl_params[15] = smc_ksign;
  task.ctrl_params[16] = smc_kdisc;
  task.ctrl_params[17] = pid_p_psi;
  task.ctrl_params[18] = pid_i_psi;
  task.ctrl_params[19] = pid_d_psi;
  task.ctrl_params[20] = pid_p_psi_clamp;
  task.ctrl_params[21] = pid_i_psi_clamp;
  task.ctrl_params[22] = smc_i_k1;
  task.ctrl_params[23] = smc_i_k2;
  task.ctrl_params[24] = smc_i_k3;
  task.ctrl_params[25] = smc_i_k4;
  task.ctrl_params[26] = smc_i_k5;
  task.ctrl_params[27] = smc_i_k6;
  task.ctrl_params[28] = smc_i_k7;
  task.ctrl_params[29] = smc_i_k8;

  _failureListParsed.push_back(task);

  return true;
}

// -----------------------------------------------------------------------------
// parsing initial parameters
bool Parser::parseParameters(const Setting &pre_flight_parameters)
{
  std::cout << "Parsing Parameters" << std::endl;
  double m_q(0.0), m_p(0.0), r_q_x(0.0), r_q_y(0.0), r_q_z(0.0), spawn_bias_D(0.0);
  double i_xx_q(0.0), i_yy_q(0.0), i_zz_q(0.0), max_thrust(0.0);
  double i_xx_l(0.0), i_yy_l(0.0), i_zz_l(0.0);
  double spawn_bias_N_uav1(0.0), spawn_bias_N_uav2(0.0), spawn_bias_N_uav3(0.0);
  double spawn_bias_E_uav1(0.0), spawn_bias_E_uav2(0.0), spawn_bias_E_uav3(0.0);
  double payload_anchor_N_rod1(0.0), payload_anchor_E_rod1(0.0), payload_anchor_D_rod1(0.0);
  double payload_anchor_N_rod2(0.0), payload_anchor_E_rod2(0.0), payload_anchor_D_rod2(0.0);
  double payload_anchor_N_rod3(0.0), payload_anchor_E_rod3(0.0), payload_anchor_D_rod3(0.0);
  double formation_change(0.0);
  std::string path = "";


  if(!pre_flight_parameters.lookupValue("m_q", m_q))
  {
    std::cerr << "Quadrotor's mass not found for Pre Flight Parameters, please add it by m_q = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("m_p", m_p))
  {
    std::cerr << "Payload's mass not found for Pre Flight Parameters, please add it by m_p = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("max_thrust", max_thrust))
  {
    std::cerr << "Payload's Max Thrust not found for Pre Flight Parameters, please add it by max_thrust = value" << std::endl;
    return(EXIT_FAILURE);
  }
  // if(!pre_flight_parameters.lookupValue("m_r", m_r))
  // {
  //   std::cerr << "Quadrotor's mass not found for Pre Flight Parameters, please add it by m_r = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }

  if(!pre_flight_parameters.lookupValue("i_xx_q", i_xx_q))
  {
    std::cerr << "Quadrotor's Inertia on xx not found for Pre Flight Parameters, please add it by i_xx_q = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("i_yy_q", i_yy_q))
  {
    std::cerr << "Quadrotor's Inertia on yy not found for Pre Flight Parameters, please add it by i_yy_q = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("i_zz_q", i_zz_q))
  {
    std::cerr << "Quadrotor's Inertia on zz not found for Pre Flight Parameters, please add it by i_zz_q = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("i_xx_l", i_xx_l))
  {
    std::cerr << "Payload Inertia on xx not found for Pre Flight Parameters, please add it by i_xx_l = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("i_yy_l", i_yy_l))
  {
    std::cerr << "Payload Inertia on yy not found for Pre Flight Parameters, please add it by i_yy_l = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("i_zz_l", i_zz_l))
  {
    std::cerr << "Payload Inertia on zz not found for Pre Flight Parameters, please add it by i_zz_l = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("r_q_x", r_q_x))
  {
    std::cerr << "X distance UAV-rod not found for Pre Flight Parameters, please add it by r_q_x = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("r_q_y", r_q_y))
  {
    std::cerr << "Y distance UAV-rod not found for Pre Flight Parameters, please add it by r_q_y = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("r_q_z", r_q_z))
  {
    std::cerr << "Z distance UAV-rod not found for Pre Flight Parameters, please add it by r_q_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("spawn_bias_N_uav1", spawn_bias_N_uav1))
  {
    std::cerr << "Spawn Bias on NORD for UAV 1 not found for Pre Flight Parameters, please add it by spawn_bias_N_uav1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("spawn_bias_E_uav1", spawn_bias_E_uav1))
  {
    std::cerr << "Spawn Bias on EAST for UAV 1 not found for Pre Flight Parameters, please add it by spawn_bias_E_uav1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("spawn_bias_N_uav2", spawn_bias_N_uav2))
  {
    std::cerr << "Spawn Bias on NORD for UAV 2 not found for Pre Flight Parameters, please add it by spawn_bias_N_uav2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("spawn_bias_E_uav2", spawn_bias_E_uav2))
  {
    std::cerr << "Spawn Bias on EAST for UAV 2 not found for Pre Flight Parameters, please add it by spawn_bias_E_uav2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("spawn_bias_N_uav3", spawn_bias_N_uav3))
  {
    std::cerr << "Spawn Bias on NORD for UAV 3 not found for Pre Flight Parameters, please add it by spawn_bias_N_uav3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("spawn_bias_E_uav3", spawn_bias_E_uav3))
  {
    std::cerr << "Spawn Bias on EAST for UAV 3 not found for Pre Flight Parameters, please add it by spawn_bias_E_uav3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("spawn_bias_D", spawn_bias_D))
  {
    std::cerr << "Spawn Bias on DOWN not found for Pre Flight Parameters, please add it by spawn_bias_D = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("payload_anchor_N_rod1", payload_anchor_N_rod1))
  {
    std::cerr << "Payload anchor point 1 NORD coord not found for Pre Flight Parameters, please add it by payload_anchor_N_rod1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("payload_anchor_E_rod1", payload_anchor_E_rod1))
  {
    std::cerr << "Payload anchor point 1 EAST coord not found for Pre Flight Parameters, please add it by payload_anchor_E_rod1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("payload_anchor_D_rod1", payload_anchor_D_rod1))
  {
    std::cerr << "Payload anchor point 1 DOWN coord not found for Pre Flight Parameters, please add it by payload_anchor_D_rod1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("payload_anchor_N_rod2", payload_anchor_N_rod2))
  {
    std::cerr << "Payload anchor point 2 NORD coord not found for Pre Flight Parameters, please add it by payload_anchor_N_rod2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("payload_anchor_E_rod2", payload_anchor_E_rod2))
  {
    std::cerr << "Payload anchor point 2 EAST coord not found for Pre Flight Parameters, please add it by payload_anchor_E_rod2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("payload_anchor_D_rod2", payload_anchor_D_rod2))
  {
    std::cerr << "Payload anchor point 2 DOWN coord not found for Pre Flight Parameters, please add it by payload_anchor_D_rod2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("payload_anchor_N_rod3", payload_anchor_N_rod3))
  {
    std::cerr << "Payload anchor point 3 NORD coord not found for Pre Flight Parameters, please add it by payload_anchor_N_rod3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("payload_anchor_E_rod3", payload_anchor_E_rod3))
  {
    std::cerr << "Payload anchor point 3 EAST coord not found for Pre Flight Parameters, please add it by payload_anchor_E_rod3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("payload_anchor_D_rod3", payload_anchor_D_rod3))
  {
    std::cerr << "Payload anchor point 3 DOWN coord not found for Pre Flight Parameters, please add it by payload_anchor_D_rod3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("formation_change", formation_change))
  {
    std::cerr << "Formation Change directive not found for Pre Flight Parameters, please add it by formation_change = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!pre_flight_parameters.lookupValue("path", path))
  {
    std::cerr << "Payload Path for Data Files Directory not found for Pre Flight Parameters, please add it by path = value" << std::endl;
    return(EXIT_FAILURE);
  }

  std::cout << "Pre Flight Parameters parsed successfully" << std::endl;

  ros2_muavp_interface::msg::StatesInfo task;
  task.curr_state = states::PARAMETERS;
  task.params[0] = m_q;
  task.params[1] = m_p;
  task.params[2] = r_q_x;
  task.params[3] = r_q_y;
  task.params[4] = r_q_z;
  task.params[5] = spawn_bias_N_uav1;
  task.params[6] = spawn_bias_E_uav1;
  task.params[7] = spawn_bias_N_uav2;
  task.params[8] = spawn_bias_E_uav2;
  task.params[9] = spawn_bias_N_uav3;
  task.params[10] = spawn_bias_E_uav3;
  task.params[11] = spawn_bias_D;
  // task.params[5] = spawn_bias_xy;
  // task.params[6] = spawn_bias_z;
  // task.params[7] = payload_anchor_x;
  // task.params[8] = payload_anchor_y;
  // task.params[9] = payload_anchor_z;
  task.params[12] = payload_anchor_N_rod1;
  task.params[13] = payload_anchor_E_rod1;
  task.params[14] = payload_anchor_D_rod1;
  task.params[15] = payload_anchor_N_rod2;
  task.params[16] = payload_anchor_E_rod2;
  task.params[17] = payload_anchor_D_rod2;
  task.params[18] = payload_anchor_N_rod3;
  task.params[19] = payload_anchor_E_rod3;
  task.params[20] = payload_anchor_D_rod3;
  task.params[21] = formation_change;
  task.path = path;

  // putting inertia on ctrl_params cause there is no more space in params
  task.ctrl_params[0] = i_xx_q;
  task.ctrl_params[1] = i_yy_q;
  task.ctrl_params[2] = i_zz_q;
  task.ctrl_params[3] = max_thrust;
  task.ctrl_params[4] = i_xx_l;
  task.ctrl_params[5] = i_yy_l;
  task.ctrl_params[6] = i_zz_l;

  _taskListParsed.push_back(task);

  return true;

}

// -----------------------------------------------------------------------------
// parsing pre-takeoff task
bool Parser::parsePreTakeOff(const Setting &tasks_pre_takeoff)
{
  std::cout << "Parsing Pre-Takeoff Task" << std::endl;

  double k_1(0.0), l_t_1(0.0), b_1(0.0);
  double k_2(0.0), l_t_2(0.0), b_2(0.0);
  double k_3(0.0), l_t_3(0.0), b_3(0.0);
  double pid_p_z(0.0), pid_i_z(0.0), pid_d_z(0.0);

  double pid_p_clamp(0.0), pid_i_clamp(0.0);
  double smc_k1(0.0), smc_k2(0.0), smc_k3(0.0), smc_k4(0.0);
  double smc_k5(0.0), smc_k6(0.0), smc_k7(0.0), smc_k8(0.0);
  double epsilon(0.0), rho(0.0);
  double smc_i_k1(0.0), smc_i_k2(0.0), smc_i_k3(0.0), smc_i_k4(0.0);
  double smc_i_k5(0.0), smc_i_k6(0.0), smc_i_k7(0.0), smc_i_k8(0.0);
  double smc_ksign(0.0), smc_kdisc(0.0);

  double pid_p_psi(0.0), pid_i_psi(0.0), pid_d_psi(0.0);
  double pid_p_psi_clamp(0.0), pid_i_psi_clamp(0.0);

  double x(0.0), z(0.0), thld_x(0.0), thld_z(0.0);
  double yaw_des_uav_1(0.0), yaw_des_uav_2(0.0), yaw_des_uav_3(0.0);

  double pid_p_T(0.0), pid_i_T(0.0), pid_d_T(0.0), tension_des(0.0);


  if(!tasks_pre_takeoff.lookupValue("k_1", k_1))
  {
    std::cerr << "Rod's elastic coefficient not found for PreTakeoff task, please add it by k_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("l_t_1", l_t_1))
  {
    std::cerr << "Rod's min length not found for PreTakeoff task, please add it by l_t_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("b_1", b_1))
  {
    std::cerr << "Rod's damping coefficient not found for PreTakeoff task, please add it by b_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("k_2", k_2))
  {
    std::cerr << "Rod's elastic coefficient not found for PreTakeoff task, please add it by k_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("l_t_2", l_t_2))
  {
    std::cerr << "Rod's min length not found for PreTakeoff task, please add it by l_t_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("b_2", b_2))
  {
    std::cerr << "Rod's damping coefficient not found for PreTakeoff task, please add it by b_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }
  if(!tasks_pre_takeoff.lookupValue("k_3", k_3))
  {
    std::cerr << "Rod's elastic coefficient not found for PreTakeoff task, please add it by k_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("l_t_3", l_t_3))
  {
    std::cerr << "Rod's min length not found for PreTakeoff task, please add it by l_t_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("b_3", b_3))
  {
    std::cerr << "Rod's damping coefficient not found for PreTakeoff task, please add it by b_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("pid_p_z", pid_p_z))
  {
    std::cerr << "Quadrotor's PID proportional gain not found for PreTakeoff task, please add it by pid_p_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("pid_i_z", pid_i_z))
  {
    std::cerr << "Quadrotor's PID integral gain not found for PreTakeoff task, please add it by pid_i_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("pid_d_z", pid_d_z))
  {
    std::cerr << "Quadrotor's PID derivative gain not found for PreTakeoff task, please add it by pid_d_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("pid_p_clamp", pid_p_clamp))
  {
    std::cerr << "Proportional Threshold not found for PreTakeoff task, please add it by pid_p_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("pid_i_clamp", pid_i_clamp))
  {
    std::cerr << "Integrative Threshold not found for PreTakeoff task, please add it by pid_i_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("pid_p_T", pid_p_T))
  {
    std::cerr << "Quadrotor's PID on Tension proportional gain not found for PreTakeoff task, please add it by pid_p_T = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("pid_i_T", pid_i_T))
  {
    std::cerr << "Quadrotor's PID on Tension integral gain not found for PreTakeoff task, please add it by pid_i_T = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("pid_d_T", pid_d_T))
  {
    std::cerr << "Quadrotor's PID on Tension derivative gain not found for PreTakeoff task, please add it by pid_d_T = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("tension_des", tension_des))
  {
    std::cerr << "Quadrotor's Desired Tension not found for PreTakeoff task, please add it by tension_des = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("epsilon", epsilon))
  {
    std::cerr << "Quadrotor's Exponential gain for sign function (SMC) not found for PreTakeoff task, please add it by epsilon = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("rho", rho))
  {
    std::cerr << "Quadrotor's Exponential decay rate for sign function (SMC) not found for PreTakeoff task, please add it by rho = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("pid_p_psi", pid_p_psi))
  {
    std::cerr << "Quadrotor's PID proportional gain not found for PreTakeoff task, please add it by pid_p_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("pid_i_psi", pid_i_psi))
  {
    std::cerr << "Quadrotor's PID integral gain not found for PreTakeoff task, please add it by pid_i_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("pid_d_psi", pid_d_psi))
  {
    std::cerr << "Quadrotor's PID derivative gain not found for PreTakeoff task, please add it by pid_d_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("pid_p_psi_clamp", pid_p_psi_clamp))
  {
    std::cerr << "Quadrotor's PID proportional clamp limit not found for PreTakeoff task, please add it by pid_p_psi_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("pid_i_psi_clamp", pid_i_psi_clamp))
  {
    std::cerr << "Quadrotor's PID integrative clamp limit not found for PreTakeoff task, please add it by pid_i_psi_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("smc_k1", smc_k1))
  {
    std::cerr << "Quadrotor's SMC k1 Gain not found for PreTakeoff task, please add it by smc_k1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("smc_k2", smc_k2))
  {
    std::cerr << "Quadrotor's SMC k2 Gain not found for PreTakeoff task, please add it by smc_k2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("smc_k3", smc_k3))
  {
    std::cerr << "Quadrotor's SMC k3 Gain not found for PreTakeoff task, please add it by smc_k3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("smc_k4", smc_k4))
  {
    std::cerr << "Quadrotor's SMC k4 Gain not found for PreTakeoff task, please add it by smc_k4 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("smc_k5", smc_k5))
  {
    std::cerr << "Quadrotor's SMC k5 Gain not found for PreTakeoff task, please add it by smc_k5 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("smc_k6", smc_k6))
  {
    std::cerr << "Quadrotor's SMC k6 Gain not found for PreTakeoff task, please add it by smc_k6 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("smc_k7", smc_k7))
  {
    std::cerr << "Quadrotor's SMC k7 Gain not found for PreTakeoff task, please add it by smc_k7 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("smc_k8", smc_k8))
  {
    std::cerr << "Quadrotor's SMC k8 Gain not found for PreTakeoff task, please add it by smc_k8 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("smc_i_k1", smc_i_k1))
  {
    std::cerr << "Quadrotor's SMC k1 Integral Gain not found for PreTakeoff task, please add it by smc_i_k1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("smc_i_k2", smc_i_k2))
  {
    std::cerr << "Quadrotor's SMC k2 Integral Gain not found for PreTakeoff task, please add it by smc_i_k2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("smc_i_k3", smc_i_k3))
  {
    std::cerr << "Quadrotor's SMC k3 Integral Gain not found for PreTakeoff task, please add it by smc_i_k3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("smc_i_k4", smc_i_k4))
  {
    std::cerr << "Quadrotor's SMC k4 integral Gain not found for PreTakeoff task, please add it by smc_i_k4 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("smc_i_k5", smc_i_k5))
  {
    std::cerr << "Quadrotor's SMC k5 Integral Gain not found for PreTakeoff task, please add it by smc_i_k5 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("smc_i_k6", smc_i_k6))
  {
    std::cerr << "Quadrotor's SMC k6 Integral Gain not found for PreTakeoff task, please add it by smc_i_k6 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("smc_i_k7", smc_i_k7))
  {
    std::cerr << "Quadrotor's SMC k7 Integral Gain not found for PreTakeoff task, please add it by smc_i_k7 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("smc_i_k8", smc_i_k8))
  {
    std::cerr << "Quadrotor's SMC k8 Integral Gain not found for PreTakeoff task, please add it by smc_i_k8 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("smc_ksign", smc_ksign))
  {
    std::cerr << "Quadrotor's SMC Gain for sign function not found for PreTakeoff task, please add it by smc_ksign = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("smc_kdisc", smc_kdisc))
  {
    std::cerr << "Quadrotor's SMC Discontinuous Gain not found for PreTakeoff task, please add it by smc_kdisc = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("x", x))
  {
    std::cerr << "Desired Position not found for PreTakeoff task, please add it by x = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("z", z))
  {
    std::cerr << "Desired Position not found for PreTakeoff task, please add it by z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("threshold_x", thld_x))
  {
    std::cerr << "Desired Threshold not found for PreTakeoff task, please add it by threshold_x = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("threshold_z", thld_z))
  {
    std::cerr << "Desired Threshold not found for PreTakeoff task, please add it by threshold_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("yaw_des_uav_1", yaw_des_uav_1))
  {
    std::cerr << "Desired Yaw for first UAV not found for PreTakeoff task, please add it by yaw_des_uav_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("yaw_des_uav_2", yaw_des_uav_2))
  {
    std::cerr << "Desired Yaw for second UAV not found for PreTakeoff task, please add it by yaw_des_uav_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_pre_takeoff.lookupValue("yaw_des_uav_3", yaw_des_uav_3))
  {
    std::cerr << "Desired Yaw for third UAV not found for PreTakeoff task, please add it by yaw_des_uav_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }


  // if(!tasks_pre_takeoff.lookupValue("clamp_p_z", clamp_p_z))
  // {
  //   std::cerr << "Proportional Threshold not found for PreTakeoff task, please add it by clamp_p_z = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!tasks_pre_takeoff.lookupValue("clamp_i_z", clamp_i_z))
  // {
  //   std::cerr << "Integrative Threshold not found for PreTakeoff task, please add it by clamp_i_z = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }

  std::cout << "Pre-Takeoff Task parsed successfully" << std::endl;

  ros2_muavp_interface::msg::StatesInfo task;
  task.curr_state = states::PRE_TAKE_OFF;
  task.x = x;
  task.y = x;
  task.z = z;
  // task.params[0] = thld_x;
  // task.params[1] = thld_x;
  // task.params[2] = thld_z;
  // task.params[3] = k;
  // task.params[4] = rod_max_l;
  // task.params[6] = yaw_des_uav_1;
  // task.params[7] = yaw_des_uav_2;
  // task.params[8] = yaw_des_uav_3;
  // task.ctrl_params[0] = pid_p_x;
  // task.ctrl_params[1] = pid_i_x;
  // task.ctrl_params[2] = pid_d_x;
  // task.ctrl_params[3] = clamp_p_x;
  // task.ctrl_params[4] = clamp_i_x;
  // task.ctrl_params[5] = pid_p_y;
  // task.ctrl_params[6] = pid_i_y;
  // task.ctrl_params[7] = pid_d_y;
  // task.ctrl_params[8] = clamp_p_y;
  // task.ctrl_params[9] = clamp_i_y;
  // task.ctrl_params[10] = pid_p_z;
  // task.ctrl_params[11] = pid_i_z;
  // task.ctrl_params[12] = pid_d_z;
  // task.ctrl_params[13] = clamp_p_z;
  // task.ctrl_params[14] = clamp_i_z;


  task.params[0] = k_1;
  task.params[1] = l_t_1;
  task.params[2] = b_1;
  task.params[3] = k_2;
  task.params[4] = l_t_2;
  task.params[5] = b_2;
  task.params[6] = k_3;
  task.params[7] = l_t_3;
  task.params[8] = b_3;
  task.params[9] = thld_x;
  task.params[10] = thld_x;
  task.params[11] = thld_z;
  task.params[12] = yaw_des_uav_1;
  task.params[13] = yaw_des_uav_2;
  task.params[14] = yaw_des_uav_3;
  // task.params[3] = smc_k2_f;
  // task.params[4] = smc_k6_f;
  task.params[15] = epsilon;
  task.params[16] = rho;
  task.params[17] = tension_des;
  // task.params[7] = dist_thld;
  // task.params[8] = increment_gain;


  task.ctrl_params[0] = pid_p_z;
  task.ctrl_params[1] = pid_i_z;
  task.ctrl_params[2] = pid_d_z;
  task.ctrl_params[3] = pid_p_clamp;
  task.ctrl_params[4] = pid_i_clamp;
  // task.ctrl_params[5] = cb_gain_prev;
  // task.ctrl_params[6] = cb_gain_next;
  task.ctrl_params[7] = smc_k1;
  task.ctrl_params[8] = smc_k2;
  task.ctrl_params[9] = smc_k3;
  task.ctrl_params[10] = smc_k4;
  task.ctrl_params[11] = smc_k5;
  task.ctrl_params[12] = smc_k6;
  task.ctrl_params[13] = smc_k7;
  task.ctrl_params[14] = smc_k8;
  task.ctrl_params[15] = smc_ksign;
  task.ctrl_params[16] = smc_kdisc;
  task.ctrl_params[17] = pid_p_psi;
  task.ctrl_params[18] = pid_i_psi;
  task.ctrl_params[19] = pid_d_psi;
  task.ctrl_params[20] = pid_p_psi_clamp;
  task.ctrl_params[21] = pid_i_psi_clamp;
  task.ctrl_params[22] = smc_i_k1;
  task.ctrl_params[23] = smc_i_k2;
  task.ctrl_params[24] = smc_i_k3;
  task.ctrl_params[25] = smc_i_k4;
  task.ctrl_params[26] = smc_i_k5;
  task.ctrl_params[27] = smc_i_k6;
  task.ctrl_params[28] = smc_i_k7;
  task.ctrl_params[29] = smc_i_k8;
  task.ctrl_params[30] = pid_p_T;
  task.ctrl_params[31] = pid_i_T;
  task.ctrl_params[32] = pid_d_T;


  _taskListParsed.push_back(task);

  return true;

}

// -----------------------------------------------------------------------------
// parsing takeoff task
bool Parser::parseTakeOff(const Setting &tasks_takeoff)
{
  std::cout << "Parsing Takeoff Task" << std::endl;
  double k_1(0.0), l_t_1(0.0), b_1(0.0);
  double k_2(0.0), l_t_2(0.0), b_2(0.0);
  double k_3(0.0), l_t_3(0.0), b_3(0.0);
  double pid_p_z(0.0), pid_i_z(0.0), pid_d_z(0.0);

  double pid_p_clamp(0.0), pid_i_clamp(0.0);
  double smc_k1(0.0), smc_k2(0.0), smc_k3(0.0), smc_k4(0.0);
  double smc_k5(0.0), smc_k6(0.0), smc_k7(0.0), smc_k8(0.0);
  double epsilon(0.0), rho(0.0);
  double smc_i_k1(0.0), smc_i_k2(0.0), smc_i_k3(0.0), smc_i_k4(0.0);
  double smc_i_k5(0.0), smc_i_k6(0.0), smc_i_k7(0.0), smc_i_k8(0.0);
  double smc_ksign(0.0), smc_kdisc(0.0);

  double pid_p_psi(0.0), pid_i_psi(0.0), pid_d_psi(0.0);
  double pid_p_psi_clamp(0.0), pid_i_psi_clamp(0.0);

  double x(0.0), z(0.0), thld_x(0.0), thld_z(0.0);
  double yaw_des_uav_1(0.0), yaw_des_uav_2(0.0), yaw_des_uav_3(0.0);
  double cb_gain_prev(0.0), cb_gain_next(0.0);


  if(!tasks_takeoff.lookupValue("k_1", k_1))
  {
    std::cerr << "Rod's elastic coefficient not found for Takeoff task, please add it by k_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("l_t_1", l_t_1))
  {
    std::cerr << "Rod's min length not found for Takeoff task, please add it by l_t_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("b_1", b_1))
  {
    std::cerr << "Rod's damping coefficient not found for Takeoff task, please add it by b_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("k_2", k_2))
  {
    std::cerr << "Rod's elastic coefficient not found for Takeoff task, please add it by k_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("l_t_2", l_t_2))
  {
    std::cerr << "Rod's min length not found for Takeoff task, please add it by l_t_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("b_2", b_2))
  {
    std::cerr << "Rod's damping coefficient not found for Takeoff task, please add it by b_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }
  if(!tasks_takeoff.lookupValue("k_3", k_3))
  {
    std::cerr << "Rod's elastic coefficient not found for Takeoff task, please add it by k_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("l_t_3", l_t_3))
  {
    std::cerr << "Rod's min length not found for Takeoff task, please add it by l_t_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("b_3", b_3))
  {
    std::cerr << "Rod's damping coefficient not found for Takeoff task, please add it by b_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("pid_p_z", pid_p_z))
  {
    std::cerr << "Quadrotor's PID proportional gain not found for Takeoff task, please add it by pid_p_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("pid_i_z", pid_i_z))
  {
    std::cerr << "Quadrotor's PID integral gain not found for Takeoff task, please add it by pid_i_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("pid_d_z", pid_d_z))
  {
    std::cerr << "Quadrotor's PID derivative gain not found for Takeoff task, please add it by pid_d_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("pid_p_clamp", pid_p_clamp))
  {
    std::cerr << "Proportional Threshold not found for Takeoff task, please add it by pid_p_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("pid_i_clamp", pid_i_clamp))
  {
    std::cerr << "Integrative Threshold not found for Takeoff task, please add it by pid_i_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("epsilon", epsilon))
  {
    std::cerr << "Quadrotor's Exponential gain for sign function (SMC) not found for Takeoff task, please add it by epsilon = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("rho", rho))
  {
    std::cerr << "Quadrotor's Exponential decay rate for sign function (SMC) not found for Takeoff task, please add it by rho = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("pid_p_psi", pid_p_psi))
  {
    std::cerr << "Quadrotor's PID proportional gain not found for Takeoff task, please add it by pid_p_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("pid_i_psi", pid_i_psi))
  {
    std::cerr << "Quadrotor's PID integral gain not found for Takeoff task, please add it by pid_i_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("pid_d_psi", pid_d_psi))
  {
    std::cerr << "Quadrotor's PID derivative gain not found for Takeoff task, please add it by pid_d_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("pid_p_psi_clamp", pid_p_psi_clamp))
  {
    std::cerr << "Quadrotor's PID proportional clamp limit not found for Takeoff task, please add it by pid_p_psi_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("pid_i_psi_clamp", pid_i_psi_clamp))
  {
    std::cerr << "Quadrotor's PID integrative clamp limit not found for Takeoff task, please add it by pid_i_psi_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("smc_k1", smc_k1))
  {
    std::cerr << "Quadrotor's SMC k1 Gain not found for Takeoff task, please add it by smc_k1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("smc_k2", smc_k2))
  {
    std::cerr << "Quadrotor's SMC k2 Gain not found for Takeoff task, please add it by smc_k2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("smc_k3", smc_k3))
  {
    std::cerr << "Quadrotor's SMC k3 Gain not found for Takeoff task, please add it by smc_k3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("smc_k4", smc_k4))
  {
    std::cerr << "Quadrotor's SMC k4 Gain not found for Takeoff task, please add it by smc_k4 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("smc_k5", smc_k5))
  {
    std::cerr << "Quadrotor's SMC k5 Gain not found for Takeoff task, please add it by smc_k5 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("smc_k6", smc_k6))
  {
    std::cerr << "Quadrotor's SMC k6 Gain not found for Takeoff task, please add it by smc_k6 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("smc_k7", smc_k7))
  {
    std::cerr << "Quadrotor's SMC k7 Gain not found for Takeoff task, please add it by smc_k7 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("smc_k8", smc_k8))
  {
    std::cerr << "Quadrotor's SMC k8 Gain not found for Takeoff task, please add it by smc_k8 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("smc_i_k1", smc_i_k1))
  {
    std::cerr << "Quadrotor's SMC k1 Integral Gain not found for Takeoff task, please add it by smc_i_k1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("smc_i_k2", smc_i_k2))
  {
    std::cerr << "Quadrotor's SMC k2 Integral Gain not found for Takeoff task, please add it by smc_i_k2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("smc_i_k3", smc_i_k3))
  {
    std::cerr << "Quadrotor's SMC k3 Integral Gain not found for Takeoff task, please add it by smc_i_k3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("smc_i_k4", smc_i_k4))
  {
    std::cerr << "Quadrotor's SMC k4 integral Gain not found for Takeoff task, please add it by smc_i_k4 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("smc_i_k5", smc_i_k5))
  {
    std::cerr << "Quadrotor's SMC k5 Integral Gain not found for Takeoff task, please add it by smc_i_k5 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("smc_i_k6", smc_i_k6))
  {
    std::cerr << "Quadrotor's SMC k6 Integral Gain not found for Takeoff task, please add it by smc_i_k6 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("smc_i_k7", smc_i_k7))
  {
    std::cerr << "Quadrotor's SMC k7 Integral Gain not found for Takeoff task, please add it by smc_i_k7 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("smc_i_k8", smc_i_k8))
  {
    std::cerr << "Quadrotor's SMC k8 Integral Gain not found for Takeoff task, please add it by smc_i_k8 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("smc_ksign", smc_ksign))
  {
    std::cerr << "Quadrotor's SMC Gain for sign function not found for Takeoff task, please add it by smc_ksign = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("smc_kdisc", smc_kdisc))
  {
    std::cerr << "Quadrotor's SMC Discontinuous Gain not found for Takeoff task, please add it by smc_kdisc = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("x", x))
  {
    std::cerr << "Desired Position not found for Takeoff task, please add it by x = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("z", z))
  {
    std::cerr << "Desired Position not found for Takeoff task, please add it by z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("threshold_x", thld_x))
  {
    std::cerr << "Desired Threshold not found for Takeoff task, please add it by threshold_x = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("threshold_z", thld_z))
  {
    std::cerr << "Desired Threshold not found for Takeoff task, please add it by threshold_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("yaw_des_uav_1", yaw_des_uav_1))
  {
    std::cerr << "Desired Yaw for first UAV not found for Takeoff task, please add it by yaw_des_uav_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("yaw_des_uav_2", yaw_des_uav_2))
  {
    std::cerr << "Desired Yaw for second UAV not found for Takeoff task, please add it by yaw_des_uav_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("yaw_des_uav_3", yaw_des_uav_3))
  {
    std::cerr << "Desired Yaw for third UAV not found for Takeoff task, please add it by yaw_des_uav_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }


  if(!tasks_takeoff.lookupValue("cb_gain_prev", cb_gain_prev))
  {
    std::cerr << "Quadrotor's Consensus Based gain for previous UAV info not found for Takeoff task, please add it by cb_gain_prev = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_takeoff.lookupValue("cb_gain_next", cb_gain_next))
  {
    std::cerr << "Quadrotor's Consensus Based gain for previous UAV info not found for Takeoff task, please add it by cb_gain_next = value" << std::endl;
    return(EXIT_FAILURE);
  }


  std::cout << "Takeoff Task parsed successfully" << std::endl;

  ros2_muavp_interface::msg::StatesInfo task;
  task.curr_state = states::TAKE_OFF;
  task.x = x;
  task.y = x;
  task.z = z;

  task.params[0] = k_1;
  task.params[1] = l_t_1;
  task.params[2] = b_1;
  task.params[3] = k_2;
  task.params[4] = l_t_2;
  task.params[5] = b_2;
  task.params[6] = k_3;
  task.params[7] = l_t_3;
  task.params[8] = b_3;
  task.params[9] = thld_x;
  task.params[10] = thld_x;
  task.params[11] = thld_z;
  task.params[12] = yaw_des_uav_1;
  task.params[13] = yaw_des_uav_2;
  task.params[14] = yaw_des_uav_3;
  // task.params[3] = smc_k2_f;
  // task.params[4] = smc_k6_f;
  task.params[15] = epsilon;
  task.params[16] = rho;
  // task.params[7] = dist_thld;
  // task.params[8] = increment_gain;


  task.ctrl_params[0] = pid_p_z;
  task.ctrl_params[1] = pid_i_z;
  task.ctrl_params[2] = pid_d_z;
  task.ctrl_params[3] = pid_p_clamp;
  task.ctrl_params[4] = pid_i_clamp;
  task.ctrl_params[5] = cb_gain_prev;
  task.ctrl_params[6] = cb_gain_next;
  task.ctrl_params[7] = smc_k1;
  task.ctrl_params[8] = smc_k2;
  task.ctrl_params[9] = smc_k3;
  task.ctrl_params[10] = smc_k4;
  task.ctrl_params[11] = smc_k5;
  task.ctrl_params[12] = smc_k6;
  task.ctrl_params[13] = smc_k7;
  task.ctrl_params[14] = smc_k8;
  task.ctrl_params[15] = smc_ksign;
  task.ctrl_params[16] = smc_kdisc;
  task.ctrl_params[17] = pid_p_psi;
  task.ctrl_params[18] = pid_i_psi;
  task.ctrl_params[19] = pid_d_psi;
  task.ctrl_params[20] = pid_p_psi_clamp;
  task.ctrl_params[21] = pid_i_psi_clamp;
  task.ctrl_params[22] = smc_i_k1;
  task.ctrl_params[23] = smc_i_k2;
  task.ctrl_params[24] = smc_i_k3;
  task.ctrl_params[25] = smc_i_k4;
  task.ctrl_params[26] = smc_i_k5;
  task.ctrl_params[27] = smc_i_k6;
  task.ctrl_params[28] = smc_i_k7;
  task.ctrl_params[29] = smc_i_k8;


  _taskListParsed.push_back(task);

  return true;
}

// -----------------------------------------------------------------------------
// parsing rotate task
bool Parser::parseRotate(const Setting &tasks_rotate)
{
  double x(0.0), y(0.0), yaw_des(0.0), thld_rot(0.0);

  if(!tasks_rotate.lookupValue("yaw_des", yaw_des))
  {
    if(!tasks_rotate.lookupValue("x", x) && !tasks_rotate.lookupValue("y", y))
    {
      std::cerr << "Desired Positions not found for Rotate task, please add it by x = value and y = value" << std::endl;
      return(EXIT_FAILURE);
    }
  }

  if(!tasks_rotate.lookupValue("threshold_rot", thld_rot))
  {
    std::cerr << "Desired Threshold not found for Rotate task, please add it by threshold_rot = value (degrees)" << std::endl;
    return(EXIT_FAILURE);
  }

  std::cout << "Rotate Task parsed successfully" << std::endl;

  ros2_muavp_interface::msg::StatesInfo task;
  task.curr_state = states::ROTATE;
  task.x = x;
  task.y = y;
  task.yaw = yaw_des;
  task.params[0] = thld_rot;

  _taskListParsed.push_back(task);

  return true;

}

// -----------------------------------------------------------------------------
// parsing move task
bool Parser::parseMove(const Setting &tasks_move)
{
  std::cout << "Parsing Move Task" << std::endl;
  double k_1(0.0), l_t_1(0.0), b_1(0.0);
  double k_2(0.0), l_t_2(0.0), b_2(0.0);
  double k_3(0.0), l_t_3(0.0), b_3(0.0);
  double pid_p_z(0.0), pid_i_z(0.0), pid_d_z(0.0);

  double pid_p_clamp(0.0), pid_i_clamp(0.0);
  double smc_k1(0.0), smc_k2(0.0), smc_k3(0.0), smc_k4(0.0);
  double smc_k5(0.0), smc_k6(0.0), smc_k7(0.0), smc_k8(0.0);
  double epsilon(0.0), rho(0.0);
  double smc_i_k1(0.0), smc_i_k2(0.0), smc_i_k3(0.0), smc_i_k4(0.0);
  double smc_i_k5(0.0), smc_i_k6(0.0), smc_i_k7(0.0), smc_i_k8(0.0);
  double smc_ksign(0.0), smc_kdisc(0.0);

  double pid_p_psi(0.0), pid_i_psi(0.0), pid_d_psi(0.0);
  double pid_p_psi_clamp(0.0), pid_i_psi_clamp(0.0);

  double x(0.0), y(0.0), z(0.0), thld_x(0.0), thld_y(0.0), thld_z(0.0);
  double yaw_des_uav_1(0.0), yaw_des_uav_2(0.0), yaw_des_uav_3(0.0);
  double cb_gain_prev(0.0), cb_gain_next(0.0), move_dist_thld(0.0), payload_dist(0.0);



  if(!tasks_move.lookupValue("x", x))
  {
    std::cerr << "Desired Position not found for Move task, please add it by x = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("y", y))
  {
    std::cerr << "Desired Position not found for Move task, please add it by y = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("z", z))
  {
    std::cerr << "Desired Position not found for Move task, please add it by z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("threshold_x", thld_x))
  {
    std::cerr << "Desired Threshold not found for Move task, please add it by threshold_x = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("threshold_y", thld_y))
  {
    std::cerr << "Desired Threshold not found for Move task, please add it by threshold_y = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("threshold_z", thld_z))
  {
    std::cerr << "Desired Threshold not found for Move task, please add it by threshold_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("yaw_des_uav_1", yaw_des_uav_1))
  {
    std::cerr << "Desired Yaw for first UAV not found for Move task, please add it by yaw_des_uav_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("yaw_des_uav_2", yaw_des_uav_2))
  {
    std::cerr << "Desired Yaw for second UAV not found for Move task, please add it by yaw_des_uav_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("yaw_des_uav_3", yaw_des_uav_3))
  {
    std::cerr << "Desired Yaw for third UAV not found for Move task, please add it by yaw_des_uav_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("pid_p_z", pid_p_z))
  {
    std::cerr << "Proportional Gain not found for Move task, please add it by pid_p_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("pid_i_z", pid_i_z))
  {
    std::cerr << "Integral Gain not found for Move task, please add it by pid_i_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("pid_d_z", pid_d_z))
  {
    std::cerr << "Derivative Gain not found for Move task, please add it by pid_d_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("pid_p_clamp", pid_p_clamp))
  {
    std::cerr << "Proportional Threshold not found for Move task, please add it by pid_p_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("pid_i_clamp", pid_i_clamp))
  {
    std::cerr << "Integrative Threshold not found for Move task, please add it by pid_i_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("move_dist_thld", move_dist_thld))
  {
    std::cerr << "Move Distance Threshold not found for Move task, please add it by move_dist_thld = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("payload_dist", payload_dist))
  {
    std::cerr << "Local Distance to Payload not found for Move task, please add it by payload_dist = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("k_1", k_1))
  {
    std::cerr << "Rod's elastic coefficient not found for Move task, please add it by k_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("l_t_1", l_t_1))
  {
    std::cerr << "Rod's min length not found for Move task, please add it by l_t_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("b_1", b_1))
  {
    std::cerr << "Rod's damping coefficient not found for Move task, please add it by b_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("k_2", k_2))
  {
    std::cerr << "Rod's elastic coefficient not found for Move task, please add it by k_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("l_t_2", l_t_2))
  {
    std::cerr << "Rod's min length not found for Move task, please add it by l_t_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("b_2", b_2))
  {
    std::cerr << "Rod's damping coefficient not found for Move task, please add it by b_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }
  if(!tasks_move.lookupValue("k_3", k_3))
  {
    std::cerr << "Rod's elastic coefficient not found for Move task, please add it by k_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("l_t_3", l_t_3))
  {
    std::cerr << "Rod's min length not found for Move task, please add it by l_t_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("b_3", b_3))
  {
    std::cerr << "Rod's damping coefficient not found for Move task, please add it by b_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }
  if(!tasks_move.lookupValue("epsilon", epsilon))
  {
    std::cerr << "Quadrotor's Exponential gain for sign function (SMC) not found for Move task, please add it by epsilon = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("rho", rho))
  {
    std::cerr << "Quadrotor's Exponential decay rate for sign function (SMC) not found for Move task, please add it by rho = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("pid_p_psi", pid_p_psi))
  {
    std::cerr << "Quadrotor's PID proportional gain not found for Move task, please add it by pid_p_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("pid_i_psi", pid_i_psi))
  {
    std::cerr << "Quadrotor's PID integral gain not found for Move task, please add it by pid_i_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("pid_d_psi", pid_d_psi))
  {
    std::cerr << "Quadrotor's PID derivative gain not found for Move task, please add it by pid_d_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("pid_p_psi_clamp", pid_p_psi_clamp))
  {
    std::cerr << "Quadrotor's PID proportional clamp limit not found for Move task, please add it by pid_p_psi_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("pid_i_psi_clamp", pid_i_psi_clamp))
  {
    std::cerr << "Quadrotor's PID integrative clamp limit not found for Move task, please add it by pid_i_psi_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("smc_k1", smc_k1))
  {
    std::cerr << "Quadrotor's SMC k1 Gain not found for Move task, please add it by smc_k1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("smc_k2", smc_k2))
  {
    std::cerr << "Quadrotor's SMC k2 Gain not found for Move task, please add it by smc_k2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("smc_k3", smc_k3))
  {
    std::cerr << "Quadrotor's SMC k3 Gain not found for Move task, please add it by smc_k3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("smc_k4", smc_k4))
  {
    std::cerr << "Quadrotor's SMC k4 Gain not found for Move task, please add it by smc_k4 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("smc_k5", smc_k5))
  {
    std::cerr << "Quadrotor's SMC k5 Gain not found for Move task, please add it by smc_k5 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("smc_k6", smc_k6))
  {
    std::cerr << "Quadrotor's SMC k6 Gain not found for Move task, please add it by smc_k6 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("smc_k7", smc_k7))
  {
    std::cerr << "Quadrotor's SMC k7 Gain not found for Move task, please add it by smc_k7 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("smc_k8", smc_k8))
  {
    std::cerr << "Quadrotor's SMC k8 Gain not found for Move task, please add it by smc_k8 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("smc_i_k1", smc_i_k1))
  {
    std::cerr << "Quadrotor's SMC k1 Integral Gain not found for Move task, please add it by smc_i_k1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("smc_i_k2", smc_i_k2))
  {
    std::cerr << "Quadrotor's SMC k2 Integral Gain not found for Move task, please add it by smc_i_k2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("smc_i_k3", smc_i_k3))
  {
    std::cerr << "Quadrotor's SMC k3 Integral Gain not found for Move task, please add it by smc_i_k3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("smc_i_k4", smc_i_k4))
  {
    std::cerr << "Quadrotor's SMC k4 integral Gain not found for Move task, please add it by smc_i_k4 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("smc_i_k5", smc_i_k5))
  {
    std::cerr << "Quadrotor's SMC k5 Integral Gain not found for Move task, please add it by smc_i_k5 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("smc_i_k6", smc_i_k6))
  {
    std::cerr << "Quadrotor's SMC k6 Integral Gain not found for Move task, please add it by smc_i_k6 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("smc_i_k7", smc_i_k7))
  {
    std::cerr << "Quadrotor's SMC k7 Integral Gain not found for Move task, please add it by smc_i_k7 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("smc_i_k8", smc_i_k8))
  {
    std::cerr << "Quadrotor's SMC k8 Integral Gain not found for Move task, please add it by smc_i_k8 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("smc_ksign", smc_ksign))
  {
    std::cerr << "Quadrotor's SMC Gain for sign function not found for Move task, please add it by smc_ksign = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("smc_kdisc", smc_kdisc))
  {
    std::cerr << "Quadrotor's SMC Discontinuous Gain not found for Move task, please add it by smc_kdisc = value" << std::endl;
    return(EXIT_FAILURE);
  }


  if(!tasks_move.lookupValue("cb_gain_prev", cb_gain_prev))
  {
    std::cerr << "Quadrotor's Consensus Based gain for previous UAV info not found for Move task, please add it by cb_gain_prev = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_move.lookupValue("cb_gain_next", cb_gain_next))
  {
    std::cerr << "Quadrotor's Consensus Based gain for previous UAV info not found for Move task, please add it by cb_gain_next = value" << std::endl;
    return(EXIT_FAILURE);
  }


  std::cout << "Move Task parsed successfully" << std::endl;

  ros2_muavp_interface::msg::StatesInfo task;
  task.curr_state = states::MOVE;

  task.x = x;
  task.y = y;
  task.z = z;

  task.params[0] = k_1;
  task.params[1] = l_t_1;
  task.params[2] = b_1;
  task.params[3] = k_2;
  task.params[4] = l_t_2;
  task.params[5] = b_2;
  task.params[6] = k_3;
  task.params[7] = l_t_3;
  task.params[8] = b_3;
  task.params[9] = thld_x;
  task.params[10] = thld_y;
  task.params[11] = thld_z;
  task.params[12] = yaw_des_uav_1;
  task.params[13] = yaw_des_uav_2;
  task.params[14] = yaw_des_uav_3;
  // task.params[3] = smc_k2_f;
  // task.params[4] = smc_k6_f;
  task.params[15] = epsilon;
  task.params[16] = rho;
  task.params[17] = move_dist_thld;
  task.params[18] = payload_dist;
  // task.params[7] = dist_thld;
  // task.params[8] = increment_gain;


  task.ctrl_params[0] = pid_p_z;
  task.ctrl_params[1] = pid_i_z;
  task.ctrl_params[2] = pid_d_z;
  task.ctrl_params[3] = pid_p_clamp;
  task.ctrl_params[4] = pid_i_clamp;
  task.ctrl_params[5] = cb_gain_prev;
  task.ctrl_params[6] = cb_gain_next;
  task.ctrl_params[7] = smc_k1;
  task.ctrl_params[8] = smc_k2;
  task.ctrl_params[9] = smc_k3;
  task.ctrl_params[10] = smc_k4;
  task.ctrl_params[11] = smc_k5;
  task.ctrl_params[12] = smc_k6;
  task.ctrl_params[13] = smc_k7;
  task.ctrl_params[14] = smc_k8;
  task.ctrl_params[15] = smc_ksign;
  task.ctrl_params[16] = smc_kdisc;
  task.ctrl_params[17] = pid_p_psi;
  task.ctrl_params[18] = pid_i_psi;
  task.ctrl_params[19] = pid_d_psi;
  task.ctrl_params[20] = pid_p_psi_clamp;
  task.ctrl_params[21] = pid_i_psi_clamp;
  task.ctrl_params[22] = smc_i_k1;
  task.ctrl_params[23] = smc_i_k2;
  task.ctrl_params[24] = smc_i_k3;
  task.ctrl_params[25] = smc_i_k4;
  task.ctrl_params[26] = smc_i_k5;
  task.ctrl_params[27] = smc_i_k6;
  task.ctrl_params[28] = smc_i_k7;
  task.ctrl_params[29] = smc_i_k8;

  _taskListParsed.push_back(task);

  return true;
}

// -----------------------------------------------------------------------------
//@ Added Movement for Z axys By: Luigi Thea
bool Parser::parsePathMove(const Setting &tasks_path_move)
{
  std::cout << "Parsing Path Parameters for Move Task" << std::endl;
  double final_point_1_x(0.0), final_point_1_y(0.0),final_point_1_z(0.0);
  double final_point_2_x(0.0), final_point_2_y(0.0),final_point_2_z(0.0);
  double final_point_3_x(0.0), final_point_3_y(0.0),final_point_3_z(0.0);
  double final_point_4_x(0.0), final_point_4_y(0.0),final_point_4_z(0.0);
  double final_point_5_x(0.0), final_point_5_y(0.0),final_point_5_z(0.0);
  double final_point_6_x(0.0), final_point_6_y(0.0),final_point_6_z(0.0);
  string traj_type_1 = "";
  string traj_type_2 = "";
  string traj_type_3 = "";
  string traj_type_4 = "";
  string traj_type_5 = "";
  string traj_type_6 = "";
  string traj_type_7 = "";


  if(!tasks_path_move.lookupValue("final_point_1_x", final_point_1_x))
  {
    std::cerr << "Final Position (x) for first trajectory not found for Path Parameters for Move task, please add it by final_point_1_x = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_path_move.lookupValue("final_point_1_y", final_point_1_y))
  {
    std::cerr << "Final Position (y) for first trajectory not found for Path Parameters for Move task, please add it by final_point_1_y = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_path_move.lookupValue("final_point_1_z", final_point_1_z))
  {
    std::cerr << "Final Position (z) for first trajectory not found for Path Parameters for Move task, please add it by final_point_1_z = value" << std::endl;
    return(EXIT_FAILURE);
  }


  if(!tasks_path_move.lookupValue("final_point_2_x", final_point_2_x))
  {
    std::cerr << "Final Position (x) for second trajectory not found for Path Parameters for Move task, please add it by final_point_2_x = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_path_move.lookupValue("final_point_2_y", final_point_2_y))
  {
    std::cerr << "Final Position (y) for second trajectory not found for Path Parameters for Move task, please add it by final_point_2_y = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_path_move.lookupValue("final_point_2_z", final_point_2_z))
  {
    std::cerr << "Final Position (z) for first trajectory not found for Path Parameters for Move task, please add it by final_point_2_z = value" << std::endl;
    return(EXIT_FAILURE);
  }


  if(!tasks_path_move.lookupValue("final_point_3_x", final_point_3_x))
  {
    std::cerr << "Final Position (x) for third trajectory not found for Path Parameters for Move task, please add it by final_point_3_x = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_path_move.lookupValue("final_point_3_y", final_point_3_y))
  {
    std::cerr << "Final Position (y) for third trajectory not found for Path Parameters for Move task, please add it by final_point_3_y = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_path_move.lookupValue("final_point_3_z", final_point_3_z))
  {
    std::cerr << "Final Position (z) for first trajectory not found for Path Parameters for Move task, please add it by final_point_3_z = value" << std::endl;
    return(EXIT_FAILURE);
  }


  if(!tasks_path_move.lookupValue("final_point_4_x", final_point_4_x))
  {
    std::cerr << "Final Position (x) for fourth trajectory not found for Path Parameters for Move task, please add it by final_point_4_x = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_path_move.lookupValue("final_point_4_y", final_point_4_y))
  {
    std::cerr << "Final Position (y) for fourth trajectory not found for Path Parameters for Move task, please add it by final_point_4_y = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_path_move.lookupValue("final_point_4_z", final_point_4_z))
  {
    std::cerr << "Final Position (z) for first trajectory not found for Path Parameters for Move task, please add it by final_point_4_z = value" << std::endl;
    return(EXIT_FAILURE);
  }


  if(!tasks_path_move.lookupValue("final_point_5_x", final_point_5_x))
  {
    std::cerr << "Final Position (x) for fifth trajectory not found for Path Parameters for Move task, please add it by final_point_5_x = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_path_move.lookupValue("final_point_5_y", final_point_5_y))
  {
    std::cerr << "Final Position (y) for fifth trajectory not found for Path Parameters for Move task, please add it by final_point_5_y = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_path_move.lookupValue("final_point_5_z", final_point_5_z))
  {
    std::cerr << "Final Position (z) for first trajectory not found for Path Parameters for Move task, please add it by final_point_5_z = value" << std::endl;
    return(EXIT_FAILURE);
  }


  if(!tasks_path_move.lookupValue("final_point_6_x", final_point_6_x))
  {
    std::cerr << "Final Position (x) for sixth trajectory not found for Path Parameters for Move task, please add it by final_point_6_x = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_path_move.lookupValue("final_point_6_y", final_point_6_y))
  {
    std::cerr << "Final Position (y) for sixth trajectory not found for Path Parameters for Move task, please add it by final_point_6_y = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_path_move.lookupValue("final_point_6_z", final_point_6_z))
  {
    std::cerr << "Final Position (z) for first trajectory not found for Path Parameters for Move task, please add it by final_point_6_z = value" << std::endl;
    return(EXIT_FAILURE);
  }


  if(!tasks_path_move.lookupValue("traj_type_1", traj_type_1))
  {
    std::cerr << "Type for first trajectory not found for Path Parameters for Move task, please add it by traj_type_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_path_move.lookupValue("traj_type_2", traj_type_2))
  {
    std::cerr << "Type for second trajectory not found for Path Parameters for Move task, please add it by traj_type_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_path_move.lookupValue("traj_type_3", traj_type_3))
  {
    std::cerr << "Type for third trajectory not found for Path Parameters for Move task, please add it by traj_type_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_path_move.lookupValue("traj_type_4", traj_type_4))
  {
    std::cerr << "Type for fourth trajectory not found for Path Parameters for Move task, please add it by traj_type_4 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_path_move.lookupValue("traj_type_5", traj_type_5))
  {
    std::cerr << "Type for fifth trajectory not found for Path Parameters for Move task, please add it by traj_type_5 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_path_move.lookupValue("traj_type_6", traj_type_6))
  {
    std::cerr << "Type for sixth trajectory not found for Path Parameters for Move task, please add it by traj_type_6 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_path_move.lookupValue("traj_type_7", traj_type_7))
  {
    std::cerr << "Type for seventh trajectory not found for Path Parameters for Move task, please add it by traj_type_7 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  std::cout << "Path Move Task parsed successfully" << std::endl;

  ros2_muavp_interface::srv::PathInfo::Request task;
  task.curr_state = states::MOVE;

  task.params[0] = final_point_1_x;
  task.params[1] = final_point_1_y;
  task.params[2] = final_point_1_z;
  task.params[3] = final_point_2_x;
  task.params[4] = final_point_2_y;
  task.params[5] = final_point_2_z;
  task.params[6] = final_point_3_x;
  task.params[7] = final_point_3_y;
  task.params[8] = final_point_3_z;
  task.params[9] = final_point_4_x;
  task.params[10] = final_point_4_y;
  task.params[11] = final_point_4_z;
  task.params[12] = final_point_5_x;
  task.params[13] = final_point_5_y;
  task.params[14] = final_point_5_z;
  task.params[15] = final_point_6_x;
  task.params[16] = final_point_6_y;
  task.params[17] = final_point_6_z;


  task.traj_type[0] = traj_type_1;
  task.traj_type[1] = traj_type_2;
  task.traj_type[2] = traj_type_3;
  task.traj_type[3] = traj_type_4;
  task.traj_type[4] = traj_type_5;
  task.traj_type[5] = traj_type_6;
  task.traj_type[6] = traj_type_7;

  _movePathListParsed.push_back(task);

  return true;
}

// -----------------------------------------------------------------------------
// parsing land task
bool Parser::parseLand(const Setting &tasks_land)
{
  std::cout << "Parsing Land Task" << std::endl;

  double z_min(0.0);
  double k_1(0.0), l_t_1(0.0), b_1(0.0);
  double k_2(0.0), l_t_2(0.0), b_2(0.0);
  double k_3(0.0), l_t_3(0.0), b_3(0.0);
  double pid_p_z(0.0), pid_i_z(0.0), pid_d_z(0.0);

  double pid_p_clamp(0.0), pid_i_clamp(0.0);
  double smc_k1(0.0), smc_k2(0.0), smc_k3(0.0), smc_k4(0.0);
  double smc_k5(0.0), smc_k6(0.0), smc_k7(0.0), smc_k8(0.0);
  double epsilon(0.0), rho(0.0);
  double smc_i_k1(0.0), smc_i_k2(0.0), smc_i_k3(0.0), smc_i_k4(0.0);
  double smc_i_k5(0.0), smc_i_k6(0.0), smc_i_k7(0.0), smc_i_k8(0.0);
  double smc_ksign(0.0), smc_kdisc(0.0);

  double pid_p_psi(0.0), pid_i_psi(0.0), pid_d_psi(0.0);
  double pid_p_psi_clamp(0.0), pid_i_psi_clamp(0.0);

  // double x(0.0), y(0.0), z(0.0), thld_x(0.0), thld_y(0.0), thld_z(0.0);
  double yaw_des_uav_1(0.0), yaw_des_uav_2(0.0), yaw_des_uav_3(0.0);
  double cb_gain_prev(0.0), cb_gain_next(0.0);
  double tension_comp(0.0);



  if(!tasks_land.lookupValue("z_min", z_min))
  {
    std::cerr << "Desired Position not found for Land task, please add it by z_min = value" << std::endl;
    return(EXIT_FAILURE);
  }

  // if(!tasks_land.lookupValue("x", x))
  // {
  //   std::cerr << "Desired Position not found for Land task, please add it by x = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!tasks_land.lookupValue("y", y))
  // {
  //   std::cerr << "Desired Position not found for Land task, please add it by y = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!tasks_land.lookupValue("z", z))
  // {
  //   std::cerr << "Desired Position not found for Land task, please add it by z = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }

  // if(!tasks_land.lookupValue("threshold_x", thld_x))
  // {
  //   std::cerr << "Desired Threshold not found for Land task, please add it by threshold_x = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!tasks_land.lookupValue("threshold_y", thld_y))
  // {
  //   std::cerr << "Desired Threshold not found for Land task, please add it by threshold_y = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }
  //
  // if(!tasks_land.lookupValue("threshold_z", thld_z))
  // {
  //   std::cerr << "Desired Threshold not found for Land task, please add it by threshold_z = value" << std::endl;
  //   return(EXIT_FAILURE);
  // }

  if(!tasks_land.lookupValue("yaw_des_uav_1", yaw_des_uav_1))
  {
    std::cerr << "Desired Yaw for first UAV not found for Land task, please add it by yaw_des_uav_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("yaw_des_uav_2", yaw_des_uav_2))
  {
    std::cerr << "Desired Yaw for second UAV not found for Land task, please add it by yaw_des_uav_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("yaw_des_uav_3", yaw_des_uav_3))
  {
    std::cerr << "Desired Yaw for third UAV not found for Land task, please add it by yaw_des_uav_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("pid_p_z", pid_p_z))
  {
    std::cerr << "Proportional Gain not found for Land task, please add it by pid_p_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("pid_i_z", pid_i_z))
  {
    std::cerr << "Integral Gain not found for Land task, please add it by pid_i_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("pid_d_z", pid_d_z))
  {
    std::cerr << "Derivative Gain not found for Land task, please add it by pid_d_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("pid_p_clamp", pid_p_clamp))
  {
    std::cerr << "Proportional Threshold not found for Land task, please add it by pid_p_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("pid_i_clamp", pid_i_clamp))
  {
    std::cerr << "Integrative Threshold not found for Land task, please add it by pid_i_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("k_1", k_1))
  {
    std::cerr << "Rod's elastic coefficient not found for Land task, please add it by k_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("l_t_1", l_t_1))
  {
    std::cerr << "Rod's min length not found for Land task, please add it by l_t_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("b_1", b_1))
  {
    std::cerr << "Rod's damping coefficient not found for Land task, please add it by b_1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("k_2", k_2))
  {
    std::cerr << "Rod's elastic coefficient not found for Land task, please add it by k_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("l_t_2", l_t_2))
  {
    std::cerr << "Rod's min length not found for Land task, please add it by l_t_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("b_2", b_2))
  {
    std::cerr << "Rod's damping coefficient not found for Land task, please add it by b_2 = value" << std::endl;
    return(EXIT_FAILURE);
  }
  if(!tasks_land.lookupValue("k_3", k_3))
  {
    std::cerr << "Rod's elastic coefficient not found for Land task, please add it by k_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("l_t_3", l_t_3))
  {
    std::cerr << "Rod's min length not found for Land task, please add it by l_t_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("b_3", b_3))
  {
    std::cerr << "Rod's damping coefficient not found for Land task, please add it by b_3 = value" << std::endl;
    return(EXIT_FAILURE);
  }
  if(!tasks_land.lookupValue("epsilon", epsilon))
  {
    std::cerr << "Quadrotor's Exponential gain for sign function (SMC) not found for Land task, please add it by epsilon = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("rho", rho))
  {
    std::cerr << "Quadrotor's Exponential decay rate for sign function (SMC) not found for Land task, please add it by rho = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("pid_p_psi", pid_p_psi))
  {
    std::cerr << "Quadrotor's PID proportional gain not found for Land task, please add it by pid_p_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("pid_i_psi", pid_i_psi))
  {
    std::cerr << "Quadrotor's PID integral gain not found for Land task, please add it by pid_i_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("pid_d_psi", pid_d_psi))
  {
    std::cerr << "Quadrotor's PID derivative gain not found for Land task, please add it by pid_d_psi = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("pid_p_psi_clamp", pid_p_psi_clamp))
  {
    std::cerr << "Quadrotor's PID proportional clamp limit not found for Land task, please add it by pid_p_psi_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("pid_i_psi_clamp", pid_i_psi_clamp))
  {
    std::cerr << "Quadrotor's PID integrative clamp limit not found for Land task, please add it by pid_i_psi_clamp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_k1", smc_k1))
  {
    std::cerr << "Quadrotor's SMC k1 Gain not found for Land task, please add it by smc_k1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_k2", smc_k2))
  {
    std::cerr << "Quadrotor's SMC k2 Gain not found for Land task, please add it by smc_k2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_k3", smc_k3))
  {
    std::cerr << "Quadrotor's SMC k3 Gain not found for Land task, please add it by smc_k3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_k4", smc_k4))
  {
    std::cerr << "Quadrotor's SMC k4 Gain not found for Land task, please add it by smc_k4 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_k5", smc_k5))
  {
    std::cerr << "Quadrotor's SMC k5 Gain not found for Land task, please add it by smc_k5 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_k6", smc_k6))
  {
    std::cerr << "Quadrotor's SMC k6 Gain not found for Land task, please add it by smc_k6 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_k7", smc_k7))
  {
    std::cerr << "Quadrotor's SMC k7 Gain not found for Land task, please add it by smc_k7 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_k8", smc_k8))
  {
    std::cerr << "Quadrotor's SMC k8 Gain not found for Land task, please add it by smc_k8 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_i_k1", smc_i_k1))
  {
    std::cerr << "Quadrotor's SMC k1 Integral Gain not found for Land task, please add it by smc_i_k1 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_i_k2", smc_i_k2))
  {
    std::cerr << "Quadrotor's SMC k2 Integral Gain not found for Land task, please add it by smc_i_k2 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_i_k3", smc_i_k3))
  {
    std::cerr << "Quadrotor's SMC k3 Integral Gain not found for Land task, please add it by smc_i_k3 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_i_k4", smc_i_k4))
  {
    std::cerr << "Quadrotor's SMC k4 integral Gain not found for Land task, please add it by smc_i_k4 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_i_k5", smc_i_k5))
  {
    std::cerr << "Quadrotor's SMC k5 Integral Gain not found for Land task, please add it by smc_i_k5 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_i_k6", smc_i_k6))
  {
    std::cerr << "Quadrotor's SMC k6 Integral Gain not found for Land task, please add it by smc_i_k6 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_i_k7", smc_i_k7))
  {
    std::cerr << "Quadrotor's SMC k7 Integral Gain not found for Land task, please add it by smc_i_k7 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_i_k8", smc_i_k8))
  {
    std::cerr << "Quadrotor's SMC k8 Integral Gain not found for Land task, please add it by smc_i_k8 = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_ksign", smc_ksign))
  {
    std::cerr << "Quadrotor's SMC Gain for sign function not found for Land task, please add it by smc_ksign = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("smc_kdisc", smc_kdisc))
  {
    std::cerr << "Quadrotor's SMC Discontinuous Gain not found for Land task, please add it by smc_kdisc = value" << std::endl;
    return(EXIT_FAILURE);
  }


  if(!tasks_land.lookupValue("cb_gain_prev", cb_gain_prev))
  {
    std::cerr << "Quadrotor's Consensus Based gain for previous UAV info not found for Land task, please add it by cb_gain_prev = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("cb_gain_next", cb_gain_next))
  {
    std::cerr << "Quadrotor's Consensus Based gain for previous UAV info not found for Land task, please add it by cb_gain_next = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_land.lookupValue("tension_comp", tension_comp))
  {
    std::cerr << "Quadrotor's Comparing Tension not found for Land task, please add it by tension_comp = value" << std::endl;
    return(EXIT_FAILURE);
  }

  std::cout << "Land Task parsed successfully" << std::endl;

  ros2_muavp_interface::msg::StatesInfo task;
  task.curr_state = states::LAND;

  // task.x = x;
  // task.y = y;
  // task.z = z;

  task.params[0] = k_1;
  task.params[1] = l_t_1;
  task.params[2] = b_1;
  task.params[3] = k_2;
  task.params[4] = l_t_2;
  task.params[5] = b_2;
  task.params[6] = k_3;
  task.params[7] = l_t_3;
  task.params[8] = b_3;
  task.params[9] = z_min;
  task.params[10] = tension_comp;
  // task.params[9] = thld_x;
  // task.params[10] = thld_y;
  // task.params[11] = thld_z;
  task.params[12] = yaw_des_uav_1;
  task.params[13] = yaw_des_uav_2;
  task.params[14] = yaw_des_uav_3;
  // task.params[3] = smc_k2_f;
  // task.params[4] = smc_k6_f;
  task.params[15] = epsilon;
  task.params[16] = rho;
  // task.params[7] = dist_thld;
  // task.params[8] = increment_gain;


  task.ctrl_params[0] = pid_p_z;
  task.ctrl_params[1] = pid_i_z;
  task.ctrl_params[2] = pid_d_z;
  task.ctrl_params[3] = pid_p_clamp;
  task.ctrl_params[4] = pid_i_clamp;
  task.ctrl_params[5] = cb_gain_prev;
  task.ctrl_params[6] = cb_gain_next;
  task.ctrl_params[7] = smc_k1;
  task.ctrl_params[8] = smc_k2;
  task.ctrl_params[9] = smc_k3;
  task.ctrl_params[10] = smc_k4;
  task.ctrl_params[11] = smc_k5;
  task.ctrl_params[12] = smc_k6;
  task.ctrl_params[13] = smc_k7;
  task.ctrl_params[14] = smc_k8;
  task.ctrl_params[15] = smc_ksign;
  task.ctrl_params[16] = smc_kdisc;
  task.ctrl_params[17] = pid_p_psi;
  task.ctrl_params[18] = pid_i_psi;
  task.ctrl_params[19] = pid_d_psi;
  task.ctrl_params[20] = pid_p_psi_clamp;
  task.ctrl_params[21] = pid_i_psi_clamp;
  task.ctrl_params[22] = smc_i_k1;
  task.ctrl_params[23] = smc_i_k2;
  task.ctrl_params[24] = smc_i_k3;
  task.ctrl_params[25] = smc_i_k4;
  task.ctrl_params[26] = smc_i_k5;
  task.ctrl_params[27] = smc_i_k6;
  task.ctrl_params[28] = smc_i_k7;
  task.ctrl_params[29] = smc_i_k8;

  _taskListParsed.push_back(task);

  return true;
}

// -----------------------------------------------------------------------------
// parsing post-failure idle task
bool Parser::parseIdlePF(const Setting &tasks_idle)
{
  std::cout << "Parsing Idle Task After Failure" << std::endl;
  double z(0.0);
  double pid_p_z(0.0), pid_i_z(0.0), pid_d_z(0.0);
  double clamp_p_z(0.0), clamp_i_z(0.0);

  if(!tasks_idle.lookupValue("z", z))
  {
    std::cerr << "Desired Position not found for Idle task, please add it by z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_idle.lookupValue("pid_p_z", pid_p_z))
  {
    std::cerr << "Proportional Gain not found for Idle task, please add it by pid_p_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_idle.lookupValue("pid_i_z", pid_i_z))
  {
    std::cerr << "Integral Gain not found for Idle task, please add it by pid_i_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_idle.lookupValue("pid_d_z", pid_d_z))
  {
    std::cerr << "Derivative Gain not found for Idle task, please add it by pid_d_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_idle.lookupValue("clamp_p_z", clamp_p_z))
  {
    std::cerr << "Proportional Threshold not found for Idle task, please add it by clamp_p_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  if(!tasks_idle.lookupValue("clamp_i_z", clamp_i_z))
  {
    std::cerr << "Integrative Threshold not found for Idle task, please add it by clamp_i_z = value" << std::endl;
    return(EXIT_FAILURE);
  }

  std::cout << "Idle Task after Failure parsed successfully" << std::endl;

  ros2_muavp_interface::msg::StatesInfo task;
  task.curr_state = states::IDLE;
  task.z = z;
  task.ctrl_params[10] = pid_p_z;
  task.ctrl_params[11] = pid_i_z;
  task.ctrl_params[12] = pid_d_z;
  task.ctrl_params[13] = clamp_p_z;
  task.ctrl_params[14] = clamp_i_z;

  _postFailureListParsed.push_back(task);

  return true;
}
