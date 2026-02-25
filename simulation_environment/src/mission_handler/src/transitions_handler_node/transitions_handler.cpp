#include "transitions_handler.hpp"
#include "Parser.h"


// %%%--------------------------%%%
// --------- CONSTRUCTOR ----------

TransitionsHandler::TransitionsHandler(std::string agent_name) : Node("transitions_handler", "transitions_handler", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  // saving the agent name
  _agent_name = agent_name;
  // the ROS2 sub for Odometry data
  _odometry_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/" + agent_name + "/fmu/out/vehicle_odometry",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg_odom)
          {
            _drone_pose = *msg_odom;

            if(_agent_name == "x500_1")
            {
              _drone_pose.position[0] += _parameters.params[5];
              _drone_pose.position[1] += _parameters.params[6];
            }
            else if(_agent_name == "x500_2")
            {
              _drone_pose.position[0] += _parameters.params[7];
              _drone_pose.position[1] += _parameters.params[8];
            }
            else if(_agent_name == "x500_3")
            {
              _drone_pose.position[0] += _parameters.params[9];
              _drone_pose.position[1] += _parameters.params[10];
            }
            _drone_pose.position[2] += _parameters.params[11];

            quaternionToEuler(_drone_pose.q);
          });
  // get common timestamp
  // _timesync_sub = this->create_subscription<px4_msgs::msg::Timesync>("/" + agent_name + "/fmu/timesync/out", 1,
  //                 [this](const px4_msgs::msg::Timesync::UniquePtr msg_ts)
  //                 {
  //                   _timestamp.store(msg_ts->timestamp);
  //                 });


  // listening to the current payload pose
  _agent_pose_sub = this->create_subscription<ros2_muavp_interface::msg::RigidBodyPose>("/" + agent_name + "/pose",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::RigidBodyPose::SharedPtr msg_ap)
          {
            _agent_pose_data = *msg_ap;
          });

  // and the one for Force coming from slung + load system
  _force_joint_sub = this->create_subscription<ros2_muavp_interface::msg::JointForceTorque>("/" + agent_name + "/rod_force_torque",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::JointForceTorque::SharedPtr msg_force)
          {
            _joint_data = *msg_force;
          });

  _ready_params_sub = this->create_subscription<ros2_muavp_interface::msg::ReadySignals>("/agents/ready_signal_parameters",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::ReadySignals::SharedPtr msg_ready)
          {
            // getting the data from the topic and  saving it
            _ready_signal_params_data = *msg_ready;
          });

  _ready_muavp_ekf_sub = this->create_subscription<ros2_muavp_interface::msg::ReadySignals>("/agents/ready_signal_muavp_ekf",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::ReadySignals::SharedPtr msg_ready)
          {
            // getting the data from the topic and  saving it
            _ready_signal_muavp_ekf_data = *msg_ready;
          });

  // listening to other vehicles to check if they are ready for the takeoff
  _ready_takeoff_sub = this->create_subscription<ros2_muavp_interface::msg::ReadySignals>("/agents/ready_signal_takeoff",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::ReadySignals::SharedPtr msg_ready)
          {
            _ready_signal_takeoff_data = *msg_ready;
          });

  _ready_move_sub = this->create_subscription<ros2_muavp_interface::msg::ReadySignals>("/agents/ready_signal_move",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::ReadySignals::SharedPtr msg_ready)
          {
            _ready_signal_move_data = *msg_ready;
          });

  _ready_land_sub = this->create_subscription<ros2_muavp_interface::msg::ReadySignals>("/agents/ready_signal_land",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::ReadySignals::SharedPtr msg_ready)
          {
            _ready_signal_land_data = *msg_ready;
          });

  _ready_bfr_sub = this->create_subscription<ros2_muavp_interface::msg::ReadySignals>("/agents/ready_signal_back_from_recovery",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::ReadySignals::SharedPtr msg_ready)
          {
            _ready_signal_bfr_data = *msg_ready;
          });

  // listening to the current task information
  _states_info_sub = this->create_subscription<ros2_muavp_interface::msg::StatesInfo>("/" + agent_name + "/states_info",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::StatesInfo::SharedPtr msg_si)
          {
            _states_info_data = *msg_si;
          });

  // listening to the possible failure directives
  _failure_action_sub = this->create_subscription<ros2_muavp_interface::msg::FailureAction>("/" + agent_name + "/failure_action",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::FailureAction::SharedPtr msg_fa)
          {
            _failure_action_data = *msg_fa;
            _failure_occured = true;
          });

  _ready_move_pub = this->create_publisher<ros2_muavp_interface::msg::ReadySignals>("/agents/ready_signal_move", 1);
  _ready_land_pub = this->create_publisher<ros2_muavp_interface::msg::ReadySignals>("/agents/ready_signal_land", 1);
  _ready_takeoff_pub = this->create_publisher<ros2_muavp_interface::msg::ReadySignals>("/agents/ready_signal_takeoff", 1);
  _ready_bfr_pub = this->create_publisher<ros2_muavp_interface::msg::ReadySignals>("/agents/ready_signal_back_from_recovery", 1);
  _states_info_pub = this->create_publisher<ros2_muavp_interface::msg::StatesInfo>("/" + agent_name + "/states_info", 1);
  _failure_action_pub = this->create_publisher<ros2_muavp_interface::msg::FailureAction>("/" + agent_name + "/failure_action", 1);

  // initializing the clients
  _path_info_clt = this->create_client<ros2_muavp_interface::srv::PathInfo>("/" + agent_name + "/path_info_srv_clt");
  _failure_info_clt = this->create_client<ros2_muavp_interface::srv::FailureInfo>("/" + agent_name + "/failure_info_srv_clt");

  // initializing message with false values
  _ready_signal_takeoff_data.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  _ready_signal_takeoff_data.uav_1_ready = false;
  _ready_signal_takeoff_data.uav_2_ready = false;
  _ready_signal_takeoff_data.uav_3_ready = false;

  _ready_takeoff_pub->publish(_ready_signal_takeoff_data);

  // initializing message with false values
  _ready_signal_move_data.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  _ready_signal_move_data.uav_1_ready = false;
  _ready_signal_move_data.uav_2_ready = false;
  _ready_signal_move_data.uav_3_ready = false;

  _ready_move_pub->publish(_ready_signal_move_data);

  _ready_signal_bfr_data.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  _ready_signal_bfr_data.uav_1_ready = false;
  _ready_signal_bfr_data.uav_2_ready = false;
  _ready_signal_bfr_data.uav_3_ready = false;

  _ready_bfr_pub->publish(_ready_signal_bfr_data);

  _ready_signal_land_data.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  _ready_signal_land_data.uav_1_ready = false;
  _ready_signal_land_data.uav_2_ready = false;
  _ready_signal_land_data.uav_3_ready = false;

  _ready_land_pub->publish(_ready_signal_land_data);

  _ready_signal_muavp_ekf_data.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  _ready_signal_muavp_ekf_data.uav_1_ready = false;
  _ready_signal_muavp_ekf_data.uav_2_ready = false;
  _ready_signal_muavp_ekf_data.uav_3_ready = false;

  // init variables
  _min_iter_stored_param = 0;
  _min_iter_takeoff[0] = 0;
  _min_iter_takeoff[1] = 0;
  _min_iter_takeoff[2] = 0;
  _min_iter_cond_takeoff = 0;
  _min_iter_move[0] = 0;
  _min_iter_move[1] = 0;
  _min_iter_move[2] = 0;
  _min_iter_cond_move = 0;
  _min_iter_bfr[0] = 0;
  _min_iter_bfr[1] = 0;
  _min_iter_bfr[2] = 0;
  _min_iter_cond_bfr = 0;
  _newTask = true;
  _firstPrint = true;
  _first_pass = true;
  _failure_occured = false;
  _system_recovered = false;
  _around_there_tension = false;
  _seq_pre_takeoff = false;
  // parsing task list file
  parseTaskList();
  // parsing path list file
  parsePathList();
  // parsing failure list file
  parseFailureList();
  // parsing post failure tasks list file
  parsePFTaskList();

  // sending the path for move task to state handler (service)
  if(!sendPathInfoData())
    return;
  // sending the failure parameters to state handler (service)
  // and the tasks to be executed after failure
  if(!sendFailureInfoData())
    return;

  // lambda that executes the main task
  auto timercallback = [this]() -> void
  {
    transitionsController();
  };
  // executing the callback every tot ms, when the timer expires
	_timer = this->create_wall_timer(std::chrono::milliseconds(10), timercallback);
}


// %%%--------------------------%%%
// --------- STATES TASKS PARSER ----------

void TransitionsHandler::parseTaskList()
{
  Parser p;
  // getting the folder path of this package in the install directory
  std::string confPath = ament_index_cpp::get_package_share_directory("mission_handler");
    // and getting then the path to the configuration file
  confPath.append("/conf/tasks_list.cfg");
  // preparing the parse
  p.loadFile(confPath);
  // and if the parse is successful
  if(p.parse())
  {
    // getting the list of the parsed tasks
    setList(p.getTaskListParsed());
  }
  else
  {
    std::cout << "Unable to parse the file Task List" << std::endl;
    rclcpp::shutdown();
  }

}


// %%%--------------------------%%%
// --------- PATH PARSER ----------

void TransitionsHandler::parsePathList()
{
  // doing the same for recovery
  Parser p;

  std::string confPath = ament_index_cpp::get_package_share_directory("mission_handler");

  confPath.append("/conf/paths_list.cfg");

  p.loadFile(confPath);
  // and if the parse is successful
  if(p.parsePath())
  {
    // getting the list of the parsed tasks
    setPathList(p.getPathListParsed());
  }
  else
  {
    std::cout << "Unable to parse the file Path List" << std::endl;
    rclcpp::shutdown();
  }
}



// %%%--------------------------%%%
// --------- FAILURE STATES PARSER ----------

void TransitionsHandler::parseFailureList()
{
  // doing the same for recovery
  Parser p;

  std::string confPath = ament_index_cpp::get_package_share_directory("mission_handler");

  confPath.append("/conf/failure_list.cfg");

  p.loadFile(confPath);
  // and if the parse is successful
  if(p.parseFailure())
  {
    // getting the list of the parsed tasks
    setFailureList(p.getFailureListParsed());
  }
  else
  {
    std::cout << "Unable to parse the file Failure List" << std::endl;
    rclcpp::shutdown();
  }
}


// %%%--------------------------%%%
// --------- POST FAILURE TASKS PARSER ----------

void TransitionsHandler::parsePFTaskList()
{
  // and for the tasks to be executed after failure
  Parser p;

  std::string confPath = ament_index_cpp::get_package_share_directory("mission_handler");

  confPath.append("/conf/tasks_list_pf.cfg");

  p.loadFile(confPath);
  // and if the parse is successful
  if(p.parsePostFailure())
  {
    // getting the list of the parsed tasks
    setPFTasksMap(p.getPostFailureListParsed());
  }
  else
  {
    std::cout << "Unable to parse the file Failure List" << std::endl;
    rclcpp::shutdown();
  }
}


// %%%--------------------------%%%
// --------- TRANSITIONS SM HANDLER ----------

void TransitionsHandler::transitionsController()
{
  if(_failure_occured && !_system_recovered)
  {
    if(failureTransitionsChecker())
    {
      // emptying the tasks list cause now there are 2 quadrotors,
      // hence new tasks need to be taken into consideration
      _taskListParsed.clear();
      taskListAdaptation();
    }
  }
  else
  {
    // the first check is about the presence of new tasks, in this case the
    // loop works even if the task is the same.
    if(_newTask)
    {
      loadTask();
    }
    // a call to a check on the transitions from the current state is then performed
    if(transitionsChecker())
    {
      // and if the condition is verified -> change state/task
      if(_taskListParsed.size() > 0)
      {
        _taskListParsed.pop_front();
        _taskListParsed.shrink_to_fit();
        // if nothing more to do,
        if (_taskListParsed.empty()) _newTask = false;
        else                         _newTask = true;
      }
      else
      {
        if(_firstPrint)
        {
          std::cout << "Empty list, sending last task" << std::endl;
          _firstPrint = false;
        }
        // no more task to parse
        _newTask = false;
      }
    }
  }
}


// %%%--------------------------%%%
// --------- PATH MOVE INFO CLIENT ----------

bool TransitionsHandler::sendPathInfoData()
{
  // for each failure task in the parsed list
  for(int i = 0; i < (int)_movePathListParsed.size(); i++)
  {
    auto request = std::make_shared<ros2_muavp_interface::srv::PathInfo::Request>();

    request->curr_state = _movePathListParsed[i].curr_state;
    request->params = _movePathListParsed[i].params;
    request->traj_type = _movePathListParsed[i].traj_type;

    while(!_path_info_clt->wait_for_service(std::chrono::seconds(5)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Path service not available, waiting again...");
    }

    // sending the request (this node requests the StatesHandler to receive the data)
    auto future_result = _path_info_clt->async_send_request(request);
    // Wait for the result.
    while(rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Waiting to complete service");
    }
    // else
    //  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service failure_info_srv_clt");

    _path_info_result = future_result.get()->response;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Path Data Stored, Result: %ld", _path_info_result);

  }

  return true;
}


// %%%--------------------------%%%
// --------- FAILURE INFO CLIENT ----------

bool TransitionsHandler::sendFailureInfoData()
{
  // for each failure task in the parsed list
  for(int i = 0; i < (int)_failureListParsed.size(); i++)
  {
    auto request = std::make_shared<ros2_muavp_interface::srv::FailureInfo::Request>();

    request->curr_state = _failureListParsed[i].curr_state;
    request->params = _failureListParsed[i].params;
    request->ctrl_params = _failureListParsed[i].ctrl_params;

    while(!_failure_info_clt->wait_for_service(std::chrono::seconds(5)))
    {
      if (!rclcpp::ok())
      {
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
        return false;
      }
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failure service not available, waiting again...");
    }

    // sending the request (this node requests the StatesHandler to receive the data)
    auto future_result = _failure_info_clt->async_send_request(request);
    // Wait for the result.
    while(rclcpp::spin_until_future_complete(this->get_node_base_interface(), future_result, std::chrono::seconds(5)) != rclcpp::FutureReturnCode::SUCCESS)
    {
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Waiting to complete service");
    }
    // else
    //  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service failure_info_srv_clt");

    _failure_info_result = future_result.get()->response;

    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Failure Data Stored, Result: %ld", _failure_info_result);

  }

  return true;
}


// %%%--------------------------%%%
// --------- TRANSITIONS CHECKER HANDLER ----------

bool TransitionsHandler::transitionsChecker()
{
  bool trans_cond = false;
  bool ekf_ready = false;
  // depending on the state the agent is, different conditions need to be satisfied
  switch((int)_currentTask.curr_state)
  {
      case states::PARAMETERS:
        _parameters = _currentTask;
        trans_cond = _ready_signal_params_data.uav_1_ready && _ready_signal_params_data.uav_2_ready && _ready_signal_params_data.uav_3_ready && _failure_info_result;
        // checking for the readiness of the EKF (IMPORTANT)
        ekf_ready = _ready_signal_muavp_ekf_data.uav_1_ready && _ready_signal_muavp_ekf_data.uav_2_ready && _ready_signal_muavp_ekf_data.uav_3_ready;

        // sequential takeoff.
        // the first UAV doesn't need conditions
        // the remaining two gets the signal to takeoff once the previous one is
        // already in position
        // if(_agent_name == "x500_1")
        //   _seq_pre_takeoff = true;
        // else if(_agent_name == "x500_2")
        //   _seq_pre_takeoff = _ready_signal_takeoff_data.uav_1_ready;
        // else if(_agent_name == "x500_3")
        //   _seq_pre_takeoff = _ready_signal_takeoff_data.uav_2_ready;


        if(trans_cond ) //_seq_pre_takeoff
          _min_iter_stored_param++;
        else
          _min_iter_stored_param = 0;


        return (_min_iter_stored_param > 200);
        // _min_iter_stored_param++;
        // if(_min_iter_stored_param > 50)
        //   trans_cond = true;
        // return trans_cond;

      case states::PRE_TAKE_OFF:
        // this state is executed when the squad needs to perform a takeoff.
        // every agent must be in a certain position before the whole squad can carry the payload in the air.
        checkTakeoffReady();
        trans_cond = _ready_signal_takeoff_data.uav_1_ready && _ready_signal_takeoff_data.uav_2_ready && _ready_signal_takeoff_data.uav_3_ready;
        if(trans_cond)
          _min_iter_cond_takeoff++;
        else
          _min_iter_cond_takeoff = 0;

        return (_min_iter_cond_takeoff > 200);

      case states::TAKE_OFF:
        checkMoveReady();
        trans_cond = _ready_signal_move_data.uav_1_ready && _ready_signal_move_data.uav_2_ready && _ready_signal_move_data.uav_3_ready;
        if(trans_cond)
          _min_iter_cond_move++;
        else
          _min_iter_cond_move = 0;

        // return (_min_iter_cond_move > 200);
        return false;

      case states::MOVE:
        checkLandReady();
        trans_cond = _ready_signal_land_data.uav_1_ready && _ready_signal_land_data.uav_2_ready && _ready_signal_land_data.uav_3_ready;
        if(trans_cond)
          _min_iter_cond_land++;
        else
          _min_iter_cond_land = 0;

        return (_min_iter_cond_land > 200);
        // return false;

      case states::ROTATE:
        return false;

      case states::LAND:
        return false;

      case states::IDLE:
        return false;

      default:
        return false;
  }
}


// %%%--------------------------%%%
// --------- FAILURE TRANSITIONS CHECKER HANDLER ----------

bool TransitionsHandler::failureTransitionsChecker()
{
  bool fail_trans_cond = false;

  switch((int)_failure_action_data.curr_action)
  {
    case fail_states::STOP:
      return fail_trans_cond;

    case fail_states::RECOVERY:
    {
      // checkBackFromRecoveryReady();
      // fail_trans_cond = _ready_signal_bfr_data.uav_1_ready && _ready_signal_bfr_data.uav_2_ready && _ready_signal_bfr_data.uav_3_ready;
      fail_trans_cond = _ready_signal_land_data.uav_1_ready && _ready_signal_land_data.uav_2_ready && _ready_signal_land_data.uav_3_ready;
      // fail_trans_cond = false;
      if(fail_trans_cond)
        _min_iter_cond_bfr ++;
      else
        _min_iter_cond_bfr = 0;

      bool ret_cond = (fail_trans_cond && _min_iter_cond_bfr > 100);
      // return false;
      if(ret_cond)
      {
        _failure_action_data.curr_action = fail_states::LAND_F;
        _failure_action_data.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        _failure_action_pub->publish(_failure_action_data);
      }
      return ret_cond;
    }
    case fail_states::KEEP_ON:
      return fail_trans_cond;

    case fail_states::GO_TO:
      return fail_trans_cond;

    case fail_states::LAND_F:
      return fail_trans_cond;

    default:
      return false;
  }
}

// %%%--------------------------%%%
// --------- TAKEOFF STATE TRANSITION ----------

void TransitionsHandler::checkTakeoffReady()
{
  tensionRodComputation();
  // I just need to know one time if I am around that tension, meaning that the
  // control law has taken around the desired point where cables are taut
  if(_tensions.tension_norm >= _currentTask.params[17]*0.7) // 70%
    _around_there_tension = true;

  // depending on the agent, different conditions on the position must be valid
  if(_agent_name == "x500_1")
  {
      double corrected_task_x = _currentTask.x*cosf(_currentTask.params[12]) + _parameters.params[5];
      double corrected_task_y = _currentTask.y*sinf(_currentTask.params[12]) + _parameters.params[6];
      // and if the condition is verified, a counter is increased
      // std::cout << "x500_1_dist_to_goal: " << sqrt(pow(_drone_pose.x - _parameters.params[5] - corrected_task_x,2) + pow(_drone_pose.y - _parameters.params[6] - corrected_task_y,2)) << std::endl;
      // if(sqrt(pow(_drone_pose.position[0] - _parameters.params[5] - corrected_task_x,2) + pow(_drone_pose.position[1] - _parameters.params[6] - corrected_task_y,2)) <= _currentTask.params[10] && abs(_drone_pose.position[2] - _currentTask.z) <= _currentTask.params[11])
      if(abs(_agent_pose_data.position[0] - corrected_task_x) <= _currentTask.params[9] && abs(_agent_pose_data.position[1] - corrected_task_y) <= _currentTask.params[10] && abs(_agent_pose_data.position[2] - _currentTask.z) <= _currentTask.params[11])
        _min_iter_takeoff[0]++;
      else
        _min_iter_takeoff[0] = 0;

      // the idea is: if the condition is valid for a certain number of consecutive
      // iterations, aka the agent is in a "stable" situation, then send the ready signal
      if(_min_iter_takeoff[0] >= 200 && _around_there_tension)
        sendReadySignalTakeoff();
  }
  else if(_agent_name == "x500_2")
  {
      double corrected_task_x = _currentTask.x*cosf(_currentTask.params[13]) + _parameters.params[7];
      double corrected_task_y = _currentTask.y*sinf(_currentTask.params[13]) + _parameters.params[8];
      // if(sqrt(pow(_drone_pose.position[0] - _parameters.params[7] - corrected_task_x,2) + pow(_drone_pose.position[1] - _parameters.params[8] - corrected_task_y,2)) <= _currentTask.params[9] && abs(_drone_pose.position[2] - _currentTask.z) <= _currentTask.params[11])
      if(abs(_agent_pose_data.position[0] - corrected_task_x) <= _currentTask.params[9] && abs(_agent_pose_data.position[1] - corrected_task_y) <= _currentTask.params[10] && abs(_agent_pose_data.position[2] - _currentTask.z) <= _currentTask.params[11])
        _min_iter_takeoff[1]++;
      else
        _min_iter_takeoff[1] = 0;

      if(_min_iter_takeoff[1] >= 200 && _around_there_tension)
        sendReadySignalTakeoff();
  }
  else if(_agent_name == "x500_3")
  {
      double corrected_task_x = _currentTask.x*cosf(_currentTask.params[14]) + _parameters.params[9];
      double corrected_task_y = _currentTask.y*sinf(_currentTask.params[14]) + _parameters.params[10];
      // if(sqrt(pow(_drone_pose.position[0] - _parameters.params[9] - corrected_task_x,2) + pow(_drone_pose.position[1] - _parameters.params[10] - corrected_task_y,2)) <= _currentTask.params[10] && abs(_drone_pose.position[2] - _currentTask.z) <= _currentTask.params[11])
      if(abs(_agent_pose_data.position[0] - corrected_task_x) <= _currentTask.params[9] && abs(_agent_pose_data.position[1] - corrected_task_y) <= _currentTask.params[10] && abs(_agent_pose_data.position[2] - _currentTask.z) <= _currentTask.params[11])
        _min_iter_takeoff[2]++;
      else
        _min_iter_takeoff[2] = 0;

      if(_min_iter_takeoff[2] >= 200 && _around_there_tension)
        sendReadySignalTakeoff();
  }

  // std::cout << _agent_name << " z diff: " << _drone_pose.z - _currentTask.z << std::endl;
}


// %%%--------------------------%%%
// --------- MOVE STATE TRANSITION ----------

void TransitionsHandler::checkMoveReady()
{
  // depending on the agent, different conditions on the position must be valid
  if(_agent_name == "x500_1")
  {
      double corrected_task_x = _currentTask.x*cosf(_currentTask.params[12]) + _parameters.params[5];
      double corrected_task_y = _currentTask.y*sinf(_currentTask.params[12]) + _parameters.params[6];
      // and if the condition is verified, a counter is increased
      // if(sqrt(pow(_drone_pose.position[0] - _parameters.params[5] - corrected_task_x,2) + pow(_drone_pose.position[1] - _parameters.params[6] - corrected_task_y,2)) <= _currentTask.params[10] && abs(_drone_pose.position[2] - _currentTask.z) <= _currentTask.params[11])
      if(abs(_agent_pose_data.position[0] - corrected_task_x) <= _currentTask.params[9] && abs(_agent_pose_data.position[1] - corrected_task_y) <= _currentTask.params[10] && abs(_agent_pose_data.position[2] - _currentTask.z) <= _currentTask.params[11])
        _min_iter_move[0]++;
      else
        _min_iter_move[0] = 0;
      // the idea is: if the condition is valid for a certain number of consecutive
      // iterations, aka the agent is in a "stable" situation, then send the ready signal
      if(_min_iter_move[0] >= 200)
        sendReadySignalMove();
  }
  else if(_agent_name == "x500_2")
  {
      double corrected_task_x = _currentTask.x*cosf(_currentTask.params[13]) + _parameters.params[7];
      double corrected_task_y = _currentTask.y*sinf(_currentTask.params[13]) + _parameters.params[8];
      // if(sqrt(pow(_drone_pose.position[0] - _parameters.params[7] - corrected_task_x,2) + pow(_drone_pose.position[1] - _parameters.params[8] - corrected_task_y,2)) <= _currentTask.params[9] && abs(_drone_pose.position[2] - _currentTask.z) <= _currentTask.params[11])
      if(abs(_agent_pose_data.position[0] - corrected_task_x) <= _currentTask.params[9] && abs(_agent_pose_data.position[1] - corrected_task_y) <= _currentTask.params[10] && abs(_agent_pose_data.position[2] - _currentTask.z) <= _currentTask.params[11])
        _min_iter_move[1]++;
      else
        _min_iter_move[1] = 0;

      if(_min_iter_move[1] >= 200)
        sendReadySignalMove();
  }
  else if(_agent_name == "x500_3")
  {
      double corrected_task_x = _currentTask.x*cosf(_currentTask.params[14]) + _parameters.params[9];
      double corrected_task_y = _currentTask.y*sinf(_currentTask.params[14]) + _parameters.params[10];
      // if(sqrt(pow(_drone_pose.position[0] - _parameters.params[9] - corrected_task_x,2) + pow(_drone_pose.position[1] - _parameters.params[10] - corrected_task_y,2)) <= _currentTask.params[10] && abs(_drone_pose.position[2] - _currentTask.z) <= _currentTask.params[11])
      if(abs(_agent_pose_data.position[0] - corrected_task_x) <= _currentTask.params[9] && abs(_agent_pose_data.position[1] - corrected_task_y) <= _currentTask.params[10] && abs(_agent_pose_data.position[2] - _currentTask.z) <= _currentTask.params[11])
        _min_iter_move[2]++;
      else
        _min_iter_move[2] = 0;

      if(_min_iter_move[2] >= 200)
        sendReadySignalMove();
  }
}

// %%%--------------------------%%%
// --------- LAND STATE TRANSITION ----------

void TransitionsHandler::checkLandReady()
{
  double corrected_task_x = _currentTask.x - _currentTask.params[18]*cos(_euler_angles[2]);
  double corrected_task_y = _currentTask.y - _currentTask.params[18]*sin(_euler_angles[2]);
  // depending on the agent, different conditions on the position must be valid
  if(_agent_name == "x500_1")
  {
      // and if the condition is verified, a counter is increased
      // if(sqrt(pow(_drone_pose.position[0] - _parameters.params[5] - corrected_task_x,2) + pow(_drone_pose.position[1] - _parameters.params[6] - corrected_task_y,2)) <= _currentTask.params[10] && abs(_drone_pose.position[2] - _currentTask.z) <= _currentTask.params[11])
      if(abs(_agent_pose_data.position[0] - corrected_task_x) <= _currentTask.params[9] && abs(_agent_pose_data.position[1] - corrected_task_y) <= _currentTask.params[10] && abs(_agent_pose_data.position[2] - _currentTask.z) <= _currentTask.params[11])
        _min_iter_land[0]++;
      else
        _min_iter_land[0] = 0;
      // the idea is: if the condition is valid for a certain number of consecutive
      // iterations, aka the agent is in a "stable" situation, then send the ready signal
      if(_min_iter_land[0] >= 200)
        sendReadySignalLand();
  }
  else if(_agent_name == "x500_2")
  {
      // if(sqrt(pow(_drone_pose.position[0] - _parameters.params[7] - corrected_task_x,2) + pow(_drone_pose.position[1] - _parameters.params[8] - corrected_task_y,2)) <= _currentTask.params[9] && abs(_drone_pose.position[2] - _currentTask.z) <= _currentTask.params[11])
      if(abs(_agent_pose_data.position[0] - corrected_task_x) <= _currentTask.params[9] && abs(_agent_pose_data.position[1] - corrected_task_y) <= _currentTask.params[10] && abs(_agent_pose_data.position[2] - _currentTask.z) <= _currentTask.params[11])
        _min_iter_land[1]++;
      else
        _min_iter_land[1] = 0;

      if(_min_iter_land[1] >= 200)
        sendReadySignalLand();
  }
  else if(_agent_name == "x500_3")
  {
      // if(sqrt(pow(_drone_pose.position[0] - _parameters.params[9] - corrected_task_x,2) + pow(_drone_pose.position[1] - _parameters.params[10] - corrected_task_y,2)) <= _currentTask.params[10] && abs(_drone_pose.position[2] - _currentTask.z) <= _currentTask.params[11])
      if(abs(_agent_pose_data.position[0] - corrected_task_x) <= _currentTask.params[9] && abs(_agent_pose_data.position[1] - corrected_task_y) <= _currentTask.params[10] && abs(_agent_pose_data.position[2] - _currentTask.z) <= _currentTask.params[11])
        _min_iter_land[2]++;
      else
        _min_iter_land[2] = 0;

      if(_min_iter_land[2] >= 200)
        sendReadySignalLand();
  }
}

// %%%--------------------------%%%
// --------- BACK FROM RECOVERY FAILURE STATE TRANSITION ----------

void TransitionsHandler::checkBackFromRecoveryReady()
{
  // only the non-failed agents can enter this function

  // WARN: SUPPOSING RECOVERY FROM MOVE, HENCE THE ALTITUDE REFERENCE IS TOOK
  // FROM THE CURRENT TASK LIST
  if(_agent_name == "x500_1")
  {
    if(abs(_drone_pose.position[2] - _currentTask.z) <= 0.03 && abs(_drone_pose.velocity[2]) <= 0.11) // 0.05
      _min_iter_bfr[0]++;
    else
      _min_iter_bfr[0] = 0;

    if(_min_iter_bfr[0] >= 300)
      sendReadySignalBackFromRecovery();
  }
  else if(_agent_name == "x500_2")
  {
    if(abs(_drone_pose.position[2] - _currentTask.z) <= 0.03 && abs(_drone_pose.velocity[2]) <= 0.11) // 0.05
      _min_iter_bfr[1]++;
    else
      _min_iter_bfr[1] = 0;

    if(_min_iter_bfr[1] >= 300)
      sendReadySignalBackFromRecovery();
  }
  else if(_agent_name == "x500_3")
  {
    if(abs(_drone_pose.position[2] - _currentTask.z) <= 0.03 && abs(_drone_pose.velocity[2]) <= 0.11) // 0.05
      _min_iter_bfr[2]++;
    else
      _min_iter_bfr[2] = 0;

    if(_min_iter_bfr[2] >= 300)
      sendReadySignalBackFromRecovery();
  }
}


// %%%--------------------------%%%
// --------- TAKEOFF SIGNAL SENDER ----------

void TransitionsHandler::sendReadySignalTakeoff()
{
  _ready_signal_takeoff_data.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  // filling the right variable, depening on the agent accessing this function
  if(_agent_name == "x500_1" && !_ready_signal_takeoff_data.uav_1_ready)
    _ready_signal_takeoff_data.uav_1_ready = true;
  else if(_agent_name == "x500_2" && !_ready_signal_takeoff_data.uav_2_ready)
    _ready_signal_takeoff_data.uav_2_ready = true;
  else if(_agent_name == "x500_3" && !_ready_signal_takeoff_data.uav_3_ready)
    _ready_signal_takeoff_data.uav_3_ready = true;

  _ready_takeoff_pub->publish(_ready_signal_takeoff_data);
}


// %%%--------------------------%%%
// --------- MOVE SIGNAL SENDER ----------

void TransitionsHandler::sendReadySignalMove()
{
  _ready_signal_move_data.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  // filling the right variable, depening on the agent accessing this function
  if(_agent_name == "x500_1" && !_ready_signal_move_data.uav_1_ready)
    _ready_signal_move_data.uav_1_ready = true;
  else if(_agent_name == "x500_2" && !_ready_signal_move_data.uav_2_ready)
    _ready_signal_move_data.uav_2_ready = true;
  else if(_agent_name == "x500_3" && !_ready_signal_move_data.uav_3_ready)
    _ready_signal_move_data.uav_3_ready = true;

  _ready_move_pub->publish(_ready_signal_move_data);
}


// %%%--------------------------%%%
// --------- LAND SIGNAL SENDER ----------

void TransitionsHandler::sendReadySignalLand()
{
  _ready_signal_land_data.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  // filling the right variable, depening on the agent accessing this function
  if(_agent_name == "x500_1" && !_ready_signal_land_data.uav_1_ready)
    _ready_signal_land_data.uav_1_ready = true;
  else if(_agent_name == "x500_2" && !_ready_signal_land_data.uav_2_ready)
    _ready_signal_land_data.uav_2_ready = true;
  else if(_agent_name == "x500_3" && !_ready_signal_land_data.uav_3_ready)
    _ready_signal_land_data.uav_3_ready = true;

  _ready_land_pub->publish(_ready_signal_land_data);
}


// %%%--------------------------%%%
// --------- BACK FROM RECOVERY FAILURE STATE SIGNAL SENDER ----------

void TransitionsHandler::sendReadySignalBackFromRecovery()
{
  _ready_signal_bfr_data.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  // filling the right variable, depening on the agent accessing this function
  // setting also to true the condition of the failed agent, cause it will not end up here
  if((_agent_name == "x500_1" || !(int)_failure_action_data.agents_alive[0]) && !_ready_signal_bfr_data.uav_1_ready)
    _ready_signal_bfr_data.uav_1_ready = true;
  else if((_agent_name == "x500_2" || !(int)_failure_action_data.agents_alive[1]) && !_ready_signal_bfr_data.uav_2_ready)
    _ready_signal_bfr_data.uav_2_ready = true;
  else if((_agent_name == "x500_3" || !(int)_failure_action_data.agents_alive[2]) && !_ready_signal_bfr_data.uav_3_ready)
    _ready_signal_bfr_data.uav_3_ready = true;

  _ready_bfr_pub->publish(_ready_signal_bfr_data);

}

// %%%--------------------------%%%
// --------- FUNCTION THAT ADAPTS THE TASK LIST ----------

void TransitionsHandler::taskListAdaptation()
{
  // depending on the current failure state, different actions are taken
  // basically: if there is the need to change the task rather than coming back
  // to the pre-failure one, creating a new state to fill the queue with.
  // putting it in front of the queue then, and turning off the failure bool
  // lastly, triggering the _newTask variable, to notify the new task coming
  ros2_muavp_interface::msg::StatesInfo new_task_land;
  ros2_muavp_interface::msg::StatesInfo new_task_idle;

  switch((int)_failure_action_data.curr_action)
  {
    case fail_states::STOP:
      break;

    case fail_states::RECOVERY:
      {
        // in the recovery case, I want the system to be IDLE after
        // (normally the system should keep on the mission with MOVE)
        // here inserting a list of consecutive tasks the remaining agents
        // have to perform. Starting with the landing (last task)
        new_task_land.curr_state = states::LAND;
        new_task_land.z = _pfTasksMapParsed[states::LAND].z;
        new_task_land.params = _pfTasksMapParsed[states::LAND].params;

        auto iterator = _taskListParsed.begin();
        _taskListParsed.insert(iterator, new_task_land);

        // then IDLE (first task)
        new_task_idle.curr_state = states::IDLE;
        new_task_idle.x = _pfTasksMapParsed[states::IDLE].x;
        new_task_idle.y = _pfTasksMapParsed[states::IDLE].y;
        new_task_idle.z = _pfTasksMapParsed[states::IDLE].z;
        new_task_idle.yaw = _pfTasksMapParsed[states::IDLE].yaw;
        new_task_idle.params = _pfTasksMapParsed[states::IDLE].params;
        new_task_idle.ctrl_params = _pfTasksMapParsed[states::IDLE].ctrl_params;

        iterator = _taskListParsed.begin();
        _taskListParsed.insert(iterator, new_task_idle);
      }
      break;

    case fail_states::KEEP_ON:
      break;

    case fail_states::GO_TO:
      break;

    case fail_states::LAND_F:
      break;

    default:
      break;

  }
  _system_recovered = true;
  _failure_occured = false;
  _newTask = true;
}

// %%%--------------------------%%%
// --------- ACTUAL TASK SENDER ----------

void TransitionsHandler::loadTask()
{
  _currentTask = _taskListParsed[0];
  _currentTask.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  _states_info_pub->publish(_currentTask);
}


// %%%--------------------------%%%
// --------- STATES LIST SETTER ----------

void TransitionsHandler::setList(std::deque<ros2_muavp_interface::msg::StatesInfo> _list)
{
  _taskListParsed = _list;
}

// %%%--------------------------%%%
// --------- MOVE TRAJECTORY LIST SETTER ----------

void TransitionsHandler::setPathList(std::deque<ros2_muavp_interface::srv::PathInfo::Request> _list)
{
  _movePathListParsed = _list;
}

// %%%--------------------------%%%
// --------- FAILURES LIST SETTER ----------

void TransitionsHandler::setFailureList(std::deque<ros2_muavp_interface::srv::FailureInfo::Request> _list)
{
  _failureListParsed = _list;
}

// %%%--------------------------%%%
// --------- POST FAILURE TASKS LIST SETTER ----------

void TransitionsHandler::setPFTasksMap(std::deque<ros2_muavp_interface::msg::StatesInfo> _list)
{
  for(int i = 0; i < _list.size(); i++)
  {
    _pfTasksMapParsed[(int)_list[i].curr_state] = _list[i];
  }
}

// %%%--------------------------%%%
// --------- TENSION ON ROD IN FRAMES ----------

void TransitionsHandler::tensionRodComputation()
{

  // transforming the tension data from joint's frame to body one
  double alpha = _joint_data.orientation[0];
  double beta = _joint_data.orientation[1];
  double gamma = _joint_data.orientation[2];

  // defining the variables needed to transform the tension
  VectorXd measured_tension = VectorXd(3);
  VectorXd body_tension_body_frame = VectorXd(3);
  MatrixXd R_x = MatrixXd(3,3);
  MatrixXd R_y = MatrixXd(3,3);
  MatrixXd R_z = MatrixXd(3,3);
  MatrixXd R = MatrixXd(3,3);
  MatrixXd R_I = MatrixXd(3,3);

  // storing the measures in a VectorXd object, so to be handled properly
  measured_tension << _joint_data.force_joint_1[0], _joint_data.force_joint_1[1], _joint_data.force_joint_1[2];

  // filling the single rotational matrices, for roll
  R_x << 1, 0, 0,
        0, cosf(alpha), -sinf(alpha),
        0, sinf(alpha), cosf(alpha);
  // for pitch
  R_y << cosf(beta), 0, sinf(beta),
        0, 1, 0,
        -sinf(beta), 0, cosf(beta);
  // and for yaw
  R_z << cosf(gamma), -sinf(gamma), 0,
        sinf(gamma), cosf(gamma), 0,
        0, 0, 1;
  // the rotational matrix between the cable and the UAV is the product of the three
  // following the chosen ZYX convention.
  R = R_z * R_y * R_x;
  // rotating the measures
  body_tension_body_frame = R*measured_tension;
  // and assigning them. changing from the Gazebo body frame (X - Front, Y - Left, Z - Up)
  // to the desired one (X - Front, Y - Right, Z - Down)
  _tensions.body_tension << body_tension_body_frame[0], -body_tension_body_frame[1], -body_tension_body_frame[2];

  // BONUS: (not needed though) Body to Inertial rotation.
  // getting the UAV's angles
  double phi = _euler_angles[0];
  double theta = _euler_angles[1];
  double psi = _euler_angles[2];

  // building the Rotation matrix from body frame to inertial frame (same as the product of previously declared rotational matrices)
  R_I << cos(psi)*cos(theta), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi),
      cos(theta)*sin(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi),
      -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta);
  // rotating it and getting the measures in the inertial frame as well.
  _tensions.inert_tension = R_I*_tensions.body_tension;

  // taking the norm finally
  double body_tension_norm = norm(_tensions.body_tension);
  // double inert_tension_norm = norm(_tensions.inert_tension);
  _tensions.tension_norm = body_tension_norm;
  // _tensions.tension_norm = inert_tension_norm;

}

// %%%--------------------------%%%
// --------- EULER ANGLES FROM QUATERNIONS ----------

void TransitionsHandler::quaternionToEuler(std::array<float, 4> q)
{
  double euler_angles[3];
  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
  double cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
  euler_angles[0] = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
  if (std::abs(sinp) >= 1)
      euler_angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
      euler_angles[1] = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
  double cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
  euler_angles[2] = atan2(siny_cosp, cosy_cosp);

  if(euler_angles[2] < 0.0)
    euler_angles[2] += 2*PI;

  _euler_angles[0] = euler_angles[0];
  _euler_angles[1] = euler_angles[1];
  _euler_angles[2] = euler_angles[2];
}

// %%%--------------------------%%%
// --------- NORM ----------

double TransitionsHandler::norm(VectorXd vector)
{
  return sqrt(pow(vector(0),2) + pow(vector(1),2) + pow(vector(2),2));
}


// %%%--------------------------%%%
// --------- DESTRUCTOR ----------

TransitionsHandler::~TransitionsHandler()
{

}
