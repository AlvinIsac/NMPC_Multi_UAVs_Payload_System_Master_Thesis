#include "states_handler.hpp"


// %%%--------------------------%%%
// --------- CONSTRUCTOR ----------

StatesHandler::StatesHandler(std::string agent_name) : Node("states_handler", "states_handler", rclcpp::NodeOptions().use_intra_process_comms(true))
{
  // saving the agent name
  _agent_name = agent_name;
  if(_agent_name == "x500_1")
  {
    _prev_agent_name = "x500_3";
    _next_agent_name = "x500_2";
  }
  else if(_agent_name == "x500_2")
  {
    _prev_agent_name = "x500_1";
    _next_agent_name = "x500_3";
  }
  else if(_agent_name == "x500_3")
  {
    _prev_agent_name = "x500_2";
    _next_agent_name = "x500_1";
  }

  // get common timestamp
  // _timesync_sub = this->create_subscription<px4_msgs::msg::Timesync>("/" + agent_name + "/fmu/timesync/out", 1, // /Timesync_PubSubTopic
  //                 [this](const px4_msgs::msg::Timesync::UniquePtr msg_ts)
  //                 {
  //                   _timestamp.store(msg_ts->timestamp);
  //                 });

  // listening to the current task information
  _states_info_sub = this->create_subscription<ros2_muavp_interface::msg::StatesInfo>("/" + agent_name + "/states_info",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::StatesInfo::SharedPtr msg_si)
          {
            // getting the data from the topic and saving it
            _states_info_data = *msg_si;

            // ---- JUST FOR FAILURES ----
            // checking if it's needed to come back to state tasks, aka if the
            // system has recovered from the failure state
            if(_ready_signal_bfr_data.uav_1_ready && _ready_signal_bfr_data.uav_2_ready && _ready_signal_bfr_data.uav_3_ready)
            {
              // TODO: check on timestamps OR clocks instead of on states
              if((int)_states_info_data.curr_state == states::IDLE)
              {
                _system_recovered = true;
                _states_info_data.x = _msg_stop.setpoint[0];
                _states_info_data.y = _msg_stop.setpoint[1];
              }
            }
          });
  
  //changes here
//   failure_sub_ = this->create_subscription<ros2_muavp_interface::msg::FailureCheck>(
//         "/agents/failure_check", 10, std::bind(&StatesHandler::failureCallback, this, std::placeholders::_1));
// // changes end

  _ready_params_sub = this->create_subscription<ros2_muavp_interface::msg::ReadySignals>("/agents/ready_signal_parameters",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::ReadySignals::SharedPtr msg_ready)
          {
            _ready_signal_params_data = *msg_ready;
          });

  // listening to the current payload pose
  _payload_pose_sub = this->create_subscription<ros2_muavp_interface::msg::RigidBodyPose>("/payload/pose",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::RigidBodyPose::SharedPtr msg_pp)
          {
            _payload_pose_data = *msg_pp;
          });

  // listening to the current payload pose
  _agent_pose_sub = this->create_subscription<ros2_muavp_interface::msg::RigidBodyPose>("/" + agent_name + "/pose",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::RigidBodyPose::SharedPtr msg_ap)
          {
            _agent_pose_data = *msg_ap;
            // saving agent "spawn" position
            if(_first_msg)
            {
              _agent_pose_home = *msg_ap;
              _first_msg = false;
            }
          });


  _prev_agent_pose_sub = this->create_subscription<ros2_muavp_interface::msg::RigidBodyPose>("/" + _prev_agent_name + "/pose",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::RigidBodyPose::SharedPtr msg_ap)
          {
            _prev_agent_pose_data = *msg_ap;
          });

  _next_agent_pose_sub = this->create_subscription<ros2_muavp_interface::msg::RigidBodyPose>("/" + _next_agent_name + "/pose",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::RigidBodyPose::SharedPtr msg_ap)
          {
            _next_agent_pose_data = *msg_ap;
          });

  _ei_prev_agent_sub = this->create_subscription<ros2_muavp_interface::msg::ExchangeInfo>("/" + _prev_agent_name + "/exchange_info",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::ExchangeInfo::SharedPtr msg_ap)
          {
            _ei_prev_agent_data = *msg_ap;
          });

  _ei_next_agent_sub = this->create_subscription<ros2_muavp_interface::msg::ExchangeInfo>("/" + _next_agent_name + "/exchange_info",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::ExchangeInfo::SharedPtr msg_ap)
          {
            _ei_next_agent_data = *msg_ap;
          });

  // listening to the current odometry information
  _odom_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/" + agent_name + "/fmu/out/vehicle_odometry", // /VehicleOdometry_PubSubTopic
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg_odom)
          {
            _odom_data = *msg_odom;
            fixingOdometry(_odom_data, _agent_name);

            quaternionToEuler(_odom_data.q, 0);

            // angular speeds in body-axis
            VectorXd body_rates = VectorXd(3);
            body_rates << _odom_data.angular_velocity[0], _odom_data.angular_velocity[1], _odom_data.angular_velocity[2];

            // transforming the angular rates in euler angles rates through the relation omega = W_eta * eta_dot
            MatrixXd W_eta_inv = MatrixXd(3,3);

            double sec_theta = 1/cos(_euler_angles[1]);

            W_eta_inv << 1, -sin(_euler_angles[0])*tan(_euler_angles[1]), -cos(_euler_angles[0])*tan(_euler_angles[1]),
                        0, cos(_euler_angles[0]), -sin(_euler_angles[0]),
                        0, sin(_euler_angles[0])*sec_theta, cos(_euler_angles[0])*sec_theta;

            // applying the inverse matrix to the body angular rates
            _euler_rates = W_eta_inv*body_rates;

          });

  // listening to the current odometry information of the previous agent in the communication graph
  _odom_pa_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/" + _prev_agent_name + "/fmu/out/vehicle_odometry", // /VehicleOdometry_PubSubTopic
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg_pa_odom)
          {
            _pa_odom_data = *msg_pa_odom;
            fixingOdometry(_pa_odom_data, _prev_agent_name);

            quaternionToEuler(_pa_odom_data.q, -1);
          });

  // listening to the current odometry information of the next agent in the communication graph
  _odom_na_sub = this->create_subscription<px4_msgs::msg::VehicleOdometry>("/" + _next_agent_name + "/fmu/out/vehicle_odometry", // /VehicleOdometry_PubSubTopic
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const px4_msgs::msg::VehicleOdometry::SharedPtr msg_na_odom)
          {
            _na_odom_data = *msg_na_odom;
            fixingOdometry(_na_odom_data, _next_agent_name);

            quaternionToEuler(_na_odom_data.q, 1);
          });

  // listening to the current odometry information
  _act_armed_sub = this->create_subscription<px4_msgs::msg::ActuatorArmed>("/" + agent_name + "/fmu/out/actuator_armed", // /ActuatorArmed_PubSubTopic
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const px4_msgs::msg::ActuatorArmed::SharedPtr msg_aa)
          {
            _actuator_armed_data = *msg_aa;
          });

  // and the one for Force coming from slung + load system
  _force_joint_sub = this->create_subscription<ros2_muavp_interface::msg::JointForceTorque>("/" + agent_name + "/rod_force_torque",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::JointForceTorque::SharedPtr msg_force)
          {
            _joint_data = *msg_force;
          });

  // needed to switch the controllers in the move status
  _ready_move_sub = this->create_subscription<ros2_muavp_interface::msg::ReadySignals>("/agents/ready_signal_move",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::ReadySignals::SharedPtr msg_ready)
          {
            _ready_signal_move_data = *msg_ready;
          });


  _ready_bfr_sub = this->create_subscription<ros2_muavp_interface::msg::ReadySignals>("/agents/ready_signal_back_from_recovery",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::ReadySignals::SharedPtr msg_ready)
          {
            _ready_signal_bfr_data = *msg_ready;
          });

  // listening to the possible failure directives
  _failure_action_sub = this->create_subscription<ros2_muavp_interface::msg::FailureAction>("/" + agent_name + "/failure_action",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::FailureAction::SharedPtr msg_fa)
          {
            _failure_action_data = *msg_fa;
            _failure_occured = true;
          });

  _ready_land_sub = this->create_subscription<ros2_muavp_interface::msg::ReadySignals>("/agents/ready_signal_land",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::ReadySignals::SharedPtr msg_ready)
          {
            _ready_signal_land_data = *msg_ready;
          });

  _setpoint_sub = this->create_subscription<ros2_muavp_interface::msg::InputSetpoint>("/" + agent_name + "/InputSetpoint_PubSubTopic",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::InputSetpoint::SharedPtr msg_sp)
          {
            _setpoint_msg = *msg_sp;
          });

  _ei_agent_pub = this->create_publisher<ros2_muavp_interface::msg::ExchangeInfo>("/" + agent_name + "/exchange_info", 1);
  _setpoint_pub = this->create_publisher<ros2_muavp_interface::msg::InputSetpoint>("/" + agent_name + "/input_setpoint_pid", 1);
  _ready_params_pub = this->create_publisher<ros2_muavp_interface::msg::ReadySignals>("/agents/ready_signal_parameters", 1);
  _ready_land_pub = this->create_publisher<ros2_muavp_interface::msg::ReadySignals>("/agents/ready_signal_land", 1);
  _failure_pub = this->create_publisher<ros2_muavp_interface::msg::FailureCheck>("/agents/failure_check", 1);

  // _ready_back_from_failure_pub = this->create_publisher<ros2_muavp_interface::msg::ReadySignals>("/agents/ready_back_from_failure", 1);

  // initializing the path info service (this is the node that receives the data - aka asks for request - service)
  _path_info_srv = this->create_service<ros2_muavp_interface::srv::PathInfo>("/" + agent_name + "/path_info_srv_clt",
          [this](const std::shared_ptr<ros2_muavp_interface::srv::PathInfo::Request> request, std::shared_ptr<ros2_muavp_interface::srv::PathInfo::Response> response)
          {
            // getting the data from the server and saving it
            _path_info_data[(int)request->curr_state] = *request;
            // _failure_info_data = *request;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Storing Path data");
            // sending the feedback
            response->response = true;
          });

  // initializing the failure info service (this is the node that receives the data - aka asks for request - service)
  _failure_info_srv = this->create_service<ros2_muavp_interface::srv::FailureInfo>("/" + agent_name + "/failure_info_srv_clt",
          [this](const std::shared_ptr<ros2_muavp_interface::srv::FailureInfo::Request> request, std::shared_ptr<ros2_muavp_interface::srv::FailureInfo::Response> response)
          {
            // getting the data from the server and saving it
            _failure_info_data[(int)request->curr_state] = *request;
            // _failure_info_data = *request;
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Storing Failure data");
            // sending the feedback
            response->response = true;
          });

  _integral_dist_z = 0;
  _exact_integral_dist_z = 0;
  _a = 0;
  _counter_Jacobian = 0;

  _x_des = 0.0;
  _y_des = 0.0;
  _yaw_des = 0.0;

  // _curr = _clocksis.now();

  // some boolean flags
  _first_pre = true;
  _first_take = true;
  _first_move = true;
  _first_move_fc = true;
  _first_land = true;
  _first_idle = true;
  _first_msg = true;
  _first_des_pos = true;
  _failure_occured = false;
  _system_recovered = false;
  _recoverying_already = false;
  _recovery_arrived = false;
  _changed_pid_params = false;
  _first_agent = false;
  _first_stop = true;
  _first_recovery = true;
  _first_failure = true;
  _first_failure_traj = true;
  _first_smc = true;
  _first_smc_pos = true;
  _first_bearing = true;
  _arrived = false;

  _recovery_stop = false;
  _fc_sp_params.first_gen = true;

  // initializing message with false values
  _ready_signal_params_data.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  _ready_signal_params_data.uav_1_ready = false;
  _ready_signal_params_data.uav_2_ready = false;
  _ready_signal_params_data.uav_3_ready = false;

  _ready_params_pub->publish(_ready_signal_params_data);

  // just setting the structure here, no need to publish cause it's done already
  // in the transitions_handler node
  _ready_signal_move_data.uav_1_ready = false;
  _ready_signal_move_data.uav_2_ready = false;
  _ready_signal_move_data.uav_3_ready = false;

  _ready_signal_land_data.uav_1_ready = false;
  _ready_signal_land_data.uav_2_ready = false;
  _ready_signal_land_data.uav_3_ready = false;

  // lambda that executes the main task
  auto timercallback = [this]() -> void
  {
    if(_failure_occured && !_system_recovered)
      failuresRunner();
    else
      statesRunner();
  };
  // executing the callback every tot ms, when the timer expires
	_timer = this->create_wall_timer(std::chrono::milliseconds(20), timercallback);
}


// %%%--------------------------%%%
// --------- STATES SM ----------

void StatesHandler::statesRunner()
{
  switch(_states_info_data.curr_state)
  {
    case states::PARAMETERS:
      storeParameters();
      storeControlEquilibriumData();
      // storedKLQRControlMatrix();
      checkParamsStored();

      break;

    case states::PRE_TAKE_OFF:
      if(_first_pre)
      {
        _integral_dist_x = 0.0;
        _exact_integral_dist_x = 0.0;
        _integral_dist_y = 0.0;
        _exact_integral_dist_y = 0.0;
        _integral_dist_z = 0.0;
        _exact_integral_dist_z = 0.0;
        _exact_integral_tension = 0.0;

        _curr = _clocksis.now();

        _first_pre = false;
      }

      handlePreTakeOff();
      break;

    case states::TAKE_OFF:
      if(_first_take)
      {
        _integral_dist_x = 0.0;
        _exact_integral_dist_x = 0.0;
        _integral_dist_y = 0.0;
        _exact_integral_dist_y = 0.0;
        _integral_dist_z = 0.0;
        _exact_integral_dist_z = 0.0;
        _exact_integral_tension = 0.0;
        // saving this for later when entering MOVE phase
        _takeoff_setpoint_z = _states_info_data.z;

        _first_take = false;
      }
      handleTakeOff();
      break;

    case states::MOVE:
      if(_first_move)
      {
        // _integral_dist_x = 0.0;
        // _exact_integral_dist_x = 0.0;
        // _integral_dist_y = 0.0;
        // _exact_integral_dist_y = 0.0;
        // _integral_dist_z = 0.0;
        // _exact_integral_dist_z = 0.0;
        _exact_integral_tension = 0.0;
        _first_move = false;
      }
      handleMove();
      break;

    case states::ROTATE:
      break;

    case states::LAND:
      if(_first_land)
      {
        _x_des = _agent_pose_data.position[0];
        _y_des = _agent_pose_data.position[1];
        _yaw_des = _euler_angles[2];
        _msg_land.setpoint[2] = _agent_pose_data.position[2];

        _first_land = false;
      }
      handleLand();
      break;

    case states::IDLE:
      handleIdle();
      break;

    default:
      break;
  }
}


// %%%--------------------------%%%
// --------- FAILURES SM ----------
//chages here
// void StatesHandler::failureCallback(const ros2_muavp_interface::msg::FailureCheck::SharedPtr msg)
// {
//     if (msg->uav_1_failed || msg->uav_2_failed || msg->uav_3_failed) {
//         RCLCPP_INFO(this->get_logger(), "Failure detected, shutting down node for agent %s", _agent_name.c_str());
//         rclcpp::shutdown();  // Trigger node shutdown
//     }
// }
//changes end

void StatesHandler::failuresRunner()
{

  switch((int)_failure_action_data.curr_action)
  {
    case fail_states::STOP:
      stopAndKeepPosition();
      break;

    case fail_states::RECOVERY:
      // stopAndKeepPosition();
      if(_first_recovery)
      {
        // _integral_dist_z = 0.0;
        // _exact_integral_dist_z = 0.0;
        survivedUAVcheck();
        _first_recovery = false;
      }
      tensionRodComputation();
      storeDataTensionIdentificationHandler();
      detachmentControl();
      // if(_a >= 0 && _a < 1000)
      //   printStatesOnFIle();
      // _a++;
      break;

    case fail_states::KEEP_ON:
      break;

    case fail_states::GO_TO:
      break;

    case fail_states::LAND_F:
      handleLand();
      break;

    default:
      break;
  }
}



// %%%--------------------------%%%
// --------- DESTRUCTOR ----------

StatesHandler::~StatesHandler()
{
  // myfile.flush();
  // myfile2.flush();
  // myfile3.flush();
  // myfile4.flush();
  myfile5.flush();
  myfile6.flush();
  myfile7.flush();
  // myfile_T.flush();
  // myfile_P.flush();
  // myfile.close();
  // myfile2.close();
  // myfile3.close();
  // myfile4.close();
  myfile5.close();
  myfile6.close();
  myfile7.close();
  // myfile_T.close();
  // myfile_P.close();
}
