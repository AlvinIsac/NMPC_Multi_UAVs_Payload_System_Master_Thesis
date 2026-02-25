#include <cstdio>
#include "offboard_rtps_handler.hpp"

// Constructor, a node
OffboardRTPSHandler::OffboardRTPSHandler(std::string agent_name, double force_limit_1m, double l_12, double l_34) : Node("offboard_rtps")
{
  // saving the parameters
  _agent_name = agent_name;
  _force_limit_1m = force_limit_1m;
  _l_12 = l_12;
  _l_34 = l_34;
  _force_limit = _force_limit_1m*4.0;
  _tau_limit_min[0] = -_force_limit_1m*(_l_12 + _l_34);
  _tau_limit_min[1] = -2*_force_limit_1m*_l_34;
  _tau_limit_min[2] = -_force_limit_1m*(_l_12 + _l_34);

  _tau_limit_max[0] = _force_limit_1m*(_l_12 + _l_34);
  _tau_limit_max[1] = 2*_force_limit_1m*_l_12;
  _tau_limit_max[2] = _force_limit_1m*(_l_12 + _l_34);

  // TODO: PARAMETRIZE PATH
  _myfile1.open("/home/andrea/SafeFly/Test_data/" + _agent_name + "_control_inputs.txt", std::ofstream::out);

  _myfile1 << "thrust, tau_phi, tau_theta, tau_psi\n";
  _i = 0;

  // initializing some vars
  _offboard_setpoint_counter = 0;
  _armed = false;
  _offboard = false;
  _failure_occured = false;
  _active = false;  // initially not active

  // creating the topics
  _offboard_control_mode_pub = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/" + agent_name + "/fmu/in/offboard_control_mode", 10); // /OffboardControlMode_PubSubTopic
  _setpoint_publisher = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/" + agent_name + "/fmu/in/trajectory_setpoint", 10); // /TrajectorySetpoint_PubSubTopic
  _vehicle_command_pub = this->create_publisher<px4_msgs::msg::VehicleCommand>("/" + agent_name + "/fmu/in/vehicle_command", 10); // /VehicleCommand_PubSubTopic
  // _vehicle_act_ctrl_pub = this->create_publisher<px4_msgs::msg::ActuatorMotors>("/" + agent_name + "/fmu/in/actuator_motors", 10); // /VehicleCommand_PubSubTopic
  _vehicle_thrust_ctrl_pub = this->create_publisher<px4_msgs::msg::VehicleThrustSetpoint>("/" + agent_name + "/fmu/in/vehicle_thrust_setpoint", 10);
  _vehicle_torque_ctrl_pub = this->create_publisher<px4_msgs::msg::VehicleTorqueSetpoint>("/" + agent_name + "/fmu/in/vehicle_torque_setpoint", 10);

  // _offboard_control_mode_pub = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/OffboardControlMode_PubSubTopic", 10);
  // _setpoint_publisher = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/TrajectorySetpoint_PubSubTopic", 10);
  // _vehicle_command_pub = this->create_publisher<px4_msgs::msg::VehicleCommand>("/VehicleCommand_PubSubTopic", 10);
  // _actuator_controls_pub = this->create_publisher<std_msgs::msg::Float64>("/float64multiarray", 10);


  // std::cout << now().seconds() << std::endl;

  // setpoints publisher
  // _vehicle_ang_acc_pub = this->create_publisher<px4_msgs::msg::VehicleAngularAccelerationSetpoint>("/VehicleAngularAccelerationSetpoint_PubSubTopic", 10);
  // get common timestamp
  // _timesync_sub = this->create_subscription<px4_msgs::msg::Timesync>("/" + agent_name + "/fmu/timesync/out", 1, // /Timesync_PubSubTopic
  //                 [this](const px4_msgs::msg::Timesync::UniquePtr msg_ts)
  //                 {
  //                   _timestamp.store(msg_ts->timestamp);
  //                 });

  // getting armed flag and control mode
  _vcm_sub = this->create_subscription<px4_msgs::msg::VehicleControlMode>("/" + agent_name + "/fmu/out/vehicle_control_mode",
                  rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data), // /VehicleControlMode_PubSubTopic
                  [this](const px4_msgs::msg::VehicleControlMode::UniquePtr msg_vcm)
                  {
                    _armed = msg_vcm->flag_armed;
                    _offboard = msg_vcm->flag_control_offboard_enabled;
                  });


  // // getting armed flag
  // _actuator_armed_sub = this->create_subscription<px4_msgs::msg::ActuatorArmed>("/ActuatorArmed_PubSubTopic", 1,
  //                         [this](const px4_msgs::msg::ActuatorArmed::UniquePtr msg_aa)
  //                         {
  //                           _armed = msg_aa->armed;
  //                         });
  // and the setpoint stream
  _setpoint_sub = this->create_subscription<ros2_muavp_interface::msg::InputSetpoint>("/" + agent_name + "/input_setpoint", 1, std::bind(&OffboardRTPSHandler::setpointCallback, this, std::placeholders::_1));

  // listening to the possible failure directives
  _failure_action_sub = this->create_subscription<ros2_muavp_interface::msg::FailureAction>("/" + agent_name + "/failure_action", 1,
                        [this](const ros2_muavp_interface::msg::FailureAction::SharedPtr msg_fa)
                        {
                          _failure_action_data = *msg_fa;
                          _failure_occured = true;
                        });

  // listening to failure check messages
  _failure_check_sub = this->create_subscription<ros2_muavp_interface::msg::FailureCheck>("/failure_check", 1,
                       std::bind(&OffboardRTPSHandler::failureCheckCb, this, std::placeholders::_1));

  _ready_params_sub = this->create_subscription<ros2_muavp_interface::msg::ReadySignals>("/agents/ready_signal_parameters",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::ReadySignals::SharedPtr msg_ready)
          {
            // getting the data from the topic and  saving it
            _ready_signal_params_data = *msg_ready;
          });

  // listening to the current task information
  _states_info_sub = this->create_subscription<ros2_muavp_interface::msg::StatesInfo>("/" + agent_name + "/states_info",
          rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
          [this](const ros2_muavp_interface::msg::StatesInfo::SharedPtr msg_si)
          {
            // getting the data from the topic and saving it
            _states_info_data = *msg_si;
          });


  // lambda that executes the main task: arm -> offboard
	auto _timercallback = [this]() -> void {

		// if (_offboard_setpoint_counter == 10) {
		// 	// Change to Offboard mode after 10 setpoints
		// 	this->publishVehicleCommand(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
    //
		// 	// Arm the vehicle
		// 	this->arm();
		// }
    //
    // // offboard_control_mode needs to be paired with trajectory setpoint
		// publishOffboardControlMode();
		// publishSetpointRTPS();


    if(_setpoint_msg.setpoint_type == 4)
    {
      VectorXd params = VectorXd(7);
      params << 1, 12, NULL, NULL, NULL, NULL, NULL;
      this->publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, params); // precision landing
    }
    else if(_setpoint_msg.out_of_mocap)
    {
      VectorXd params = VectorXd(7);
      params << 1, 8, NULL, NULL, _setpoint_msg.setpoint[0], _setpoint_msg.setpoint[1], _setpoint_msg.setpoint[2];
      // params << 1, 8, NULL, NULL, NULL, NULL, NULL;
      this->publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, params); // hold
    }
    else if(_offboard_setpoint_counter >= 10 && !_armed)     // if the minimum number of setpoints is reached AND the vehicle is not armed yet
    {
      VectorXd params = VectorXd(7);
      params << 1, 6, NULL, NULL, NULL, NULL, NULL;
      // Change to Offboard mode after 10 setpoints
      this->publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, params);
      // Arm the vehicle
      this->arm();
    }
    // this allows each iteration to keep sending the arm command if the drone doesn't arm itself


    if(!_setpoint_msg.out_of_mocap)
    {
      // offboard_control_mode needs to be paired with trajectory setpoint
      publishOffboardControlMode();
      publishSetpointRTPS();
    }

    // incrementing the counter 'til it reaches the minimum amount of iterations before going offboard
    // considering also a triggering condition, based on wether the parameters phase has been successfully completed
    int curr_state = (int)_states_info_data.curr_state;
    bool offboard_cond = _ready_signal_params_data.uav_1_ready && _ready_signal_params_data.uav_2_ready && _ready_signal_params_data.uav_3_ready && curr_state > 0;

    if(_offboard_setpoint_counter < 11 && offboard_cond)
      _offboard_setpoint_counter++;
	};
  // executing the callback every tot ms, when the timer expires
	_timer = this->create_wall_timer(std::chrono::milliseconds(10), _timercallback);
}



// function to send arm command to drone
void OffboardRTPSHandler::arm()
{
  VectorXd params = VectorXd(7);
  params << 1.0, NULL, NULL, NULL, NULL, NULL, NULL;
  publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, params);

	// RCLCPP_INFO(this->get_logger(), "Arm command sent");
}
// and the one to disarm it
void OffboardRTPSHandler::disarm()
{
  VectorXd params = VectorXd(7);
  params << 0.0, NULL, NULL, NULL, NULL, NULL, NULL;
  publishVehicleCommand(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, params);

  RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

// this function enables offboard control mode
void OffboardRTPSHandler::publishOffboardControlMode()
{
  // the allowed modalities are specified below
	px4_msgs::msg::OffboardControlMode msg{};
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

// --------------------------- WITHOUT LOCKSTEP ----------------------------
  if(_setpoint_msg.setpoint_type == 0)
  {
    msg.position = true;
  	msg.acceleration = true;
  }
  else if(_setpoint_msg.setpoint_type == 1)
  {
    msg.position = true;
  	msg.acceleration = false;
  }
  else if(_setpoint_msg.setpoint_type == 3)
  {
    msg.position = false;
  	msg.velocity = true;
  	msg.acceleration = true;
  }

  if(_setpoint_msg.setpoint_type == 2) // _failure_occured &&
  {
    msg.position = false;
  	msg.acceleration = false;
    msg.thrust_and_torque = true;
  }
  else
  {
    msg.thrust_and_torque = false;
  }

	msg.velocity = false;
	msg.attitude = false;
	msg.body_rate = false;

  // and the message is sent on the topic
	_offboard_control_mode_pub->publish(msg);
}

// this function pubishes the setpoints coming from the control algorithm
// in the RTPS topic
void OffboardRTPSHandler::publishSetpointRTPS()
{

  // LOCAL X = EAST
  // LOCAL Y = NORTH
  // the check on msg type is needed also to avoid the "failed agent" to give
  // actuator setpoints as the other agents, but keep giving position ones instead
  if(_setpoint_msg.setpoint_type == 2) // _failure_occured &&
  {

    _actCtrlData.control[0] = _setpoint_msg.setpoint[0];
    _actCtrlData.control[1] = _setpoint_msg.setpoint[1];
    _actCtrlData.control[2] = _setpoint_msg.setpoint[2];
    _actCtrlData.control[3] = _setpoint_msg.setpoint[3];

    // converting them
    conversionToActuatorsInput();
    // and sending them
    sendActuatorSetpoints();
  }
  else if(_setpoint_msg.setpoint_type == 0)
  {

    px4_msgs::msg::TrajectorySetpoint msg{};

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    msg.position[0] = _setpoint_msg.setpoint[0];
    msg.position[1] = _setpoint_msg.setpoint[1];
    msg.position[2] = NULL;
    msg.acceleration[0] = NULL;
    msg.acceleration[1] = NULL;
    msg.acceleration[2] = _setpoint_msg.setpoint[2];
    msg.yaw = _setpoint_msg.setpoint[3];

    _setpoint_publisher->publish(msg);
  }
  else if(_setpoint_msg.setpoint_type == 1)
  {
    px4_msgs::msg::TrajectorySetpoint msg{};

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;

    msg.position[0] = _setpoint_msg.setpoint[0];
    msg.position[1] = _setpoint_msg.setpoint[1];
    msg.position[2] = _setpoint_msg.setpoint[2];
    msg.acceleration[0] = NULL;
    msg.acceleration[1] = NULL;
    msg.acceleration[2] = NULL;

    msg.yaw = _setpoint_msg.setpoint[3];

    _setpoint_publisher->publish(msg);
  }
  else if(_setpoint_msg.setpoint_type == 3)
  {
    px4_msgs::msg::TrajectorySetpoint msg{};

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.position[0] = NAN;
    msg.position[1] = NAN;
    msg.position[2] = NAN;
    msg.velocity[0] = _setpoint_msg.setpoint[0];
    msg.velocity[1] = _setpoint_msg.setpoint[1];
    msg.velocity[2] = NULL;
    msg.acceleration[0] = NULL;
    msg.acceleration[1] = NULL;
    msg.acceleration[2] = _setpoint_msg.setpoint[2];
    // std::cout << "input acc "<< _setpoint_msg.setpoint[2] << std::endl;

    msg.yaw = _setpoint_msg.setpoint[3];

    _setpoint_publisher->publish(msg);
  }

}


void OffboardRTPSHandler::conversionToActuatorsInput()
{

  _actCtrlData.control[0] = std::clamp(_actCtrlData.control[0], _tau_limit_min[0], _tau_limit_max[0]);
  _actCtrlData.control[1] = std::clamp(_actCtrlData.control[1], _tau_limit_min[1], _tau_limit_max[1]);
  _actCtrlData.control[2] = std::clamp(-_actCtrlData.control[2], 0.0, _force_limit);
  _actCtrlData.control[3] = std::clamp(_actCtrlData.control[3], _tau_limit_min[2], _tau_limit_max[2]);
  // converting them to actuator setpoints
  _actCtrlData.control[0] = _actCtrlData.control[0]/_tau_limit_max[0];
  // handling the case of different saturations limits as well
  if(_actCtrlData.control[1] < 0.0)
    _actCtrlData.control[1] = -_actCtrlData.control[1]/_tau_limit_min[1];
  else
    _actCtrlData.control[1] = _actCtrlData.control[1]/_tau_limit_max[1];

  _actCtrlData.control[3] = _actCtrlData.control[3]/_tau_limit_max[2];
  _actCtrlData.control[2] = _actCtrlData.control[2]/_force_limit;

}

void OffboardRTPSHandler::sendActuatorSetpoints()
{

  px4_msgs::msg::VehicleThrustSetpoint msg_thurst_pub{};
  px4_msgs::msg::VehicleTorqueSetpoint msg_torque_pub{};


  // std::cout << _agent_name << " u: " << _actCtrlData.control[2] << std::endl;
  // std::cout << _agent_name << " phi: " << _actCtrlData.control[0] << std::endl;
  // std::cout << _agent_name << " theta: " << _actCtrlData.control[1] << std::endl;
  // std::cout << _agent_name << " psi: " << _actCtrlData.control[3] << std::endl;

  // msg_pub.reversible_flags = 6;

  msg_thurst_pub.xyz[0] = 0.0;
  msg_thurst_pub.xyz[1] = 0.0;
  msg_thurst_pub.xyz[2] = -_actCtrlData.control[2];

  msg_torque_pub.xyz[0] = _actCtrlData.control[0];
  msg_torque_pub.xyz[1] = _actCtrlData.control[1];
  msg_torque_pub.xyz[2] = _actCtrlData.control[3];

  msg_thurst_pub.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  msg_torque_pub.timestamp = msg_thurst_pub.timestamp;

  // _vehicle_act_ctrl_pub->publish(msg_pub);
  _vehicle_thrust_ctrl_pub->publish(msg_thurst_pub);
  _vehicle_torque_ctrl_pub->publish(msg_torque_pub);

  // printing on file the controls for logging
  if(_i < 10000 && _i % 2 == 0)
    _myfile1 << std::to_string(_actCtrlData.control[2]) + "," + std::to_string(_actCtrlData.control[0]) + "," + std::to_string(_actCtrlData.control[1]) + "," + std::to_string(_actCtrlData.control[3]) + "\n";
  _i++;
}

// this function is used to arm/disarm the vehicle, depending on the input
void OffboardRTPSHandler::publishVehicleCommand(uint16_t command, VectorXd params)
{
  // filling the message accordingly
	px4_msgs::msg::VehicleCommand msg{};
  msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
  if(params(0) != NULL)
	  msg.param1 = params(0);
  if(params(1) != NULL)
 	  msg.param2 = params(1);
  if(params(2) != NULL)
 	  msg.param3 = params(2);
  if(params(3) != NULL)
    msg.param4 = params(3);
  if(params(4) != NULL)
 	  msg.param5 = params(4);
  if(params(5) != NULL)
 	  msg.param6 = params(5);
  if(params(6) != NULL)
    msg.param7 = params(6);

	msg.command = command;
  if(_agent_name == "x500_1")
  {
  	msg.target_system = 1;
  }
  else if(_agent_name == "x500_2")
  {
  	msg.target_system = 2;
  }
  else if(_agent_name == "x500_3")
  {
  	msg.target_system = 3;
  }
  msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
  // and publishing it
	_vehicle_command_pub->publish(msg);
}

void OffboardRTPSHandler::setpointCallback(const ros2_muavp_interface::msg::InputSetpoint::SharedPtr msg_sp)
{
  // Only filter after failure - if failure is active, only allow timestamp == 1
  if (_active && msg_sp->timestamp != 1) {
    return;
  }
  
  _setpoint_msg = *msg_sp;
  _setpoint_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
}

void OffboardRTPSHandler::failureCheckCb(const ros2_muavp_interface::msg::FailureCheck::SharedPtr msg) {
    if (!_active && (msg->uav_1_failed || msg->uav_2_failed || msg->uav_3_failed)) {
        _active = true;
        _failure_check_data = *msg;
    }
}
