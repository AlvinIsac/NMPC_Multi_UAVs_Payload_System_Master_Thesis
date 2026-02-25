#include <cstdio>
#include <rclcpp/rclcpp.hpp>
// #include <px4_msgs/msg/actuator_motors.hpp>
#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_control_mode.hpp>
#include <px4_msgs/msg/vehicle_thrust_setpoint.hpp>
#include <px4_msgs/msg/vehicle_torque_setpoint.hpp>
// #include <px4_msgs/msg/timesync.hpp>
#include <px4_msgs/msg/actuator_armed.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_local_position_setpoint.hpp>
#include "ros2_muavp_interface/msg/input_setpoint.hpp"
#include "ros2_muavp_interface/msg/failure_action.hpp"
#include "ros2_muavp_interface/msg/failure_check.hpp"
#include "ros2_muavp_interface/msg/ready_signals.hpp"
#include "ros2_muavp_interface/msg/states_info.hpp"
#include <std_msgs/msg/float64.hpp>
#include <fstream>
// #include "udp_utils.hpp"

#include <chrono>
#include <Eigen/Dense>
using Eigen::VectorXd;

class OffboardRTPSHandler : public rclcpp::Node
{
  public:
    OffboardRTPSHandler(std::string agent_name, double force_limit_1m, double l_12, double l_34);

    void arm();
    void disarm();

  private:

    rclcpp::TimerBase::SharedPtr _timer;

    std::ofstream _myfile1;
    int _i;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr _offboard_control_mode_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr _vehicle_command_pub;
    // rclcpp::Publisher<px4_msgs::msg::VehicleAngularAccelerationSetpoint>::SharedPtr _vehicle_ang_acc_pub;
    // rclcpp::Publisher<px4_msgs::msg::ActuatorMotors>::SharedPtr _vehicle_act_ctrl_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleThrustSetpoint>::SharedPtr _vehicle_thrust_ctrl_pub;
    rclcpp::Publisher<px4_msgs::msg::VehicleTorqueSetpoint>::SharedPtr _vehicle_torque_ctrl_pub;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr _actuator_controls_pub;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr _setpoint_publisher;

    // rclcpp::Subscription<px4_msgs::msg::Timesync>::SharedPtr _timesync_sub;
    rclcpp::Subscription<px4_msgs::msg::VehicleControlMode>::SharedPtr _vcm_sub;
    rclcpp::Subscription<ros2_muavp_interface::msg::InputSetpoint>::SharedPtr _setpoint_sub;
    rclcpp::Subscription<ros2_muavp_interface::msg::FailureAction>::SharedPtr _failure_action_sub;
    rclcpp::Subscription<ros2_muavp_interface::msg::FailureCheck>::SharedPtr _failure_check_sub;
    rclcpp::Subscription<ros2_muavp_interface::msg::ReadySignals>::SharedPtr _ready_params_sub;
    rclcpp::Subscription<ros2_muavp_interface::msg::StatesInfo>::SharedPtr _states_info_sub;


    ros2_muavp_interface::msg::InputSetpoint _setpoint_msg;
    ros2_muavp_interface::msg::FailureAction _failure_action_data;
    ros2_muavp_interface::msg::FailureCheck _failure_check_data;
    ros2_muavp_interface::msg::ReadySignals _ready_signal_params_data;
    ros2_muavp_interface::msg::StatesInfo _states_info_data;

  	// std::atomic<uint64_t> _timestamp;

    std::string _agent_name = "";
    double _force_limit_1m = 0.0;
    double _l_12 = 0.0;
    double _l_34 = 0.0;
    double _force_limit = 0.0;
    double _tau_limit_min[3] = {0.0, 0.0, 0.0};
    double _tau_limit_max[3] = {0.0, 0.0, 0.0};

    // std::string _port_actuator_controls, _ip;
    // udp_utils::UDPSenderSocket *_udpSenderActCtrl;

    struct ActCtrlData
    {
      double control[4];
    }_actCtrlData;

    bool _armed;
    bool _offboard;
    bool _failure_occured;
    bool _active;  // tracks if failure is active
    //counter for the number of setpoints sent
    uint64_t _offboard_setpoint_counter;

    void setpointCallback(const ros2_muavp_interface::msg::InputSetpoint::SharedPtr msg_sp);
    void failureCheckCb(const ros2_muavp_interface::msg::FailureCheck::SharedPtr msg);

    void publishOffboardControlMode();
    void publishSetpointRTPS();
    void publishVehicleCommand(uint16_t command, VectorXd params);

    void conversionToActuatorsInput();
    void sendActuatorSetpoints();

};
