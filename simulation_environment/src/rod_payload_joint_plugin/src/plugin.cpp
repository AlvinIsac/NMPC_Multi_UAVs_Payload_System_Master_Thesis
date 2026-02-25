#include <rclcpp/rclcpp.hpp>
#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
// #include <stdlib.h>
#include <math.h>
#include <vector>
#include "ros2_muavp_interface/msg/failure_check.hpp"
#include "ros2_muavp_interface/msg/rigid_body_pose.hpp"
#include "ros2_muavp_interface/msg/rigid_body_measures.hpp"
#include "ros2_muavp_interface/msg/ready_signals.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <ignition/math/Pose3.hh>

// #include <chrono>
#define PI 3.14159265

namespace gazebo
{

    class RodPayloadJointHandler : public WorldPlugin
    {

    private:

        physics::WorldPtr _world;
        physics::ModelPtr _x5001;
        physics::ModelPtr _x5002;
        physics::ModelPtr _x5003;
        physics::ModelPtr _payload;
        event::ConnectionPtr _updateConnection;
        physics::JointPtr _joint_cable1_p;
        physics::JointPtr _joint_cable2_p;
        physics::JointPtr _joint_cable3_p;
        physics::LinkPtr _lp = NULL;
        physics::LinkPtr _li1 = NULL;
        physics::LinkPtr _li2 = NULL;
        physics::LinkPtr _li3 = NULL;

        physics::PhysicsEnginePtr _physics;

        ignition::math::Pose3d _initial_joint_pose_1;
        ignition::math::Pose3d _initial_joint_pose_2;
        ignition::math::Pose3d _initial_joint_pose_3;

        rclcpp::Node::SharedPtr _node;
        rclcpp::TimerBase::SharedPtr _timer;
        std::atomic<uint64_t> _timestamp;


        rclcpp::Time _curr;
        rclcpp::Clock _clocksis;

        ros2_muavp_interface::msg::FailureCheck  _failure_check_data;
        ros2_muavp_interface::msg::RigidBodyPose _rb_pose_data;
        ros2_muavp_interface::msg::RigidBodyMeasures _payload_measures_data;
        ros2_muavp_interface::msg::ReadySignals _ready_signal_params_data;

        rclcpp::Subscription<ros2_muavp_interface::msg::FailureCheck>::SharedPtr _failure_check_sub;
        rclcpp::Publisher<ros2_muavp_interface::msg::FailureCheck>::SharedPtr _failure_check_pub;
        rclcpp::Subscription<ros2_muavp_interface::msg::ReadySignals>::SharedPtr _ready_params_sub;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr _imu_payload_sub;

        rclcpp::Publisher<ros2_muavp_interface::msg::RigidBodyPose>::SharedPtr _rb_pose_pub;
        rclcpp::Publisher<ros2_muavp_interface::msg::RigidBodyMeasures>::SharedPtr _payload_measures_pub;
        rclcpp::Publisher<ros2_muavp_interface::msg::RigidBodyPose>::SharedPtr _x5001_pose_pub;
        rclcpp::Publisher<ros2_muavp_interface::msg::RigidBodyPose>::SharedPtr _x5002_pose_pub;
        rclcpp::Publisher<ros2_muavp_interface::msg::RigidBodyPose>::SharedPtr _x5003_pose_pub;

        bool _joints_created;
        bool _takeoff_plats_deleted;


    public:

        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the world
            this->_world = _world;
            // and its physics tools
            this->_physics = this->_world->Physics();
            // Listen to the update event.
            this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&RodPayloadJointHandler::OnUpdate, this));
            // initiaizing the initial pose of each created joint wrt their parent
            // _initial_joint_pose_1 = ignition::math::Pose3d(0.0, 0, -0.0, 0, -1.57, 0);
            // _initial_joint_pose_2 = ignition::math::Pose3d(0, 0.0, -0.0, 0, -1.57, 0);
            // _initial_joint_pose_3 = ignition::math::Pose3d(-0.0, 0, -0.0, 0, -1.57, 0);
            // _initial_joint_pose_3 = ignition::math::Pose3d(0, -0.0, -0.0, 0, -1.57, 0);

            // old (x, y , z)
            _initial_joint_pose_1 = ignition::math::Pose3d(0.009, -0.0, -0.008, 0, -0, 0);
            _initial_joint_pose_2 = ignition::math::Pose3d(0.012, -0.0, -0.007, 0, -0, 0);
            _initial_joint_pose_3 = ignition::math::Pose3d(0.005, 0.0, -0.005, 0, -0, 0);

            // new (z, y ,x)
            // _initial_joint_pose_1 = ignition::math::Pose3d(-0.0, -0.0, 0.00, 0, -0, 0);
            // // _initial_joint_pose_2 = ignition::math::Pose3d(0.04, -0.0, -0.005, 0, -0, 0);
            // _initial_joint_pose_2 = ignition::math::Pose3d(0.00, -0.0, 0.05, 0, -0, 0);
            // _initial_joint_pose_3 = ignition::math::Pose3d(0.03, -0.0, -0.0, 0, -0, 0);


            // _initial_joint_pose_1 = ignition::math::Pose3d(0.0, -0.0, 0.00, 0, -0, 0);
            // // _initial_joint_pose_2 = ignition::math::Pose3d(0.04, -0.0, -0.005, 0, -0, 0);
            // _initial_joint_pose_2 = ignition::math::Pose3d(0.05, -0.0, -0.04, 0, -0, 0);
            // _initial_joint_pose_3 = ignition::math::Pose3d(0.02, -0.0, -0.01, 0, -0, 0);

            // init some vars
            _joints_created = false;
            _takeoff_plats_deleted = false;
              // creating a ros2 node
            if(!rclcpp::ok())
              rclcpp::init(0, nullptr);

            _node = rclcpp::Node::make_shared("_rods_payload_joint_plugin_ros2");

            _failure_check_sub = _node->create_subscription<ros2_muavp_interface::msg::FailureCheck>("/agents/failure_check",
                    rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
                    std::bind(&RodPayloadJointHandler::failureCheckCallback, this, std::placeholders::_1));

            _ready_params_sub = _node->create_subscription<ros2_muavp_interface::msg::ReadySignals>("/agents/ready_signal_parameters",
                    rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
                    std::bind(&RodPayloadJointHandler::signalReadyParamsCallback, this, std::placeholders::_1));

            _imu_payload_sub = _node->create_subscription<sensor_msgs::msg::Imu>("/payload/imu_plugin/out",
                    rclcpp::QoS(rclcpp::QoSInitialization(rmw_qos_profile_sensor_data.history, rmw_qos_profile_sensor_data.depth), rmw_qos_profile_sensor_data),
                    std::bind(&RodPayloadJointHandler::signalImuPluginCallback, this, std::placeholders::_1));

            _failure_check_pub = _node->create_publisher<ros2_muavp_interface::msg::FailureCheck>("/agents/failure_check", 10);

            _rb_pose_pub = _node->create_publisher<ros2_muavp_interface::msg::RigidBodyPose>("/payload/pose", 10);
            _payload_measures_pub = _node->create_publisher<ros2_muavp_interface::msg::RigidBodyMeasures>("/payload/measures", 10);

            _x5001_pose_pub = _node->create_publisher<ros2_muavp_interface::msg::RigidBodyPose>("/x500_1/pose", 10);
            _x5002_pose_pub = _node->create_publisher<ros2_muavp_interface::msg::RigidBodyPose>("/x500_2/pose", 10);
            _x5003_pose_pub = _node->create_publisher<ros2_muavp_interface::msg::RigidBodyPose>("/x500_3/pose", 10);

            // resetting the message at start
            _failure_check_data.uav_1_failed = false;
            _failure_check_data.uav_2_failed = false;
            _failure_check_data.uav_3_failed = false;
            _failure_check_data.uav_1_detached = false;
            _failure_check_data.uav_2_detached = false;
            _failure_check_data.uav_3_detached = false;
            _failure_check_data.timestamp = _node->get_clock()->now().nanoseconds() / 1000;

            _failure_check_pub->publish(_failure_check_data);


            // lambda that executes the main task
            auto timercallback = [this]() -> void
            {
              if(!_takeoff_plats_deleted)
                deletePlatsCheck();

              failureChecker();
              if(_lp != NULL)
                payloadPosePub();
              if(_li1 != NULL)
                x5001PosePub();
              if(_li2 != NULL)
                x5002PosePub();
              if(_li3 != NULL)
                x5003PosePub();
            };
            // executing the callback every tot ms, when the timer expires
            _timer = _node->create_wall_timer(std::chrono::milliseconds(20), timercallback);
        }

        void OnUpdate()
        {
          // Creating the needed connections, after the last model spawns
          if(this->_world->ModelByName("x500_3") != NULL && !_joints_created)
          {
            CreateJoints();
            _joints_created = true;
          }

          rclcpp::spin_some(_node);
        }

        void failureCheckCallback(const ros2_muavp_interface::msg::FailureCheck::SharedPtr msg_fc)
        {
          _failure_check_data = *msg_fc;
        }

        void signalReadyParamsCallback(const ros2_muavp_interface::msg::ReadySignals::SharedPtr msg_rs)
        {
          _ready_signal_params_data = *msg_rs;
        }

        double adjustYaw(double yaw)
        {
          // Convert yaw angle from x-axis to N-axis (it is taken wrt E axis in Gazebo)
          double yaw_adjusted = yaw + PI/2;

           // Adjust the angle to stay within (-pi, pi]
           if (yaw_adjusted <= -PI)
               yaw_adjusted += 2*PI;

           if (yaw_adjusted > PI)
               yaw_adjusted -= 2*PI;

           return yaw_adjusted;
        }

        void payloadPosePub()
        {
          auto payload_pose = _lp->WorldPose();
          auto initial_payload_rel_pose = _lp->InitialRelativePose();
          auto payload_vel = _lp->WorldLinearVel();
          auto payload_ang_vel = _lp->WorldAngularVel();
          auto payload_acc = _lp->WorldLinearAccel();
          // data given in local gazebo frame, transforming in NED
          _rb_pose_data.position[0] = payload_pose.Pos().Y();
          _rb_pose_data.position[1] = payload_pose.Pos().X();
          _rb_pose_data.position[2] = -payload_pose.Pos().Z();
          _rb_pose_data.orientation[0] = payload_pose.Rot().Pitch();
          _rb_pose_data.orientation[1] = payload_pose.Rot().Roll();
          _rb_pose_data.orientation[2] = -payload_pose.Rot().Yaw();

          _rb_pose_data.orientation[2] = adjustYaw(_rb_pose_data.orientation[2]);

          _rb_pose_data.lin_velocity[0] = payload_vel.Y();
          _rb_pose_data.lin_velocity[1] = payload_vel.X();
          _rb_pose_data.lin_velocity[2] = -payload_vel.Z();
          _rb_pose_data.lin_acceleration[0] = payload_acc.Y();
          _rb_pose_data.lin_acceleration[1] = payload_acc.X();
          _rb_pose_data.lin_acceleration[2] = -payload_acc.Z();
          _rb_pose_data.ang_velocity[0] = payload_ang_vel.Y();
          _rb_pose_data.ang_velocity[1] = payload_ang_vel.X();
          _rb_pose_data.ang_velocity[2] = -payload_ang_vel.Z();
          // _rb_pose_data.orientation[2] = -payload_vel.Z();

          // Add timestamp
          _rb_pose_data.timestamp = _node->get_clock()->now().nanoseconds() / 1000;

          // std::cout << "payload_yaw: " << _rb_pose_data.orientation[2] << std::endl;
          // std::cout << "payload_yaw_dot: "<< _rb_pose_data.ang_velocity[2] << std::endl;
          _rb_pose_pub->publish(_rb_pose_data);


          // std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
          //
          // auto duration = now.time_since_epoch();
          // auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
          //
          // bool cond_1 = _failure_check_data.uav_1_failed && _failure_check_data.uav_1_detached;
          // bool cond_2 = _failure_check_data.uav_2_failed && _failure_check_data.uav_2_detached;
          // bool cond_3 = _failure_check_data.uav_3_failed && _failure_check_data.uav_3_detached;
          //
          // if(cond_1 || cond_2 || cond_3)
          //   std::cout << "payload-send-at: " << millis << std::endl;
        }

        void signalImuPluginCallback(const sensor_msgs::msg::Imu::SharedPtr msg_ip)
        {
          // auto payload_body_lin_acc = _lp->RelativeLinearAccel();
          // auto payload_body_ang_vel = _lp->RelativeAngularVel();
          //
          // _payload_measures_data.body_acceleration[0] = payload_body_lin_acc.Y();
          // _payload_measures_data.body_acceleration[1] = payload_body_lin_acc.X();
          // _payload_measures_data.body_acceleration[2] = -payload_body_lin_acc.Z();
          //
          // _payload_measures_data.body_angular_velocity[0] = payload_body_ang_vel.Y();
          // _payload_measures_data.body_angular_velocity[1] = payload_body_ang_vel.X();
          // _payload_measures_data.body_angular_velocity[2] = -payload_body_ang_vel.Z();


          _payload_measures_data.body_acceleration[0] = msg_ip->linear_acceleration.y;
          _payload_measures_data.body_acceleration[1] = msg_ip->linear_acceleration.x;
          _payload_measures_data.body_acceleration[2] = -msg_ip->linear_acceleration.z;

          _payload_measures_data.body_angular_velocity[0] = msg_ip->angular_velocity.y;
          _payload_measures_data.body_angular_velocity[1] = msg_ip->angular_velocity.x;
          _payload_measures_data.body_angular_velocity[2] = -msg_ip->angular_velocity.z;

          // Add timestamp
          _payload_measures_data.timestamp = _node->get_clock()->now().nanoseconds() / 1000;

          // TODO: PUT NOISE ON MEASURES

          _payload_measures_pub->publish(_payload_measures_data);
        }


        void x5001PosePub()
        {
          ros2_muavp_interface::msg::RigidBodyPose  x5001_pose_data;

          auto rb_pose = _li1->WorldPose();
          auto rb_vel = _li1->WorldLinearVel();
          auto rb_ang_vel = _li1->WorldAngularVel();
          auto initial_rb_rel_pose = _li1->InitialRelativePose();
          // auto rb_vel = _li1->WorldLinearVel();
          // data given in local gazebo frame, transforming in NED
          x5001_pose_data.position[0] = rb_pose.Pos().Y();
          x5001_pose_data.position[1] = rb_pose.Pos().X();
          x5001_pose_data.position[2] = -rb_pose.Pos().Z();
          x5001_pose_data.lin_velocity[0] = rb_vel.Y();
          x5001_pose_data.lin_velocity[1] = rb_vel.X();
          x5001_pose_data.lin_velocity[2] = -rb_vel.Z();
          x5001_pose_data.orientation[0] = rb_pose.Rot().Pitch();
          x5001_pose_data.orientation[1] = rb_pose.Rot().Roll();
          x5001_pose_data.orientation[2] = -rb_pose.Rot().Yaw();

          x5001_pose_data.orientation[2] = adjustYaw(x5001_pose_data.orientation[2]);

          // Change to body frame angular rates (p, q, r)
          auto rb_body_ang_vel = _li1->RelativeAngularVel();
          x5001_pose_data.ang_velocity[0] = rb_body_ang_vel.Y();  // p (roll rate)
          x5001_pose_data.ang_velocity[1] = rb_body_ang_vel.X();  // q (pitch rate)
          x5001_pose_data.ang_velocity[2] = -rb_body_ang_vel.Z(); // r (yaw rate)
          
          // Add timestamp
          x5001_pose_data.timestamp = _node->get_clock()->now().nanoseconds() / 1000;
          
          _x5001_pose_pub->publish(x5001_pose_data);
          // std::cout << "x5001_x: " << rb_pose.Pos().Y() << std::endl;
          // std::cout << "x5001_y: "<< rb_pose.Pos().X() << std::endl;
        }

        void x5002PosePub()
        {
          ros2_muavp_interface::msg::RigidBodyPose  x5002_pose_data;

          auto rb_pose = _li2->WorldPose();
          auto rb_vel = _li2->WorldLinearVel();
          auto rb_ang_vel = _li2->WorldAngularVel();
          auto initial_rb_rel_pose = _li2->InitialRelativePose();
          // auto rb_vel = _li1->WorldLinearVel();
          // data given in local gazebo frame, transforming in NED
          x5002_pose_data.position[0] = rb_pose.Pos().Y();
          x5002_pose_data.position[1] = rb_pose.Pos().X();
          x5002_pose_data.position[2] = -rb_pose.Pos().Z();
          x5002_pose_data.lin_velocity[0] = rb_vel.Y();
          x5002_pose_data.lin_velocity[1] = rb_vel.X();
          x5002_pose_data.lin_velocity[2] = -rb_vel.Z();
          x5002_pose_data.orientation[0] = rb_pose.Rot().Pitch();
          x5002_pose_data.orientation[1] = rb_pose.Rot().Roll();
          x5002_pose_data.orientation[2] = -rb_pose.Rot().Yaw();

          x5002_pose_data.orientation[2] = adjustYaw(x5002_pose_data.orientation[2]);

          // Change to body frame angular rates (p, q, r)
          auto rb_body_ang_vel = _li2->RelativeAngularVel();
          x5002_pose_data.ang_velocity[0] = rb_body_ang_vel.Y();  // p (roll rate)
          x5002_pose_data.ang_velocity[1] = rb_body_ang_vel.X();  // q (pitch rate)
          x5002_pose_data.ang_velocity[2] = -rb_body_ang_vel.Z(); // r (yaw rate)
          
          // Add timestamp
          x5002_pose_data.timestamp = _node->get_clock()->now().nanoseconds() / 1000;
          
          _x5002_pose_pub->publish(x5002_pose_data);
        }

        void x5003PosePub()
        {
          ros2_muavp_interface::msg::RigidBodyPose  x5003_pose_data;

          auto rb_pose = _li3->WorldPose();
          auto rb_vel = _li3->WorldLinearVel();
          auto rb_ang_vel = _li3->WorldAngularVel();
          auto initial_rb_rel_pose = _li3->InitialRelativePose();
          // auto rb_vel = _li1->WorldLinearVel();
          // data given in local gazebo frame, transforming in NED
          x5003_pose_data.position[0] = rb_pose.Pos().Y();
          x5003_pose_data.position[1] = rb_pose.Pos().X();
          x5003_pose_data.position[2] = -rb_pose.Pos().Z();
          x5003_pose_data.lin_velocity[0] = rb_vel.Y();
          x5003_pose_data.lin_velocity[1] = rb_vel.X();
          x5003_pose_data.lin_velocity[2] = -rb_vel.Z();
          x5003_pose_data.orientation[0] = rb_pose.Rot().Pitch();
          x5003_pose_data.orientation[1] = rb_pose.Rot().Roll();
          x5003_pose_data.orientation[2] = -rb_pose.Rot().Yaw();

          x5003_pose_data.orientation[2] = adjustYaw(x5003_pose_data.orientation[2]);

          // Change to body frame angular rates (p, q, r)
          auto rb_body_ang_vel = _li3->RelativeAngularVel();
          x5003_pose_data.ang_velocity[0] = rb_body_ang_vel.Y();  // p (roll rate)
          x5003_pose_data.ang_velocity[1] = rb_body_ang_vel.X();  // q (pitch rate)
          x5003_pose_data.ang_velocity[2] = -rb_body_ang_vel.Z(); // r (yaw rate)
          
          // Add timestamp
          x5003_pose_data.timestamp = _node->get_clock()->now().nanoseconds() / 1000;
          
          _x5003_pose_pub->publish(x5003_pose_data);
        }


        void failureChecker()
        {
          // checking if any agent failed from the data coming from the topic
          // and if the joint has not been detached yet
          if(_failure_check_data.uav_1_failed && !_failure_check_data.uav_1_detached)
          {
            std::cout << "AGENT UAV_1 FAILED, DETACHING" << std::endl;
            std::cout << "DETACHMENT TIME: " << this->_world->SimTime() << std::endl;
            // if the agent i failed, detaching the joint
            for(int i=0; i<30; i++)
            {
              _joint_cable1_p->Update();
              _joint_cable1_p->Detach();
            }
            // and publishing on the topic the detachment
            _failure_check_data.uav_1_detached = true;
            _failure_check_data.timestamp = _node->get_clock()->now().nanoseconds() / 1000;
            _failure_check_pub->publish(_failure_check_data);
          }

          if(_failure_check_data.uav_2_failed && !_failure_check_data.uav_2_detached)
          {
            std::cout << "AGENT UAV_2 FAILED, DETACHING" << std::endl;
            std::cout << "DETACHMENT TIME: " << this->_world->SimTime() << std::endl;
            for(int i=0; i<30; i++)
            {
              _joint_cable2_p->Update();
              _joint_cable2_p->Detach();
            }
            _failure_check_data.uav_2_detached = true;
            _failure_check_data.timestamp = _node->get_clock()->now().nanoseconds() / 1000;
            _failure_check_pub->publish(_failure_check_data);
          }

          if(_failure_check_data.uav_3_failed && !_failure_check_data.uav_3_detached)
          {
            std::cout << "AGENT UAV_3 FAILED, DETACHING" << std::endl;
            std::cout << "DETACHMENT TIME: " << this->_world->SimTime() << std::endl;
            for(int i=0; i<30; i++)
            {
              _joint_cable3_p->Update();
              _joint_cable3_p->Detach();
            }
            _failure_check_data.uav_3_detached = true;
            _failure_check_data.timestamp = _node->get_clock()->now().nanoseconds() / 1000;
            _failure_check_pub->publish(_failure_check_data);
          }

        }


        void deletePlatsCheck()
        {
          if(_ready_signal_params_data.uav_1_ready && _ready_signal_params_data.uav_2_ready &&
              _ready_signal_params_data.uav_3_ready)
          {
            // deleting the takeoff platforms
            this->_world->RemoveModel("takeoff_plat_0");
            this->_world->RemoveModel("takeoff_plat_1");
            this->_world->RemoveModel("takeoff_plat_2");

            _takeoff_plats_deleted = true;
          }
        }


        void CreateJoints()
        {
          // getting the models from the world
          physics::BasePtr b0 = this->_world->ModelByName("x500_1");
          physics::BasePtr b1 = this->_world->ModelByName("x500_2");
          physics::BasePtr b2 = this->_world->ModelByName("x500_3");
          physics::BasePtr bp = this->_world->ModelByName("payload_3");
          // integrity check
          if(b0 == NULL || b1 == NULL || b2 == NULL || bp == NULL)
          {
            gzerr << "Some models are missing! Exiting";
            return;
          }
          // casting then to models
          physics::ModelPtr m0(dynamic_cast<physics::Model*>(b0.get()));
          physics::ModelPtr m1(dynamic_cast<physics::Model*>(b1.get()));
          physics::ModelPtr m2(dynamic_cast<physics::Model*>(b2.get()));
          physics::ModelPtr mp(dynamic_cast<physics::Model*>(bp.get()));

          // and entities, to set final poses
          // physics::EntityPtr e0(dynamic_cast<physics::Entity*>(b0.get()));
          // physics::EntityPtr e1(dynamic_cast<physics::Entity*>(b1.get()));
          // physics::EntityPtr e2(dynamic_cast<physics::Entity*>(b2.get()));
          // physics::EntityPtr e3(dynamic_cast<physics::Entity*>(b3.get()));

          // and assigning them to our private variables
          _x5001 = m0;
          _x5002 = m1;
          _x5003 = m2;
          _payload = mp;

          // getting now the links to be connected together
          physics::LinkPtr l0 = _x5001->GetLink("rod_uav::eighth_cylinder");
          physics::LinkPtr l1 = _x5002->GetLink("rod_uav::eighth_cylinder");
          physics::LinkPtr l2 = _x5003->GetLink("rod_uav::eighth_cylinder");
          _li1 = _x5001->GetLink("base_link");
          _li2 = _x5002->GetLink("base_link");
          _li3 = _x5003->GetLink("base_link");
          _lp = _payload->GetLink("base_link");
          physics::LinkPtr lp1 = _payload->GetLink("payload_cylinder_1");
          physics::LinkPtr lp2 = _payload->GetLink("payload_cylinder_2");
          physics::LinkPtr lp3 = _payload->GetLink("payload_cylinder_3");


          // creating the first joint, between x5000::rod and payload
          _joint_cable1_p = this->_physics->CreateJoint("universal", _x5001);
          _joint_cable2_p = this->_physics->CreateJoint("universal", _x5002);
          _joint_cable3_p = this->_physics->CreateJoint("universal", _x5003);

          _joint_cable1_p->SetName("payload_joint_0");
          _joint_cable2_p->SetName("payload_joint_1");
          _joint_cable3_p->SetName("payload_joint_2");

          // setting them to their parent model
          _joint_cable1_p->SetModel(_x5001);
          _joint_cable2_p->SetModel(_x5002);
          _joint_cable3_p->SetModel(_x5003);

          _joint_cable1_p->SetWorld(this->_world);
          _joint_cable2_p->SetWorld(this->_world);
          _joint_cable3_p->SetWorld(this->_world);

          // and loading them into the simulation, with their initial pose wrt child link
          _joint_cable1_p->Load(l0, lp1, _initial_joint_pose_1);
          _joint_cable2_p->Load(l1, lp2, _initial_joint_pose_2);
          _joint_cable3_p->Load(l2, lp3, _initial_joint_pose_3);

          // attaching the links
          _joint_cable1_p->Attach(l0, lp1);
          _joint_cable2_p->Attach(l1, lp2);
          _joint_cable3_p->Attach(l2, lp3);

          //
          // Initializing the whole
          _joint_cable1_p->Init();
          std::cout << "Created Joint between " << _x5001->GetName() << " and " << _payload->GetName() << std::endl;
          _joint_cable2_p->Init();
          std::cout << "Created Joint between " << _x5002->GetName() << " and " << _payload->GetName() << std::endl;
          _joint_cable3_p->Init();
          std::cout << "Created Joint between " << _x5003->GetName() << " and " << _payload->GetName() << std::endl;

          this->_physics->UpdateCollision();

        }

        ~RodPayloadJointHandler()
        {
          rclcpp::shutdown();
        }

    };

    // Register this plugin with the simulator
    GZ_REGISTER_WORLD_PLUGIN(RodPayloadJointHandler)
}
