#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <math.h>
#include "rclcpp/rclcpp.hpp"
#include "ros2_muavp_interface/msg/joint_force_torque.hpp"
#include <vector>

namespace gazebo
{

    class JointManager : public ModelPlugin
    {

    private:

        physics::ModelPtr _model;
        event::ConnectionPtr _updateConnection;
        physics::JointPtr _joint_0;
        physics::JointPtr _joint_1;
        physics::JointWrench _joint_wrench_0;
        physics::JointWrench _joint_wrench_1;
        physics::LinkPtr _fake_link = NULL;
        physics::LinkPtr _first_link = NULL;

        std::string _rod_force_torque_topic;

        rclcpp::Node::SharedPtr _node;
        rclcpp::Publisher<ros2_muavp_interface::msg::JointForceTorque>::SharedPtr _ros2_joint_force_pub;

        double _actual_sim_time;
        double _prev_sim_time;
        double _dt;
        double _offset_x;
        double _offset_y;

        // void manageTime(const common::UpdateInfo & _info)
        // {
        //
        //     double t = (double)_info.simTime.nsec;
        //
        //     _actual_sim_time = t;
        //     _dt = _actual_sim_time - _prev_sim_time;
        //     _prev_sim_time = _actual_sim_time;
        //
        // }


    public:

        void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
        {
            // Store the pointer to the model
            this->_model = _parent;
        //    _model.get()->SetStatic(true);
        //    _model.get()->SetGravityMode(false);

            // Listen to the update event. This event is broadcast every simulation iteration.
            this->_updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&JointManager::OnUpdate, this, _1));
            // getting the interested joint
            // this->_joint_0 = this->_model->GetJoint("rod_uav_joint_0");
            this->_joint_1 = this->_model->GetJoint("rod_uav::rod_uav_joint_1");
            // //
            // setting the topics where to publish
            _rod_force_torque_topic = "/" + this->_model.get()->GetName() + "/rod_force_torque";
            // //
            std::cout << "Plugin registered for joints: " << std::endl;
            // std::cout << "   " << this->_joint_0.get()->GetName() << std::endl;
            std::cout << "   " << this->_joint_1.get()->GetName() << std::endl;
            std::cout << "Publishing into Topic: " << _rod_force_torque_topic << std::endl;
            //
            std::string node_name = this->_model.get()->GetName() + "_joint_plugin_gazebo_ros2";

            // initializing rod model, to get the fake link
            _fake_link = this->_model->GetLink("rod_uav::fake_cylinder");
            _first_link = this->_model->GetLink("rod_uav::first_cylinder");

            if(_fake_link == NULL || _first_link == NULL)
            {
              std::cout << "LINK NOT FOUND, TERMINATING" << std::endl;
              return;
            }

            // creating a ros2 node
            if(!rclcpp::ok())
              rclcpp::init(0, nullptr);

            _node = rclcpp::Node::make_shared(node_name);
            // allocating the advertiser
            _ros2_joint_force_pub = _node->create_publisher<ros2_muavp_interface::msg::JointForceTorque>(_rod_force_torque_topic, 10);
            // init some vars
            _actual_sim_time = 0;
            _prev_sim_time = 0;

        }

        // Called by the world update start event
        void OnUpdate(const common::UpdateInfo & _info)
        {
            // manageTime(_info);
            // getting the force and torque from the joint
            // _joint_wrench_0 = this->_joint_0->GetForceTorque(0u);
            _joint_wrench_1 = this->_joint_1->GetForceTorque(0u);
            auto fake_link_relative_pose = this->_fake_link->RelativePose();
            // // creating the message for the topic
            auto ros2_joint_force_msg = ros2_muavp_interface::msg::JointForceTorque();
            // // filling then the topic
            ros2_joint_force_msg.timestamp = _actual_sim_time;
            // // first with the Forces applied by the child link to parent one
            // ros2_joint_force_msg.force_joint_0[0] = _joint_wrench_0.body2Force[0];
            // ros2_joint_force_msg.force_joint_0[1] = _joint_wrench_0.body2Force[1];
            // ros2_joint_force_msg.force_joint_0[2] = _joint_wrench_0.body2Force[2];

            // first with the Forces applied by the child link to parent one
            ros2_joint_force_msg.force_joint_1[0] = _joint_wrench_1.body2Force[0];
            ros2_joint_force_msg.force_joint_1[1] = _joint_wrench_1.body2Force[1];
            ros2_joint_force_msg.force_joint_1[2] = _joint_wrench_1.body2Force[2];
            // then torques
            // ros2_joint_force_msg.torque_joint_0[0] = _joint_wrench_0.body2Torque[0];
            // ros2_joint_force_msg.torque_joint_0[1] = _joint_wrench_0.body2Torque[1];
            // ros2_joint_force_msg.torque_joint_0[2] = _joint_wrench_0.body2Torque[2];

            ros2_joint_force_msg.torque_joint_1[0] = _joint_wrench_1.body2Torque[0];
            ros2_joint_force_msg.torque_joint_1[1] = _joint_wrench_1.body2Torque[1];
            ros2_joint_force_msg.torque_joint_1[2] = _joint_wrench_1.body2Torque[2];
            // and then orientation
            // ros2_joint_force_msg.orientation[0] = (float)_joint_1->Position(1);
            // ros2_joint_force_msg.orientation[1] = (float)_joint_1->Position(0);

            // taking the relative orientation w.r.t. the parent (UAV)
            ros2_joint_force_msg.orientation[0] = fake_link_relative_pose.Rot().Roll();
            ros2_joint_force_msg.orientation[1] = fake_link_relative_pose.Rot().Pitch();
            ros2_joint_force_msg.orientation[2] = fake_link_relative_pose.Rot().Yaw();

            // std::cout << this->_model.get()->GetName() << " - roll: " << ros2_joint_force_msg.orientation[0] << std::endl;
            // std::cout << this->_model.get()->GetName() << " - pitch: " << ros2_joint_force_msg.orientation[1] << std::endl;
            // std::cout << this->_model.get()->GetName() << " - yaw: " << ros2_joint_force_msg.orientation[2] << std::endl;
            // std::cout << "--------------------------------------------------" << std::endl;

            // // publishing it finally
            _ros2_joint_force_pub->publish(ros2_joint_force_msg);
            rclcpp::spin_some(_node);
        }

        ~JointManager()
        {
          rclcpp::shutdown();
        }

    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(JointManager)
}
