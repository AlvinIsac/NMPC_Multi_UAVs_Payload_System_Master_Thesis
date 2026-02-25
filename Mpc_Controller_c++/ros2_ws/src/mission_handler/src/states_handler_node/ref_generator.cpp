#include <rclcpp/rclcpp.hpp>
#include "ros2_muavp_interface/msg/rigid_body_pose.hpp"
#include <cmath>

using ros2_muavp_interface::msg::RigidBodyPose;

struct PositionNED {
    double n;
    double e;
    double d;
};

class PlannerForMPC : public rclcpp::Node {
public:
    PlannerForMPC() : Node("plannerfor_mpc") {
        using std::placeholders::_1;

        sub_uav2_ = create_subscription<RigidBodyPose>(
            "/x500_2/pose", 10, std::bind(&PlannerForMPC::uav2Callback, this, _1));

        sub_uav3_ = create_subscription<RigidBodyPose>(
            "/x500_3/pose", 10, std::bind(&PlannerForMPC::uav3Callback, this, _1));

        sub_payload_ = create_subscription<RigidBodyPose>(
            "/payload/pose", 10, std::bind(&PlannerForMPC::payloadCallback, this, _1));

        pub_ref_uav2_ = create_publisher<RigidBodyPose>("/x500_2/ref", 10);
        pub_ref_uav3_ = create_publisher<RigidBodyPose>("/x500_3/ref", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(20),  // 50 Hz
            std::bind(&PlannerForMPC::process, this));
    }

private:
    rclcpp::Subscription<RigidBodyPose>::SharedPtr sub_uav2_, sub_uav3_, sub_payload_;
    rclcpp::Publisher<RigidBodyPose>::SharedPtr pub_ref_uav2_, pub_ref_uav3_;
    rclcpp::TimerBase::SharedPtr timer_;

    PositionNED uav2_pos_, uav3_pos_, payload_pos_;
    double uav3_yaw_ = 0.0;
    bool uav2_ready_ = false, uav3_ready_ = false, payload_ready_ = false;

    void uav2Callback(const RigidBodyPose::SharedPtr msg) {
        uav2_pos_ = {msg->position[0], msg->position[1], msg->position[2]};
        uav2_ready_ = true;
    }

    void uav3Callback(const RigidBodyPose::SharedPtr msg) {
        uav3_pos_ = {msg->position[0], msg->position[1], msg->position[2]};
        uav3_yaw_ = msg->orientation[2];
        uav3_ready_ = true;
    }

    void payloadCallback(const RigidBodyPose::SharedPtr msg) {
        payload_pos_ = {msg->position[0], msg->position[1], msg->position[2]};
        payload_ready_ = true;
    }

    void process() {
        if (!(uav2_ready_ && uav3_ready_ && payload_ready_)) return;

        double current_distance = std::abs(uav2_pos_.e - uav3_pos_.e);
        double L_min = 0.2;
        double L_max = 0.5;
        double danger_increment = 0.2;
        double normal_increment = 0.03;

        double uav2_e_ref = uav2_pos_.e;
        double uav3_e_ref = uav3_pos_.e;

        // ==== 4 Explicit Conditions ====
        if (current_distance < L_min) {
            if (uav2_pos_.e <= uav3_pos_.e) {
                uav2_e_ref -= danger_increment;
                uav3_e_ref += danger_increment;
            } else {
                uav2_e_ref += danger_increment;
                uav3_e_ref -= danger_increment;
            }
        } else if (current_distance == L_max) {
            // Ideal distance — keep same
        } else if (current_distance < L_max) {
            if (uav2_pos_.e <= uav3_pos_.e) {
                uav2_e_ref -= normal_increment;
                uav3_e_ref += normal_increment;
            } else {
                uav2_e_ref += normal_increment;
                uav3_e_ref -= normal_increment;
            }
        } else {
            if (uav2_pos_.e <= uav3_pos_.e) {
                uav2_e_ref += danger_increment;
                uav3_e_ref -= danger_increment;
            } else {
                uav2_e_ref -= danger_increment;
                uav3_e_ref += danger_increment;
            }
        }

        // === Payload swing check (projected along UAV3's heading) ===
        double dx = payload_pos_.n - uav3_pos_.n;
        double dy = payload_pos_.e - uav3_pos_.e;
        double ux = std::cos(uav3_yaw_);
        double uy = std::sin(uav3_yaw_);
        double swing_disp = dx * ux + dy * uy;

        double swing_threshold = 0.3;
        double swing_velocity = 0.5;
        double uav2_vel = 0.0, uav3_vel = 0.0;

        if (std::abs(swing_disp) > swing_threshold) {
            uav2_vel = swing_velocity;
            uav3_vel = swing_velocity;
        }

        // === Publish UAV2 ref ===
        RigidBodyPose ref_uav2;
        ref_uav2.timestamp = this->now().nanoseconds();
        ref_uav2.position = {uav2_pos_.n, uav2_e_ref, uav2_pos_.d};
        ref_uav2.lin_velocity = {0.0, uav2_vel, 0.0};
        pub_ref_uav2_->publish(ref_uav2);

        // === Publish UAV3 ref ===
        RigidBodyPose ref_uav3;
        ref_uav3.timestamp = this->now().nanoseconds();
        ref_uav3.position = {uav3_pos_.n, uav3_e_ref, uav3_pos_.d};
        ref_uav3.lin_velocity = {0.0, uav3_vel, 0.0};
        pub_ref_uav3_->publish(ref_uav3);

        RCLCPP_INFO(this->get_logger(),
                    "[REF] E_UAV2: %.2f | E_UAV3: %.2f | SwingDisp: %.3f | V_ref: %.2f",
                    uav2_e_ref, uav3_e_ref, swing_disp, uav3_vel);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PlannerForMPC>());
    rclcpp::shutdown();
    return 0;
}
