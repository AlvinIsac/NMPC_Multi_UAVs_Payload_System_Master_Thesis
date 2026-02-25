#include "rclcpp/rclcpp.hpp"
#include "ros2_muavp_interface/msg/failure_check.hpp"
#include <cstdlib>
#include <string>
#include <vector>

using std::placeholders::_1;

class FailureMonitor : public rclcpp::Node
{
public:
    FailureMonitor()
    : Node("failure_monitor")
    {
        subscription_ = this->create_subscription<ros2_muavp_interface::msg::FailureCheck>(
            "/agents/failure_check", 10, std::bind(&FailureMonitor::callback, this, _1));
    }

private:
    bool already_triggered_ = false;

    void callback(const ros2_muavp_interface::msg::FailureCheck::SharedPtr msg)
    {
        if (already_triggered_) return;

        if (msg->uav_1_failed || msg->uav_2_failed || msg->uav_3_failed)
        {
            RCLCPP_WARN(this->get_logger(), "A UAV FAILURE DETECTED! Shutting down all handler nodes...");
            kill_all_handlers();
            already_triggered_ = true;
        }
    }

    // void kill_all_handlers()
    // {
    //     std::vector<std::string> uav_ids = {"x500_1", "x500_2", "x500_3"};
    //     std::vector<std::string> process_names = {
    //         "transitions_handler_main",
    //         "states_handler_main",
    //         "failures_handler_main"
    //     };

    //     for (const auto &uav : uav_ids)
    //     {
    //         for (const auto &proc : process_names)
    //         {
    //             std::string full_match = proc + " " + uav;

    //             // Debug print
    //             std::string check_cmd = "ps aux | grep \"" + full_match + "\" | grep -v grep";
    //             std::system(check_cmd.c_str());

    //             // Kill
    //             std::string kill_cmd = "pkill -f \"" + full_match + "\"";
    //             int result = std::system(kill_cmd.c_str());

    //             if (result == 0)
    //                 RCLCPP_INFO(this->get_logger(), "Killed process: %s", full_match.c_str());
    //             else
    //                 RCLCPP_ERROR(this->get_logger(), "Failed to kill or not found: %s", full_match.c_str());
    //         }
    //     }
    // }

void kill_all_handlers()
{
    std::vector<std::string> uav_ids = {"x500_1", "x500_2", "x500_3"};
    std::vector<std::string> process_names = {
        "transitions_handler_main",
        "states_handler_main",
        "failures_handler_main"
    };

    for (const auto &uav : uav_ids)
    {
        for (const auto &proc : process_names)
        {
            std::string full_match = proc + " " + uav;

            // Step 1: find matching PIDs
            std::string pgrep_cmd = "pgrep -f \"" + full_match + "\"";
            FILE* pipe = popen(pgrep_cmd.c_str(), "r");

            if (!pipe) {
                RCLCPP_ERROR(this->get_logger(), "Failed to run pgrep for: %s", full_match.c_str());
                continue;
            }

            char buffer[128];
            bool found = false;

            while (fgets(buffer, sizeof(buffer), pipe) != nullptr)
            {
                int pid = std::stoi(buffer);
                std::string kill_cmd = "kill " + std::to_string(pid);
                int result = std::system(kill_cmd.c_str());
                if (result == 0) {
                    RCLCPP_INFO(this->get_logger(), "Killed PID %d for: %s", pid, full_match.c_str());
                    found = true;
                }
            }

            pclose(pipe);

            if (!found)
                RCLCPP_WARN(this->get_logger(), "No matching process found for: %s", full_match.c_str());
        }
    }
}


    rclcpp::Subscription<ros2_muavp_interface::msg::FailureCheck>::SharedPtr subscription_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FailureMonitor>());
    rclcpp::shutdown();
    return 0;
}
