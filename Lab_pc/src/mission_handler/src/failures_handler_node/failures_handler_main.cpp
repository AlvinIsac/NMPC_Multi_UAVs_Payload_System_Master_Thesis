#include "failures_handler.hpp"

int main(int argc, char ** argv)
{

    std::string agent_name = "";
    if(argc>2)
    {
      agent_name = std::string(argv[1]);
      std::cout << "New registered Failures Handler Node with Agent: " << agent_name << std::endl;
    }
    else
    {
      std::cout << "No Agent Name provided for Failures Handler Node, stopping" << std::endl;
      return 0;
    }


    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FailuresHandler>(agent_name));
    rclcpp::shutdown();
    return 0;
}
