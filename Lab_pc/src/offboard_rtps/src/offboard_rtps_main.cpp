#include <cstdio>
#include "offboard_rtps_handler.hpp"

int main(int argc, char ** argv)
{
  // (void) argc;
  // (void) argv;
  std::string agent_name = "";
  double force_limit_1m{0.0}, l_12{0.0}, l_34{0.0};
  if(argc>2)
  {
    agent_name = std::string(argv[1]);
    force_limit_1m = std::stod(argv[2]);
    l_12 = std::stod(argv[3]);
    l_34 = std::stod(argv[4]);
    std::cout << "New registered Offboard-RTPS Control Node with Agent: " << agent_name << std::endl;
  }
  else
  {
    std::cout << "No Agent Name provided for Offboard-RTPS Control Node, stopping" << std::endl;
    return 0;
  }

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OffboardRTPSHandler>(agent_name, force_limit_1m, l_12, l_34));

  rclcpp::shutdown();
  return 0;
}
