#ifndef PARSER_H
#define PARSER_H

// #define strtk_no_tr1_or_boost
// #include "strtk/strtk.hpp"

#include <libconfig.h++>
#include <iostream>
#include <stdlib.h>
#include <string>
#include <cstring>
#include <sstream>
#include <fstream>
#include <deque>
#include <vector>
#include <list>
#include <cmath>
#include "ament_index_cpp/get_package_share_directory.hpp"
#include "ros2_muavp_interface/msg/states_info.hpp"

#include "ros2_muavp_interface/srv/failure_info.hpp"
#include "ros2_muavp_interface/srv/path_info.hpp"

#include "common.hpp"

using namespace libconfig;
using namespace common;
using namespace std;

class Parser
{

public:

    Parser();

    bool parse();
    bool parsePath();
    bool parseFailure();
    bool parsePostFailure();
    bool loadFile(std::string file);
    std::deque<ros2_muavp_interface::msg::StatesInfo> getTaskListParsed();
    std::deque<ros2_muavp_interface::srv::PathInfo::Request> getPathListParsed();
    std::deque<ros2_muavp_interface::srv::FailureInfo::Request> getFailureListParsed();
    std::deque<ros2_muavp_interface::msg::StatesInfo> getPostFailureListParsed();

private:

    Config cfg;

    std::deque<ros2_muavp_interface::msg::StatesInfo> _taskListParsed;
    std::deque<ros2_muavp_interface::srv::PathInfo::Request> _movePathListParsed;
    std::deque<ros2_muavp_interface::srv::FailureInfo::Request> _failureListParsed;
    std::deque<ros2_muavp_interface::msg::StatesInfo> _postFailureListParsed;
    // parsers for tasks
    bool parseParameters(const Setting &pre_flight_parameters);
    bool parsePreTakeOff(const Setting &tasks_pre_takeoff);
    bool parseTakeOff(const Setting &tasks_pre_takeoff);
    bool parseRotate(const Setting &tasks_rotate);
    bool parseMove(const Setting &tasks_move);
    bool parsePathMove(const Setting &tasks_path_move);
    bool parseLand(const Setting &tasks_land);
    bool parseIdlePF(const Setting &tasks_idle);
    // parsers for failures
    bool parseRecovery(const Setting &recovery_parameters);
    bool parseStop(const Setting &stop_parameters);
    bool parseLandFailure(const Setting &tasks_land);

};

#endif //MOCAP2MAV_PARSER_H
