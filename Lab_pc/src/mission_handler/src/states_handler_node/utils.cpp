#include "states_handler.hpp"

// %%%--------------------------%%%
// --------- FIXING ODOMETRY ----------

// Each Quadrotor considers its spawning point as its local (0,0,0), but the
// quadrotor is in a different point that the true origin (where the payload is
// spawned). Correcting then its position and orientation wrt gazebo (and payload) local frame
void StatesHandler::fixingOdometry(px4_msgs::msg::VehicleOdometry &odom_data, std::string agent_name)
{
  if(agent_name == "x500_1")
  {
    odom_data.position[0] += _parameters.params[5];
    odom_data.position[1] += _parameters.params[6];
  }
  else if(agent_name == "x500_2")
  {
    odom_data.position[0] += _parameters.params[7];
    odom_data.position[1] += _parameters.params[8];
  }
  else if(agent_name == "x500_3")
  {
    odom_data.position[0] += _parameters.params[9];
    odom_data.position[1] += _parameters.params[10];
  }

  odom_data.position[2] += _parameters.params[11];
}


// %%%--------------------------%%%
// --------- FILLING ANCHOR POINTS STRUCT ----------

void StatesHandler::anchorPointsStructFiller()
{
  // saving then in a vector the distance between the uav and the payload
  // double uav_payload_dist[3] = {_payload_pose_data.position[0] - _odom_data.position[0], _payload_pose_data.position[1] - _odom_data.position[1], _payload_pose_data.position[2] - _odom_data.position[2]};
  _ancPointsData.uav_payload_exact_dist << _payload_pose_data.position[0] - _agent_pose_data.position[0], _payload_pose_data.position[1] - _agent_pose_data.position[1], _payload_pose_data.position[2] - _agent_pose_data.position[2];
  // taking into account the fact the given position from the topic is the
  // payload's pos wrt its center of mass, computing then the norm between it
  // and a generic anchor point on the payload (assuming they are placed
  // specularly with respect to the center of mass)
  // double payload_anchor_dist_norm = 0;
  VectorXd r_l1 = VectorXd(3);
  VectorXd r_l2 = VectorXd(3);
  VectorXd r_l3 = VectorXd(3);
  r_l1 << _parameters.params[12], _parameters.params[13], _parameters.params[14];
  r_l2 << _parameters.params[15], _parameters.params[16], _parameters.params[17];
  r_l3 << _parameters.params[18], _parameters.params[19], _parameters.params[20];

  // doing the same for the quadrotor as well, given that its anchor point is
  // not it its center of mass, but shifted
  // double uav_anchor_dist_norm = norm(_ancPointsData.r_uav);

  double phi_uav = _euler_angles[0];
  double theta_uav = _euler_angles[1];
  double psi_uav = _euler_angles[2];

  double uav_anchor_point_x = (cos(psi_uav)*cos(theta_uav))*_parameters.params[2] + (sin(phi_uav)*sin(theta_uav)*cos(psi_uav)-cos(phi_uav)*sin(psi_uav))*_parameters.params[3] + (sin(phi_uav)*sin(psi_uav)+cos(phi_uav)*sin(theta_uav)*cos(psi_uav))*_parameters.params[4];
  double uav_anchor_point_y = (cos(theta_uav)*sin(psi_uav))*_parameters.params[2] + (cos(phi_uav)*cos(psi_uav)+sin(phi_uav)*sin(theta_uav)*sin(psi_uav))*_parameters.params[3] + (cos(phi_uav)*sin(theta_uav)*sin(psi_uav)-sin(phi_uav)*cos(psi_uav))*_parameters.params[4];
  double uav_anchor_point_z = -sin(theta_uav)*_parameters.params[2] + sin(phi_uav)*cos(theta_uav)*_parameters.params[3] + cos(phi_uav)*cos(theta_uav)*_parameters.params[4];
  _ancPointsData.uav_anchor_point << uav_anchor_point_x, uav_anchor_point_y, uav_anchor_point_z;

  double load_anchor_point_1_x = cos(_payload_pose_data.orientation[2])*cos(_payload_pose_data.orientation[1])*_parameters.params[12] + (sin(_payload_pose_data.orientation[0])*sin(_payload_pose_data.orientation[1])*cos(_payload_pose_data.orientation[2])-cos(_payload_pose_data.orientation[0])*sin(_payload_pose_data.orientation[2]))*_parameters.params[13] + (sin(_payload_pose_data.orientation[0])*sin(_payload_pose_data.orientation[2])+cos(_payload_pose_data.orientation[0])*sin(_payload_pose_data.orientation[1])*cos(_payload_pose_data.orientation[2]))*_parameters.params[14];
  double load_anchor_point_1_y = cos(_payload_pose_data.orientation[1])*sin(_payload_pose_data.orientation[2])*_parameters.params[12] + (cos(_payload_pose_data.orientation[0])*cos(_payload_pose_data.orientation[2])+sin(_payload_pose_data.orientation[0])*sin(_payload_pose_data.orientation[1])*sin(_payload_pose_data.orientation[2]))*_parameters.params[13] + (cos(_payload_pose_data.orientation[0])*sin(_payload_pose_data.orientation[1])*sin(_payload_pose_data.orientation[2])-sin(_payload_pose_data.orientation[0])*cos(_payload_pose_data.orientation[2]))*_parameters.params[14];
  double load_anchor_point_1_z = -sin(_payload_pose_data.orientation[1])*_parameters.params[12] + sin(_payload_pose_data.orientation[0])*cos(_payload_pose_data.orientation[1])*_parameters.params[13] + cos(_payload_pose_data.orientation[0])*cos(_payload_pose_data.orientation[1])*_parameters.params[14];
  _ancPointsData.load_anchor_point_1 << load_anchor_point_1_x, load_anchor_point_1_y, load_anchor_point_1_z;

  double load_anchor_point_2_x = cos(_payload_pose_data.orientation[2])*cos(_payload_pose_data.orientation[1])*_parameters.params[15] + (sin(_payload_pose_data.orientation[0])*sin(_payload_pose_data.orientation[1])*cos(_payload_pose_data.orientation[2])-cos(_payload_pose_data.orientation[0])*sin(_payload_pose_data.orientation[2]))*_parameters.params[16] + (sin(_payload_pose_data.orientation[0])*sin(_payload_pose_data.orientation[2])+cos(_payload_pose_data.orientation[0])*sin(_payload_pose_data.orientation[1])*cos(_payload_pose_data.orientation[2]))*_parameters.params[17];
  double load_anchor_point_2_y = cos(_payload_pose_data.orientation[1])*sin(_payload_pose_data.orientation[2])*_parameters.params[15] + (cos(_payload_pose_data.orientation[0])*cos(_payload_pose_data.orientation[2])+sin(_payload_pose_data.orientation[0])*sin(_payload_pose_data.orientation[1])*sin(_payload_pose_data.orientation[2]))*_parameters.params[16] + (cos(_payload_pose_data.orientation[0])*sin(_payload_pose_data.orientation[1])*sin(_payload_pose_data.orientation[2])-sin(_payload_pose_data.orientation[0])*cos(_payload_pose_data.orientation[2]))*_parameters.params[17];
  double load_anchor_point_2_z = -sin(_payload_pose_data.orientation[1])*_parameters.params[15] + sin(_payload_pose_data.orientation[0])*cos(_payload_pose_data.orientation[1])*_parameters.params[16] + cos(_payload_pose_data.orientation[0])*cos(_payload_pose_data.orientation[1])*_parameters.params[17];
  _ancPointsData.load_anchor_point_2 << load_anchor_point_2_x, load_anchor_point_2_y, load_anchor_point_2_z;


  double load_anchor_point_3_x = cos(_payload_pose_data.orientation[2])*cos(_payload_pose_data.orientation[1])*_parameters.params[18] + (sin(_payload_pose_data.orientation[0])*sin(_payload_pose_data.orientation[1])*cos(_payload_pose_data.orientation[2])-cos(_payload_pose_data.orientation[0])*sin(_payload_pose_data.orientation[2]))*_parameters.params[19] + (sin(_payload_pose_data.orientation[0])*sin(_payload_pose_data.orientation[2])+cos(_payload_pose_data.orientation[0])*sin(_payload_pose_data.orientation[1])*cos(_payload_pose_data.orientation[2]))*_parameters.params[20];
  double load_anchor_point_3_y = cos(_payload_pose_data.orientation[1])*sin(_payload_pose_data.orientation[2])*_parameters.params[18] + (cos(_payload_pose_data.orientation[0])*cos(_payload_pose_data.orientation[2])+sin(_payload_pose_data.orientation[0])*sin(_payload_pose_data.orientation[1])*sin(_payload_pose_data.orientation[2]))*_parameters.params[19] + (cos(_payload_pose_data.orientation[0])*sin(_payload_pose_data.orientation[1])*sin(_payload_pose_data.orientation[2])-sin(_payload_pose_data.orientation[0])*cos(_payload_pose_data.orientation[2]))*_parameters.params[20];
  double load_anchor_point_3_z = -sin(_payload_pose_data.orientation[1])*_parameters.params[18] + sin(_payload_pose_data.orientation[0])*cos(_payload_pose_data.orientation[1])*_parameters.params[19] + cos(_payload_pose_data.orientation[0])*cos(_payload_pose_data.orientation[1])*_parameters.params[20];
  _ancPointsData.load_anchor_point_3 << load_anchor_point_3_x, load_anchor_point_3_y, load_anchor_point_3_z;

  // getting the anchor points positions depending on the UAV
  _ancPointsData.uav_payload_exact_dist[0] -= _ancPointsData.uav_anchor_point[0];
  _ancPointsData.uav_payload_exact_dist[1] -= _ancPointsData.uav_anchor_point[1];
  _ancPointsData.uav_payload_exact_dist[2] -= _ancPointsData.uav_anchor_point[2];
  if(_agent_name == "x500_1")
  {
    _ancPointsData.uav_payload_exact_dist[0] += _ancPointsData.load_anchor_point_1[0];
    _ancPointsData.uav_payload_exact_dist[1] += _ancPointsData.load_anchor_point_1[1];
    _ancPointsData.uav_payload_exact_dist[2] += _ancPointsData.load_anchor_point_1[2];
  }
  else if(_agent_name == "x500_2")
  {
    _ancPointsData.uav_payload_exact_dist[0] += _ancPointsData.load_anchor_point_2[0];
    _ancPointsData.uav_payload_exact_dist[1] += _ancPointsData.load_anchor_point_2[1];
    _ancPointsData.uav_payload_exact_dist[2] += _ancPointsData.load_anchor_point_2[2];
  }
  else if(_agent_name == "x500_3")
  {
    _ancPointsData.uav_payload_exact_dist[0] += _ancPointsData.load_anchor_point_3[0];
    _ancPointsData.uav_payload_exact_dist[1] += _ancPointsData.load_anchor_point_3[1];
    _ancPointsData.uav_payload_exact_dist[2] += _ancPointsData.load_anchor_point_3[2];
  }


  double dist_norm_exact_2 = norm(_ancPointsData.uav_payload_exact_dist);

  _ancPointsData.dist_norm_exact = dist_norm_exact_2;

}

// %%%--------------------------%%%
// --------- TRANSFORMING TENSIONS FROM INERTIAL TO BODY ----------

VectorXd StatesHandler::tensionInertialToBody(double T_x, double T_y, double T_z)
{
  // Constructing the inverse of the rotational matrix from body to inertial frame
  // MatrixXd R_inv = MatrixXd(3,3);
  MatrixXd R = MatrixXd(3,3);

  // vector to store the inertial tension values
  VectorXd T_inert = VectorXd(3);
  T_inert << T_x, T_y, T_z;

  // getting the angles
  double phi = _euler_angles[0];
  double theta = _euler_angles[1];
  double psi = _euler_angles[2];

  // building the Rotation matrix from body frame to inertial frame
  R << cos(psi)*cos(theta), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi),
      cos(theta)*sin(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi),
      -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta);

  // body tension is returned
  // R is an orthogonal matrix, hence its inverse is its transpose
  return R.transpose()*T_inert;

}


// %%%--------------------------%%%
// --------- BEARING ANGLE ----------

double StatesHandler::bearingAngleComputation()
{
  double x_diff = _detachUAVData.surv_position[0] - _agent_pose_data.position[0];
  double y_diff = _detachUAVData.surv_position[1] - _agent_pose_data.position[1];

  double bearing = atan2(y_diff,x_diff);
  // double bearing = atan(y_diff/x_diff);
  // std::cout << _agent_name << " - BEARING: " << bearing << std::endl;

  double yaw_des = bearing;
  // adapting the yaw in the interval [0, 2*PI]
  if(yaw_des < 0.0)
    yaw_des += 2*PI;
  // else if(yaw_des >= 2*PI)
  //   yaw_des -= 2*PI;
  // std::cout << _agent_name << " - YAW_DES: " << yaw_des << std::endl;

  return yaw_des;
}

// %%%--------------------------%%%
// --------- ANCHOR POINTS VELOCITY ----------

VectorXd StatesHandler::anchorPointsAndVelComputation()
{
  double phi_me = _euler_angles[0];
  double theta_me = _euler_angles[1];
  double psi_me = _euler_angles[2];

  double me_anchor_point_x = (cos(psi_me)*cos(theta_me))*_parameters.params[2] + (sin(phi_me)*sin(theta_me)*cos(psi_me)-cos(phi_me)*sin(psi_me))*_parameters.params[3] + (sin(phi_me)*sin(psi_me)+cos(phi_me)*sin(theta_me)*cos(psi_me))*_parameters.params[4];
  double me_anchor_point_y = (cos(theta_me)*sin(psi_me))*_parameters.params[2] + (cos(phi_me)*cos(psi_me)+sin(phi_me)*sin(theta_me)*sin(psi_me))*_parameters.params[3] + (cos(phi_me)*sin(theta_me)*sin(psi_me)-sin(phi_me)*cos(psi_me))*_parameters.params[4];
  double me_anchor_point_z = -sin(theta_me)*_parameters.params[2] + sin(phi_me)*cos(theta_me)*_parameters.params[3] + cos(phi_me)*cos(theta_me)*_parameters.params[4];

  double phi_surv(0.0), theta_surv(0.0), psi_surv(0.0);
  double x_surv(0.0), y_surv(0.0), z_surv(0.0);
  double x_surv_vel(0.0), y_surv_vel(0.0), z_surv_vel(0.0);

  survivedAgentPoseAssignment();

  phi_surv = _detachUAVData.surv_orientation(0);
  theta_surv = _detachUAVData.surv_orientation(1);
  psi_surv = _detachUAVData.surv_orientation(2);
  x_surv = _detachUAVData.surv_position(0);
  y_surv = _detachUAVData.surv_position(1);
  z_surv = _detachUAVData.surv_position(2);
  x_surv_vel = _detachUAVData.surv_lin_vel(0);
  y_surv_vel = _detachUAVData.surv_lin_vel(1);
  z_surv_vel = _detachUAVData.surv_lin_vel(2);

  // std::cout << x_surv << std::endl;
  // std::cout << y_surv << std::endl;
  // std::cout << z_surv << std::endl;

  double surv_uav_anchor_point_x = (cos(psi_surv)*cos(theta_surv))*_parameters.params[2] + (sin(phi_surv)*sin(theta_surv)*cos(psi_surv)-cos(phi_surv)*sin(psi_surv))*_parameters.params[3] + (sin(phi_surv)*sin(psi_surv)+cos(phi_surv)*sin(theta_surv)*cos(psi_surv))*_parameters.params[4];
  double surv_uav_anchor_point_y = (cos(theta_surv)*sin(psi_surv))*_parameters.params[2] + (cos(phi_surv)*cos(psi_surv)+sin(phi_surv)*sin(theta_surv)*sin(psi_surv))*_parameters.params[3] + (cos(phi_surv)*sin(theta_surv)*sin(psi_surv)-sin(phi_surv)*cos(psi_surv))*_parameters.params[4];
  double surv_uav_anchor_point_z = -sin(theta_surv)*_parameters.params[2] + sin(phi_surv)*cos(theta_surv)*_parameters.params[3] + cos(phi_surv)*cos(theta_surv)*_parameters.params[4];

  _ancPointsData.surv_uav_anchor_point << surv_uav_anchor_point_x + x_surv, surv_uav_anchor_point_y + y_surv, surv_uav_anchor_point_z + z_surv;
  _ancPointsData.uav_anchor_point << me_anchor_point_x + _agent_pose_data.position[0], me_anchor_point_y + _agent_pose_data.position[1], me_anchor_point_z + _agent_pose_data.position[2];

  VectorXd lin_vel_mean = VectorXd(3);
  lin_vel_mean << (_agent_pose_data.lin_velocity[0] + x_surv_vel)/2.0, (_agent_pose_data.lin_velocity[1] + y_surv_vel)/2.0, (_agent_pose_data.lin_velocity[2] + z_surv_vel)/2.0;

  return lin_vel_mean;
}


// %%%--------------------------%%%
// --------- TENSION ON ROD IN FRAMES ----------

void StatesHandler::tensionRodComputation()
{
  // updating the struct
  anchorPointsStructFiller();

  // transforming the tension data from joint's frame to body one
  double alpha = _joint_data.orientation[0];
  double beta = _joint_data.orientation[1];
  double gamma = _joint_data.orientation[2];

  // defining the variables needed to transform the tension
  VectorXd measured_tension = VectorXd(3);
  VectorXd body_tension_body_frame = VectorXd(3);
  MatrixXd R_x = MatrixXd(3,3);
  MatrixXd R_y = MatrixXd(3,3);
  MatrixXd R_z = MatrixXd(3,3);
  MatrixXd R = MatrixXd(3,3);
  MatrixXd R_I = MatrixXd(3,3);

  // storing the measures in a VectorXd object, so to be handled properly
  measured_tension << _joint_data.force_joint_1[0], _joint_data.force_joint_1[1], _joint_data.force_joint_1[2];

  // filling the single rotational matrices, for roll
  R_x << 1, 0, 0,
        0, cosf(alpha), -sinf(alpha),
        0, sinf(alpha), cosf(alpha);
  // for pitch
  R_y << cosf(beta), 0, sinf(beta),
        0, 1, 0,
        -sinf(beta), 0, cosf(beta);
  // and for yaw
  R_z << cosf(gamma), -sinf(gamma), 0,
        sinf(gamma), cosf(gamma), 0,
        0, 0, 1;
  // the rotational matrix between the cable and the UAV is the product of the three
  // following the chosen ZYX convention.
  R = R_z * R_y * R_x;
  // rotating the measures
  body_tension_body_frame = R*measured_tension;
  // and assigning them. changing from the Gazebo body frame (X - Front, Y - Left, Z - Up)
  // to the desired one (X - Front, Y - Right, Z - Down)
  _tensions.body_tension << body_tension_body_frame[0], -body_tension_body_frame[1], -body_tension_body_frame[2];

  // BONUS: (not needed though) Body to Inertial rotation.
  // getting the UAV's angles
  double phi = _euler_angles[0];
  double theta = _euler_angles[1];
  double psi = _euler_angles[2];

  // building the Rotation matrix from body frame to inertial frame (same as the product of previously declared rotational matrices)
  R_I << cos(psi)*cos(theta), sin(phi)*sin(theta)*cos(psi)-cos(phi)*sin(psi), sin(phi)*sin(psi)+cos(phi)*sin(theta)*cos(psi),
      cos(theta)*sin(psi), cos(phi)*cos(psi)+sin(phi)*sin(theta)*sin(psi), cos(phi)*sin(theta)*sin(psi)-sin(phi)*cos(psi),
      -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta);
  // rotating it and getting the measures in the inertial frame as well.
  _tensions.inert_tension = R_I*_tensions.body_tension;

  // taking the norm finally
  double body_tension_norm = norm(_tensions.body_tension);
  // double inert_tension_norm = norm(_tensions.inert_tension);
  _tensions.tension_norm = body_tension_norm;
  // _tensions.tension_norm = inert_tension_norm;

}

// %%%--------------------------%%%
// --------- INERTIAL TENSION ESTIMATOR ----------

Vector3d StatesHandler::tensionEstimation()
{

    // retrieving tension parameters
    double k(0.0), l_t(0.0), b(0.0);
    if(_failure_occured)
    {
      int fail_state = (int)_failure_action_data.curr_action;
      if(_agent_name == "x500_1")
      {
        k = _failure_info_data[fail_state].params[0];
        l_t = _failure_info_data[fail_state].params[1];
        b = _failure_info_data[fail_state].params[2];
      }
      else if(_agent_name == "x500_2")
      {
        k = _failure_info_data[fail_state].params[9];
        l_t = _failure_info_data[fail_state].params[10];
        b = _failure_info_data[fail_state].params[11];
      }
      else if(_agent_name == "x500_3")
      {
        k = _failure_info_data[fail_state].params[12];
        l_t = _failure_info_data[fail_state].params[13];
        b = _failure_info_data[fail_state].params[14];
      }
      else
      {
        std::cout << "No UAV recognized, setting default values for tension" << std::endl;
      }
    }
    else
    {
      if(_agent_name == "x500_1")
      {
        k = _states_info_data.params[0];
        l_t = _states_info_data.params[1];
        b = _states_info_data.params[2];
      }
      else if(_agent_name == "x500_2")
      {
        k = _states_info_data.params[3];
        l_t = _states_info_data.params[4];
        b = _states_info_data.params[5];
      }
      else if(_agent_name == "x500_3")
      {
        k = _states_info_data.params[6];
        l_t = _states_info_data.params[7];
        b = _states_info_data.params[8];
      }
      else
      {
        std::cout << "No UAV recognized, setting default values for tension" << std::endl;
      }
    }

    anchorPointsStructFiller();


    double r_l[3] = {0, 0, 0};

    if(_agent_name == "x500_1")
    {
      r_l[0] = _ancPointsData.r_l1[0];
      r_l[1] = _ancPointsData.r_l1[1];
      r_l[2] = _ancPointsData.r_l1[2];
    }
    else if(_agent_name == "x500_2")
    {
      r_l[0] = _ancPointsData.r_l2[0];
      r_l[1] = _ancPointsData.r_l2[1];
      r_l[2] = _ancPointsData.r_l2[2];
    }
    else if(_agent_name == "x500_3")
    {
      r_l[0] = _ancPointsData.r_l3[0];
      r_l[1] = _ancPointsData.r_l3[1];
      r_l[2] = _ancPointsData.r_l3[2];
    }

    // double phi_dot = _euler_rates[0];
    // double theta_dot = _euler_rates[1];
    // double psi_dot = _euler_rates[2];

    double payload_anc_vel_x(0.0), payload_anc_vel_y(0.0), payload_anc_vel_z(0.0);
    double me_anc_vel_x(0.0), me_anc_vel_y(0.0), me_anc_vel_z(0.0);

    // rotation
    double phi_uav = _euler_angles[0];
    double theta_uav = _euler_angles[1];
    double psi_uav = _euler_angles[2];

    double phi_load = _payload_pose_data.orientation[0];
    double theta_load = _payload_pose_data.orientation[1];
    double psi_load = _payload_pose_data.orientation[2];

    double phi_dot_load = _payload_pose_data.ang_velocity[0];
    double theta_dot_load = _payload_pose_data.ang_velocity[1];
    double psi_dot_load = _payload_pose_data.ang_velocity[2];

    MatrixXd R_uav = MatrixXd(3,3);

    MatrixXd R_load = MatrixXd(3,3);

    R_uav << cos(psi_uav)*cos(theta_uav), sin(phi_uav)*sin(theta_uav)*cos(psi_uav)-cos(phi_uav)*sin(psi_uav), sin(phi_uav)*sin(psi_uav)+cos(phi_uav)*sin(theta_uav)*cos(psi_uav),
        cos(theta_uav)*sin(psi_uav), cos(phi_uav)*cos(psi_uav)+sin(phi_uav)*sin(theta_uav)*sin(psi_uav), cos(phi_uav)*sin(theta_uav)*sin(psi_uav)-sin(phi_uav)*cos(psi_uav),
        -sin(theta_uav), sin(phi_uav)*cos(theta_uav), cos(phi_uav)*cos(theta_uav);


    R_load << cos(psi_load)*cos(theta_load), sin(phi_load)*sin(theta_load)*cos(psi_load)-cos(phi_load)*sin(psi_load), sin(phi_load)*sin(psi_load)+cos(phi_load)*sin(theta_load)*cos(psi_load),
        cos(theta_load)*sin(psi_load), cos(phi_load)*cos(psi_load)+sin(phi_load)*sin(theta_load)*sin(psi_load), cos(phi_load)*sin(theta_load)*sin(psi_load)-sin(phi_load)*cos(psi_load),
        -sin(theta_load), sin(phi_load)*cos(theta_load), cos(phi_load)*cos(theta_load);

    VectorXd load_ang_vel_inert = VectorXd(3);
    VectorXd load_ang_vel_body = VectorXd(3);

    load_ang_vel_inert << phi_dot_load, theta_dot_load, psi_dot_load;

    load_ang_vel_body = R_load.transpose()*load_ang_vel_inert;


    double payload_anc_vel_x_body = load_ang_vel_body(1)*r_l[2] - load_ang_vel_body(2)*r_l[1];
    double payload_anc_vel_y_body = load_ang_vel_body(2)*r_l[0] - load_ang_vel_body(0)*r_l[2];
    double payload_anc_vel_z_body = load_ang_vel_body(0)*r_l[1] - load_ang_vel_body(1)*r_l[0];

    double me_anc_vel_x_body = _odom_data.angular_velocity[1]*_ancPointsData.r_uav[2] - _odom_data.angular_velocity[2]*_ancPointsData.r_uav[1];
    double me_anc_vel_y_body = _odom_data.angular_velocity[2]*_ancPointsData.r_uav[0] - _odom_data.angular_velocity[0]*_ancPointsData.r_uav[2];
    double me_anc_vel_z_body = _odom_data.angular_velocity[0]*_ancPointsData.r_uav[1] - _odom_data.angular_velocity[1]*_ancPointsData.r_uav[0];

    VectorXd me_anc_vel_body = VectorXd(3);
    VectorXd load_anc_vel_body = VectorXd(3);

    me_anc_vel_body << me_anc_vel_x_body, me_anc_vel_y_body, me_anc_vel_z_body;
    load_anc_vel_body << payload_anc_vel_x_body, payload_anc_vel_y_body, payload_anc_vel_z_body;

    VectorXd me_anc_vel_inert = VectorXd(3);
    VectorXd load_anc_vel_inert = VectorXd(3);

    me_anc_vel_inert = R_uav*me_anc_vel_body;
    load_anc_vel_inert = R_load*load_anc_vel_body;

    payload_anc_vel_x = _payload_pose_data.lin_velocity[0] + load_anc_vel_inert(0);
    payload_anc_vel_y = _payload_pose_data.lin_velocity[1] + load_anc_vel_inert(1);
    payload_anc_vel_z = _payload_pose_data.lin_velocity[2] + load_anc_vel_inert(2);

    me_anc_vel_x = _agent_pose_data.lin_velocity[0] + me_anc_vel_inert(0);
    me_anc_vel_y = _agent_pose_data.lin_velocity[1] + me_anc_vel_inert(1);
    me_anc_vel_z = _agent_pose_data.lin_velocity[2] + me_anc_vel_inert(2);

    VectorXd anc_points_vel_diff = VectorXd(3);
    anc_points_vel_diff << payload_anc_vel_x - me_anc_vel_x, payload_anc_vel_y - me_anc_vel_y, payload_anc_vel_z - me_anc_vel_z;

    double anc_points_vel_diff_norm = norm(anc_points_vel_diff);

    Vector3d estimated_inertial_tension = Vector3d();
    double uav_payload_dist = _ancPointsData.dist_norm_exact - l_t;

    if(uav_payload_dist > 0.0)
    {
      estimated_inertial_tension(0) = k*(uav_payload_dist)*_ancPointsData.uav_payload_exact_dist[0]/_ancPointsData.dist_norm_exact - b*anc_points_vel_diff(0);
      estimated_inertial_tension(1) = k*(uav_payload_dist)*_ancPointsData.uav_payload_exact_dist[1]/_ancPointsData.dist_norm_exact - b*anc_points_vel_diff(1);
      estimated_inertial_tension(2) = k*(uav_payload_dist)*_ancPointsData.uav_payload_exact_dist[2]/_ancPointsData.dist_norm_exact - b*anc_points_vel_diff(2);
    }

    // std::cout << _agent_name << " - tension_x: "<< tension_x << std::endl;
    // std::cout << _agent_name << " - tension_y: "<< tension_y << std::endl;
    // std::cout << _agent_name << " - tension_z: "<< tension_z << std::endl;
    // std::cout << _agent_name << " - tension_body_x: "<< tension_body[0] << std::endl;
    // std::cout << _agent_name << " - tension_body_y: "<< tension_body[1] << std::endl;
    // std::cout << _agent_name << " - tension_body_z: "<< tension_body[2] << std::endl;
    // std::cout << _agent_name << " - tension_body_norm: "<< norm(tension_body) << std::endl;
    // std::cout << _agent_name << " - tension_inert_norm: "<< norm(tension_inert) << std::endl;
    // std::cout << _agent_name << " - tension_ctrl: "<< - round(tension_z*10.0/_parameters.params[0])/10.0 << std::endl;

    return estimated_inertial_tension;
}


// %%%--------------------------%%%
// --------- DATA WRITE FOR TENSIONS ----------

void StatesHandler::storeTensions(double T_x, double T_y, double T_z)
{
  if(_counter_Jacobian < _m)
  {
    // myfile_T << std::to_string(_tensions.body_tension(0)) + "," + std::to_string(_tensions.body_tension(1)) + "," + std::to_string(_tensions.body_tension(2)) + "," + std::to_string(_joint_data.force_joint_1[0]) + "," + std::to_string(_joint_data.force_joint_1[1]) + "," + std::to_string(_joint_data.force_joint_1[2]) + "," + std::to_string(this->now().seconds()) << std::endl;
  }
}

// %%%--------------------------%%%
// --------- DATA WRITE FOR IDENTIFICATION ----------

void StatesHandler::storeDataTensionIdentificationHandler()
{
  // depending on who is alive, the stored data changes
  if(_detachUAVData.me[0] && _detachUAVData.next_agent_alive)
    storeDataTensionIdentification(_tensions.tension_norm, _ancPointsData.load_anchor_point_1, _ancPointsData.load_anchor_point_2, _ancPointsData.r_l1, _ancPointsData.r_l2);
  else if(_detachUAVData.me[1] && _detachUAVData.prev_agent_alive)
    storeDataTensionIdentification(_tensions.tension_norm, _ancPointsData.load_anchor_point_2, _ancPointsData.load_anchor_point_1, _ancPointsData.r_l2, _ancPointsData.r_l1);
  else if(_detachUAVData.me[0] && _detachUAVData.prev_agent_alive)
    storeDataTensionIdentification(_tensions.tension_norm, _ancPointsData.load_anchor_point_1, _ancPointsData.load_anchor_point_3, _ancPointsData.r_l1, _ancPointsData.r_l3);
  else if(_detachUAVData.me[2] && _detachUAVData.next_agent_alive)
    storeDataTensionIdentification(_tensions.tension_norm, _ancPointsData.load_anchor_point_3, _ancPointsData.load_anchor_point_1, _ancPointsData.r_l3, _ancPointsData.r_l1);
  else if(_detachUAVData.me[1] && _detachUAVData.next_agent_alive)
    storeDataTensionIdentification(_tensions.tension_norm, _ancPointsData.load_anchor_point_2, _ancPointsData.load_anchor_point_3, _ancPointsData.r_l2, _ancPointsData.r_l3);
  else if(_detachUAVData.me[2] && _detachUAVData.prev_agent_alive)
    storeDataTensionIdentification(_tensions.tension_norm, _ancPointsData.load_anchor_point_3, _ancPointsData.load_anchor_point_2, _ancPointsData.r_l3, _ancPointsData.r_l2);
}


void StatesHandler::storeDataTensionIdentification(double residual, VectorXd load_anchor_point_1, VectorXd load_anchor_point_2, VectorXd r_1, VectorXd r_2)
{

  if(_counter_Jacobian < _m)
  {
      std::cout << _agent_name << " _counter_Jacobian: " << _counter_Jacobian << std::endl;
      _counter_Jacobian++;

      double payload_anc_vel_1_x(0.0), payload_anc_vel_1_y(0.0), payload_anc_vel_1_z(0.0);
      double payload_anc_vel_2_x(0.0), payload_anc_vel_2_y(0.0), payload_anc_vel_2_z(0.0);
      double me_anc_vel_x(0.0), me_anc_vel_y(0.0), me_anc_vel_z(0.0);

      double phi_uav = _euler_angles[0];
      double theta_uav = _euler_angles[1];
      double psi_uav = _euler_angles[2];

      double phi_load = _payload_pose_data.orientation[0];
      double theta_load = _payload_pose_data.orientation[1];
      double psi_load = _payload_pose_data.orientation[2];

      double phi_dot_load = _payload_pose_data.ang_velocity[0];
      double theta_dot_load = _payload_pose_data.ang_velocity[1];
      double psi_dot_load = _payload_pose_data.ang_velocity[2];

      MatrixXd R_uav = MatrixXd(3,3);

      MatrixXd R_load = MatrixXd(3,3);

      R_uav << cos(psi_uav)*cos(theta_uav), sin(phi_uav)*sin(theta_uav)*cos(psi_uav)-cos(phi_uav)*sin(psi_uav), sin(phi_uav)*sin(psi_uav)+cos(phi_uav)*sin(theta_uav)*cos(psi_uav),
          cos(theta_uav)*sin(psi_uav), cos(phi_uav)*cos(psi_uav)+sin(phi_uav)*sin(theta_uav)*sin(psi_uav), cos(phi_uav)*sin(theta_uav)*sin(psi_uav)-sin(phi_uav)*cos(psi_uav),
          -sin(theta_uav), sin(phi_uav)*cos(theta_uav), cos(phi_uav)*cos(theta_uav);

      R_load << cos(psi_load)*cos(theta_load), sin(phi_load)*sin(theta_load)*cos(psi_load)-cos(phi_load)*sin(psi_load), sin(phi_load)*sin(psi_load)+cos(phi_load)*sin(theta_load)*cos(psi_load),
          cos(theta_load)*sin(psi_load), cos(phi_load)*cos(psi_load)+sin(phi_load)*sin(theta_load)*sin(psi_load), cos(phi_load)*sin(theta_load)*sin(psi_load)-sin(phi_load)*cos(psi_load),
          -sin(theta_load), sin(phi_load)*cos(theta_load), cos(phi_load)*cos(theta_load);

      VectorXd load_ang_vel_inert = VectorXd(3);
      VectorXd load_ang_vel_body = VectorXd(3);

      load_ang_vel_inert << phi_dot_load, theta_dot_load, psi_dot_load;

      load_ang_vel_body = R_load.transpose()*load_ang_vel_inert;


      double payload_anc_vel_1_x_body = load_ang_vel_body(1)*r_1[2] - load_ang_vel_body(2)*r_1[1];
      double payload_anc_vel_1_y_body = load_ang_vel_body(2)*r_1[0] - load_ang_vel_body(0)*r_1[2];
      double payload_anc_vel_1_z_body = load_ang_vel_body(0)*r_1[1] - load_ang_vel_body(1)*r_1[0];

      double payload_anc_vel_2_x_body = load_ang_vel_body(1)*r_2[2] - load_ang_vel_body(2)*r_2[1];
      double payload_anc_vel_2_y_body = load_ang_vel_body(2)*r_2[0] - load_ang_vel_body(0)*r_2[2];
      double payload_anc_vel_2_z_body = load_ang_vel_body(0)*r_2[1] - load_ang_vel_body(1)*r_2[0];

      double me_anc_vel_x_body = _odom_data.angular_velocity[1]*_ancPointsData.r_uav[2] - _odom_data.angular_velocity[2]*_ancPointsData.r_uav[1];
      double me_anc_vel_y_body = _odom_data.angular_velocity[2]*_ancPointsData.r_uav[0] - _odom_data.angular_velocity[0]*_ancPointsData.r_uav[2];
      double me_anc_vel_z_body = _odom_data.angular_velocity[0]*_ancPointsData.r_uav[1] - _odom_data.angular_velocity[1]*_ancPointsData.r_uav[0];

      VectorXd me_anc_vel_body = VectorXd(3);
      VectorXd load_anc_vel_1_body = VectorXd(3);
      VectorXd load_anc_vel_2_body = VectorXd(3);

      me_anc_vel_body << me_anc_vel_x_body, me_anc_vel_y_body, me_anc_vel_z_body;
      load_anc_vel_1_body << payload_anc_vel_1_x_body, payload_anc_vel_1_y_body, payload_anc_vel_1_z_body;
      load_anc_vel_2_body << payload_anc_vel_2_x_body, payload_anc_vel_2_y_body, payload_anc_vel_2_z_body;

      VectorXd me_anc_vel_inert = VectorXd(3);
      VectorXd load_anc_vel_1_inert = VectorXd(3);
      VectorXd load_anc_vel_2_inert = VectorXd(3);

      me_anc_vel_inert = R_uav*me_anc_vel_body;
      load_anc_vel_1_inert = R_load*load_anc_vel_1_body;
      load_anc_vel_2_inert = R_load*load_anc_vel_2_body;

      payload_anc_vel_1_x = _payload_pose_data.lin_velocity[0] + load_anc_vel_1_inert(0);
      payload_anc_vel_1_y = _payload_pose_data.lin_velocity[1] + load_anc_vel_1_inert(1);
      payload_anc_vel_1_z = _payload_pose_data.lin_velocity[2] + load_anc_vel_1_inert(2);

      payload_anc_vel_2_x = _payload_pose_data.lin_velocity[0] + load_anc_vel_2_inert(0);
      payload_anc_vel_2_y = _payload_pose_data.lin_velocity[1] + load_anc_vel_2_inert(1);
      payload_anc_vel_2_z = _payload_pose_data.lin_velocity[2] + load_anc_vel_2_inert(2);

      me_anc_vel_x = _agent_pose_data.lin_velocity[0] + me_anc_vel_inert(0);
      me_anc_vel_y = _agent_pose_data.lin_velocity[1] + me_anc_vel_inert(1);
      me_anc_vel_z = _agent_pose_data.lin_velocity[2] + me_anc_vel_inert(2);

      // rememeber that, given the formation of W_eta, the rates correspondence is reversed
      double phi_dot = _euler_rates[0];
      double theta_dot = _euler_rates[1];
      double psi_dot = _euler_rates[2];
      //
      // double payload_anc_1_vel_x(0.0), payload_anc_1_vel_y(0.0), payload_anc_1_vel_z(0.0), payload_anc_2_vel_x(0.0), payload_anc_2_vel_y(0.0), payload_anc_2_vel_z(0.0);
      // double me_anc_vel_x(0.0), me_anc_vel_y(0.0), me_anc_vel_z(0.0);
      //
      // payload_anc_1_vel_x = _payload_pose_data.lin_velocity[0] + r_1[1]*(_payload_pose_data.orientation[1]*_payload_pose_data.ang_velocity[1] - _payload_pose_data.ang_velocity[2]) + r_1[2]*(_payload_pose_data.orientation[0]*_payload_pose_data.ang_velocity[2] + _payload_pose_data.ang_velocity[1]);
      // payload_anc_1_vel_y = _payload_pose_data.lin_velocity[1] + r_1[0]*(_payload_pose_data.ang_velocity[2] - _payload_pose_data.orientation[1]*_payload_pose_data.ang_velocity[1]) + r_1[2]*(_payload_pose_data.orientation[1]*_payload_pose_data.ang_velocity[2] - _payload_pose_data.ang_velocity[0]);
      // payload_anc_1_vel_z = _payload_pose_data.lin_velocity[2] + r_1[1]*(_payload_pose_data.ang_velocity[0] - _payload_pose_data.orientation[1]*_payload_pose_data.ang_velocity[2]) - r_1[0]*(_payload_pose_data.orientation[0]*_payload_pose_data.ang_velocity[2] + _payload_pose_data.ang_velocity[1]);
      // payload_anc_2_vel_x = _payload_pose_data.lin_velocity[0] + r_2[1]*(_payload_pose_data.orientation[1]*_payload_pose_data.ang_velocity[1] - _payload_pose_data.ang_velocity[2]) + r_2[2]*(_payload_pose_data.orientation[0]*_payload_pose_data.ang_velocity[2] + _payload_pose_data.ang_velocity[1]);
      // payload_anc_2_vel_y = _payload_pose_data.lin_velocity[1] + r_2[0]*(_payload_pose_data.ang_velocity[2] - _payload_pose_data.orientation[1]*_payload_pose_data.ang_velocity[1]) + r_2[2]*(_payload_pose_data.orientation[1]*_payload_pose_data.ang_velocity[2] - _payload_pose_data.ang_velocity[0]);
      // payload_anc_2_vel_z = _payload_pose_data.lin_velocity[2] + r_2[1]*(_payload_pose_data.ang_velocity[0] - _payload_pose_data.orientation[1]*_payload_pose_data.ang_velocity[2]) - r_2[0]*(_payload_pose_data.orientation[0]*_payload_pose_data.ang_velocity[2] + _payload_pose_data.ang_velocity[1]);
      //
      // me_anc_vel_x = _agent_pose_data.lin_velocity[0] + _ancPointsData.r_uav[1]*(_euler_angles[1]*theta_dot - psi_dot) + _ancPointsData.r_uav[2]*(_euler_angles[0]*psi_dot + theta_dot);
      // me_anc_vel_y = _agent_pose_data.lin_velocity[1] + _ancPointsData.r_uav[0]*(psi_dot - _euler_angles[1]*theta_dot) + _ancPointsData.r_uav[2]*(_euler_angles[1]*psi_dot - phi_dot);
      // me_anc_vel_z = _agent_pose_data.lin_velocity[2] + _ancPointsData.r_uav[1]*(phi_dot - _euler_angles[1]*psi_dot) - _ancPointsData.r_uav[0]*(_euler_angles[0]*psi_dot + theta_dot);

      // myfile << std::to_string(0) + "," + std::to_string(0) + "," + std::to_string(residual) + "," + std::to_string(_ancPointsData.dist_norm_exact) + "," + std::to_string(this->now().seconds()) << std::endl;
      // myfile2 << std::to_string(_payload_pose_data.position[0]+load_anchor_point_1[0]) + "," + std::to_string(_payload_pose_data.position[1]+load_anchor_point_1[1]) + "," + std::to_string(_payload_pose_data.position[2]+load_anchor_point_1[2]) + "," + std::to_string(_payload_pose_data.position[0]+load_anchor_point_2[0]) + "," + std::to_string(_payload_pose_data.position[1]+load_anchor_point_2[1]) + "," + std::to_string(_payload_pose_data.position[2]+load_anchor_point_2[2]) + "," + std::to_string(payload_anc_vel_1_x) + "," + std::to_string(payload_anc_vel_1_y) + "," + std::to_string(payload_anc_vel_1_z) + "," + std::to_string(payload_anc_vel_2_x) + "," +  std::to_string(payload_anc_vel_2_y) + "," +  std::to_string(payload_anc_vel_2_z) + "," +  std::to_string(this->now().seconds()) << std::endl;
      // myfile3 << std::to_string(_agent_pose_data.position[0]+_ancPointsData.uav_anchor_point[0]) + "," + std::to_string(_agent_pose_data.position[1]+_ancPointsData.uav_anchor_point[1]) + "," + std::to_string(_agent_pose_data.position[2]+_ancPointsData.uav_anchor_point[2]) + "," + std::to_string(me_anc_vel_x) + "," + std::to_string(me_anc_vel_y) + "," + std::to_string(me_anc_vel_z) + "," + std::to_string(this->now().seconds()) << std::endl;
  }
  else
  {
    // myfile.flush();
    // myfile2.flush();
    // myfile3.flush();
    // myfile.close();
    // myfile2.close();
    // myfile3.close();
  }
}


void StatesHandler::printStatesOnFIle(VectorXd des_state)
{
  // if(_a < 5000)
  // {
    // if(_agent_name == "x500_2")
    //   std::cout << _agent_name << " a: " << _a << std::endl;

    myfile5 << std::to_string(_agent_pose_data.position[0]) + "," + std::to_string(_agent_pose_data.lin_velocity[0]) + "," + std::to_string(_agent_pose_data.position[1]) + "," +  std::to_string(_agent_pose_data.lin_velocity[1]) + "," + std::to_string(_agent_pose_data.position[2]) + "," + std::to_string(_agent_pose_data.lin_velocity[2]) + "," +
              std::to_string(_euler_angles[0])  + "," + std::to_string(_euler_rates[0])  + "," + std::to_string(_euler_angles[1])  + "," + std::to_string(_euler_rates[1])  + "," + std::to_string(_euler_angles[2]) + "," + std::to_string(_euler_rates[2]) + "," + std::to_string(this->now().seconds()) << std::endl; //  "," + std::to_string(_tensions.tension_norm) +

    myfile6 << std::to_string(_payload_pose_data.position[0])  + "," + std::to_string(_payload_pose_data.lin_velocity[0]) + "," + std::to_string(_payload_pose_data.position[1]) + "," + std::to_string(_payload_pose_data.lin_velocity[1]) + "," + std::to_string(_payload_pose_data.position[2]) + "," + std::to_string(_payload_pose_data.lin_velocity[2]) + "," +
              std::to_string(_payload_pose_data.orientation[0]) + "," + std::to_string(_payload_pose_data.ang_velocity[0]) + "," + std::to_string(_payload_pose_data.orientation[1]) + "," + std::to_string(_payload_pose_data.ang_velocity[1]) + "," + std::to_string(_payload_pose_data.orientation[2]) + "," + std::to_string(_payload_pose_data.ang_velocity[2]) + "," + std::to_string(this->now().seconds()) << std::endl;

    myfile7 << std::to_string(des_state(0)) + "," + std::to_string(des_state(1)) + "," + std::to_string(des_state(2)) + "," + std::to_string(des_state(3)) + "," + std::to_string(des_state(4)) + "," + std::to_string(des_state(5)) + "," +
              std::to_string(des_state(6)) + "," + std::to_string(des_state(7)) + "," + std::to_string(des_state(8)) + "," + std::to_string(des_state(9)) + "," + std::to_string(des_state(10)) + "," + std::to_string(des_state(11)) << std::endl;
  // }
  // _a++;
}

// %%%--------------------------%%%
// --------- SURVIVED UAV CHECK/DISCOVERING ----------

void StatesHandler::survivedUAVcheck()
{
  auto agents_alive = _failure_action_data.agents_alive;

  if(_agent_name == "x500_1")
    _detachUAVData.me[0] = 1;
  else if(_agent_name == "x500_2")
    _detachUAVData.me[1] = 1;
  else if(_agent_name == "x500_3")
    _detachUAVData.me[2] = 1;

  for(int i = 0; i < 3; i++)
    _detachUAVData.surv_agent[i] = agents_alive[i] - _detachUAVData.me[i];

  if((_detachUAVData.me[0] && _detachUAVData.surv_agent[1]) || (_detachUAVData.me[1] && _detachUAVData.surv_agent[2]) || (_detachUAVData.me[2] && _detachUAVData.surv_agent[0]))
    _detachUAVData.next_agent_alive = true;
  else if((_detachUAVData.me[0] && _detachUAVData.surv_agent[2]) || (_detachUAVData.me[1] && _detachUAVData.surv_agent[0]) || (_detachUAVData.me[2] && _detachUAVData.surv_agent[1]))
    _detachUAVData.prev_agent_alive = true;

  survivedAgentPoseAssignment();

}

// %%%--------------------------%%%
// --------- NORM ----------

double StatesHandler::norm(VectorXd vector)
{
  return sqrt(pow(vector(0),2) + pow(vector(1),2) + pow(vector(2),2));
}

// %%%--------------------------%%%
// --------- EULER ANGLES FROM QUATERNIONS ----------

void StatesHandler::quaternionToEuler(std::array<float, 4> q, int pos)
{
  double euler_angles[3];
  // roll (x-axis rotation)
  double sinr_cosp = 2 * (q[0] * q[1] + q[2] * q[3]);
  double cosr_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2]);
  euler_angles[0] = atan2(sinr_cosp, cosr_cosp);

  // pitch (y-axis rotation)
  double sinp = 2 * (q[0] * q[2] - q[3] * q[1]);
  if (std::abs(sinp) >= 1)
      euler_angles[1] = std::copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  else
      euler_angles[1] = std::asin(sinp);

  // yaw (z-axis rotation)
  double siny_cosp = 2 * (q[0] * q[3] + q[1] * q[2]);
  double cosy_cosp = 1 - 2 * (q[2] * q[2] + q[3] * q[3]);
  euler_angles[2] = atan2(siny_cosp, cosy_cosp);

  if(euler_angles[2] < 0.0)
    euler_angles[2] += 2*PI;

  if(pos == 0)
  {
    _euler_angles[0] = euler_angles[0];
    _euler_angles[1] = euler_angles[1];
    _euler_angles[2] = euler_angles[2];
  }
  else if(pos == 1)
  {
    _na_euler_angles[0] = euler_angles[0];
    _na_euler_angles[1] = euler_angles[1];
    _na_euler_angles[2] = euler_angles[2];
  }
  else if(pos == -1)
  {
    _pa_euler_angles[0] = euler_angles[0];
    _pa_euler_angles[1] = euler_angles[1];
    _pa_euler_angles[2] = euler_angles[2];
  }
}
