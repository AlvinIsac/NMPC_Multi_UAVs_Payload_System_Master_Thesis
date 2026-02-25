/*! \file formation_optimizer.cpp
	\brief implementation of the functions that handle the rotation to the optimal formation position and that handle the formation correction process

	Details.
*/

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// @ AUTHOR: Luca Bricarello ---------------------------------------------------
// @ REVISITED BY Andrea Delbene

#include "states_handler.hpp"

// ----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

double StatesHandler::YawAdjuster(VectorXd formationCenter)
{

  double desiredYaw;

  double distance_from_center_vector[2];
  distance_from_center_vector[0] = formationCenter[0] - _agent_pose_data.position[0];
  distance_from_center_vector[1] = formationCenter[1] - _agent_pose_data.position[1];

  desiredYaw = atan2(distance_from_center_vector[1], distance_from_center_vector[0]);

  // Yaw is desired between 0 and 2PI degrees
  if (desiredYaw < 0.0)
  {
    desiredYaw += 2*PI;
  }

  return desiredYaw;

}
