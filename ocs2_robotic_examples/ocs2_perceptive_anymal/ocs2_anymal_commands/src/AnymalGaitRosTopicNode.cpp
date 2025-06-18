#include <ros/package.h>

#include "ocs2_anymal_commands/ModeSequenceRosTopic.h"

int main(int argc, char* argv[]) {
  const std::string robotName = "anymal";
  std::string gaitFile = ros::package::getPath("ocs2_anymal_commands") + "/config/gait.info";
  std::cerr << "Loading gait file: " << gaitFile << std::endl;

  ros::init(argc, argv, robotName + "_mpc_mode_sequence");
  ros::NodeHandle nodeHandle;
  switched_model::ModeSequenceRosTopic modeSequenceCommand(nodeHandle, gaitFile, robotName, true);

  ros::spin();

  // Successful exit
  return 0;
}
