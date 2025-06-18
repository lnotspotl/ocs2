//
// Created by rgrandia on 18.03.20.
//

#include "ocs2_anymal_commands/ModeSequenceRosTopic.h"

#include <ocs2_core/misc/CommandLine.h>
#include <ocs2_core/misc/LoadData.h>

#include <ocs2_msgs/mode_schedule.h>

namespace switched_model {

ModeSequenceRosTopic::ModeSequenceRosTopic(ros::NodeHandle nodeHandle, const std::string& gaitFile, const std::string& robotName,
                                           bool verbose) {
  ROS_INFO_STREAM(robotName + "_mpc_mode_schedule node is setting up ...");
  ocs2::loadData::loadStdVector(gaitFile, "list", gaitList_, verbose);

  modeSequenceTemplatePublisher_ = nodeHandle.advertise<ocs2_msgs::mode_schedule>(robotName + "_mpc_mode_schedule", 1, true);
  gaitSubscriber_ = nodeHandle.subscribe(robotName + "_gait", 1, &ModeSequenceRosTopic::gaitCallback, this);

  gaitMap_.clear();
  for (const auto& gaitName : gaitList_) {
    gaitMap_.insert({gaitName, loadModeSequenceTemplate(gaitFile, gaitName, verbose)});
  }
  ROS_INFO_STREAM(robotName + "_mpc_mode_schedule command node is ready.");
}

void ModeSequenceRosTopic::gaitCallback(const std_msgs::String::ConstPtr& msg) {
  const std::string gaitCommand = msg->data;

  try {
    ModeSequenceTemplate modeSequenceTemplate = gaitMap_.at(gaitCommand);
    publishModeSequenceTemplate(modeSequenceTemplate);
  } catch (const std::out_of_range& e) {
    std::cout << "Gait \"" << gaitCommand << "\" not found.\n";
    printGaitList(gaitList_);
  }
}

void ModeSequenceRosTopic::printGaitList(const std::vector<std::string>& gaitList) const {
  std::cout << "List of available gaits:\n";
  size_t itr = 0;
  for (const auto& s : gaitList) {
    std::cout << "[" << itr++ << "]: " << s << "\n";
  }
  std::cout << std::endl;
}

void ModeSequenceRosTopic::publishModeSequenceTemplate(const ModeSequenceTemplate& modeSchedule) {
  modeSequenceTemplatePublisher_.publish(createModeSequenceTemplateMsg(modeSchedule));
}

}  // namespace switched_model
