/*
 * ModeSequence_Keyboard_Quadruped.h
 *
 *  Created on: Oct 11, 2018
 *      Author: farbod
 */

#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <ocs2_switched_model_interface/core/SwitchedModel.h>
#include <ocs2_switched_model_interface/logic/ModeSequenceTemplate.h>

namespace switched_model {

/** This class implements ModeSequence communication using ROS. */
class ModeSequenceRosTopic {
 public:
  ModeSequenceRosTopic(ros::NodeHandle nodeHandle, const std::string& gaitFile, const std::string& robotName, bool verbose = false);

 private:
  /** Publishes the mode sequence template. */
  void publishModeSequenceTemplate(const ModeSequenceTemplate& modeSchedule);

  /** Prints the list of available gaits. */
  void printGaitList(const std::vector<std::string>& gaitList) const;

  /** Callback function for the gait subscriber. */
  void gaitCallback(const std_msgs::String::ConstPtr& msg);

  std::vector<std::string> gaitList_;
  std::map<std::string, ModeSequenceTemplate> gaitMap_;

  ros::Publisher modeSequenceTemplatePublisher_;
  ros::Subscriber gaitSubscriber_;
};

}  // end of namespace switched_model
