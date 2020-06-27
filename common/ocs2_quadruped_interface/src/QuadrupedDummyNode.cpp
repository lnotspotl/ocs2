//
// Created by rgrandia on 17.02.20.
//

#include "ocs2_quadruped_interface/QuadrupedDummyNode.h"

#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Dummy_Loop.h>
#include <ocs2_comm_interfaces/ocs2_ros_interfaces/mrt/MRT_ROS_Interface.h>

#include <ocs2_quadruped_interface/QuadrupedVisualizer.h>

namespace switched_model {

void quadrupedDummyNode(ros::NodeHandle& nodeHandle, const QuadrupedInterface& quadrupedInterface, const ocs2::RolloutBase* rolloutPtr,
                        double mrtDesiredFrequency, double mpcDesiredFrequency) {
  const std::string robotName = "anymal";

  // MRT
  ocs2::MRT_ROS_Interface mrt(robotName);
  if (rolloutPtr != nullptr) {
    mrt.initRollout(rolloutPtr);
  }
  mrt.launchNodes(nodeHandle);

  // Visualization
  auto visualizer = std::make_shared<switched_model::QuadrupedVisualizer>(quadrupedInterface.getKinematicModel(),
                                                                          quadrupedInterface.getComModel(), nodeHandle);

  // Dummy MRT
  ocs2::MRT_ROS_Dummy_Loop dummySimulator(mrt, mrtDesiredFrequency, mpcDesiredFrequency);
  dummySimulator.subscribeObservers({visualizer});

  // initial state
  ocs2::SystemObservation initObservation;
  initObservation.state() = quadrupedInterface.getInitialState();
  initObservation.input() = vector_t::Zero(INPUT_DIM);
  initObservation.subsystem() = switched_model::ModeNumber::STANCE;

  // initial command
  ocs2::CostDesiredTrajectories initCostDesiredTrajectories({0.0}, {initObservation.state()}, {initObservation.input()});

  // run dummy
  dummySimulator.run(initObservation, initCostDesiredTrajectories);
}

}  // namespace switched_model
