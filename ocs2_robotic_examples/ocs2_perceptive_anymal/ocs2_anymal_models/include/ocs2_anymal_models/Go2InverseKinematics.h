#pragma once


#include <ocs2_switched_model_interface/core/InverseKinematicsModelBase.h>

#include <ocs2_switched_model_interface/analytical_inverse_kinematics/LegInverseKinematicParameters.h>

#include <ocs2_pinocchio_interface/PinocchioInterface.h>

#include "ocs2_anymal_models/QuadrupedPinocchioMapping.h"

namespace anymal {

class Go2InverseKinematics final : public switched_model::InverseKinematicsModelBase {
 public:
  Go2InverseKinematics(const FrameDeclaration& frameDeclaration, const ocs2::PinocchioInterface& pinocchioInterface);

  ~Go2InverseKinematics() override = default;

  Go2InverseKinematics* clone() const override;

  switched_model::vector3_t getLimbJointPositionsFromPositionBaseToFootInBaseFrame(
      size_t footIndex, const switched_model::vector3_t& positionBaseToFootInBaseFrame) const override;

  switched_model::vector3_t getLimbVelocitiesFromFootVelocityRelativeToBaseInBaseFrame(
      size_t footIndex, const switched_model::vector3_t& footVelocityRelativeToBaseInBaseFrame,
      const joint_jacobian_block_t& jointJacobian, switched_model::scalar_t damping) const override;

 private:
  Go2InverseKinematics(const Go2InverseKinematics& other) = default;

  switched_model::feet_array_t<switched_model::analytical_inverse_kinematics::LegInverseKinematicParameters> parameters_;
};

};  // namespace anymal