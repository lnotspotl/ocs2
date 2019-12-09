/******************************************************************************
Copyright (c) 2017, Farbod Farshidian. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#pragma once

#include "ocs2_core/misc/LinearInterpolation.h"
#include "ocs2_core/model_data/ModelDataBase.h"

// Declares an access function of name FIELD (e.g., time, flowMap)
#define CREATE_INTERPOLATION_ACCESS_FUNCTION(FIELD)                                                             \
  inline auto FIELD(const ocs2::ModelDataBase::array_t::size_type ind, const ocs2::ModelDataBase::array_t* vec) \
      ->const decltype((*vec)[ind].FIELD##_)& {                                                                 \
    return (*vec)[ind].FIELD##_;                                                                                \
  }

namespace ocs2 {
namespace ModelData {

using LinearInterpolation = ocs2::LinearInterpolation<ModelDataBase, Eigen::aligned_allocator<ModelDataBase>>;

// time
CREATE_INTERPOLATION_ACCESS_FUNCTION(time)

// dynamics flow
CREATE_INTERPOLATION_ACCESS_FUNCTION(flowMap)
CREATE_INTERPOLATION_ACCESS_FUNCTION(flowMapStateDerivative)
CREATE_INTERPOLATION_ACCESS_FUNCTION(flowMapInputDerivative)

// cost
CREATE_INTERPOLATION_ACCESS_FUNCTION(cost)
CREATE_INTERPOLATION_ACCESS_FUNCTION(costStateDerivative)
CREATE_INTERPOLATION_ACCESS_FUNCTION(costInputDerivative)
CREATE_INTERPOLATION_ACCESS_FUNCTION(costStateSecondDerivative)
CREATE_INTERPOLATION_ACCESS_FUNCTION(costInputSecondDerivative)
CREATE_INTERPOLATION_ACCESS_FUNCTION(costInputStateDerivative)

// state equality constraints
CREATE_INTERPOLATION_ACCESS_FUNCTION(stateEqConstr)
CREATE_INTERPOLATION_ACCESS_FUNCTION(stateEqConstrStateDerivative)

// state-input equality constraints
CREATE_INTERPOLATION_ACCESS_FUNCTION(stateInputEqConstr)
CREATE_INTERPOLATION_ACCESS_FUNCTION(stateInputEqConstrStateDerivative)
CREATE_INTERPOLATION_ACCESS_FUNCTION(stateInputEqConstrInputDerivative)

}  // namespace ModelData
}  // namespace ocs2

#undef CREATE_INTERPOLATION_ACCESS_FUNCTION
