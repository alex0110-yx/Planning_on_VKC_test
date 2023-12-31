/**
 * @file trajopt_utils.h
 * @brief Contains utilities for converting to trajopt_ifopt types to trajopt_sco or Tesseract types
 *
 * @author Matthew Powelson
 * @date May 18, 2020
 * @version TODO
 * @bug No known bugs
 *
 * @copyright Copyright (c) 2020, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#ifndef TRAJOPT_IFOPT_TRAJOPT_UTILS_H
#define TRAJOPT_IFOPT_TRAJOPT_UTILS_H

#include <trajopt_utils/macros.h>
TRAJOPT_IGNORE_WARNINGS_PUSH
#include <tesseract_common/types.h>
#include <tesseract_common/joint_state.h>
#include <Eigen/Core>
TRAJOPT_IGNORE_WARNINGS_POP
#include <trajopt_ifopt/variable_sets/joint_position_variable.h>

namespace trajopt_ifopt
{
/**
 * @brief Converts a vector of trajopt variables into the legacy TrajArray format
 * @param joint_positions Vector of joint positions. Must be in order and all the same length
 * @return TrajArray - size = [joint_positions.size(), joint_positions.n_dof]
 */
tesseract_common::TrajArray toTrajArray(const std::vector<trajopt_ifopt::JointPosition::ConstPtr>& joint_positions);

/**
 * @brief Converts a vector of trajopt variables into tesseract_common JointTrajectory
 * @param joint_positions Vector of joint positions. Must be in order and all the same length
 * @return JointTrajectory
 */
tesseract_common::JointTrajectory
toJointTrajectory(const std::vector<trajopt_ifopt::JointPosition::ConstPtr>& joint_positions);

}  // namespace trajopt_ifopt
#endif
