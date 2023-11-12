/**
 * @file null_waypoint.cpp
 * @brief
 *
 * @author Levi Armstrong
 * @date June 15, 2020
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

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <iostream>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/mixed_waypoint.h>

namespace tesseract_planning
{
void MixedWaypoint::print(const std::string& prefix) const
{
  {
    std::cout << prefix << "Mixed WP: joint_names: ";
    for (auto name : joint_names_)
    {
      std::cout << name << ", ";
    }
  }
}

std::vector<std::string> MixedWaypoint::getJointNames() { return joint_names_; }
const std::vector<std::string> MixedWaypoint::getJointNames() const { return joint_names_; }

void MixedWaypoint::addJointTarget(std::string joint_name, double joint_value)
{
  joint_targets_[joint_name] = joint_value;
}

std::map<std::string, double> MixedWaypoint::getJointTargets() { return joint_targets_; }
const std::map<std::string, double> MixedWaypoint::getJointTargets() const { return joint_targets_; }

std::map<int, double> MixedWaypoint::getJointIndexTargets()
{
  std::map<int, double> joint_index_targets;
  for (auto const& jt : joint_targets_)
  {
    auto itr = std::find(joint_names_.begin(), joint_names_.end(), jt.first);
    if (itr == joint_names_.end())
    {
      throw std::logic_error("cannot find target joint name");
    }
    int index = std::distance(joint_names_.begin(), itr);
    joint_index_targets[index] = jt.second;
  }
  return joint_index_targets;
}
const std::map<int, double> MixedWaypoint::getJointIndexTargets() const
{
  std::map<int, double> joint_index_targets;
  for (auto const& jt : joint_targets_)
  {
    auto itr = std::find(joint_names_.begin(), joint_names_.end(), jt.first);
    if (itr == joint_names_.end())
    {
      throw std::logic_error("cannot find target joint name");
    }
    int index = std::distance(joint_names_.begin(), itr);
    joint_index_targets[index] = jt.second;
  }
  return joint_index_targets;
}

void MixedWaypoint::addLinkTarget(std::string link_name, Eigen::Isometry3d& link_tf)
{
  link_targets_[link_name] = link_tf;
}

std::map<std::string, Eigen::Isometry3d> MixedWaypoint::getLinkTargets() { return link_targets_; }
const std::map<std::string, Eigen::Isometry3d> MixedWaypoint::getLinkTargets() const { return link_targets_; }

void MixedWaypoint::addLinkConstraint(std::string link_name, Eigen::Isometry3d& link_tf)
{
  detail_mixed_waypoint::CartesianConstraint constraint = { link_tf, Eigen::VectorXd() };
  link_constraints_[link_name] = constraint;
}

void MixedWaypoint::addLinkConstraint(std::string link_name,
                                      Eigen::Isometry3d& link_tf,
                                      const Eigen::VectorXd& cartesian_coeff)
{
  detail_mixed_waypoint::CartesianConstraint constraint = { link_tf, cartesian_coeff };
  link_constraints_[link_name] = constraint;
}

std::map<std::string, detail_mixed_waypoint::CartesianConstraint> MixedWaypoint::getLinkConstraints() { return link_constraints_; }
const std::map<std::string, detail_mixed_waypoint::CartesianConstraint> MixedWaypoint::getLinkConstraints() const { return link_constraints_; }

bool MixedWaypoint::operator==(const MixedWaypoint& /*rhs*/) const
{
  return true;  // TODO
}
bool MixedWaypoint::operator!=(const MixedWaypoint& /*rhs*/) const { return false; }

template <class Archive>
void MixedWaypoint::serialize(Archive& /*ar*/, const unsigned int /*version*/)
{
}
}  // namespace tesseract_planning

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::MixedWaypoint)
TESSERACT_MIXED_WAYPOINT_EXPORT_IMPLEMENT(tesseract_planning::MixedWaypoint);
