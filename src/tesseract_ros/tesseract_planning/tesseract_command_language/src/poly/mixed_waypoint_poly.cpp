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

#include <tesseract_command_language/poly/mixed_waypoint_poly.h>

template <class Archive>
void tesseract_planning::detail_mixed_waypoint::MixedWaypointInterface::serialize(
    Archive& ar,
    const unsigned int /*version*/)  // NOLINT
{
}

std::vector<std::string> tesseract_planning::MixedWaypointPoly::getJointNames()
{
  return getInterface().getJointNames();
}

const std::vector<std::string> tesseract_planning::MixedWaypointPoly::getJointNames() const
{
  return getInterface().getJointNames();
}

void tesseract_planning::MixedWaypointPoly::addJointTarget(std::string joint_name, double joint_value)
{
  getInterface().addJointTarget(joint_name, joint_value);
}

std::map<std::string, double> tesseract_planning::MixedWaypointPoly::getJointTargets()
{
  return getInterface().getJointTargets();
}

const std::map<std::string, double> tesseract_planning::MixedWaypointPoly::getJointTargets() const
{
  return getInterface().getJointTargets();
}

std::map<int, double> tesseract_planning::MixedWaypointPoly::getJointIndexTargets()
{
  return getInterface().getJointIndexTargets();
}
const std::map<int, double> tesseract_planning::MixedWaypointPoly::getJointIndexTargets() const
{
  return getInterface().getJointIndexTargets();
}

void tesseract_planning::MixedWaypointPoly::addLinkTarget(std::string link_name, Eigen::Isometry3d& link_tf)
{
  getInterface().addLinkTarget(link_name, link_tf);
}

std::map<std::string, Eigen::Isometry3d> tesseract_planning::MixedWaypointPoly::getLinkTargets()
{
  return getInterface().getLinkTargets();
}

const std::map<std::string, Eigen::Isometry3d> tesseract_planning::MixedWaypointPoly::getLinkTargets() const
{
  return getInterface().getLinkTargets();
}

void tesseract_planning::MixedWaypointPoly::addLinkConstraint(std::string link_name, Eigen::Isometry3d& link_tf)
{
  getInterface().addLinkConstraint(link_name, link_tf);
}

void tesseract_planning::MixedWaypointPoly::addLinkConstraint(std::string link_name,
                                                              Eigen::Isometry3d& link_tf,
                                                              const Eigen::VectorXd& coeff)
{
  getInterface().addLinkConstraint(link_name, link_tf, coeff);
}

std::map<std::string, tesseract_planning::detail_mixed_waypoint::CartesianConstraint>
tesseract_planning::MixedWaypointPoly::getLinkConstraints()
{
  return getInterface().getLinkConstraints();
}

const std::map<std::string, tesseract_planning::detail_mixed_waypoint::CartesianConstraint>
tesseract_planning::MixedWaypointPoly::getLinkConstraints() const
{
  return getInterface().getLinkConstraints();
}

void tesseract_planning::MixedWaypointPoly::print(const std::string& prefix) const { getInterface().print(prefix); }

template <class Archive>
void tesseract_planning::MixedWaypointPoly::serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
{
  ar& boost::serialization::make_nvp("base", boost::serialization::base_object<MixedWaypointPolyBase>(*this));
}

#include <tesseract_common/serialization.h>
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::detail_mixed_waypoint::MixedWaypointInterface)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::MixedWaypointPolyBase)
TESSERACT_SERIALIZE_ARCHIVES_INSTANTIATE(tesseract_planning::MixedWaypointPoly)

BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::detail_mixed_waypoint::MixedWaypointInterface)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::MixedWaypointPolyBase)
BOOST_CLASS_EXPORT_IMPLEMENT(tesseract_planning::MixedWaypointPoly)

TESSERACT_WAYPOINT_EXPORT_IMPLEMENT(tesseract_planning::MixedWaypointPoly);
