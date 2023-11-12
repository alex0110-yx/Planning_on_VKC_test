#ifndef TESSERACT_COMMAND_LANGUAGE_MIXED_WAYPOINT_H
#define TESSERACT_COMMAND_LANGUAGE_MIXED_WAYPOINT_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <map>
#include <boost/serialization/base_object.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/mixed_waypoint_poly.h>
#include <tesseract_common/utils.h>

namespace tesseract_planning
{

class MixedWaypoint
{
public:
  MixedWaypoint() = default;

  void print(const std::string& prefix = "") const;

  /*<< ", joint_targets_: " << this->joint_targets_.size()
<< ", link_targets: " << this->link_targets.size() << std::endl;
}*/

  MixedWaypoint(std::vector<std::string> joint_names) : joint_names_(std::move(joint_names)) {}

  bool operator==(const MixedWaypoint& rhs) const;
  bool operator!=(const MixedWaypoint& rhs) const;

  // all joint names for manipulator
  std::vector<std::string> joint_names_;
  // target joint name and value
  std::map<std::string, double> joint_targets_;
  // target link name and pose
  std::map<std::string, Eigen::Isometry3d> link_targets_;
  std::map<std::string, detail_mixed_waypoint::CartesianConstraint> link_constraints_;

  std::vector<std::string> getJointNames();
  const std::vector<std::string> getJointNames() const;
  void addJointTarget(std::string joint_name, double joint_value);
  std::map<std::string, double> getJointTargets();
  const std::map<std::string, double> getJointTargets() const;
  std::map<int, double> getJointIndexTargets();
  const std::map<int, double> getJointIndexTargets() const;
  void addLinkTarget(std::string link_name, Eigen::Isometry3d& link_tf);
  std::map<std::string, Eigen::Isometry3d> getLinkTargets();
  const std::map<std::string, Eigen::Isometry3d> getLinkTargets() const;
  void addLinkConstraint(std::string link_name, Eigen::Isometry3d& link_tf);
  void addLinkConstraint(std::string link_name, Eigen::Isometry3d& link_tf, const Eigen::VectorXd& cartesian_coeff);
  std::map<std::string, detail_mixed_waypoint::CartesianConstraint> getLinkConstraints();
  const std::map<std::string, detail_mixed_waypoint::CartesianConstraint> getLinkConstraints() const;

private:
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& /*ar*/, const unsigned int /*version*/);  // NOLINT
};
}  // namespace tesseract_planning

#ifdef SWIG
%tesseract_command_language_add_waypoint_type(MixedWaypoint)
#else
TESSERACT_MIXED_WAYPOINT_EXPORT_KEY(tesseract_planning, MixedWaypoint);
#endif  // SWIG

#endif  // TESSERACT_COMMAND_LANGUAGE_NULL_WAYPOINT_H
