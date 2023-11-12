#ifndef TESSERACT_COMMAND_LANGUAGE_MIXED_WAYPOINT_POLY_H
#define TESSERACT_COMMAND_LANGUAGE_MIXED_WAYPOINT_POLY_H

#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <string>
#include <map>
#include <boost/serialization/base_object.hpp>
#include <boost/serialization/export.hpp>
#include <boost/concept_check.hpp>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_common/serialization.h>
#include <tesseract_common/type_erasure.h>

/** @brief If shared library, this must go in the header after the class definition */
#define TESSERACT_MIXED_WAYPOINT_EXPORT_KEY(N, C)                                                                      \
  namespace N                                                                                                          \
  {                                                                                                                    \
  using C##InstanceBase =                                                                                              \
      tesseract_common::TypeErasureInstance<C, tesseract_planning::detail_mixed_waypoint::MixedWaypointInterface>;     \
  using C##Instance = tesseract_planning::detail_mixed_waypoint::MixedWaypointInstance<C>;                             \
  using C##InstanceWrapper = tesseract_common::TypeErasureInstanceWrapper<C##Instance>;                                \
  }                                                                                                                    \
  BOOST_CLASS_EXPORT_KEY(N::C##InstanceBase)                                                                           \
  BOOST_CLASS_EXPORT_KEY(N::C##Instance)                                                                               \
  BOOST_CLASS_EXPORT_KEY(N::C##InstanceWrapper)                                                                        \
  BOOST_CLASS_TRACKING(N::C##InstanceBase, boost::serialization::track_never)                                          \
  BOOST_CLASS_TRACKING(N::C##Instance, boost::serialization::track_never)                                              \
  BOOST_CLASS_TRACKING(N::C##InstanceWrapper, boost::serialization::track_never)

/** @brief If shared library, this must go in the cpp after the implicit instantiation of the serialize function */
#define TESSERACT_MIXED_WAYPOINT_EXPORT_IMPLEMENT(inst)                                                                \
  BOOST_CLASS_EXPORT_IMPLEMENT(inst##InstanceBase)                                                                     \
  BOOST_CLASS_EXPORT_IMPLEMENT(inst##Instance)                                                                         \
  BOOST_CLASS_EXPORT_IMPLEMENT(inst##InstanceWrapper)

/**
 * @brief This should not be used within shared libraries use the two above.
 * If not in a shared library it can go in header or cpp
 */
#define TESSERACT_MIXED_WAYPOINT_EXPORT(N, C)                                                                          \
  TESSERACT_MIXED_WAYPOINT_EXPORT_KEY(N, C)                                                                            \
  TESSERACT_MIXED_WAYPOINT_EXPORT_IMPLEMENT(N::C)

namespace tesseract_planning::detail_mixed_waypoint
{
struct CartesianConstraint
{
  Eigen::Isometry3d pose;
  Eigen::VectorXd coeff;
};

template <typename T>
struct MixedWaypointConcept  // NOLINT
  : boost::Assignable<T>,
    boost::CopyConstructible<T>,
    boost::EqualityComparable<T>
{
  BOOST_CONCEPT_USAGE(MixedWaypointConcept)
  {
    T cp(c);
    T assign = c;
    bool eq = (c == cp);
    bool neq = (c != cp);
    UNUSED(assign);
    UNUSED(eq);
    UNUSED(neq);

    std::vector<std::string> names_ref = c.getJointNames();
    UNUSED(names_ref);

    std::string joint_name;
    double joint_value;
    c.addJointTarget(joint_name, joint_value);

    std::map<std::string, double> joint_targets = c.getJointTargets();
    UNUSED(joint_targets);

    std::map<int, double> joint_index_targets = c.getJointIndexTargets();
    UNUSED(joint_index_targets);

    std::string link_name;
    Eigen::Isometry3d link_tf;
    c.addLinkTarget(link_name, link_tf);

    std::map<std::string, Eigen::Isometry3d> link_targets = c.getLinkTargets();
    UNUSED(link_targets);

    c.addLinkConstraint(link_name, link_tf);

    std::map<std::string, CartesianConstraint> link_constraints = c.getLinkConstraints();
    UNUSED(link_constraints);

    c.print();
    c.print("prefix_");
  }

private:
  T c;
};

struct MixedWaypointInterface : tesseract_common::TypeErasureInterface
{
  virtual void addJointTarget(std::string joint_name, double joint_value) = 0;
  virtual std::vector<std::string> getJointNames() = 0;
  virtual const std::vector<std::string> getJointNames() const = 0;
  virtual std::map<std::string, double> getJointTargets() = 0;
  virtual const std::map<std::string, double> getJointTargets() const = 0;
  virtual std::map<int, double> getJointIndexTargets() = 0;
  virtual const std::map<int, double> getJointIndexTargets() const = 0;

  virtual void addLinkTarget(std::string link_name, Eigen::Isometry3d& link_tf) = 0;
  virtual std::map<std::string, Eigen::Isometry3d> getLinkTargets() = 0;
  virtual const std::map<std::string, Eigen::Isometry3d> getLinkTargets() const = 0;

  virtual void addLinkConstraint(std::string link_name, Eigen::Isometry3d& link_tf) = 0;
  virtual void addLinkConstraint(std::string link_name, Eigen::Isometry3d& link_tf, const Eigen::VectorXd& coeff) = 0;
  virtual std::map<std::string, CartesianConstraint> getLinkConstraints() = 0;
  virtual const std::map<std::string, CartesianConstraint> getLinkConstraints() const = 0;
  virtual void print(const std::string& prefix) const = 0;

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int version);  // NOLINT
};

template <typename T>
struct MixedWaypointInstance : tesseract_common::TypeErasureInstance<T, MixedWaypointInterface>
{
  using BaseType = tesseract_common::TypeErasureInstance<T, MixedWaypointInterface>;
  MixedWaypointInstance() = default;
  MixedWaypointInstance(const T& x) : BaseType(x) {}
  MixedWaypointInstance(MixedWaypointInstance&& x) noexcept : BaseType(std::move(x)) {}

  BOOST_CONCEPT_ASSERT((MixedWaypointConcept<T>));

  std::vector<std::string> getJointNames() final { return this->get().getJointNames(); }
  const std::vector<std::string> getJointNames() const final { return this->get().getJointNames(); }

  void addJointTarget(std::string joint_name, double joint_value) final
  {
    this->get().addJointTarget(joint_name, joint_value);
  }

  std::map<std::string, double> getJointTargets() final { return this->get().getJointTargets(); }
  const std::map<std::string, double> getJointTargets() const final { return this->get().getJointTargets(); };
  std::map<int, double> getJointIndexTargets() final { return this->get().getJointIndexTargets(); }
  const std::map<int, double> getJointIndexTargets() const final { return this->get().getJointIndexTargets(); }

  void addLinkTarget(std::string link_name, Eigen::Isometry3d& link_tf) final
  {
    this->get().addLinkTarget(link_name, link_tf);
  }

  std::map<std::string, Eigen::Isometry3d> getLinkTargets() final { return this->get().getLinkTargets(); }
  const std::map<std::string, Eigen::Isometry3d> getLinkTargets() const final { return this->get().getLinkTargets(); }

  void addLinkConstraint(std::string link_name, Eigen::Isometry3d& link_tf) final
  {
    this->get().addLinkConstraint(link_name, link_tf);
  }

  void addLinkConstraint(std::string link_name, Eigen::Isometry3d& link_tf, const Eigen::VectorXd& coeff) final
  {
    this->get().addLinkConstraint(link_name, link_tf, coeff);
  }
  std::map<std::string, CartesianConstraint> getLinkConstraints() final { return this->get().getLinkConstraints(); }
  const std::map<std::string, CartesianConstraint> getLinkConstraints() const final
  {
    return this->get().getLinkConstraints();
  }

  void print(const std::string& prefix) const final { this->get().print(prefix); }

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/)  // NOLINT
  {
    ar& boost::serialization::make_nvp("base", boost::serialization::base_object<BaseType>(*this));
  }
};

}  // namespace tesseract_planning::detail_mixed_waypoint

namespace tesseract_planning
{
using MixedWaypointPolyBase = tesseract_common::TypeErasureBase<detail_mixed_waypoint::MixedWaypointInterface,
                                                                detail_mixed_waypoint::MixedWaypointInstance>;

struct MixedWaypointPoly : MixedWaypointPolyBase
{
  using MixedWaypointPolyBase::MixedWaypointPolyBase;
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
  /**
   * @brief add link constraint with coeff. first 3 element of coeff will be pos coeff, last 3 element of coeff will be rot coeff.
   * 
   * @param link_name 
   * @param link_tf 
   * @param coeff 
   */
  void addLinkConstraint(std::string link_name, Eigen::Isometry3d& link_tf, const Eigen::VectorXd& coeff);
  std::map<std::string, detail_mixed_waypoint::CartesianConstraint> getLinkConstraints();
  const std::map<std::string, detail_mixed_waypoint::CartesianConstraint> getLinkConstraints() const;

  void print(const std::string& prefix = "") const;

private:
  friend class boost::serialization::access;
  friend struct tesseract_common::Serialization;

  template <class Archive>
  void serialize(Archive& ar, const unsigned int /*version*/);  // NOLINT
};
}  // namespace tesseract_planning

TESSERACT_WAYPOINT_EXPORT_KEY(tesseract_planning, MixedWaypointPoly);

#endif  // TESSERACT_COMMAND_LANGUAGE_NULL_WAYPOINT_H
