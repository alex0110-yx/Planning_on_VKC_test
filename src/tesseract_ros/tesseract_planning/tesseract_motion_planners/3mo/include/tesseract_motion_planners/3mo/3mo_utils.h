#ifndef TESSERACT_MOTION_PLANNERS_3MO_UTILS_H
#define TESSERACT_MOTION_PLANNERS_3MO_UTILS_H

#include <tesseract_collision/core/discrete_contact_manager.h>
#include <tesseract_kinematics/core/utils.h>
#include <tesseract_motion_planners/simple/profile/simple_planner_utils.h>
#include <tesseract_command_language/mixed_waypoint.h>
#include <AStar.hpp>

namespace tesseract_planning
{
struct MapInfo
{
  int map_x;
  int map_y;
  double step_size;
  int grid_size_x;
  int grid_size_y;

  MapInfo(int x = 15, int y = 15, double step = 0.4) : map_x(x), map_y(y), step_size(step)
  {
    grid_size_x = int(map_x / step_size) + 1;
    grid_size_y = int(map_y / step_size) + 1;
  }
};

struct IKWithCost
{
  Eigen::VectorXd ik;
  double cost;
  IKWithCost(Eigen::VectorXd ik, double cost) : ik{ std::move(ik) }, cost{ cost } {}
  friend bool operator<(IKWithCost const& ik1, IKWithCost const& ik2) { return ik1.cost < ik2.cost; }
  friend bool operator>(IKWithCost const& ik1, IKWithCost const& ik2) { return ik1.cost > ik2.cost; }
};

void setupAstarGenerator(AStar::Generator& generator,
                         tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager,
                         const MapInfo& map,
                         const std::string& base_link_name,
                         double base_link_z);

bool isEmptyCell(tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager,
                 std::string link_name,
                 Eigen::Isometry3d& tf,
                 tesseract_collision::ContactResultMap& contact_results);

tesseract_kinematics::IKSolutions getIKs(const tesseract_environment::Environment::ConstPtr env,
                                         tesseract_kinematics::KinematicGroup::Ptr manip,
                                         Eigen::VectorXd prev_joint,
                                         const MixedWaypointPoly& waypoint,
                                         const std::string working_frame,
                                         double tolerance = 0.2);

std::vector<std::pair<Eigen::VectorXd, double>> getIKsWithCost(const tesseract_environment::Environment::ConstPtr env,
                                                               tesseract_kinematics::KinematicGroup::Ptr manip,
                                                               const MixedWaypointPoly& waypoint,
                                                               const std::string working_frame,
                                                               const Eigen::VectorXd& prev_joints,
                                                               const Eigen::VectorXd& cost_coeff = Eigen::VectorXd());

double getIKCost(const tesseract_planning::MixedWaypointPoly& wp,
                 const Eigen::VectorXd& target,
                 const Eigen::VectorXd& base,
                 const Eigen::VectorXd& cost_coeff);

double getIKGoalCost(const Eigen::VectorXd& ik, const MixedWaypointPoly& wp, double tolerance);

std::size_t getIKCollisionCount(const tesseract_environment::Environment::ConstPtr env,
                                tesseract_kinematics::KinematicGroup::Ptr kin_group,
                                Eigen::VectorXd joints);

tesseract_kinematics::IKSolutions filterCollisionIK(tesseract_environment::Environment::ConstPtr env,
                                                    tesseract_kinematics::KinematicGroup::Ptr kin_group,
                                                    tesseract_kinematics::IKSolutions ik_input);

std::vector<std::pair<Eigen::VectorXd, double>>
filterCollisionIK(tesseract_environment::Environment::ConstPtr env,
                  tesseract_kinematics::KinematicGroup::Ptr kin_group,
                  std::vector<std::pair<Eigen::VectorXd, double>> ik_input);

Eigen::VectorXd refineIK(tesseract_kinematics::KinematicGroup::Ptr manip,
                         const Eigen::VectorXd& ik_result,
                         const Eigen::VectorXd& init_config);

std::vector<Eigen::VectorXd> refineIK2(tesseract_kinematics::KinematicGroup::Ptr manip,
                                       const Eigen::VectorXd& ik_result,
                                       const Eigen::VectorXd& init_config);

}  // namespace tesseract_planning

#endif