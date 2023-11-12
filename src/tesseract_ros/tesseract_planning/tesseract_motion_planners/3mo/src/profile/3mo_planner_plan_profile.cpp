/**
 * @file 3mo_planner_simple_plan_profile.cpp
 * @brief
 *
 * @author Tyler Marr
 * @date Septemeber 16, 2020
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

#include <AStar.hpp>
#include <tesseract_motion_planners/3mo/profile/3mo_planner_plan_profile.h>
#include <tesseract_motion_planners/3mo/3mo_utils.h>
#include <tesseract_motion_planners/core/utils.h>

using namespace tesseract_kinematics;

namespace tesseract_planning
{
// MMMOPlannerPlanProfile::MMMOPlannerPlanProfile(MapInfo& map,
//                                                int min_steps,
//                                                double state_longest_valid_segment_length,
//                                                double translation_longest_valid_segment_length,
//                                                double rotation_longest_valid_segment_length)
//   : map_(MapInfo(map.map_x, map.map_y, map.step_size))
//   , state_longest_valid_segment_length(state_longest_valid_segment_length)
//   , translation_longest_valid_segment_length(translation_longest_valid_segment_length)
//   , rotation_longest_valid_segment_length(rotation_longest_valid_segment_length)
//   , min_steps(min_steps)
// {
// }

MMMOPlannerPlanProfile::MMMOPlannerPlanProfile(int min_steps,
                                               double state_longest_valid_segment,
                                               double translation_longest_valid_segment_length,
                                               double rotation_longest_valid_segment_length)
  : map_(MapInfo(15, 15, 0.4))
  , min_steps(min_steps)
  , state_longest_valid_segment_length(state_longest_valid_segment)
  , translation_longest_valid_segment_length(translation_longest_valid_segment_length)
  , rotation_longest_valid_segment_length(rotation_longest_valid_segment_length)
{
  // std::cout << "map step size:" << map_.step_size << std::endl;
}

CompositeInstruction MMMOPlannerPlanProfile::generate(const MoveInstructionPoly& prev_instruction,
                                                      const MoveInstructionPoly& /*prev_seed*/,
                                                      const MoveInstructionPoly& base_instruction,
                                                      const InstructionPoly& /*next_instruction*/,
                                                      const PlannerRequest& request,
                                                      const tesseract_common::ManipulatorInfo& global_manip_info) const
{
  KinematicGroupInstructionInfo info1(prev_instruction, request, global_manip_info);
  KinematicGroupInstructionInfo info2(base_instruction, request, global_manip_info);

  if (!info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
    if (info2.has_mixed_waypoint)
      return stateJointMixedWaypoint(info1, info2, request);
  return stateJointJointWaypoint(info1, info2, request);

  if (!info1.has_cartesian_waypoint && info2.has_cartesian_waypoint)
    return stateJointCartWaypoint(info1, info2, request);

  if (info1.has_cartesian_waypoint && !info2.has_cartesian_waypoint)
    return stateCartJointWaypoint(info1, info2);

  return stateCartCartWaypoint(info1, info2, request);
}

CompositeInstruction MMMOPlannerPlanProfile::stateJointMixedWaypoint(const KinematicGroupInstructionInfo& prev,
                                                                     const KinematicGroupInstructionInfo& base,
                                                                     const PlannerRequest& request) const
{
  CONSOLE_BRIDGE_logInform("generating joint -> mixed waypoint composite instruction...");
  const Eigen::VectorXd j1 = prev.extractJointPosition();
  std::stringstream ss;
  ss << j1.transpose();
  CONSOLE_BRIDGE_logDebug("prev state: %s", ss.str().c_str());

  // calculate possible iks with heuristic
  tesseract_kinematics::KinematicGroup::Ptr kin_group =
      std::move(request.env->getKinematicGroup(prev.manip->getName()));
  MixedWaypointPoly wp = base.instruction.getWaypoint().as<MixedWaypointPoly>();

  Eigen::VectorXd joint_target;

  if (wp.getLinkTargets().empty())
  {
    // cartesian waypoint not specified, no need to calculate ik
    CONSOLE_BRIDGE_logDebug("no cartesian target specified in mixed waypoint, will not calculate ik");
    joint_target = j1;
    for (auto& joint : wp.getJointTargets())
    {
      auto joint_names_ = wp.getJointNames();
      auto joint_idx = std::find(joint_names_.begin(), joint_names_.end(), joint.first);
      if (joint_idx == joint_names_.end())
      {
        CONSOLE_BRIDGE_logError("joint target name: %s not found.", joint.first.c_str());
        throw std::runtime_error("joint target name not found in joints");
      }
      // std::cout << joint_idx - joint_names_.begin() << std::endl;
      // std::cout << joint_target.transpose() << std::endl;
      joint_target[joint_idx - joint_names_.begin()] = joint.second;
    }
    std::cout << "joint state target: " << joint_target.transpose() << std::endl;
  }
  else
  {
    // cartesian waypoint specified
    CONSOLE_BRIDGE_logDebug("calculating ik for mixed waypoint...");
    auto filtered_ik_result = getIKsWithCost(request.env, kin_group, wp, base.working_frame, j1, cost_coeff);
    // auto filtered_ik_result = filterCollisionIK(request.env, kin_group, ik_result);
    // std::cout << "total solutions: " << filtered_ik_result.size() << std::endl
    //           << "best sol: " << filtered_ik_result.at(0).transpose() << std::endl;
    std::stringstream ss;
    ss << filtered_ik_result.at(0).first.transpose();
    CONSOLE_BRIDGE_logInform(
        "current joint target: \nik: %s, cost: %f", ss.str().c_str(), filtered_ik_result.at(0).second);
    joint_target = filtered_ik_result.at(0).first;
  }
  // get joint joint seed
  auto states = getJointJointSeed(j1, joint_target, request, kin_group);
  CompositeInstruction interpolated_composite =
      getInterpolatedComposite(kin_group->getJointNames(), states, base.instruction);

  return interpolated_composite;
}

CompositeInstruction MMMOPlannerPlanProfile::stateJointJointWaypoint(const KinematicGroupInstructionInfo& prev,
                                                                     const KinematicGroupInstructionInfo& base,
                                                                     const PlannerRequest& request) const
{
  // trans dist, rot dist, joint dist is not in use for now
  CONSOLE_BRIDGE_logInform("generating joint -> joint waypoint composite instruction...");
  // get kin group
  auto joint_target = base.extractJointPosition();
  auto joint_start = prev.extractJointPosition();
  KinematicGroup::Ptr kin_group = std::move(request.env->getKinematicGroup(prev.manip->getName()));
  auto states = getJointJointSeed(joint_start, joint_target, request, kin_group);
  return getInterpolatedComposite(kin_group->getJointNames(), states, base.instruction);
}

// get seed between 2 joint waypoints
Eigen::MatrixXd MMMOPlannerPlanProfile::getJointJointSeed(const Eigen::VectorXd& joint_start,
                                                          const Eigen::VectorXd& joint_target,
                                                          const PlannerRequest& request,
                                                          KinematicGroup::Ptr kin_group) const
{
  std::vector<Eigen::Isometry3d> base_poses;
  int steps = min_steps;
  Eigen::MatrixXd states = interpolate(joint_start, joint_target, steps);
  if (!base_joint_.first.empty() && !base_joint_.second.empty())
  {
    initBaseTrajectory_(kin_group, request.env, joint_start, joint_target, base_poses, steps);
    auto joint_names = kin_group->getJointNames();
    auto find_result = std::find(joint_names.begin(), joint_names.end(), base_joint_.first);
    assert(find_result != joint_names.end());
    auto x_idx = find_result - joint_names.begin();
    find_result = std::find(joint_names.begin(), joint_names.end(), base_joint_.second);
    assert(find_result != joint_names.end());
    auto y_idx = find_result - joint_names.begin();
    // set base pose from astar
    for (int i = 1; i < steps; ++i)
    {
      states(x_idx, i) = base_poses[i].translation()[0];
      states(y_idx, i) = base_poses[i].translation()[1];
    }
  }
  else
  {
    CONSOLE_BRIDGE_logDebug("base joint does not exist, will not apply astar base trajectory initialization.");
  }

  return states;
}

void MMMOPlannerPlanProfile::initBaseTrajectory_(KinematicGroup::Ptr kin_group,
                                                 tesseract_environment::Environment::ConstPtr env,
                                                 const Eigen::VectorXd& joint_start,
                                                 const Eigen::VectorXd& joint_target,
                                                 std::vector<Eigen::Isometry3d>& base_poses,
                                                 int n_steps) const
{
  tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager_ =
      std::move(env->getDiscreteContactManager()->clone());
  for (auto& active_link : discrete_contact_manager_->getCollisionObjects())
  {
    // std::cout << active_link << std::endl;
    discrete_contact_manager_->enableCollisionObject(active_link);
  }
  // std::cout << "map step size: " << map_.step_size << std::endl;
  tesseract_collision::ContactResultMap contact_results;
  // init base trajectory
  Eigen::Isometry3d base_start_pose;
  base_start_pose.setIdentity();
  base_start_pose.translation() = Eigen::Vector3d(joint_start[0], joint_start[1], 0.145);
  Eigen::Isometry3d base_target_pose;
  base_target_pose.setIdentity();
  base_target_pose.translation() = Eigen::Vector3d(joint_target[0], joint_target[1], 0.145);

  int base_x = int(round((base_start_pose.translation()[0] + map_.map_x / 2.0) / map_.step_size));
  int base_y = int(round((base_start_pose.translation()[1] + map_.map_y / 2.0) / map_.step_size));

  int end_x = int(round((base_target_pose.translation()[0] + map_.map_x / 2.0) / map_.step_size));
  int end_y = int(round((base_target_pose.translation()[1] + map_.map_y / 2.0) / map_.step_size));
  Eigen::Isometry3d base_tf;
  std::vector<std::string> link_names = kin_group->getActiveLinkNames();

  for (auto& link_name : link_names)
  {
    if (link_name != "base_link" && link_name != "world")
    {
      if (!discrete_contact_manager_->removeCollisionObject(link_name))
      {
        // ROS_WARN("Unable to remove collision object: %s", link_name.c_str());
      }
    }
  }

  AStar::Generator astar_generator;

  setupAstarGenerator(astar_generator,
                      discrete_contact_manager_,
                      map_,
                      "base_link",
                      env->getLinkTransform("base_link").translation()[2]);

  // generate path
  std::vector<Eigen::Isometry3d> base_poses_raw;
  CONSOLE_BRIDGE_logDebug("S/G info: %d, %d / %d, %d", base_x, base_y, end_x, end_y);
  astar_generator.removeCollision({ base_x, base_y });
  astar_generator.removeCollision({ end_x, end_y });
  auto path = astar_generator.findPath({ base_x, base_y }, { end_x, end_y });
  base_poses_raw.clear();
  base_poses.clear();
  for (auto& coordinate : path)
  {
    Eigen::Isometry3d base_target;
    base_target.setIdentity();
    base_target.translation() = Eigen::Vector3d(
        -map_.map_x / 2.0 + coordinate.x * map_.step_size, -map_.map_y / 2.0 + coordinate.y * map_.step_size, 0.13);
    base_poses_raw.push_back(base_target);
  }
  if (path.size() == 1)  // start and goal at same position
  {
    for (size_t i = 0; i < n_steps; ++i)
    {
      base_poses.push_back(base_poses_raw.front());
    }
    return;
  }
  std::reverse(base_poses_raw.begin(), base_poses_raw.end());

  // CONSOLE_BRIDGE_logDebug("base poses generated: \n");
  // for (auto& pose : base_poses_raw)
  // {
  //   std::cout << pose.translation() << std::endl;
  // }

  // interpolate
  std::vector<double> nsteps_remap;
  for (int i = 0; i < n_steps; ++i)
  {
    nsteps_remap.push_back(i / 1.0 / (n_steps - 1) * (base_poses_raw.size() - 1));
  }

  for (size_t i = 0; i < n_steps; ++i)
  {
    Eigen::Isometry3d pose;
    pose.setIdentity();
    double idx = nsteps_remap[i];
    double idx_1d = 0;
    double rem = modf(idx, &idx_1d);
    double idx_2d = idx_1d + 1;
    size_t idx_1 = size_t(idx_1d);
    size_t idx_2 = size_t(idx_2d);

    double waypoints_len =
        pow(pow(base_poses_raw[idx_2].translation()[0] - base_poses_raw[idx_1].translation()[0], 2.0) +
                pow(base_poses_raw[idx_2].translation()[1] - base_poses_raw[idx_1].translation()[1], 2.0),
            0.5);

    double grad_x = 0;
    double grad_y = 0;
    if (waypoints_len > 0)
    {
      grad_x = (base_poses_raw[idx_2].translation()[0] - base_poses_raw[idx_1].translation()[0]) / waypoints_len;
      grad_y = (base_poses_raw[idx_2].translation()[1] - base_poses_raw[idx_1].translation()[1]) / waypoints_len;
    }
    pose.translation()[0] = base_poses_raw[idx_1].translation()[0] + grad_x * rem * waypoints_len;
    pose.translation()[1] = base_poses_raw[idx_1].translation()[1] + grad_y * rem * waypoints_len;
    if (std::isnan(pose.translation()[0]) || std::isnan(pose.translation()[1]) || idx_2 > base_poses_raw.size())
    {
      pose.translation()[0] = base_poses.back().translation()[0];
      pose.translation()[1] = base_poses.back().translation()[1];
      CONSOLE_BRIDGE_logWarn("nan pose found at step %d", i);
      std::cout << pose.translation() << std::endl;
    }
    // CONSOLE_BRIDGE_logDebug("interpolated pose translation: ");
    // std::cout << pose.translation() << std::endl;
    base_poses.push_back(pose);
  }
  return;
}

CompositeInstruction MMMOPlannerPlanProfile::stateJointCartWaypoint(const KinematicGroupInstructionInfo& prev,
                                                                    const KinematicGroupInstructionInfo& base,
                                                                    const PlannerRequest& request) const
{
  // Calculate FK for start
  CONSOLE_BRIDGE_logDebug("state joint cart waypoint called");
  const Eigen::VectorXd& j1 = prev.extractJointPosition();
  Eigen::Isometry3d p1_world = prev.calcCartesianPose(j1);

  // Calculate p2 in kinematics base frame without tcp for accurate comparison with p1
  Eigen::Isometry3d p2_world = base.extractCartesianPose();

  KinematicGroup::Ptr kin_group = std::move(request.env->getKinematicGroup(prev.manip->getName()));
  // Calculate steps based on cartesian information
  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  Eigen::VectorXd j2_final = getClosestJointSolution(base, j1);
  if (j2_final.size() != 0)
  {
    double joint_dist = (j2_final - j1).norm();
    int state_steps = int(joint_dist / state_longest_valid_segment_length) + 1;
    steps = std::max(steps, state_steps);

    // Check min steps requirement
    steps = std::max(steps, min_steps);

    // Linearly interpolate in joint space
    // Eigen::MatrixXd states = interpolate(j1, j2_final, steps);
    auto states = getJointJointSeed(j1, j2_final, request, kin_group);
    return getInterpolatedComposite(base.manip->getJointNames(), states, base.instruction);
  }

  // Check min steps requirement
  steps = std::max(steps, min_steps);

  // Convert to MoveInstructions
  Eigen::MatrixXd states = j1.replicate(1, steps + 1);
  return getInterpolatedComposite(base.manip->getJointNames(), states, base.instruction);
}

CompositeInstruction MMMOPlannerPlanProfile::stateCartJointWaypoint(const KinematicGroupInstructionInfo& prev,
                                                                    const KinematicGroupInstructionInfo& base) const
{
  // Calculate FK for end
  const Eigen::VectorXd& j2 = base.extractJointPosition();
  Eigen::Isometry3d p2_world = base.calcCartesianPose(j2);

  // Calculate p1 in kinematics base frame without tcp for accurate comparison with p1
  Eigen::Isometry3d p1_world = prev.extractCartesianPose();

  // Calculate steps based on cartesian information
  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  Eigen::VectorXd j1_final = getClosestJointSolution(prev, j2);
  if (j1_final.size() != 0)
  {
    double joint_dist = (j2 - j1_final).norm();
    int state_steps = int(joint_dist / state_longest_valid_segment_length) + 1;
    steps = std::max(steps, state_steps);

    // Check min steps requirement
    steps = std::max(steps, min_steps);

    // Linearly interpolate in joint space
    Eigen::MatrixXd states = interpolate(j1_final, j2, steps);
    return getInterpolatedComposite(base.manip->getJointNames(), states, base.instruction);
  }

  // Check min steps requirement
  steps = std::max(steps, min_steps);

  // Convert to MoveInstructions
  Eigen::MatrixXd states = j2.replicate(1, steps + 1);
  return getInterpolatedComposite(base.manip->getJointNames(), states, base.instruction);
}

CompositeInstruction MMMOPlannerPlanProfile::stateCartCartWaypoint(const KinematicGroupInstructionInfo& prev,
                                                                   const KinematicGroupInstructionInfo& base,
                                                                   const PlannerRequest& request) const
{
  // Get IK seed
  Eigen::VectorXd seed = request.env_state.getJointValues(base.manip->getJointNames());
  tesseract_common::enforcePositionLimits<double>(seed, base.manip->getLimits().joint_limits);

  // Calculate IK for start and end
  Eigen::Isometry3d p1_world = prev.extractCartesianPose();
  Eigen::Isometry3d p2_world = base.extractCartesianPose();

  double trans_dist = (p2_world.translation() - p1_world.translation()).norm();
  double rot_dist = Eigen::Quaterniond(p1_world.linear()).angularDistance(Eigen::Quaterniond(p2_world.linear()));
  int trans_steps = int(trans_dist / translation_longest_valid_segment_length) + 1;
  int rot_steps = int(rot_dist / rotation_longest_valid_segment_length) + 1;
  int steps = std::max(trans_steps, rot_steps);

  std::array<Eigen::VectorXd, 2> sol = getClosestJointSolution(prev, base, seed);

  Eigen::MatrixXd states;
  if (sol[0].size() != 0 && sol[1].size() != 0)
  {
    double joint_dist = (sol[1] - sol[0]).norm();
    int state_steps = int(joint_dist / state_longest_valid_segment_length) + 1;
    steps = std::max(steps, state_steps);

    // Check min steps requirement
    steps = std::max(steps, min_steps);

    // Interpolate
    states = interpolate(sol[0], sol[1], steps);
  }
  else if (sol[0].size() != 0)
  {
    // Check min steps requirement
    steps = std::max(steps, min_steps);

    // Interpolate
    states = sol[0].replicate(1, steps + 1);
  }
  else if (sol[1].size() != 0)
  {
    // Check min steps requirement
    steps = std::max(steps, min_steps);

    // Interpolate
    states = sol[1].replicate(1, steps + 1);
  }
  else
  {
    // Check min steps requirement
    steps = std::max(steps, min_steps);

    states = seed.replicate(1, steps + 1);
  }

  // Convert to MoveInstructions
  return getInterpolatedComposite(base.manip->getJointNames(), states, base.instruction);
}
}  // namespace tesseract_planning
