#include <tesseract_motion_planners/3mo/3mo_utils.h>
#include <tesseract_command_language/mixed_waypoint.h>
#include <queue>
#include <fmt/ranges.h>
#include <math.h>

namespace tesseract_planning
{

unsigned int MAX_IK_CALC_NUM = 20000;
unsigned int MAX_IK_QUEUE_NUM = 100;

void setupAstarGenerator(AStar::Generator& generator,
                         tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager,
                         const MapInfo& map,
                         const std::string& base_link_name,
                         double base_link_z)
{
  generator.setWorldSize({ map.grid_size_x, map.grid_size_y });
  generator.setHeuristic(AStar::Heuristic::euclidean);
  generator.setDiagonalMovement(true);

  CONSOLE_BRIDGE_logDebug("map info: %d, %d, stepsize: %f, base link: %s, base link z: %f",
                          map.grid_size_x,
                          map.grid_size_y,
                          map.step_size,
                          base_link_name.c_str(),
                          base_link_z);

  tesseract_collision::ContactResultMap contact_results;
  Eigen::Isometry3d base_tf;

  for (int x = 0; x < map.grid_size_x; ++x)
  {
    for (int y = 0; y < map.grid_size_y; ++y)
    {
      base_tf.setIdentity();
      contact_results.clear();
      base_tf.translation() =
          Eigen::Vector3d(-map.map_x / 2.0 + x * map.step_size, -map.map_y / 2.0 + y * map.step_size, base_link_z);
      if (!tesseract_planning::isEmptyCell(discrete_contact_manager,
                                           base_link_name, base_tf,
                                           contact_results) /*&&
          (!(x == base_x && y == base_y) && !(x == end_x && y == end_y))*/)
      {
        // std::cout << "o";
        // std::cout << x << ":\t" << y << std::endl;
        generator.addCollision({ x, y });
        // } else if (x == base_x && y == base_y) {
        // std::cout << "S";
        // } else if (x == end_x && y == end_y) {
        // std::cout << "G";
      }
      else
      {
        // std::cout << "+";
      }
    }
    // std::cout << std::endl;
  }
}

bool isEmptyCell(tesseract_collision::DiscreteContactManager::Ptr discrete_contact_manager,
                 std::string link_name,
                 Eigen::Isometry3d& tf,
                 tesseract_collision::ContactResultMap& contact_results)
{
  assert(discrete_contact_manager != nullptr);
  discrete_contact_manager->setCollisionObjectsTransform(link_name, tf);
  discrete_contact_manager->contactTest(contact_results, tesseract_collision::ContactTestType::ALL);
  for (auto& collision : contact_results)
  {
    // std::cout << collision.first.first << ":\t" << collision.first.second <<
    // std::endl;
    if (collision.first.first == "base_link" || collision.first.second == "base_link")
    {
      return false;
    }
  }
  return true;
}

tesseract_kinematics::IKSolutions getIKs(const tesseract_environment::Environment::ConstPtr env,
                                         tesseract_kinematics::KinematicGroup::Ptr manip,
                                         Eigen::VectorXd prev_joints,
                                         const MixedWaypointPoly& waypoint,
                                         const std::string working_frame,
                                         double tolerance)
{
  CONSOLE_BRIDGE_logDebug("getting iks...");
  auto limits = manip->getLimits();
  auto redundancy_indices = manip->getRedundancyCapableJointIndices();
  tesseract_kinematics::KinGroupIKInputs ik_inputs;
  for (auto link_target : waypoint.getLinkTargets())
  {
    ik_inputs.push_back(tesseract_kinematics::KinGroupIKInput(link_target.second, working_frame, link_target.first));
  }
  tesseract_kinematics::IKSolutions solutions;
  int retry = 0;
  Eigen::VectorXd ik_seed = prev_joints;
  Eigen::VectorXd prev_ik_seed_ = prev_joints;
  std::cout << "number of solutions found / retries: " << std::endl;
  while (retry < MAX_IK_CALC_NUM)
  {
    // std::cout << "ik seed: " << ik_seed.transpose() << std::endl << "ik input size: " << ik_inputs.size() <<
    // std::endl; std::cout << fmt::format("kin group joint names: {}", manip->getJointNames()).c_str() << std::endl;
    // std::cout << fmt::format("kin group joint names: {}", manip->getAllPossibleTipLinkNames()).c_str() << std::endl;
    tesseract_kinematics::IKSolutions result;
    try
    {
      result = manip->calcInvKin(ik_inputs, ik_seed);
    }
    catch (const std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      CONSOLE_BRIDGE_logError("Can't find desired link name in kinematic chain.");
      throw e;
    }

    auto filtered_result = filterCollisionIK(env, manip, result);
    // std::cout << "result length: " << result.size() << std::endl;
    for (const auto& res : filtered_result)
    {
      if (getIKGoalCost(res, waypoint, tolerance) >= 0)
      {
        // auto ik_refined = refineIK(manip, res, prev_joints);
        // assert(ik_refined.size() > 0);
        // solutions.push_back(ik_refined);
        for (const auto& ik_refined : refineIK2(manip, res, prev_joints))
        {
          assert(ik_refined.size() > 0);
          solutions.push_back(ik_refined);
        }
      }
      // auto redundant_solutions =
      //     tesseract_kinematics::getRedundantSolutions<double>(res, limits.joint_limits, redundancy_indices);
      // for (const auto& redundant_sol : redundant_solutions)
      // {
      //   if (getIKGoalCost(redundant_sol, waypoint, tolerance) >= 0)
      //   {
      //     auto ik_refined = refineIK(manip, redundant_sol, prev_joints);
      //     assert(ik_refined.size() > 0);
      //     solutions.push_back(ik_refined);
      //   }
      // }
    }
    prev_ik_seed_ = ik_seed;
    ik_seed = tesseract_common::generateRandomNumber(limits.joint_limits);
    if (prev_ik_seed_ == ik_seed)
    {
      CONSOLE_BRIDGE_logWarn("prev ik seed is same as new ik seed");
    }
    std::cout << "\r" << solutions.size() << " / " << retry << "\t" << std::flush;
    retry++;

    if (solutions.size() > MAX_IK_QUEUE_NUM)
      break;
  }
  if (solutions.empty())
  {
    throw std::runtime_error("cannot find valid ik solution");
  }
  std::cout << std::endl;
  return solutions;
}

// seed here is the initial joint pose, not actual seed used for kinematics.
std::vector<std::pair<Eigen::VectorXd, double>> getIKsWithCost(const tesseract_environment::Environment::ConstPtr env,
                                                               tesseract_kinematics::KinematicGroup::Ptr manip,
                                                               const MixedWaypointPoly& waypoint,
                                                               const std::string working_frame,
                                                               const Eigen::VectorXd& prev_joints,
                                                               const Eigen::VectorXd& cost_coefficient_input)
{
  std::stringstream ss;

  auto limits = manip->getLimits();
  auto redundancy_indices = manip->getRedundancyCapableJointIndices();
  Eigen::VectorXd cost_coeff;
  if (cost_coefficient_input.size())
  {
    // assert(cost_coefficient_input.size() == prev_joints.size());
    cost_coeff = cost_coefficient_input;
  }
  else
  {
    CONSOLE_BRIDGE_logInform("no cost coeff found, setting cost coeffs with 1");
    cost_coeff.setOnes(prev_joints.size());
    // cost_coeff[6] = 2.;
    // cost_coeff[3] = 5.;
    // cost_coeff[0] = 2.;
    // cost_coeff[1] = 2.;
  }
  ss << cost_coeff.transpose();
  CONSOLE_BRIDGE_logInform("getting ik with heuristic for mixed waypoint with cost coeff %s", ss.str().c_str());
  // if (!info.has_mixed_waypoint)
  // {
  //   throw std::runtime_error("Instruction waypoint need to have a mixed waypoint.");
  // }
  // MixedWaypoint wp = info.instruction.getWaypoint().as<MixedWaypoint>();

  // ik_with_cost_queue is reversed when inserting elements(larger cost will be poped first), so we need greater
  std::priority_queue<IKWithCost, std::vector<IKWithCost>, std::greater<IKWithCost>> ik_with_cost_queue;
  tesseract_kinematics::KinGroupIKInputs ik_inputs;
  for (auto link_target : waypoint.getLinkTargets())
  {
    std::stringstream ss;
    ss << link_target.first << ", working frame: " << working_frame
       << ", translation: " << link_target.second.translation().transpose() << ", linear\n"
       << link_target.second.linear();
    CONSOLE_BRIDGE_logDebug("adding kin group input: %s", ss.str().c_str());
    ik_inputs.push_back(tesseract_kinematics::KinGroupIKInput(link_target.second, working_frame, link_target.first));
  }
  int retry_count = 0;
  Eigen::VectorXd ik_seed = prev_joints;
  while (retry_count < MAX_IK_CALC_NUM)
  {
    retry_count++;
    // Eigen::VectorXd ik_seed = tesseract_common::generateRandomNumber(limits.joint_limits);
    // std::cout << "ik seed: " << ik_seed.transpose() << std::endl << "ik input size: " << ik_inputs.size() <<
    // std::endl; std::cout << fmt::format("kin group joint names: {}", manip->getJointNames()).c_str() << std::endl;
    // std::cout << fmt::format("kin group joint names: {}", manip->getAllPossibleTipLinkNames()).c_str() << std::endl;
    tesseract_kinematics::IKSolutions result;
    try
    {
      result = manip->calcInvKin(ik_inputs, ik_seed);
    }
    catch (const std::exception& e)
    {
      std::cerr << e.what() << std::endl;
      CONSOLE_BRIDGE_logError("Can't find desired link name in kinematic chain.");
      throw e;
    }

    auto filtered_result = filterCollisionIK(env, manip, result);

    for (const auto& res : filtered_result)
    {
      auto ik_refined = refineIK(manip, res, prev_joints);
      // auto ik_refined = res;
      double cost = getIKCost(waypoint, ik_refined, prev_joints, cost_coeff);
      if (cost > 0)
        ik_with_cost_queue.emplace(ik_refined, cost);

      auto redundant_solutions =
          tesseract_kinematics::getRedundantSolutions<double>(ik_refined, limits.joint_limits, redundancy_indices);
      for (const auto& redundant_sol : redundant_solutions)
      {
        ik_refined = refineIK(manip, redundant_sol, prev_joints);
        // ik_refined = redundant_sol;
        cost = getIKCost(waypoint, ik_refined, prev_joints, cost_coeff);
        if (cost > 0)
          ik_with_cost_queue.emplace(ik_refined, cost);
      }
    }

    if (ik_with_cost_queue.size() > MAX_IK_QUEUE_NUM)
      break;

    ik_seed = tesseract_common::generateRandomNumber(limits.joint_limits);
  }

  // reverse the ik with cost queue and return
  std::vector<std::pair<Eigen::VectorXd, double>> solutions;

  if (ik_with_cost_queue.size() == 0)
  {
    ik_with_cost_queue.emplace(prev_joints, 1e6);
    CONSOLE_BRIDGE_logWarn("Fail to find feasible ik, use currect pose.");
  }

  while (ik_with_cost_queue.size() > 0)
  {
    // solutions.insert(solutions.begin(), ik_with_cost_queue.top().ik);
    // std::cout << ik_with_cost_queue.top().cost << ", ";
    solutions.push_back(std::make_pair(ik_with_cost_queue.top().ik, ik_with_cost_queue.top().cost));
    ik_with_cost_queue.pop();
  }

  std::stringstream buffer;
  buffer << solutions.front().first.transpose();
  std::stringstream coeff_buffer;
  coeff_buffer << cost_coeff.transpose();
  CONSOLE_BRIDGE_logDebug(
      "best ik: %s\ncost: %f, coeff: %s", buffer.str().c_str(), solutions.front().second, coeff_buffer.str().c_str());
  // reverse the ik with cost queue and return

  return solutions;

  // get more than 1000 ik solutions
}

double getIKCost(const MixedWaypointPoly& wp,
                 const Eigen::VectorXd& target,
                 const Eigen::VectorXd& base,
                 const Eigen::VectorXd& cost_coefficient)
{
  // assert(target.size() == base.size() && target.size() == cost_coefficient.size());
  double cost = 0;
  cost += (target.head(9) - base.head(9)).cwiseProduct(cost_coefficient.head(9)).array().abs().sum();
  double ik_goal_cost = getIKGoalCost(target, wp, 0.2);
  if (ik_goal_cost < 0)
  {
    return -1.0;
  }
  // cost += ik_goal_cost * 7;
  return std::abs(cost);
}

double getIKGoalCost(const Eigen::VectorXd& ik, const MixedWaypointPoly& wp, double tolerance)
{
  double cost = 0;
  for (auto const& jt : wp.getJointIndexTargets())
  {
    const double diff = std::abs(ik[jt.first] - jt.second);
    if (diff > tolerance)
    {
      return -1.0;
    }
    cost += std::pow(diff, 1);
  }
}

std::size_t getIKCollisionCount(const tesseract_environment::Environment::ConstPtr env,
                                tesseract_kinematics::KinematicGroup::Ptr kin_group,
                                Eigen::VectorXd joints)
{
  tesseract_collision::ContactResultMap contact_result;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager = env->getDiscreteContactManager()->clone();
  size_t best_collision_count = 1000;
  auto current_state = env->getState(kin_group->getJointNames(), joints);
  contact_manager->setCollisionObjectsTransform(current_state.link_transforms);
  contact_manager->contactTest(contact_result, tesseract_collision::ContactTestType::ALL);
  // for (auto& collision : contact_result)
  // {
  //   std::cout << "\t" << collision.first.first << " -->|<-- " << collision.first.second << std::endl;
  // }
  return contact_result.size();
}

tesseract_kinematics::IKSolutions filterCollisionIK(const tesseract_environment::Environment::ConstPtr env,
                                                    tesseract_kinematics::KinematicGroup::Ptr kin_group,
                                                    tesseract_kinematics::IKSolutions ik_input)
{
  // CONSOLE_BRIDGE_logDebug("filtering ik with collision...");
  tesseract_kinematics::IKSolutions result;
  // check collision
  tesseract_collision::ContactResultMap contact_result;
  tesseract_collision::ContactResultMap best_contact_result;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager = env->getDiscreteContactManager()->clone();
  size_t best_collision_count = 1000;
  if (ik_input.size() == 0)
    return ik_input;
  Eigen::VectorXd best_solution = ik_input.at(0);
  for (auto const& ik : ik_input)
  {
    contact_result.clear();
    auto current_state = env->getState(kin_group->getJointNames(), ik);
    contact_manager->setCollisionObjectsTransform(current_state.link_transforms);
    contact_manager->contactTest(contact_result, tesseract_collision::ContactTestType::ALL);
    // for (auto& collision : contact_result)
    // {
    //   std::cout << "\t" << collision.first.first << " -->|<-- " << collision.first.second << std::endl;
    // }
    if (contact_result.size() <= best_collision_count)
    {
      best_collision_count = contact_result.size();
      best_solution = ik;
      best_contact_result = contact_result;
      if (contact_result.size() == 0)
      {
        result.push_back(ik);
      }
    }
  }

  // if (result.empty())
  // {
  //   CONSOLE_BRIDGE_logWarn("ik solution is empty, saving best solution into result. best collision count: %d",
  //                          best_collision_count);
  //   for (auto& collision : best_contact_result)
  //   {
  //     std::cout << "\t" << collision.first.first << " -->|<-- " << collision.first.second << std::endl;
  //   }
  //   result.push_back(best_solution);
  // }
  // CONSOLE_BRIDGE_logDebug("collision free ik: %ld/%ld", result.size(), ik_input.size());

  return result;
}

std::vector<std::pair<Eigen::VectorXd, double>>
filterCollisionIK(tesseract_environment::Environment::ConstPtr env,
                  tesseract_kinematics::KinematicGroup::Ptr kin_group,
                  std::vector<std::pair<Eigen::VectorXd, double>> ik_input)
{
  // CONSOLE_BRIDGE_logDebug("filtering ik with collision...");
  std::vector<std::pair<Eigen::VectorXd, double>> result;
  // check collision
  tesseract_collision::ContactResultMap contact_result;
  tesseract_collision::ContactResultMap best_contact_result;
  tesseract_collision::DiscreteContactManager::Ptr contact_manager = env->getDiscreteContactManager()->clone();
  size_t best_collision_count = 1000;
  std::pair<Eigen::VectorXd, double> best_solution = ik_input.at(0);
  for (auto const& ik : ik_input)
  {
    contact_result.clear();
    auto current_state = env->getState(kin_group->getJointNames(), ik.first);
    contact_manager->setCollisionObjectsTransform(current_state.link_transforms);
    contact_manager->contactTest(contact_result, tesseract_collision::ContactTestType::ALL);
    for (auto& collision : contact_result)
    {
      std::cout << "\t" << collision.first.first << " -->|<-- " << collision.first.second << std::endl;
    }
    if (contact_result.size() <= best_collision_count)
    {
      best_collision_count = contact_result.size();
      best_solution = ik;
      best_contact_result = contact_result;
      if (contact_result.size() == 0)
      {
        result.push_back(ik);
      }
    }
  }

  if (result.empty())
  {
    CONSOLE_BRIDGE_logWarn("ik solution is empty, saving best solution into result. best collision count: %d",
                           best_collision_count);
    for (auto& collision : best_contact_result)
    {
      std::cout << "\t" << collision.first.first << " -->|<-- " << collision.first.second << std::endl;
    }
    result.push_back(best_solution);
  }
  CONSOLE_BRIDGE_logDebug("collision free ik: %ld/%ld", result.size(), ik_input.size());

  return result;
}

Eigen::VectorXd refineIK(tesseract_kinematics::KinematicGroup::Ptr manip,
                         const Eigen::VectorXd& ik_result,
                         const Eigen::VectorXd& init_config)
{
  Eigen::VectorXd ik_refined = ik_result;
  for (const auto idx : manip->getRedundancyCapableJointIndices())
  {
    if (manip->getJointNames()[idx] == "right_arm_shoulder_pan_joint" ||
        manip->getJointNames()[idx] == "base_link_base_theta" ||
        manip->getJointNames()[idx] == "right_arm_shoulder_lift_joint" ||
        manip->getJointNames()[idx] == "ur_arm_shoulder_pan_joint")
    {
      if (ik_result[idx] > init_config[idx])
      {
        if (ik_refined[idx] - init_config[idx] > M_PI)
        {
          ik_refined[idx] -= 2.0 * M_PI;
          if (!tesseract_common::satisfiesPositionLimits<double>(ik_refined, manip->getLimits().joint_limits))
            ik_refined[idx] += 2.0 * M_PI;
        }
      }
      else
      {
        if (init_config[idx] - ik_result[idx] > M_PI)
        {
          ik_refined[idx] += 2.0 * M_PI;
          if (!tesseract_common::satisfiesPositionLimits<double>(ik_refined, manip->getLimits().joint_limits))
            ik_refined[idx] -= 2.0 * M_PI;
        }
      }
    }
  }

  return ik_refined;
}

std::vector<Eigen::VectorXd> refineIK2(tesseract_kinematics::KinematicGroup::Ptr manip,
                                       const Eigen::VectorXd& ik_result,
                                       const Eigen::VectorXd& init_config)
{
  auto redundancy_indices = manip->getRedundancyCapableJointIndices();
  auto redundant_solutions = tesseract_kinematics::getRedundantSolutions<double>(
      ik_result, manip->getLimits().joint_limits, redundancy_indices);

  redundant_solutions.push_back(ik_result);
  std::priority_queue<IKWithCost, std::vector<IKWithCost>, std::greater<IKWithCost>> solutions;
  // std::cout << "redundant_solutions size: " << redundant_solutions.size() << std::endl;
  for (const auto& redundant_sol : redundant_solutions)
  {
    // std::cout << redundant_sol.transpose() << std::endl;
    auto diff = redundant_sol - init_config;
    double cost = diff.matrix().norm();
    for (const auto idx : manip->getRedundancyCapableJointIndices())
    {
      if (manip->getJointNames()[idx] == "right_arm_shoulder_pan_joint" ||
          manip->getJointNames()[idx] == "base_link_base_theta" ||
          manip->getJointNames()[idx] == "right_arm_shoulder_lift_joint" ||
          manip->getJointNames()[idx] == "ur_arm_shoulder_pan_joint")
      {
        if (abs(redundant_sol[idx] - init_config[idx]) > M_PI)
          cost += 1e3;
      }
    }
    // std::cout << "diff: " << diff.transpose() << std::endl << "norm: " << cost << std::endl;
    solutions.emplace(redundant_sol, cost);
  }
  std::vector<Eigen::VectorXd> result;
  for (int i = 0; i < redundant_solutions.size(); i++)
  {
    result.push_back(solutions.top().ik);
    // std::cout << solutions.top().cost << ": " << solutions.top().ik.transpose() << std::endl;
    solutions.pop();
  }
  return result;
}

}  // namespace tesseract_planning