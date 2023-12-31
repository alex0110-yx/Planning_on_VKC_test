#include <tesseract_common/macros.h>
TESSERACT_COMMON_IGNORE_WARNINGS_PUSH
#include <console_bridge/console.h>
#include <tesseract_environment/utils.h>
TESSERACT_COMMON_IGNORE_WARNINGS_POP

#include <tesseract_motion_planners/3mo/3mo_motion_planner.h>
#include <tesseract_motion_planners/3mo/profile/3mo_planner_plan_profile.h>
#include <tesseract_motion_planners/3mo/profile/3mo_planner_ik_plan_profile.h>
#include <tesseract_motion_planners/core/utils.h>
#include <tesseract_command_language/poly/waypoint_poly.h>
#include <tesseract_command_language/state_waypoint.h>
#include <tesseract_motion_planners/planner_utils.h>
#include <tesseract_motion_planners/3mo/3mo_utils.h>

namespace tesseract_planning
{
MMMOMotionPlannerStatusCategory::MMMOMotionPlannerStatusCategory(std::string name) : name_(std::move(name)) {}
const std::string& MMMOMotionPlannerStatusCategory::name() const noexcept { return name_; }
std::string MMMOMotionPlannerStatusCategory::message(int code) const
{
  switch (code)
  {
    case SolutionFound: {
      return "Found valid solution";
    }
    case ErrorInvalidInput: {
      return "Input to planner is invalid. Check that instructions and seed are compatible";
    }
    case FailedToFindValidSolution: {
      return "Failed to find valid solution";
    }
    default: {
      assert(false);
      return "";
    }
  }
}

MMMOMotionPlanner::MMMOMotionPlanner(std::string name)
  : name_(std::move(name)), status_category_(std::make_shared<const MMMOMotionPlannerStatusCategory>(name_))
{
  if (name_.empty())
    throw std::runtime_error("MMMOMotionPlanner name is empty!");
}

const std::string& MMMOMotionPlanner::getName() const { return name_; }

bool MMMOMotionPlanner::terminate()
{
  CONSOLE_BRIDGE_logWarn("Termination of ongoing planning is not implemented yet");
  return false;
}

void MMMOMotionPlanner::clear() {}

MotionPlanner::Ptr MMMOMotionPlanner::clone() const { return std::make_shared<MMMOMotionPlanner>(name_); }

tesseract_common::StatusCode MMMOMotionPlanner::solve(const PlannerRequest& request,
                                                      PlannerResponse& response,
                                                      bool /*verbose*/) const
{
  if (!checkUserInput(request))
  {
    response.status =
        tesseract_common::StatusCode(MMMOMotionPlannerStatusCategory::ErrorInvalidInput, status_category_);
    return response.status;
  }

  // Assume all the plan instructions have the same manipulator as the composite
  const std::string manipulator = request.instructions.getManipulatorInfo().manipulator;
  const std::string manipulator_ik_solver = request.instructions.getManipulatorInfo().manipulator_ik_solver;

  // Initialize
  tesseract_kinematics::JointGroup::UPtr manip = request.env->getJointGroup(manipulator);
  WaypointPoly start_waypoint;

  // Create seed
  CompositeInstruction seed;

  // Get the start waypoint/instruction
  MoveInstructionPoly start_instruction = getStartInstruction(request, request.env_state, *manip);

  // Set start instruction
  MoveInstructionPoly start_instruction_seed = start_instruction;
  start_instruction_seed.setMoveType(MoveInstructionType::START);

  // Process the instructions into the seed
  try
  {
    MoveInstructionPoly start_instruction_copy = start_instruction;
    MoveInstructionPoly start_instruction_seed_copy = start_instruction_seed;
    seed =
        processCompositeInstruction(request.instructions, start_instruction_copy, start_instruction_seed_copy, request);
  }
  catch (std::exception& e)
  {
    CONSOLE_BRIDGE_logError("MMMOMotionPlanner failed to solve problem: %s.", e.what());
    response.status =
        tesseract_common::StatusCode(MMMOMotionPlannerStatusCategory::ErrorInvalidInput, status_category_);
    return response.status;
  }

  // Set seed start state
  seed.setStartInstruction(start_instruction_seed);

  // Fill out the response
  response.results = seed;

  // Enforce limits
  auto results_flattened = response.results.flatten(&moveFilter);
  for (auto& inst : results_flattened)
  {
    auto& mi = inst.get().as<MoveInstructionPoly>();
    Eigen::VectorXd jp = getJointPosition(mi.getWaypoint());
    if (jp.rows() == manip->getLimits().joint_limits.rows())  // row means numbers of joint
    {
      assert(tesseract_common::satisfiesPositionLimits<double>(jp, manip->getLimits().joint_limits));
      tesseract_common::enforcePositionLimits<double>(jp, manip->getLimits().joint_limits);
      setJointPosition(mi.getWaypoint(), jp);
    }
    else
      CONSOLE_BRIDGE_logWarn("Enforce limits error: joint waypoint row(%d) is not equal to limit "
                             "row(%d)",
                             jp.rows(),
                             jp.cols(),
                             manip->getLimits().joint_limits.rows(),
                             manip->getLimits().joint_limits.cols());
  }

  // Return success
  response.status = tesseract_common::StatusCode(MMMOMotionPlannerStatusCategory::SolutionFound, status_category_);
  return response.status;
}

MoveInstructionPoly MMMOMotionPlanner::getStartInstruction(const PlannerRequest& request,
                                                           const tesseract_scene_graph::SceneState& current_state,
                                                           const tesseract_kinematics::JointGroup& manip)
{
  // Create start instruction
  if (request.instructions.hasStartInstruction())
  {
    const auto& start_instruction = request.instructions.getStartInstruction();
    assert(start_instruction.isStart());
    const auto& start_waypoint = start_instruction.getWaypoint();

    MoveInstructionPoly start_instruction_seed(start_instruction);
    if (start_waypoint.isJointWaypoint())
    {
      assert(checkJointPositionFormat(manip.getJointNames(), start_waypoint));
      const auto& jwp = start_waypoint.as<JointWaypointPoly>();
      StateWaypointPoly swp = start_instruction_seed.createStateWaypoint();
      swp.setNames(jwp.getNames());
      swp.setPosition(jwp.getPosition());
      start_instruction_seed.assignStateWaypoint(swp);
      return start_instruction_seed;
    }

    if (start_waypoint.isCartesianWaypoint())
    {
      /** @todo Update to run IK to find solution closest to start */
      StateWaypointPoly swp = start_instruction_seed.createStateWaypoint();
      swp.setNames(manip.getJointNames());
      swp.setPosition(current_state.getJointValues(manip.getJointNames()));
      start_instruction_seed.assignStateWaypoint(swp);
      return start_instruction_seed;
    }

    if (start_waypoint.isStateWaypoint())
    {
      assert(checkJointPositionFormat(manip.getJointNames(), start_waypoint));
      return start_instruction_seed;
    }

    throw std::runtime_error("Unsupported waypoint type!");
  }

  MoveInstructionPoly start_instruction_seed(*request.instructions.getFirstMoveInstruction());
  start_instruction_seed.setMoveType(MoveInstructionType::START);
  StateWaypointPoly swp = start_instruction_seed.createStateWaypoint();
  swp.setNames(manip.getJointNames());
  swp.setPosition(current_state.getJointValues(manip.getJointNames()));
  start_instruction_seed.assignStateWaypoint(swp);

  return start_instruction_seed;
}

CompositeInstruction MMMOMotionPlanner::processCompositeInstruction(const CompositeInstruction& instructions,
                                                                    MoveInstructionPoly& prev_instruction,
                                                                    MoveInstructionPoly& prev_seed,
                                                                    const PlannerRequest& request) const
{
  CompositeInstruction seed(instructions);
  seed.clear();

  for (std::size_t i = 0; i < instructions.size(); ++i)
  {
    const auto& instruction = instructions[i];

    if (instruction.isCompositeInstruction())
    {
      seed.appendInstruction(
          processCompositeInstruction(instruction.as<CompositeInstruction>(), prev_instruction, prev_seed, request));
    }
    else if (instruction.isMoveInstruction())
    {
      const auto& base_instruction = instruction.as<MoveInstructionPoly>();

      // Get the next plan instruction if it exists
      InstructionPoly next_instruction;
      for (std::size_t n = i + 1; n < instructions.size(); ++n)
      {
        if (instructions[n].isMoveInstruction())
        {
          next_instruction = instructions[n];
          break;
        }
      }

      // If a path profile exists for the instruction it should use that instead of the termination profile
      MMMOPlanProfile::ConstPtr plan_profile;
      MapInfo default_map = MapInfo();
      if (base_instruction.getPathProfile().empty())
      {
        std::string profile = getProfileString(name_, base_instruction.getProfile(), request.plan_profile_remapping);
        if (profile == "IK_TRAJ")
        {
          CONSOLE_BRIDGE_logDebug("profile name is IK_TRAJ, using 3mo planner ik profile...");
          plan_profile = getProfile<MMMOPlannerIKPlanProfile>(
              name_,
              profile,
              *request.profiles,
              std::make_shared<MMMOPlannerIKPlanProfile>(default_map.map_x, default_map.map_y, default_map.step_size));
        }
        else
        {
          plan_profile = getProfile<MMMOPlannerPlanProfile>(
              name_,
              profile,
              *request.profiles,
              std::make_shared<MMMOPlannerPlanProfile>(default_map.map_x, default_map.map_y, default_map.step_size));
        }
        plan_profile = applyProfileOverrides(name_, profile, plan_profile, base_instruction.getProfileOverrides());
      }
      else
      {
        std::string profile =
            getProfileString(name_, base_instruction.getPathProfile(), request.plan_profile_remapping);
        plan_profile = getProfile<MMMOPlannerPlanProfile>(
            name_,
            profile,
            *request.profiles,
            std::make_shared<MMMOPlannerPlanProfile>(default_map.map_x, default_map.map_y, default_map.step_size));
        plan_profile = applyProfileOverrides(name_, profile, plan_profile, base_instruction.getProfileOverrides());
      }

      if (!plan_profile)
        throw std::runtime_error("MMMOMotionPlanner: Invalid profile");

      CompositeInstruction instruction_seed = plan_profile->generate(prev_instruction,
                                                                     prev_seed,
                                                                     base_instruction,
                                                                     next_instruction,
                                                                     request,
                                                                     request.instructions.getManipulatorInfo());
      seed.appendInstruction(instruction_seed);
      CONSOLE_BRIDGE_logDebug("3mo seed generation success");

      prev_instruction = base_instruction;
      prev_seed = instruction_seed.back().as<MoveInstructionPoly>();
    }
    else if (instruction.isMoveInstruction())
    {
      throw std::runtime_error("MMMOMotionPlanner: The input program includes MoveInstructions!");
    }
    else
    {
      seed.appendInstruction(instruction);
    }
  }  // for (const auto& instruction : instructions)
  return seed;
}

bool MMMOMotionPlanner::checkUserInput(const PlannerRequest& request)
{
  // Check that parameters are valid
  if (request.env == nullptr)
  {
    CONSOLE_BRIDGE_logError("In MMMOMotionPlanner: env is a required parameter and has not been set");
    return false;
  }

  if (request.instructions.empty())
  {
    CONSOLE_BRIDGE_logError("MMMOMotionPlanner requires at least one instruction");
    return false;
  }

  return true;
}

}  // namespace tesseract_planning
