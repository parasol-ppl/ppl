#include "PathFollowingAgent.h"

#include <limits>

#include "Behaviors/Controllers/ControllerMethod.h"
#include "MPProblem/Robot/Robot.h"
#include "Simulator/BulletModel.h"
#include "Simulator/Simulation.h"

/*------------------------------ Construction --------------------------------*/

PathFollowingAgent::
PathFollowingAgent(Robot* const _r) : PlanningAgent(_r) { }


PathFollowingAgent::
PathFollowingAgent(Robot* const _r, const PathFollowingAgent& _a)
  : PlanningAgent(_r, _a)
{ }

PathFollowingAgent::
PathFollowingAgent(Robot* const _r, XMLNode& _node) : PlanningAgent(_r, _node) {
  m_waypointDm = _node.Read("waypointDm", true, "",
      "The distance metric to use for checking proximity to a path waypoint.");

  m_waypointThreshold = _node.Read("waypointThreshold", false,
      m_waypointThreshold, 0., std::numeric_limits<double>::max(),
      "The robot is considered to have reached waypoints within this threshold.");
}

std::unique_ptr<Agent>
PathFollowingAgent::
Clone(Robot* const _r) const {
  return std::unique_ptr<PathFollowingAgent>(new PathFollowingAgent(_r, *this));
}

PathFollowingAgent::
~PathFollowingAgent() {
  PathFollowingAgent::Uninitialize();
}

/*------------------------------ Agent Interface -----------------------------*/

void
PathFollowingAgent::
Uninitialize() {
  if(!m_initialized)
    return;

  ClearVisualGraph();

  PlanningAgent::Uninitialize();

  ClearPlan();
}

void
PathFollowingAgent::
ClearVisualGraph() {
  if(Simulation::Get() and m_graphVisualID != size_t(-1))
  {
    Simulation::Get()->RemoveRoadmap(m_graphVisualID);
    m_graphVisualID = size_t(-1);
  }
}

/*--------------------------------- Planning ---------------------------------*/

bool
PathFollowingAgent::
HasPlan() const {
  return !m_path.empty();
}

void
PathFollowingAgent::
ClearPlan() {
  // Remove the path visual if needed.
  if(HasPlan() and Simulation::Get() and m_pathVisualID != size_t(-1)) {
    Simulation::Get()->RemovePath(m_pathVisualID);
    m_pathVisualID = size_t(-1);
  }
  // Clear path data.
  PlanningAgent::ClearPlan();
  m_path.clear();
  m_pathIndex = 0;
}

void
PathFollowingAgent::
SetPlan(std::vector<Cfg> _path) {
  m_path = _path;
}

/*----------------------------- Planning Helpers -----------------------------*/

void
PathFollowingAgent::
WorkFunction(std::shared_ptr<MPProblem> _problem) {
  PlanningAgent::WorkFunction(_problem);

  // Extract the planned path from the solution.
  m_path = m_solution->GetPath()->Cfgs();
  m_pathIndex = 0;

  // Throw if PMPL failed to generate a solution.
  /// @TODO Develop a model for what to when PMPL fails. We would probably want
  ///       at least options like 'retry' and 'throw'.
  if(m_path.empty())
    throw RunTimeException(WHERE, "PMPL failed to produce a solution.");

  // Add a path visualization to the simulation.
  m_pathVisualID = Simulation::Get()->AddPath(m_path, glutils::color::red);
}

/*------------------------------- Task Helpers -------------------------------*/

bool
PathFollowingAgent::
EvaluateTask() {
  //hack for the moment to handle multi-task edge case where delivering robot checks receiving 
  //robot's task at the moment the receiving robot is delivering a different task.
	if(m_path.empty())
		return true;
  // Get the current configuration.
  const Cfg current = m_robot->GetSimulationModel()->GetState();

  // We consider the robot to have reached the next subgoal if it is within a
  // threshold distance. Advance the path index until the next subgoal is
  // at least one threshold away.
  auto dm = m_library->GetDistanceMetric(m_waypointDm);

  double distance = dm->Distance(current, m_path[m_pathIndex]);

  if(m_debug)
    std::cout << "\tDistance from current configuration: "
              << distance << "/" << m_waypointThreshold
              << std::endl;

  // Check if the agent has completed its task. If so, clear out any remaining
  // plan data and mark the task complete.
  if(GetTask()->EvaluateGoalConstraints(current) and !GetTask()->GetGoalConstraints().empty()) {
    if(m_debug)
      std::cout << "Reached the end of the path." << std::endl;

    GetTask()->GetStatus().complete();
    SetTask(nullptr);
    //if(m_CUSTOM_PATH){
    //  this->PauseAgent(300);
    //}
    return false;
  }

  // Advance our subgoal while we are within the distance threshold of the next
  // one.
  while(distance < m_waypointThreshold and !m_path.empty()) {
    if(m_debug)
      std::cout << "\tReached waypoint " << m_pathIndex << " at "
                << distance << "/" << m_waypointThreshold
                << "\n\t\t" << m_path[m_pathIndex].PrettyPrint()
                << std::endl;

    // Move to next cfg in path since the distance is within the threshold.
    ++m_pathIndex;

    // Check if we have completed the path. If so, this task is complete.
    if(m_pathIndex == m_path.size()) {
      if(m_debug)
        std::cout << "Reached the end of the path." << std::endl;
      GetTask()->GetStatus().complete();
      SetTask(nullptr);
      return false;
    }

    distance = dm->Distance(current, m_path[m_pathIndex]);
  }

  return true;
}

void
PathFollowingAgent::
ExecuteTask(const double _dt) {
  if(m_debug)
    std::cout << "Approaching waypoint " << m_pathIndex << " / "
              << m_path.size() - 1 << "."
              << std::endl;

  // Get the smallest safe time interval in case _dt is too fast for the hardware.
  const size_t steps = std::max(Simulation::NearestNumSteps(_dt), MinimumSteps());
  const double timeRes = m_robot->GetMPProblem()->GetEnvironment()->GetTimeRes(),
               time = timeRes * steps;

  // Ask the controller for the best action to get from the current position to
  // the next waypoint.
  const Cfg current = m_robot->GetSimulationModel()->GetState();
  auto bestControl = m_robot->GetController()->operator()(current,
      m_path[m_pathIndex], time);

  if(m_debug){
    std::cout << "Printing m_path in agent step function" << std::endl;
    for(auto cfg : m_path){
      std::cout << cfg.PrettyPrint() << std::endl;
    }
  }

  ExecuteControls({bestControl}, steps);
}

/*----------------------------------------------------------------------------*/
