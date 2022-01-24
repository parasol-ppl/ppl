#include "Plan.h"

#include "Behaviors/Agents/Coordinator.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"
#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"
#include "TaskSolution.h"
#include "Utilities/MetricUtils.h"

/*--------------------------- Construction -----------------------------*/

Plan::
Plan() {
  m_statClass = std::unique_ptr<StatClass>(new StatClass());
}

Plan::
~Plan() {}
/*---------------------------- Accessors -------------------------------*/

/// Coordinator
void
Plan::
SetCoordinator(Coordinator* _coordinator) {
  m_coordinator = _coordinator;
}

Coordinator*
Plan::
GetCoordinator() const {
  return m_coordinator;
}

/// Robot Team
void
Plan::
SetTeam(std::vector<Robot*> _team) {
  m_team = _team;
}

const std::vector<Robot*>&
Plan::
GetTeam() const {
  return m_team;
}

/// Decomposition
void
Plan::
SetDecomposition(Decomposition* _decomp) {
  m_decomposition = _decomp;
}

Decomposition*
Plan::
GetDecomposition() const {
  return m_decomposition;
}

/// Task Allocations
void
Plan::
ClearAllocations(Robot* _robot) {
  m_allocations[_robot].clear();
}

void
Plan::
ClearAllocations(RobotGroup* _group) {
  m_groupAllocations[_group].clear();
}

void
Plan::
AddAllocation(Robot* _robot, SemanticTask* _task) {
  m_allocations[_robot].push_back(_task);
}

void
Plan::
AddAllocation(RobotGroup* _group, SemanticTask* _task) {
  m_groupAllocations[_group].push_back(_task);
}

std::list<SemanticTask*>
Plan::
GetAllocations(Robot* _robot) {
  return m_allocations[_robot];
}

std::list<SemanticTask*>
Plan::
GetAllocations(RobotGroup* _group) {
  return m_groupAllocations[_group];
}

/// Task Plans
void
Plan::
SetTaskSolution(SemanticTask* _task, std::shared_ptr<TaskSolution> _solution) {
  m_taskSolutions[_task] = _solution;

  auto& allocs = m_allocations[_solution->GetRobot()];
  auto iter = std::find(allocs.begin(), allocs.end(), _task);
  if(iter != allocs.end()) {
    allocs.erase(iter);
  }
  for(iter = allocs.begin(); iter != allocs.end(); iter++) {
    auto& sol = m_taskSolutions[*iter];
    if(sol->GetStartTime() > _solution->GetStartTime()) {
      allocs.insert(iter,_task);
    }
  }
  if(allocs.empty())
    allocs.push_back(_task);
}

TaskSolution*
Plan::
GetTaskSolution(SemanticTask* _task) {
  return m_taskSolutions[_task].get();
}

std::vector<TaskSolution*>
Plan::
GetRobotTaskSolutions(Robot* _robot) {
  std::vector<TaskSolution*> solutions;

  for(auto task : m_allocations[_robot]) {
    solutions.push_back(m_taskSolutions[task].get());
  }

  return solutions;
}

const std::unordered_map<SemanticTask*,std::shared_ptr<TaskSolution>>&
Plan::
GetTaskSolutions() {
  return m_taskSolutions;
}

/// MPProblem
void
Plan::
SetMPProblem(MPProblem* _problem) {
  m_problem = _problem;
}

MPProblem*
Plan::
GetMPProblem() {
  return m_problem;
}

/// StatClass
StatClass*
Plan::
GetStatClass() {
  return m_statClass.get();
}

void
Plan::
Print() {
  for(auto sol : m_taskSolutions) {
    sol.second->Print();
  }
}
