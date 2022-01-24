#include "DefaultCoordinatorStepFunction.h"

#include "ConfigurationSpace/Cfg.h"

#include "MPLibrary/MPSolution.h"

#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/TaskHierarchy/Decomposition.h"

#include "TMPLibrary/Solution/TaskSolution.h"

/*----------------------------- Construction -------------------------*/

DefaultCoordinatorStepFunction::
DefaultCoordinatorStepFunction(Coordinator* _coordinator, XMLNode& _node) :
             StepFunction(_coordinator, _node) {

  m_coordinator = static_cast<Coordinator*>(m_agent);
}

DefaultCoordinatorStepFunction::
~DefaultCoordinatorStepFunction() { }

/*------------------------------- Interface --------------------------*/

void
DefaultCoordinatorStepFunction::
StepAgent(double _dt) {

  if(HasPlan()) {
    StepMemberAgents(_dt);
  }
  else if(HasProblem() and GetPlan()) {
    DistributePlan();
  }
}

/*---------------------------- Helper Functions  ----------------------*/

bool
DefaultCoordinatorStepFunction::
HasProblem() {

  // If we already have a set problem, return true
  if(m_decomposition)
    return true;

  // Otherwise, check if there is an incomplete decomposition
  // and, if so, assign it
  auto robot = m_coordinator->GetRobot();
  auto problem = robot->GetMPProblem();
  auto& decompositions = problem->GetDecompositions(robot);

  for(auto& decomp : decompositions) {
    if(decomp->IsComplete())
      continue;
    m_decomposition = decomp.get();
    return true;
  }

  return false;
}

bool
DefaultCoordinatorStepFunction::
HasPlan() {
  return m_plan.get();
}

bool
DefaultCoordinatorStepFunction::
GetPlan() {

  auto robot = m_coordinator->GetRobot();
  auto problem = robot->GetMPProblem();

  // Use tmplibrary to get task assignments
  m_plan = std::shared_ptr<Plan>(new Plan());
  m_plan->SetCoordinator(m_coordinator);

  std::vector<Robot*> team;
  for(auto agent : m_coordinator->GetChildAgents()) {
    team.push_back(agent->GetRobot());
  }

  m_plan->SetTeam(team);
  m_plan->SetDecomposition(problem->GetDecompositions(robot)[0].get());

  if(!m_tmpLibrary)
    m_tmpLibrary = m_coordinator->GetTMPLibrary();

  m_tmpLibrary->Solve(problem, problem->GetDecompositions(robot)[0].get(),
                      m_plan.get(), m_coordinator, team);

  if(m_debug and m_plan) {
    std::cout << "SOLUTION PLAN" << std::endl;
    m_plan->Print();
  }

  return m_plan.get();
}

void
DefaultCoordinatorStepFunction::
StepMemberAgents(double _dt) { }

void
DefaultCoordinatorStepFunction::
DistributePlan() {

  auto childAgents = m_coordinator->GetChildAgents();
  //auto memberAgents = m_coordinator->GetMemberAgents();

  for(auto agent : childAgents) {
    auto allocs = m_plan->GetAllocations(agent->GetRobot());
    std::vector<TaskSolution*> solutions;

    //Temporary dummy task
    std::shared_ptr<MPTask> temp = std::shared_ptr<MPTask>(new MPTask(agent->GetRobot()));
    agent->SetTask(temp);

    for(auto iter = allocs.begin(); iter != allocs.end(); iter++) {
      auto sol = m_plan->GetTaskSolution(*iter);
      solutions.push_back(sol);
      auto mpSol = sol->GetMotionSolution();

      if(iter == allocs.begin()) {
        agent->GetMPSolution()->SetRoadmap(agent->GetRobot(),mpSol->GetRoadmap());
        agent->GetMPSolution()->SetPath(agent->GetRobot(),mpSol->GetPath(agent->GetRobot()));
      }
      else {
        *(agent->GetMPSolution()->GetPath(agent->GetRobot())) += *(mpSol->GetPath());
      }
    }
    auto& cfgs = agent->GetMPSolution()->GetPath()->Cfgs();
    agent->SetPlan(cfgs);
    if(cfgs.empty())
      continue;
    auto goalCfg = cfgs.back();
    std::unique_ptr<CSpaceConstraint> goal =
      std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(agent->GetRobot(), goalCfg));
    temp->AddGoalConstraint(std::move(goal));
  }
}
