#include "SimpleMotionMethod.h"

#include "MPProblem/Robot/Robot.h"
#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"
#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/TaskEvaluators/TaskEvaluatorMethod.h"

#include <unordered_set>

/*------------------------- Construction ------------------------*/

SimpleMotionMethod::
SimpleMotionMethod() {
  this->SetName("SimpleMotionMethod");
}

SimpleMotionMethod::
SimpleMotionMethod(XMLNode& _node) : TMPStrategyMethod(_node) {
  this->SetName("SimpleMotionMethod");
}

SimpleMotionMethod::
~SimpleMotionMethod() {}

void
SimpleMotionMethod::
Initialize() {
  if(this->GetPlan()->GetDecomposition())
    return;

  // Create an intial Decomposition structure for the set of tasks
  Decomposition* decomp = new Decomposition();
  SemanticTask* top = new SemanticTask("top", nullptr, decomp,
                          SemanticTask::SubtaskRelation::AND, false, true);
  decomp->SetRootTask(top);

  std::unordered_set<Robot*> robots;
  for(auto mt : this->GetTMPLibrary()->GetTasks()) {
    robots.insert(mt->GetRobot());
    SemanticTask* task = new SemanticTask(mt->GetLabel(), top, decomp, mt);

    std::cout << "Creating semantic task for " << task->GetLabel() << std::endl;
  }

  this->GetPlan()->SetDecomposition(decomp);

}

/*****************************************Call Method****************************************************/

void
SimpleMotionMethod::
PlanTasks() {
  //TODO:: Move this to a simple motion plan task evaluator
  this->GetTaskEvaluator(m_teLabel)->operator()();
/*	auto decomp = this->GetPlan()->GetDecomposition();
  auto tasks = decomp->GetMotionTasks();
  auto problem = this->GetMPProblem();
  auto pmpl = this=>GetMPLibrary();

  for(auto task : tasks) {
    // Set up storage and call motion planner
    auto mt = task->GetMotionTask();
    MPSolution* mpSolution = new MPSolution(mt->GetRobot());
    if(mt->GetStatus().is_complete())
      continue;
    pmpl->Solve(problem, mt.get(), mpSolution);

    // Save solution in plan
    auto solution = std::shared_ptr<TaskSolution>(new TaskSolution(task));
    solution->SetRobot(mt->GetRobot());
    solution->SetMotionSolution(mpSolution);
    plan->AddAllocation(mt->GetRobot(), task);
    plan->SetTaskSolution(task, solution);
  }
*/
  //TODO:: MultiRobot Plans
}
