#include "SimpleMotionEvaluator.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"
#include "MPProblem/TaskHierarchy/SemanticTask.h"
#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/Solution/TaskSolution.h"

SimpleMotionEvaluator::
SimpleMotionEvaluator() {
  this->SetName("SimpleMotionEvaluator");
}

SimpleMotionEvaluator::
SimpleMotionEvaluator(XMLNode& _node) : TaskEvaluatorMethod(_node) {
  this->SetName("SimpleMotionEvaluator");
}

SimpleMotionEvaluator::
~SimpleMotionEvaluator() {}

bool
SimpleMotionEvaluator::
Run(Plan* _plan) {

  Plan* plan;

  if(_plan)
    plan = _plan;
  else
    plan = this->GetPlan();

  auto decomp = plan->GetDecomposition();
  auto tasks = decomp->GetMotionTasks();
  auto groupTasks = decomp->GetGroupMotionTasks();
  auto problem = this->GetMPProblem();
  auto pmpl = this->GetMPLibrary();

  // Call motion planner for motion tasks
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

  // Call motion planner for group motion tasks
  for(auto task : groupTasks) {
    // Set up storage and call motion planner
    auto gt = task->GetGroupMotionTask();
    MPSolution* mpSolution = new MPSolution(gt->GetRobotGroup());
    pmpl->Solve(problem, gt.get(), mpSolution);

    // Save solution in plan
    auto solution = std::shared_ptr<TaskSolution>(new TaskSolution(task));
    solution->SetRobotGroup(gt->GetRobotGroup());
    solution->SetMotionSolution(mpSolution);
    plan->AddAllocation(gt->GetRobotGroup(), task);
    plan->SetTaskSolution(task, solution);
  }
  return true;
}
