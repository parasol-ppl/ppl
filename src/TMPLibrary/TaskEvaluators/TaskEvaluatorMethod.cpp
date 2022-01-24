#include "TaskEvaluatorMethod.h"

#include "TMPLibrary/Solution/Plan.h"
#include "Simulator/Simulation.h"
#include "Utilities/MetricUtils.h"
/*------------------------------ Construction --------------------------------*/

TaskEvaluatorMethod::
TaskEvaluatorMethod() {}

TaskEvaluatorMethod::
TaskEvaluatorMethod(XMLNode& _node) : TMPBaseObject(_node) {
  m_sgLabel = _node.Read("sgLabel", false, "",
                         "Label for the state graph used by the TMPStrategy");
}

TaskEvaluatorMethod::
~TaskEvaluatorMethod() {}

bool
TaskEvaluatorMethod::
operator()(Plan* _plan) {
  this->GetPlan()->GetStatClass()->StartClock("TaskEvaluationTime");
  auto ret = Run(_plan);
  this->GetPlan()->GetStatClass()->StopClock("TaskEvaluationTime");
  return ret;
}

bool
TaskEvaluatorMethod::
Run(Plan* _plan) {
  return false;
}
