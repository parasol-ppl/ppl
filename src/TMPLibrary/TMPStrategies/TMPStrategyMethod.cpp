#include "TMPStrategyMethod.h"

#include "Behaviors/Agents/Agent.h"
#include "Behaviors/Agents/Coordinator.h"

#include "TMPLibrary/PoIPlacementMethods/PoIPlacementMethod.h"
#include "TMPLibrary/StateGraphs/StateGraph.h"
#include "TMPLibrary/Solution/Plan.h"

#include "Simulator/Simulation.h"
#include "Utilities/MetricUtils.h"


/*-------------------------------- Constructor --------------------------------*/
TMPStrategyMethod::
TMPStrategyMethod(XMLNode& _node) : TMPBaseObject(_node) {
  m_sgLabel = _node.Read("sgLabel", false, "",
                         "Label for the state graph used by the TMPStrategy");
  m_teLabel = _node.Read("teLabel", false, "",
                         "Label for the task evaluator used by the TMPStrategy");
  m_tdLabel = _node.Read("tdLabel", false, "",
                         "Label for the task decomposer used by the TMPStrategy");
  m_taLabel = _node.Read("taLabel", false, "",
                         "Label for the task allocator used by the TMPStrategy");
}

TMPStrategyMethod::
~TMPStrategyMethod() {
}

/*---------------------------------- Configure ---------------------------------*/

void
TMPStrategyMethod::
Initialize() {
}

/*--------------------------------- Call Method ---------------------------------*/
void
TMPStrategyMethod::
operator()() {
  this->GetPlan()->GetStatClass()->StartClock("TotalTMPStrategyTime");
  Initialize();
  PlanTasks();
  DecomposeTasks();
  AssignTasks();
  this->GetPlan()->GetStatClass()->StopClock("TotalTMPStrategyTime");
}

/*---------------------------------- Accessors ----------------------------------*/

/*----------------------------- Combined Roadmap --------------------------------*/

void
TMPStrategyMethod::
PlanTasks() {}

void
TMPStrategyMethod::
AssignTasks() {}

void
TMPStrategyMethod::
DecomposeTasks() {}
