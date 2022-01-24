#ifndef _PPL_DEFAULT_COORDINATOR_STEP_FUNCTION_H_
#define _PPL_DEFAULT_COORDINATOR_STEP_FUNCTION_H_

//////////////////////////////////////////////////////////////////
///
/// The defines the default behavior for a multi-agent coordinator.
/// If there is a problem and no plan, the planning library will
/// be queried. The plan will then be distributed to individual
/// agents in the system.
/// When there is a plan, the coordinator will step each agent
/// along its respective plan to keep everything in sync.
///
///////////////////////////////////////////////////////////////////

#include "Behaviors/Agents/Coordinator.h"
#include "Behaviors/Agents/StepFunctions/StepFunction.h"

#include "MPProblem/TaskHierarchy/Decomposition.h"

#include "TMPLibrary/Solution/Plan.h"
#include "TMPLibrary/TMPLibrary.h"

class DefaultCoordinatorStepFunction : public StepFunction {

  public :
    ///@name Construction
    ///@{

    DefaultCoordinatorStepFunction(Coordinator* _coordinator, XMLNode& _node);

    ~DefaultCoordinatorStepFunction();

    ///@}
    ///@name Interface
    ///@{

    virtual void StepAgent(double _dt) override;

    ///@}

  private:
    ///@name Helper Functions
    ///@name {

    bool HasProblem();

    bool HasPlan();

    bool GetPlan();

    void StepMemberAgents(double _dt);

    void DistributePlan();

    ///@}
    ///@name Internal State
    ///@{

    Coordinator* m_coordinator;

    TMPLibrary* m_tmpLibrary{nullptr};

    bool m_debug;

    std::shared_ptr<Plan> m_plan{nullptr};

    Decomposition* m_decomposition{nullptr};

    ///@}
};

#endif
