#ifndef PPL_TMP_STRATEGY_METHOD_H
#define PPL_TMP_STRATEGY_METHOD_H

#include "MPProblem/MPTask.h"

#include "TMPLibrary/TMPBaseObject.h"

class TMPStrategyMethod : public TMPBaseObject {
  public:

    ///@name Constructon
    ///@{

    TMPStrategyMethod() = default;

    TMPStrategyMethod(XMLNode& _node);

    virtual ~TMPStrategyMethod();

    ///@}
    ///@name Interface
    ///@{

    void operator()();

    ///@}
    ///@name Configure
    ///@{

    void Initialize() override;

    ///@}
  protected:

    ///@name Helper Methods
    ///@{

    /// Get plan for the input agents to perform the input tasks.
    /// _library needs to have the solution and problem set to the coordinator's
    /// values for these.
    virtual void PlanTasks();

    /// Split existing tasks into subtasks.
    virtual void DecomposeTasks();

    /// Assign subtasks to agents.
    virtual void AssignTasks();

    ///@}
    ///@name Member Variables
    ///@{

    std::string m_sgLabel; ///< State Graph Label

    std::string m_teLabel; ///< Task Evaluator Label

    std::string m_tdLabel; ///< Task Decomposer Label

    std::string m_taLabel; ///< Task Allocator Label

    ///@}
};

#endif
