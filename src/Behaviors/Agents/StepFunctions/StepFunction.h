#ifndef _PPL_STEP_FUNCTION_H_
#define _PPL_STEP_FUNCTION_H_

//////////////////////////////////////////////////////////////////
///
/// This is a virtual base class for agent functions.
/// Agent behavior will be customizeable by the step
/// function behavior.
///
///////////////////////////////////////////////////////////////////

#include "Behaviors/Agents/Agent.h"

#include <memory>

class StepFunction {

  public :
    ///@name Construction
    ///@{

    StepFunction(Agent* _agent, XMLNode& _node);

    ~StepFunction();

    /// Create a dynamically-allocated agent from an XML node.
    /// @param _r The robot which this agent will reason for.
    /// @param _node The XML node to parse.
    /// @return An agent of the type specified by _node.
    static std::unique_ptr<StepFunction> Factory(Agent* const _a, XMLNode& _node);

    ///@}
    ///@name Interface
    ///@{

    /// Function to call the step function behavior
    virtual void StepAgent(double _dt) = 0;

    ///@}

  protected:
    ///@name Internal State
    ///@{

    Agent* m_agent;

    bool m_debug{false};
    ///@}
};

#endif
