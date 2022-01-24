#ifndef _PPL_SIMPLE_MOTION_METHOD_H_
#define _PPL_SIMPLE_MOTION_METHOD_H_

#include "TMPStrategyMethod.h"

class SimpleMotionMethod : public TMPStrategyMethod {
  public:

    ///@name Construction
    ///@{

    SimpleMotionMethod();

    SimpleMotionMethod(XMLNode& _node);

    ~SimpleMotionMethod();

    virtual void Initialize() override;

    ///@}
    ///@name Accessors
    ///@{

    ///@}
    ///@name Call Method
    ///@{

    /// Get plan for the input agents to perform the input tasks.
    /// _library needs to have the solution and problem set to the coordinator's
    /// values for these.
    virtual void PlanTasks() override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    ///@}
};

#endif
