#ifndef PMPL_TASK_EVALUATOR_METHOD_H_
#define PMPL_TASK_EVALUATOR_METHOD_H_

#include "TMPLibrary/TMPBaseObject.h"

#include <iostream>

class TaskEvaluatorMethod : public TMPBaseObject {
  public:

    ///@name Construction
    ///@{

    TaskEvaluatorMethod();

    TaskEvaluatorMethod(XMLNode& _node);

    virtual ~TaskEvaluatorMethod();

    ///@}
    ///@name MapEvaluator Interface
    ///@{

    /// Evaluate a stateGraph.
    /// @return True if this stateGraph meets the evaluation criteria.
    bool operator()(Plan* _plan = nullptr);

    ///@}
  protected:
    ///Exectute
    ///@param _plan pointer
    ///@return True if exectuion is successful
    virtual bool Run(Plan* _plan = nullptr);

    std::string m_sgLabel; ///< StateGraph Label

};

/*----------------------------------------------------------------------------*/

#endif
