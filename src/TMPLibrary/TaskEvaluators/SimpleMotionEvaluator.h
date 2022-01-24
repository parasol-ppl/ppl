#ifndef _PPL_SIMPLE_MOTION_EVALUATOR_H_
#define _PPL_SIMPLE_MOTION_EVALUATOR_H_

#include <unordered_map>

#include "TMPLibrary/TaskEvaluators/TaskEvaluatorMethod.h"

class SimpleMotionEvaluator : virtual public TaskEvaluatorMethod {
  public:

    ///@name Constructor
    ///@{

    SimpleMotionEvaluator();

    SimpleMotionEvaluator(XMLNode& _node);

    ~SimpleMotionEvaluator();

    ///@}
    ///@name Call method
    ///@{

    ///@}
  private:

    virtual bool Run(Plan* _plan = nullptr) override;

};

#endif
