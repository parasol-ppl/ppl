#ifndef PMPL_TASK_DECOMPOSER_METHOD_H_
#define PMPL_TASK_DECOMPOSER_METHOD_H_

#include "TMPLibrary/TMPBaseObject.h"

#include <iostream>

class TaskDecomposerMethod : public TMPBaseObject {
  public:

    ///@name Construction
    ///@{

    TaskDecomposerMethod() = default;

    TaskDecomposerMethod(XMLNode& _node);

    virtual ~TaskDecomposerMethod() = default;

    ///@}
    ///@name Interface
    ///@{

   //@todo: Convert this to semantic task
    // virtual void BreakupTask(WholeTask* _wholeTask);

    ///@}
};

/*----------------------------------------------------------------------------*/

#endif
