#ifndef PMPL_TASK_ALLOCATOR_METHOD_H_
#define PMPL_TASK_ALLOCATOR_METHOD_H_

#include "TMPLibrary/TMPBaseObject.h"

#include <iostream>

class TaskAllocatorMethod : public TMPBaseObject {
  public:

    ///@name Construction
    ///@{

    TaskAllocatorMethod() = default;

    TaskAllocatorMethod(XMLNode& _node);

    virtual ~TaskAllocatorMethod() = default;

    ///@}
    ///@name Call Method
    ///@{

    virtual void AllocateTasks();

    ///@}
};

/*----------------------------------------------------------------------------*/

#endif
