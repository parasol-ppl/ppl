#ifndef PPL_TASK_ALLOCATOR_METHOD_TEST_H_
#define PPL_TASK_ALLOCATOR_METHOD_TEST_H_

#include "TMPLibrary/TaskAllocators/TaskAllocatorMethod.h"
#include "Testing/TestBaseObject.h"

class TaskAllocatorMethodTest : virtual public TaskAllocatorMethod,
                                 public TestBaseObject {

  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@} 
    ///@name Construction
    ///@{

    TaskAllocatorMethodTest();

    ~TaskAllocatorMethodTest();

    ///@} 
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@} 

  private:

    ///@name Test Interface Functions
    ///@{

    virtual TestResult AllocationTest() = 0;

    ///@} 

};

#endif
