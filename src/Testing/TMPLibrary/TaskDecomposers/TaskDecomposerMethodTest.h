#ifndef PPL_TASK_DECOMPOSER_METHOD_TEST_H_
#define PPL_TASK_DECOMPOSER_METHOD_TEST_H_

#include "TMPLibrary/TaskDecomposers/TaskDecomposerMethod.h"
#include "Testing/TestBaseObject.h"

class TaskDecomposerMethodTest : virtual public TaskDecomposerMethod,
                                 public TestBaseObject {

  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@} 
    ///@name Construction
    ///@{

    TaskDecomposerMethodTest();

    ~TaskDecomposerMethodTest();

    ///@} 
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@} 

  private:

    ///@name Test Interface Functions
    ///@{

    virtual TestResult DecompositionTest() = 0;

    ///@} 

};

#endif
