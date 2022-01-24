#ifndef PPL_TASK_EVALUATOR_METHOD_TEST_H_
#define PPL_TASK_EVALUATOR_METHOD_TEST_H_

#include "TMPLibrary/TaskEvaluators/TaskEvaluatorMethod.h"
#include "Testing/TestBaseObject.h"

class TaskEvaluatorMethodTest : virtual public TaskEvaluatorMethod,
                                public TestBaseObject {

  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    TaskEvaluatorMethodTest();

    ~TaskEvaluatorMethodTest();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

  protected: 

    ///@} Test Interface Functions
    ///@{

    virtual TestResult TaskEvaluatorTest() = 0;

    ///@}

};

#endif
