#ifndef PPL_SIMPLE_MOTION_EVALUATOR_TEST_H_
#define PPL_SIMPLE_MOTION_EVALUATOR_TEST_H_

#include "Testing/TMPLibrary/TaskEvaluators/TaskEvaluatorMethodTest.h"
#include "TMPLibrary/TaskEvaluators/SimpleMotionEvaluator.h"

class SimpleMotionEvaluatorTest : virtual public SimpleMotionEvaluator, 
                                  public TaskEvaluatorMethodTest {
  public:
 
    ///@name LocalTypes
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    SimpleMotionEvaluatorTest();

    SimpleMotionEvaluatorTest(XMLNode& _node);

    virtual ~SimpleMotionEvaluatorTest();

    ///@}
    
  private: 
    
    ///@} Test Interface Functions
    ///@{

    virtual TestResult TaskEvaluatorTest();

    ///@}
};

#endif
