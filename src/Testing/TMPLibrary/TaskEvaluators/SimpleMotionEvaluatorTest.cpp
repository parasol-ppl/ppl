#include "SimpleMotionEvaluatorTest.h"

/*--------------------------- Construction ---------------------------*/

SimpleMotionEvaluatorTest::
SimpleMotionEvaluatorTest() {}

SimpleMotionEvaluatorTest::
SimpleMotionEvaluatorTest(XMLNode& _node) : SimpleMotionEvaluator(_node) {}

SimpleMotionEvaluatorTest::
~SimpleMotionEvaluatorTest() {}

/*---------------------- Test Interface Functions --------------------*/

SimpleMotionEvaluatorTest::TestResult
SimpleMotionEvaluatorTest::
TaskEvaluatorTest() {
  return TestResult();
}

/*--------------------------------------------------------------------*/
