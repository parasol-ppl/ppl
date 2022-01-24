#include "TaskEvaluatorMethodTest.h"

/*--------------------------- Construction ---------------------------*/

TaskEvaluatorMethodTest::
TaskEvaluatorMethodTest() { }

TaskEvaluatorMethodTest::
~TaskEvaluatorMethodTest() { }

/*---------------------------- Interface -----------------------------*/

TaskEvaluatorMethodTest::TestResult
TaskEvaluatorMethodTest::
RunTest() { 

  bool passed = true;
  std::string message = "";

  auto result = TaskEvaluatorTest();
  passed = passed and result.first;
  message = message + result.second;

  return std::make_pair(passed,message);
}

/*--------------------------------------------------------------------*/
