#include "TaskDecomposerMethodTest.h"

/*--------------------------- Construction ---------------------------*/

TaskDecomposerMethodTest::
TaskDecomposerMethodTest() { }

TaskDecomposerMethodTest::
~TaskDecomposerMethodTest() { }

/*---------------------------- Interface -----------------------------*/

TaskDecomposerMethodTest::TestResult
TaskDecomposerMethodTest::
RunTest() { 

  bool passed = true;
  std::string message = "";

  auto result = DecompositionTest();
  passed = passed and result.first;
  message = message + result.second;

  return std::make_pair(passed,message);
}

/*--------------------------------------------------------------------*/
