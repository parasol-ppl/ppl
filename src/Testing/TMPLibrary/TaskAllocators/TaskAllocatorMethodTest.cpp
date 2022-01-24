#include "TaskAllocatorMethodTest.h"

/*--------------------------- Construction ---------------------------*/

TaskAllocatorMethodTest::
TaskAllocatorMethodTest() { }

TaskAllocatorMethodTest::
~TaskAllocatorMethodTest() { }

/*---------------------------- Interface -----------------------------*/

TaskAllocatorMethodTest::TestResult
TaskAllocatorMethodTest::
RunTest() { 

  bool passed = true;
  std::string message = "";

  auto result = AllocationTest();
  passed = passed and result.first;
  message = message + result.second;

  return std::make_pair(passed,message);
}

/*--------------------------------------------------------------------*/
