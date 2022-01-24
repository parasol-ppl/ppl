#include "MPProblemTests.h"

/*--------------------------- Construction ---------------------------*/

MPProblemTests::
MPProblemTests() {}

MPProblemTests::
MPProblemTests(const std::string& _xmlFile) : MPProblem(_xmlFile) {}

MPProblemTests::
~MPProblemTests() {}
/*----------------------------- Interface ----------------------------*/

typename MPProblemTests::TestResult
MPProblemTests::
RunTest() {
  return TestResult();
}

/*--------------------------------------------------------------------*/
