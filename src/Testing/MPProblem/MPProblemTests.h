#ifndef PPL_MP_PROBLEM_TESTS_H_
#define PPL_MP_PROBLEM_TESTS_H_

#include "Testing/TestBaseObject.h"
#include "MPProblem/MPProblem.h"

class MPProblemTests : public MPProblem, public TestBaseObject {
  public:
    ///@name LocalTypes
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    MPProblemTests();

    MPProblemTests(const std::string& _xmlFile);

    virtual ~MPProblemTests();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}
};

#endif
