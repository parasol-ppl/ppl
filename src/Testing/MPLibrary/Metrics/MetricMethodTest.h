#ifndef PPL_METRIC_METHOD_TEST_H
#define PPL_METRIC_METHOD_TEST_

#include "MPLibrary/Metrics/MetricMethod.h"
#include "Testing/TestBaseObject.h"

template <typename MPTraits>
class MetricMethodTest : virtual public MetricMethod<MPTraits>,
                         public TestBaseObject {

  public:
  
    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    MetricMethodTest();

    ~MetricMethodTest();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}

  protected:

    ///@name Interface Test Function
    ///@{

    virtual TestResult MetricTest() = 0;

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
MetricMethodTest<MPTraits>::
MetricMethodTest() : MetricMethod<MPTraits>() { }

template <typename MPTraits>
MetricMethodTest<MPTraits>::
~MetricMethodTest() { }

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>
typename MetricMethodTest<MPTraits>::TestResult
MetricMethodTest<MPTraits>::
RunTest() {
  
  bool passed = true;
  std::string message = "";

  auto result = MetricTest();
  passed = passed and result.first;
  message = message + result.second;

  return std::make_pair(passed,message);
}

/*--------------------------------------------------------------------*/

#endif
