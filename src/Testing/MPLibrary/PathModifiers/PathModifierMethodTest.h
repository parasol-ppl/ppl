#ifndef PPL_PATH_MODIFIER_METHOD_TEST_H_
#define PPL_PATH_MODIFIER_METHOD_TEST_H_

#include "MPLibrary/PathModifiers/PathModifierMethod.h"
#include "Testing/TestBaseObject.h"

template <typename MPTraits>
class PathModifierMethodTest : virtual public PathModifierMethod<MPTraits>,
                               public TestBaseObject {

  public:

    ///@name LocalTypes 
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    PathModifierMethodTest();

    ~PathModifierMethodTest();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}

  protected:

    ///@name Interface Test Functions
    ///@{

    virtual TestResult TestModify() = 0;

    ///@}
    ///@name Default Function Calls
    ///@{

    // No default function for now because you would pass all the of same parameters in
    // to call the interface function.

    ///@}

};


/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
PathModifierMethodTest<MPTraits>::
PathModifierMethodTest() { }

template <typename MPTraits>
PathModifierMethodTest<MPTraits>::
~PathModifierMethodTest() { }

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>
typename PathModifierMethodTest<MPTraits>::TestResult
PathModifierMethodTest<MPTraits>::
RunTest() {

  bool passed = true;
  std::string message = "";

  auto result  = TestModify();
  passed = passed and result.first;
  message = message + result.second;

  return std::make_pair(passed,message);
}

/*--------------------------------------------------------------------*/

#endif
