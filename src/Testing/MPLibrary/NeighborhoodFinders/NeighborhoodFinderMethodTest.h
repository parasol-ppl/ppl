#ifndef PPL_NEIGHBORHOOD_FINDER_METHOD_TEST_H_
#define PPL_NEIGHBORHOOD_FINDER_METHOD_TEST_H_

#include "MPLibrary/NeighborhoodFinders/NeighborhoodFinderMethod.h"
#include "Testing/TestBaseObject.h"

template <typename MPTraits>
class NeighborhoodFinderMethodTest : virtual public NeighborhoodFinderMethod<MPTraits>,
                                     public TestBaseObject {

  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    NeighborhoodFinderMethodTest();

    ~NeighborhoodFinderMethodTest();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}

  protected:

    ///@name Interface Test Functions
    ///@{
  
    virtual TestResult IndividualRobotFindNeighborsTest() = 0;

    virtual TestResult RobotGroupFindNeighborsTest() = 0;

    ///@}
    ///@name Default Function Calls
    ///@{

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>    
NeighborhoodFinderMethodTest<MPTraits>::
NeighborhoodFinderMethodTest() : NeighborhoodFinderMethod<MPTraits>() { }

template <typename MPTraits>    
NeighborhoodFinderMethodTest<MPTraits>::
~NeighborhoodFinderMethodTest() { }

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>    
typename NeighborhoodFinderMethodTest<MPTraits>::TestResult
NeighborhoodFinderMethodTest<MPTraits>::
RunTest() {

  bool passed = true;
  std::string message = "";

  auto result = IndividualRobotFindNeighborsTest();
  passed = passed and result.first;
  message = message + result.second;

  result = RobotGroupFindNeighborsTest();
  passed = passed and result.first;
  message = message + result.second;

  return std::make_pair(passed,message);
}

/*--------------------------------------------------------------------*/
#endif
