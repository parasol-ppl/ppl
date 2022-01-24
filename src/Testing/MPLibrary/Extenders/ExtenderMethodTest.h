#ifndef PPL_EXTENDER_METHOD_TEST_H_
#define PPL_EXTENDER_METHOD_TEST_H_

#include "MPLibrary/Extenders/ExtenderMethod.h"
#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPLibrary/LocalPlanners/GroupLPOutput.h"
#include "Testing/TestBaseObject.h"

template <typename MPTraits>
class ExtenderMethodTest : virtual public ExtenderMethod<MPTraits>,
                           public TestBaseObject {

  public:

    ///@name Local Types
    ///@{
    
    typedef TestBaseObject::TestResult TestResult;
    
    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;

    ///@}
    ///@name Construction
    ///@{
    
    ExtenderMethodTest();

    ~ExtenderMethodTest();
    
    ///@}
    ///@name Interface
    ///@{
    
    virtual TestResult RunTest() override;
    
    ///@}


  protected:

    ///@name Interface Test Function 
    ///@{
    
    virtual TestResult IndividualRobotExtendTests() = 0;

    virtual TestResult RobotGroupExtendTests() = 0;
    
    ///@}
    ///@name Default Function Calls 
    ///@{
    
    virtual bool IndividualRobotExtend(CfgType& _new, 
                        LPOutput<MPTraits>& _lpOutput);

    virtual bool RobotGroupExtend(GroupCfgType& _new,
                        GroupLPOutput<MPTraits>& _lpOutput);
    
    ///@}
    ///@name Helper Functions
    ///@{
    
    CfgType GetIndividualCfg();

    GroupCfgType GetGroupCfg();
    
    ///@}

};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
ExtenderMethodTest<MPTraits>::
ExtenderMethodTest() {}

template <typename MPTraits>
ExtenderMethodTest<MPTraits>::
~ExtenderMethodTest() {}

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>
typename ExtenderMethodTest<MPTraits>::TestResult
ExtenderMethodTest<MPTraits>::
RunTest() {
  
  bool passed = true;
  std::string message = "";

  auto result = IndividualRobotExtend();
  passed = passed and result.first;
  message = message + result.second;

  result = RobotGroupExtend();
  passed = passed and result.first;
  message = message + result.second;

  return std::make_pair(passed,message);
}

/*----------------------- Default Function Calls ---------------------*/

template <typename MPTraits>
bool
ExtenderMethodTest<MPTraits>::
IndividualRobotExtend(CfgType& _new, LPOutput<MPTraits>& _lpOutput) {
  CfgType start = GetIndividualCfg();
  CfgType goal = start;
  goal[0] = 10;

  return this->Extend(start,goal,_new,_lpOutput);
}

template <typename MPTraits>
bool
ExtenderMethodTest<MPTraits>::
RobotGroupExtend(GroupCfgType& _new, GroupLPOutput<MPTraits>& _lpOutput) {

  auto start = GetGroupCfg();
  auto& startCfg2 = start.GetRobotCfg(1);
  startCfg2[0] = 5;

  auto goal = start;
  for(size_t i = 0; i < goal.GetNumRobots(); i++) {
    auto& cfg = goal.GetRobotCfg(i);
    cfg[1] = cfg[1]+5;
  }

  return this->Extend(start,goal,_new,_lpOutput);
}

/*-------------------------- Helper Functions ------------------------*/

template <typename MPTraits>
typename MPTraits::CfgType
ExtenderMethodTest<MPTraits>::
GetIndividualCfg() {
  auto robot = this->GetMPProblem()->GetRobots()[0].get();
  CfgType cfg(robot);
  return cfg;
}

template <typename MPTraits>
typename MPTraits::GroupCfgType
ExtenderMethodTest<MPTraits>::
GetGroupCfg() {
  auto group = this->GetMPProblem()->GetRobotGroups()[0].get();
  GroupCfgType gcfg(group);
  return gcfg;
}

/*--------------------------------------------------------------------*/

#endif
