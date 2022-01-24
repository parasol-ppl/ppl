#ifndef PPL_LOCAL_PALLNER_METHOD_TEST_H_
#define PPL_LOCAL_PALLNER_METHOD_TEST_H_

#include "Testing/TestBaseObject.h"
#include "MPLibrary/LocalPlanners/LocalPlannerMethod.h"

template <typename MPTraits>
class LocalPlannerMethodTest : virtual public LocalPlannerMethod<MPTraits>,
                               public TestBaseObject {

  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult      TestResult;

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;

    ///@}
    ///@name Construction
    ///@{

    LocalPlannerMethodTest();

    ~LocalPlannerMethodTest();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;
    
    ///@}

  protected:

    ///@name Interface Test Functions
    ///@{

    virtual TestResult IndividualRobotIsConnectedTest() = 0;

    virtual TestResult IndividualRobotBlindPathTest() = 0;

    virtual TestResult RobotGroupIsConnectedTest() = 0;

    virtual TestResult RobotGroupBlindPathTest() = 0;

    ///@}
    ///@name Default Function Calls
    ///@{

    // Unfortunately, most of these will require setting up specific cases
    // for the inherted local planner method to find both a valid and an invalid
    // expected output. To create obstacles, you can setup another robot in between
    // the start and end configurations. Or we can make sure our FullUnitTest.xml
    // file uses an environment with some obstacles in it.

    virtual bool IndividualRobotIsConnected(const CfgType& _start, 
            const CfgType& _end, CfgType& _col, LPOutput<MPTraits>* _lpOutput);

    virtual std::vector<CfgType> IndividualRobotBlindPath();

    virtual bool RobotGroupIsConnected(const GroupCfgType& _start, 
            const GroupCfgType& _end, GroupCfgType& _col, LPOutput<MPTraits>* _lpOutput);

    virtual std::vector<GroupCfgType> RobotGroupBlindPath();

    ///@}
    ///@name Helper Functions
    ///@{

    void SetLibraryRobot();

    void SetLibraryGroup();

    CfgType GetIndividualCfg();

    GroupCfgType GetGroupCfg();

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
LocalPlannerMethodTest<MPTraits>::
LocalPlannerMethodTest() {}

template <typename MPTraits>
LocalPlannerMethodTest<MPTraits>::
~LocalPlannerMethodTest() {}

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>
typename LocalPlannerMethodTest<MPTraits>::TestResult
LocalPlannerMethodTest<MPTraits>::
RunTest() {
  
  bool passed = true;
  std::string message = "";
  
  auto result = IndividualRobotIsConnectedTest(); 
  passed = passed and result.first;
  message = message + result.second;

  result = IndividualRobotBlindPathTest();
  passed = passed and result.first;
  message = message + result.second;

  result = RobotGroupIsConnectedTest();
  passed = passed and result.first;
  message = message + result.second;

  result = RobotGroupBlindPathTest();
  passed = passed and result.first;
  message = message + result.second;

  return std::make_pair(passed,message);
}

/*----------------------- Default Function Calls ---------------------*/

template <typename MPTraits>
bool
LocalPlannerMethodTest<MPTraits>::
IndividualRobotIsConnected(const CfgType& _start, const CfgType& _end,
                           CfgType& _col, LPOutput<MPTraits>* _lpOutput) {

  auto env = this->GetMPProblem()->GetEnvironment();
  auto pos = env->GetPostionRes();
  auto ori = env->GetOrientationRes();
  bool checkCollision = true;
  bool savePath = true;

  return this->IsConnected(_start,_end,_col,_lpOutput,
                           pos,ori,checkCollision,savePath);
}

template <typename MPTraits>
std::vector<typename MPTraits::CfgType>
LocalPlannerMethodTest<MPTraits>::
IndividualRobotBlindPath() {

  // Set up an "L" shaped set of waypoints.
  auto start = GetIndividualCfg();

  auto middle = start;
  middle[0] = 10;

  auto end = middle;
  end[1] = 10;

  std::vector<CfgType> waypoints = {start,middle,end};

  auto env = this->GetMPProblem()->GetEnvironment();
  auto pos = env->GetPostionRes();
  auto ori = env->GetOrientationRes();

  return this->BlindPath(waypoints,pos,ori);
}

template <typename MPTraits>
bool
LocalPlannerMethodTest<MPTraits>::
RobotGroupIsConnected(const GroupCfgType& _start, 
            const GroupCfgType& _end, GroupCfgType& _col, LPOutput<MPTraits>* _lpOutput) {
  auto env = this->GetMPProblem()->GetEnvironment();
  auto pos = env->GetPostionRes();
  auto ori = env->GetOrientationRes();
  bool checkCollision = true;
  bool savePath = true;

  return this->IsConnected(_start,_end,_col,_lpOutput,
                           pos,ori,checkCollision,savePath);
}

template <typename MPTraits>
std::vector<typename MPTraits::GroupCfgType>
LocalPlannerMethodTest<MPTraits>::
RobotGroupBlindPath() {

  // Set up an "L" shaped set of waypoints for both robots.
  auto start = GetGroupCfg();
  auto& startCfg2 = start.GetRobotCfg(1);
  startCfg2[0] = 5;

  auto middle = start;
  for(size_t i = 0; i < middle.GetNumRobots(); i++) {
    auto& cfg = middle.GetRobotCfg(i);
    cfg[0] = cfg[0]+5;
  }

  auto end = middle;
  for(size_t i = 0; i < end.GetNumRobots(); i++) {
    auto& cfg = end.GetRobotCfg(i);
    cfg[1] = cfg[1]+5;
  }

  std::vector<GroupCfgType> waypoints = {start,middle,end};

  auto env = this->GetMPProblem()->GetEnvironment();
  auto pos = env->GetPostionRes();
  auto ori = env->GetOrientationRes();

  return this->BlindPath(waypoints,pos,ori);
}

/*-------------------------- Helper Functions ------------------------*/

template <typename MPTraits>
void
LocalPlannerMethodTest<MPTraits>::
SetLibraryRobot() {
  // Set MPLibrary to sample for single robot
  auto robot = this->GetMPProblem()->GetRobots()[0].get();
  auto task = this->GetMPProblem()->GetTasks(robot)[0];
  this->GetMPLibrary()->SetTask(task);
  this->GetMPLibrary()->SetGroupTask(nullptr);
}

template <typename MPTraits>
void
LocalPlannerMethodTest<MPTraits>::
SetLibraryGroup() {
  // Set MPLibrary to sample for robot group
  auto group = this->GetMPProblem()->GetRobotGroups()[0].get();
  auto task = this->GetMPProblem()->GetTasks(group)[0];
  this->GetMPLibrary()->SetGroupTask(task);
  this->GetMPLibrary()->SetTask(nullptr);
}

template <typename MPTraits>
typename MPTraits::CfgType
LocalPlannerMethodTest<MPTraits>::
GetIndividualCfg() {
  auto robot = this->GetMPProblem()->GetRobots()[0].get();
  CfgType cfg(robot);
  return cfg;
}

template <typename MPTraits>
typename MPTraits::GroupCfgType
LocalPlannerMethodTest<MPTraits>::
GetGroupCfg() {
  auto group = this->GetMPProblem()->GetRobotGroups()[0].get();
  GroupCfgType gcfg(group);
  return gcfg;
}

/*--------------------------------------------------------------------*/
#endif
