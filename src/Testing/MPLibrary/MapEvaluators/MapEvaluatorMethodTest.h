#ifndef PPL_MAP_EVALUATOR_METHOD_TEST_H_
#define PPL_MAP_EVALUATOR_METHOD_TEST_H_

#include "MPLibrary/MapEvaluators/MapEvaluatorMethod.h"
#include "Testing/TestBaseObject.h"

#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

template <typename MPTraits>
class MapEvaluatorMethodTest : virtual public MapEvaluatorMethod<MPTraits>,
                               public TestBaseObject {

  public: 
    ///@name Local Types
    ///@{

    typedef typename MPTraits::RoadmapType      RoadmapType;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    MapEvaluatorMethodTest();

    ~MapEvaluatorMethodTest();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}

  protected:
    ///@name Interface Test Function
    ///@{

    virtual TestResult MainFunctionTest() = 0;

    ///@}
    ///@name Default Function Calls
    ///@{

    virtual bool IndividualRobotMainFunction(RoadmapType* _roadmap, 
                                             MPTask* _task);

    virtual bool GroupRobotMainFunction(GroupRoadmapType* _roadmap, 
                                        GroupTask* _task);

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
MapEvaluatorMethodTest<MPTraits>::
MapEvaluatorMethodTest() : MapEvaluatorMethod<MPTraits>() { }

template <typename MPTraits>
MapEvaluatorMethodTest<MPTraits>::
~MapEvaluatorMethodTest() { }

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>
typename MapEvaluatorMethodTest<MPTraits>::TestResult
MapEvaluatorMethodTest<MPTraits>::
RunTest() {
  
  bool passed = true;
  std::string message = "";

  auto result = MainFunctionTest();
  passed = passed and result.first;
  message = message + result.second;

  return std::make_pair(passed,message);
}

/*----------------------- Default Function Calls ---------------------*/

template <typename MPTraits>
bool
MapEvaluatorMethodTest<MPTraits>::
IndividualRobotMainFunction(RoadmapType* _roadmap, MPTask* _task) {

  // Initialize MPLibrary with input variables
  auto lib = this->GetMPLibrary();
  lib->SetTask(_task);
  lib->SetGroupTask(nullptr);

  // Setup goal tracker for test roadmap and task
  this->GetMPLibrary()->GetGoalTracker()->AddMap(_roadmap, _task);

  auto robot = _task->GetRobot();
  auto solution = lib->GetMPSolution();
  solution->AddRobot(robot);
  solution->SetRoadmap(robot,_roadmap);

  return this->operator()();
}

template <typename MPTraits>
bool
MapEvaluatorMethodTest<MPTraits>::
GroupRobotMainFunction(GroupRoadmapType* _roadmap, GroupTask* _task) {

  // Initialize MPLibrary with input variables
  auto lib = this->GetMPLibrary();
  lib->SetGroupTask(_task);
  lib->SetTask(nullptr);
  
  auto group = _task->GetRobotGroup();
  auto solution = lib->GetMPSolution();

  for(auto robot : group->GetRobots()) {
    solution->AddRobot(robot);
  }

  solution->AddRobotGroup(group);
  solution->SetGroupRoadmap(group,_roadmap);

  return this->operator()();
}

/*--------------------------------------------------------------------*/

#endif
