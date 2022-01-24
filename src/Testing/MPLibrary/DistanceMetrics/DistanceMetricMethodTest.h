#ifndef PPL_DISTANCE_METRIC_METHOD_TEST_H_
#define PPL_DISTANCE_METRIC_METHOD_TEST_H_

#include "MPLibrary/DistanceMetrics/DistanceMetricMethod.h"
#include "Testing/TestBaseObject.h"

template <typename MPTraits>
class DistanceMetricMethodTest : public DistanceMetricMethod<MPTraits>,
                                 public TestBaseObject {

  public: 

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;
    typedef typename MPTraits::RoadmapType  RoadmapType;

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    DistanceMetricMethodTest();

    ~DistanceMetricMethodTest();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}

  protected:

    ///@name Interface Test Functions
    ///@{

    virtual TestResult TestIndividualCfgDistance() = 0;

    virtual TestResult TestIndividualEdgeWeight() = 0;

    virtual TestResult TestIndividualScaleCfg() = 0;

    virtual TestResult TestGroupCfgDistance() = 0;

    virtual TestResult TestGroupEdgeWeight() = 0;

    virtual TestResult TestGroupScaleCfg() = 0;

    ///@}
    ///@name Default Function Calls
    ///@{

    virtual double IndividualCfgDistance();

    virtual double IndividualEdgeWeight();

    virtual CfgType IndividualScaleCfg();

    virtual double GroupCfgDistance();

    virtual double GroupEdgeWeight();

    virtual GroupCfgType GroupScaleCfg();

    ///@}
    ///@name Helper Functions
    ///@{
  
    CfgType GetIndividualCfg();

    GroupCfgType GetGroupCfg();

    ///@}

};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
DistanceMetricMethodTest<MPTraits>::
DistanceMetricMethodTest() {}

template <typename MPTraits>
DistanceMetricMethodTest<MPTraits>::
~DistanceMetricMethodTest() {}

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>
typename DistanceMetricMethodTest<MPTraits>::TestResult
DistanceMetricMethodTest<MPTraits>::
RunTest() {

  bool passed = true;
  std::string message = "";

  auto result = TestIndividualCfgDistance();
  passed = passed and result.first;
  message = message + result.second;

  result = TestIndividualEdgeWeight();
  passed = passed and result.first;
  message = message + result.second;

  result = TestIndividualScaleCfg();
  passed = passed and result.first;
  message = message + result.second;

  result = TestGroupCfgDistance();
  passed = passed and result.first;
  message = message + result.second;

  result = TestGroupEdgeWeight();
  passed = passed and result.first;
  message = message + result.second;

  result = TestGroupScaleCfg();
  passed = passed and result.first;
  message = message + result.second;

  return std::make_pair(passed,message); 
}

/*----------------------- Default Function Calls ---------------------*/

template <typename MPTraits>
double
DistanceMetricMethodTest<MPTraits>::
IndividualCfgDistance() {
  auto cfg1 = GetIndividualCfg();
  auto cfg2 = GetIndividualCfg();
  
  for(size_t i = 0; i < cfg2.PosDOF(); i++) {
    cfg2[i] = 5;
  }

  for(size_t i = cfg2.PosDOF(); i < cfg2.DOF(); i++) {
    cfg2[i+cfg2.PosDOF()] = .5;
  }

  return this->Distance(cfg1,cfg2);
}

template <typename MPTraits>
double
DistanceMetricMethodTest<MPTraits>::
IndividualEdgeWeight() {

  // Initialize roadmap for test
  auto robot = this->GetMPProblem()->GetRobots()[0].get();
  RoadmapType roadmap(robot);

  // Get cfgs for edge
  auto cfg1 = GetIndividualCfg();
  auto cfg2 = GetIndividualCfg();
  
  for(size_t i = 0; i < cfg2.PosDOF(); i++) {
    cfg2[i] = 5;
  }

  for(size_t i = cfg2.PosDOF(); i < cfg2.DOF(); i++) {
    cfg2[i+cfg2.PosDOF()] = .5;
  }

  // Add cfgs to roadmap
  auto first = roadmap.AddVertex(cfg1);
  auto second = roadmap.AddVertex(cfg2);

  // Add edge
  DefaultWeight<CfgType> weight;
  roadmap.AddEdge(first,second,weight);

  // Call distance metric method
  return this->EdgeWeight(&roadmap,first,second);
}

template <typename MPTraits>
typename MPTraits::CfgType
DistanceMetricMethodTest<MPTraits>::
IndividualScaleCfg() {

  // Grab initial variables
  double length = 10;
  auto cfg = GetIndividualCfg();

  // Increase each dof from 0, so that scaling has an effect
  for(size_t i = 0; i < cfg.DOF(); i++) {
    cfg[i] = 0.01;
  }

  // Call distance metric method
  this->Scale(length,cfg);

  return cfg; 
}

template <typename MPTraits>
double
DistanceMetricMethodTest<MPTraits>::
GroupCfgDistance() {
  auto gcfg1 = GetGroupCfg();
  auto gcfg2 = GetGroupCfg();
  
  for(size_t i = 0; i < gcfg2.GetNumRobots(); i++) {

    // Grab individual cfg
    auto& cfg = gcfg2.GetRobotCfg(i);

    // Adjust individual cfg
    for(size_t j = 0; j < cfg.PosDOF(); j++) {
      cfg[j] = 5;
    }
    
    for(size_t j = cfg.PosDOF(); j < cfg.DOF(); j++) {
      cfg[j+cfg.PosDOF()] = .5;
    }
  }

  // Call distance metric method
  return this->Distance(gcfg1,gcfg2);
}

template <typename MPTraits>
double
DistanceMetricMethodTest<MPTraits>::
GroupEdgeWeight() {
  // TODO::Implement this default function.
  // This function does not currently exists in the 
  // distance metric method.
  return 0;
}

template <typename MPTraits>
typename MPTraits::GroupCfgType
DistanceMetricMethodTest<MPTraits>::
GroupScaleCfg() {

  double length = 10;
  auto gcfg = GetGroupCfg();

  // Increase each dof from 0, so that scaling has an effect
  for(size_t i = 0; i < gcfg.GetNumRobots(); i++) {

    // Grab individual robot cfg
    auto& cfg = gcfg.GetRobotCfg(i);

    // Update dofs
    for(size_t j = 0; j < cfg.DOF(); j++) {
      cfg[j] = 0.01;
    }
  }

  this->Scale(length,gcfg);

  return gcfg; 
}

/*-------------------------- Helper Functions ------------------------*/
  
template <typename MPTraits>
typename MPTraits::CfgType
DistanceMetricMethodTest<MPTraits>::
GetIndividualCfg() {
  auto robot = this->GetMPProblem()->GetRobots()[0].get();
  CfgType cfg(robot);
  return cfg;
}

template <typename MPTraits>
typename MPTraits::GroupCfgType
DistanceMetricMethodTest<MPTraits>::
GetGroupCfg() {
  auto group = this->GetMPProblem()->GetRobotGroups()[0].get();
  GroupCfgType gcfg(group);
  return gcfg;
}

/*--------------------------------------------------------------------*/

#endif
