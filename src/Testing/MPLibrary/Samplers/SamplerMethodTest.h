#ifndef PPL_SAMPLER_METHOD_TEST_H_
#define PPL_SAMPLER_METHOD_TEST_H_

#include "MPLibrary/Samplers/SamplerMethod.h"
#include "Testing/TestBaseObject.h"

template <typename MPTraits>
class SamplerMethodTest : virtual public SamplerMethod<MPTraits>, 
                          public TestBaseObject {
  public:

    ///@name Local Types
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    SamplerMethodTest();

    ~SamplerMethodTest();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}

  protected:

    ///@name Interface Test Functions
    ///@{

    virtual TestResult TestIndividualCfgSample() = 0;

    virtual TestResult TestIndividualCfgSampleWithEEConstraint() = 0;

    virtual TestResult TestIndividualFilter() = 0;

    virtual TestResult TestGroupCfgSampleSingleBoundary() = 0;

    virtual TestResult TestGroupCfgSampleIndividualBoundaries() = 0;

    virtual TestResult TestGroupFilter() = 0;

    ///@}
    ///@name Default Function Calls
    ///@{

    //TODO::Determine what the proper return type is as these functions
    //      get implemented. Each one should call the corresponding interface
    //      function in the SamplerMethod class and return the same output.
    //      The function should instantiate whatever input variables are 
    //      needed to test the underlying SamplerMethod function.

    virtual void IndividualCfgSample(Boundary*& _boundary, 
                    std::vector<Cfg>& _valids, std::vector<Cfg>& _invalids);

    virtual void IndividualCfgSampleWithEEConstraint(Boundary*& _boundary, 
                    Boundary* _eeBoundary, std::vector<Cfg>& _valids, 
                    std::vector<Cfg>& _invalids);

    virtual void IndividualFilter(std::vector<Cfg> _input, Boundary*& _boundary,
                    std::vector<Cfg>& _valids, std::vector<Cfg>& _invalids);

    virtual void GroupCfgSampleSingleBoundary(Boundary*& _boundary, 
                    std::vector<GroupCfg>& _valids, std::vector<GroupCfg>& _invalids);

    virtual void GroupCfgSampleIndividualBoundaries(std::map<Robot*,const Boundary*>& _boundaryMap, 
                    std::vector<GroupCfg>& _valids, std::vector<GroupCfg>& _invalids);

    virtual void GroupFilter(std::vector<GroupCfg> _input, Boundary*& _boundary,
      std::vector<GroupCfg>& _valids, std::vector<GroupCfg>& _invalids);

    ///@}
    ///@name Helper Functions
    ///@{

    void SetLibraryRobot();

    void SetLibraryGroup();

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
SamplerMethodTest<MPTraits>::
SamplerMethodTest() : SamplerMethod<MPTraits>() {}

template <typename MPTraits>
SamplerMethodTest<MPTraits>::
~SamplerMethodTest() {}

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>
typename SamplerMethodTest<MPTraits>::TestResult
SamplerMethodTest<MPTraits>::
RunTest() {

  bool passed = true;
  std::string message = "";

  auto result = TestIndividualCfgSample();
  passed = passed and result.first;
  message = message + result.second;

  result = TestIndividualCfgSampleWithEEConstraint();
  passed = passed and result.first;
  message = message + result.second;

  result = TestIndividualFilter();
  passed = passed and result.first;
  message = message + result.second;

  result = TestGroupCfgSampleSingleBoundary();
  passed = passed and result.first;
  message = message + result.second;

  result = TestGroupCfgSampleIndividualBoundaries();
  passed = passed and result.first;
  message = message + result.second;

  result = TestGroupFilter();
  passed = passed and result.first;
  message = message + result.second;

  return std::make_pair(passed,message);
}

/*----------------------- Default Function Calls ---------------------*/

template <typename MPTraits>
void
SamplerMethodTest<MPTraits>::
IndividualCfgSample(Boundary*& _boundary, std::vector<Cfg>& _valids,
                    std::vector<Cfg>& _invalids) {

  SetLibraryRobot();

  size_t numNodes = 10;
  size_t maxAttempts = 100;

  if(!_boundary)
    _boundary = this->GetMPProblem()->GetEnvironment()->GetBoundary();

  this->Sample(numNodes,maxAttempts,_boundary,std::back_inserter(_valids),
               std::back_inserter(_invalids));
}

template <typename MPTraits>
void
SamplerMethodTest<MPTraits>::
IndividualCfgSampleWithEEConstraint(Boundary*& _boundary, 
            Boundary* _eeBoundary, std::vector<Cfg>& _valids, 
            std::vector<Cfg>& _invalids) {

  SetLibraryRobot();

  // Check if robot has end effector
  auto robot = this->GetMPProblem()->GetRobots()[0].get();
  if(!robot->GetEndEffector().effectorBody)
    return;

  size_t numNodes = 10;
  size_t maxAttempts = 100;

  if(!_boundary)
    _boundary = this->GetMPProblem()->GetEnvironment()->GetBoundary();

  this->Sample(numNodes,maxAttempts,_boundary,_eeBoundary,
               std::back_inserter(_valids), std::back_inserter(_invalids));
}

template <typename MPTraits>
void
SamplerMethodTest<MPTraits>::
IndividualFilter(std::vector<Cfg> _input, Boundary*& _boundary, 
                 std::vector<Cfg>& _valids, std::vector<Cfg>& _invalids) {

  SetLibraryRobot();

  size_t maxAttempts = 100;

  if(!_boundary)
    _boundary = this->GetMPProblem()->GetEnvironment()->GetBoundary();

  this->Filter(_input.begin(), _input.end(), maxAttempts, _boundary,
               std::back_inserter(_valids), std::back_inserter(_invalids));
}

template <typename MPTraits>
void
SamplerMethodTest<MPTraits>::
GroupCfgSampleSingleBoundary(Boundary*& _boundary, std::vector<GroupCfg>& _valids,
      std::vector<GroupCfg>& _invalids) {

  SetLibraryGroup();

  size_t numNodes = 10;
  size_t maxAttempts = 100;

  if(!_boundary)
    _boundary = this->GetMPProblem()->GetEnvironment()->GetBoundary();

  this->Sample(numNodes,maxAttempts,_boundary,std::back_inserter(_valids), 
               std::back_inserter(_invalids));
}

template <typename MPTraits>
void
SamplerMethodTest<MPTraits>::
GroupCfgSampleIndividualBoundaries(std::map<Robot*,const Boundary*>& _boundaryMap, 
      std::vector<GroupCfg>& _valids, std::vector<GroupCfg>& _invalids) {

  SetLibraryGroup();

  size_t numNodes = 10;
  size_t maxAttempts = 100;

  this->Sample(numNodes,maxAttempts,_boundaryMap,std::back_inserter(_valids), 
               std::back_inserter(_invalids));
}

template <typename MPTraits>
void
SamplerMethodTest<MPTraits>::
GroupFilter(std::vector<GroupCfg> _input, Boundary*& _boundary,
      std::vector<GroupCfg>& _valids, std::vector<GroupCfg>& _invalids) {

  SetLibraryGroup();

  size_t maxAttempts = 100;
  
  this->Filter(_input.begin(), _input.end(), maxAttempts, _boundary, 
               std::back_inserter(_valids), std::back_inserter(_invalids));
}

/*-------------------------- Helper Functions ------------------------*/

template <typename MPTraits>
void
SamplerMethodTest<MPTraits>::
SetLibraryRobot() {
  // Set MPLibrary to sample for single robot
  auto robot = this->GetMPProblem()->GetRobots()[0].get();
  auto task = this->GetMPProblem()->GetTasks(robot)[0].get();
  this->GetMPLibrary()->SetTask(task);
  this->GetMPLibrary()->SetGroupTask(nullptr);
}

template <typename MPTraits>
void
SamplerMethodTest<MPTraits>::
SetLibraryGroup() {
  // Set MPLibrary to sample for robot group
  auto group = this->GetMPProblem()->GetRobotGroups()[0].get();
  auto task = this->GetMPProblem()->GetTasks(group)[0].get();
  this->GetMPLibrary()->SetGroupTask(task);
  this->GetMPLibrary()->SetTask(nullptr);
}

/*--------------------------------------------------------------------*/
#endif
