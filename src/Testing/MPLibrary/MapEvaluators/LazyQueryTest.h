#ifndef PPL_LAZY_QUERY_H_
#define PPL_LAZY_QUERY_H_

#include "MPLibrary/MapEvaluators/LazyQuery.h"
#include "Testing/MPLibrary/MapEvaluators/MapEvaluatorMethodTest.h"
#include "Utilities/MPUtils.h"
#include "MPLibrary/MPSolution.h"

#include "Vector.h"

namespace mathtool {
  class EulerAngle;
  class Transformation;
}

template <typename MPTraits>
class LazyQueryTest : virtual public LazyQuery<MPTraits>,
                      public MapEvaluatorMethodTest<MPTraits> {
  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::RoadmapType      RoadmapType;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::WeightType       WeightType;


    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Construction
    ///@{

    LazyQueryTest();

    LazyQueryTest(XMLNode& _node);

    ~LazyQueryTest();

    ///@}

  private:

    ///@name Interface Test Function
    ///@{

    virtual TestResult MainFunctionTest() override;

    ///@}

    ///@name Helper Functions
    ///@{

    /// Construct a roadmap to test that edges and nodes get
    /// invalidated properly for a single robot
    void SingleSetup();

    ///@}

    ///@name Helper Members
    ///@{

    /// Single helper members
    RoadmapType* m_singleRoadmap;
    MPTask*      m_singleTask;

    ///@}
};

/*--------------------------- Construction ---------------------------*/
template <typename MPTraits>
LazyQueryTest<MPTraits>::
LazyQueryTest() : QueryMethod<MPTraits>(), LazyQuery<MPTraits>() {
}

template <typename MPTraits>
LazyQueryTest<MPTraits>::
LazyQueryTest(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node), QueryMethod<MPTraits>(_node), LazyQuery<MPTraits>(_node) {
}

template <typename MPTraits>
LazyQueryTest<MPTraits>::
~LazyQueryTest() {}

/*----------------------Interface Test Functions ---------------------*/

template <typename MPTraits>
typename LazyQueryTest<MPTraits>::TestResult
LazyQueryTest<MPTraits>::
MainFunctionTest() {

  bool passed = true;
  std::string message = "";

  // Test Individual Robot Functionality
  SingleSetup();

  auto singleResult = this->IndividualRobotMainFunction(m_singleRoadmap,
                                                        m_singleTask);

  if(!singleResult) {
    passed = false;
    message = message + "\n\tIndividual robot functionality testing failed ";
  }

  if(!passed) {
    message = "MainFunctionTest::FAILED :(\n" + message;
    return std::make_pair(passed, message);
  }

  return std::make_pair(passed, message);
}

/*--------------------------Helper Functions -------------------------*/

template <typename MPTraits>
void
LazyQueryTest<MPTraits>::
SingleSetup() {
  // Get Robot
  auto robot = MapEvaluatorMethodTest<MPTraits>::GetMPProblem()->GetRobots()[0].get();

  /// Roadmap Construction
  RoadmapType* roadmap = new RoadmapType(robot);

  // Construct vertices and add to roadmap

  // Start & Goal: Valid robot configurations.
  auto p1 = CfgType(robot);
  std::istringstream p1Stream("0 0 0 0 0 0 0");
  p1.Read(p1Stream);
  auto start = roadmap->AddVertex(p1);

  auto p2 = CfgType(robot);
  std::istringstream p2Stream("0 20 -10 0 .3 .7 .8");
  p2.Read(p2Stream);
  auto goal = roadmap->AddVertex(p2);

  // Vertex 1: Out of boundary vertex.
  auto v1P = CfgType(robot);
  std::istringstream v1Stream("0 0 -50 0 0 0 0");
  v1P.Read(v1Stream);
  auto v1 = roadmap->AddVertex(v1P);

  // Vertex 2: In boundary vertex.
  auto v2P = CfgType(robot);
  std::istringstream v2Stream("0 20 0 0 0 0 0");
  v2P.Read(v2Stream);
  auto v2 = roadmap->AddVertex(v2P);

  // Add edges to roadmap with respective weights

  // Edge 1: Passes through object in enviornment
  // Tests to ensure edges are invalidated properly
  roadmap->AddEdge(start, goal, WeightType("a",3.0));

  // Edge 2 & 3: Passes through vertex out of environment
  // boundary. Tests to ensure vertices are invalidated
  // properly.
  roadmap->AddEdge(start, v1, WeightType("b",1.0));
  roadmap->AddEdge(v1, goal, WeightType("c",1.0));

  // Edge 4 & 5: Valid path to goal. Tests to ensure query
  // method can find path after failing first to searches.
  roadmap->AddEdge(start, v2, WeightType("d",4.0));
  roadmap->AddEdge(v2, goal, WeightType("e",4.0));

  /// Task Construction
  auto task = this->GetMPLibrary()->GetTask();
  task->SetRobot(robot);

  // Set robot start and goal constraints
  auto startConstraint = std::unique_ptr<CSpaceConstraint>(
    new CSpaceConstraint(robot, p1));

  auto goalConstraint  = std::unique_ptr<CSpaceConstraint>(
    new CSpaceConstraint(robot, p2));

  task->SetStartConstraint(std::move(startConstraint));
  task->AddGoalConstraint(std::move(goalConstraint));

  m_singleRoadmap = roadmap;
  m_singleTask    = task;
}

#endif
