#ifndef PPL_BOUNDINGSPHERESCOLLISIONDETECTION_TEST_H_
#define PPL_BOUNDINGSPHERESCOLLISIONDETECTION_TEST_H_

#include "MPLibrary/MPLibrary.h"
#include "Testing/TestBaseObject.h"

#include "Testing/MPLibrary/DistanceMetrics/DistanceMetricMethodTest.h"
#include "Testing/MPLibrary/ValidityCheckers/ValidityCheckerMethodTest.h"
#include "Testing/MPLibrary/NeighborhoodFinders/NeighborhoodFinderMethodTest.h"
#include "Testing/MPLibrary/Samplers/SamplerMethodTest.h"
#include "Testing/MPLibrary/LocalPlanners/LocalPlannerMethodTest.h"
#include "Testing/MPLibrary/Extenders/ExtenderMethodTest.h"
//#include "Testing/MPLibrary/PathModifiers/PathModifierMethodTest.h"
#include "Testing/MPLibrary/Connectors/ConnectorMethodTest.h"
#include "Testing/MPLibrary/Metrics/MetricMethodTest.h"
#include "Testing/MPLibrary/MapEvaluators/MapEvaluatorMethodTest.h"

#include "Testing/MPLibrary/ValidityCheckers/CollisionDetection/CollisionDetectionMethodTest.h"
#include "Testing/MPLibrary/ValidityCheckers/CollisionDetection/BoundingSpheresCollisionDetectionTest.h"
#include "Testing/MPLibrary/ValidityCheckers/CollisionDetection/InsideSpheresCollisionDetectionTest.h"



template <typename MPTraits>
class MPLibraryTests : public MPLibraryType<MPTraits>, public TestBaseObject {
  public:
    ///@LocalTypes
    ///@{

    typedef MPLibraryType<MPTraits>    MPLibrary;
    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Method Set Types
    ///@{

    typedef MethodSet<MPTraits, DistanceMetricMethodTest<MPTraits>> DistanceMetricTestSet;
    typedef MethodSet<MPTraits, ValidityCheckerMethodTest<MPTraits>>
                                                                    ValidityCheckerTestSet;
    typedef MethodSet<MPTraits, NeighborhoodFinderMethodTest<MPTraits>>
                                                                    NeighborhoodFinderTestSet;
    typedef MethodSet<MPTraits, SamplerMethodTest<MPTraits>>        SamplerTestSet;
    typedef MethodSet<MPTraits, LocalPlannerMethodTest<MPTraits>>   LocalPlannerTestSet;
    typedef MethodSet<MPTraits, ExtenderMethodTest<MPTraits>>       ExtenderTestSet;
    //typedef MethodSet<MPTraits, PathModifierMethodTest<MPTraits>>   PathModifierTestSet;
    typedef MethodSet<MPTraits, ConnectorMethodTest<MPTraits>>     ConnectorTestSet;
    typedef MethodSet<MPTraits, MetricMethodTest<MPTraits>>        MetricTestSet;
    typedef MethodSet<MPTraits, MapEvaluatorMethodTest<MPTraits>>   MapEvaluatorTestSet;

    ///@}
    ///@name Construction
    ///@{

    MPLibraryTests();

    MPLibraryTests(const std::string& _xmlFile);

    virtual ~MPLibraryTests();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    void InitializeMethodSets();

    void InitializeCollisionDetectionMethodTests();

    template <typename MethodTypeList>
    void RunMethodSetTests(const MethodTypeList& _mtl,size_t& _passed,
                           size_t& failed, size_t& _total);

    void RunCollisionDetectionMethodTests(size_t& _passed, size_t& failed,
                                          size_t& _total);

    ///@name XML Helpers
    ///@{

    void ReadXMLFile(const std::string& _filename);

    bool ParseChild(XMLNode& _node);

    ///@}
    ///@name Internal State
    ///@{

    bool verbose{true};
    std::map<std::string, CollisionDetectionMethodTest*> m_collisionDetectionTests;

    ///@}
    ///@name Method Sets
    ///@{
    /// Method sets hold and offer access to the motion planning objects of the
    /// corresponding type.

    DistanceMetricTestSet*     m_distanceMetricTests{nullptr};
    ValidityCheckerTestSet*    m_validityCheckerTests{nullptr};
    NeighborhoodFinderTestSet* m_neighborhoodFinderTests{nullptr};
    SamplerTestSet*            m_samplerTests{nullptr};
    LocalPlannerTestSet*       m_localPlannerTests{nullptr};
    ExtenderTestSet*           m_extenderTests{nullptr};
    //PathModifierTestSet*       m_pathModifierTests{nullptr};
    ConnectorTestSet*          m_connectorTests{nullptr};
    MetricTestSet*             m_metricTests{nullptr};
    MapEvaluatorTestSet*       m_mapEvaluatorTests{nullptr};

    ///@}
};

/*--------------------------- Construction ---------------------------*/

template <typename MPTraits>
MPLibraryTests<MPTraits>::
MPLibraryTests() {
  InitializeMethodSets();
}

template <typename MPTraits>
MPLibraryTests<MPTraits>::
MPLibraryTests(const std::string& _xmlFile) : MPLibraryType<MPTraits>(_xmlFile) {
  InitializeMethodSets();
  ReadXMLFile(_xmlFile);
}

template <typename MPTraits>
MPLibraryTests<MPTraits>::
~MPLibraryTests() {}

/*----------------------------- Interface ----------------------------*/

template <typename MPTraits>
typename MPLibraryTests<MPTraits>::TestResult
MPLibraryTests<MPTraits>::
RunTest() {

  // Init mpsolution for stat purposes (and avoiding seg faults)
  this->SetMPSolution(new MPSolutionType<MPTraits>(this->GetMPProblem()->GetRobots()[0].get()));

  size_t passed = 0;
  size_t failed = 0;
  size_t total = 0;

  // Collision detection tests
  InitializeCollisionDetectionMethodTests();
  RunCollisionDetectionMethodTests(passed, failed, total);


  // Distance metric tests
  RunMethodSetTests(*this->m_distanceMetricTests,passed,failed,total);

  // Validity checker tests
  RunMethodSetTests(*this->m_validityCheckerTests,passed,failed,total);

  // Neighborhood finder tests
  RunMethodSetTests(*this->m_neighborhoodFinderTests,passed,failed,total);

  // Sampler tests
  RunMethodSetTests(*this->m_samplerTests,passed,failed,total);

  // Local planner tests
  RunMethodSetTests(*this->m_localPlannerTests,passed,failed,total);

  // Extender tests
  RunMethodSetTests(*this->m_extenderTests,passed,failed,total);

  // Path modifier tests
  //RunMethodSetTests(*this->m_pathModifierTests,passed,failed,total);

  // Connector tests
  RunMethodSetTests(*this->m_connectorTests,passed,failed,total);

  // Metric tests
  RunMethodSetTests(*this->m_metricTests,passed,failed,total);

  // Map evaluator tests
  RunMethodSetTests(*this->m_mapEvaluatorTests,passed,failed,total);


  bool success = (failed == 0);
  std::string message = "COMPLETED TESTS"
                        "\nTotal: "  + std::to_string(total)  +
                        "\nPassed: " + std::to_string(passed) +
                        "\nFailed: " + std::to_string(failed) +
                        "\n\n\n";


  return std::make_pair(success,message);
}

/*-------------------------- Helper Functions ------------------------*/

template <typename MPTraits>
void
MPLibraryTests<MPTraits>::
InitializeMethodSets() {
  m_distanceMetricTests = new DistanceMetricTestSet(this,
      typename MPTraits::DistanceMetricMethodList(), "DistanceMetrics");
  m_validityCheckerTests = new ValidityCheckerTestSet(this,
      typename MPTraits::ValidityCheckerMethodList(), "ValidityCheckers");
  m_neighborhoodFinderTests = new NeighborhoodFinderTestSet(this,
      typename MPTraits::NeighborhoodFinderMethodList(), "NeighborhoodFinders");
  m_samplerTests = new SamplerTestSet(this,
      typename MPTraits::SamplerMethodList(), "Samplers");
  m_localPlannerTests = new LocalPlannerTestSet(this,
      typename MPTraits::LocalPlannerMethodList(), "LocalPlanners");
  m_extenderTests = new ExtenderTestSet(this,
      typename MPTraits::ExtenderMethodList(), "Extenders");
  //m_pathModifierTests = new PathModifierTestSet(this,
  //    typename MPTraits::PathModifierMethodList(), "PathModifiers");
  m_connectorTests = new ConnectorTestSet(this,
      typename MPTraits::ConnectorMethodList(), "Connectors");
  m_metricTests = new MetricTestSet(this,
      typename MPTraits::MetricMethodList(), "Metrics");
  m_mapEvaluatorTests = new MapEvaluatorTestSet(this,
      typename MPTraits::MapEvaluatorMethodList(), "MapEvaluators");
}

template<typename MPTraits>
void
MPLibraryTests<MPTraits>::
InitializeCollisionDetectionMethodTests() {
  // TODO: add addition collision detection methods here
  BoundingSpheresCollisionDetectionTest* boundingSpheres = nullptr;
  boundingSpheres = new BoundingSpheresCollisionDetectionTest(this->GetMPProblem());
  m_collisionDetectionTests["Bounding Spheres"] = boundingSpheres;

  InsideSpheresCollisionDetectionTest* insideSpheres = nullptr;
  insideSpheres = new InsideSpheresCollisionDetectionTest(this->GetMPProblem());
  m_collisionDetectionTests["Inside Spheres"] = insideSpheres;

}

template <typename MPTraits>
void
MPLibraryTests<MPTraits>::
RunCollisionDetectionMethodTests(size_t& _passed, size_t& _failed, size_t& _total) {
  for (auto method : m_collisionDetectionTests) {
    std::cout << "Running test for " << method.first << "..." << std::endl;

    auto test = dynamic_cast<TestBaseObject*>(method.second);
    auto result = test->RunTest();

    _total++;

    if(result.first) {
      std::cout << "PASSED!" << std::endl;
      _passed++;
    }
    else {
      std::cout << "FAILED :(" << std::endl;
      _failed++;
    }

    if(verbose) {
      std::cout << result.second << std::endl;
    }
  }
}


template <typename MPTraits>
template <typename MethodTypeList>
void
MPLibraryTests<MPTraits>::
RunMethodSetTests(const MethodTypeList& _mtl, size_t& _passed,
                  size_t& _failed, size_t& _total) {

  for(auto iter = _mtl.begin(); iter != _mtl.end(); iter++) {

    std::cout << "Running test for " << iter->first << "..." << std::endl;

    auto test = dynamic_cast<TestBaseObject*>(iter->second.get());
    auto result = test->RunTest();

    _total++;

    if(result.first) {
      std::cout << "PASSED!" << std::endl;
      _passed++;
    }
    else {
      std::cout << "FAILED :(" << std::endl;
      _failed++;
    }

    if(verbose) {
      std::cout << result.second << std::endl;
    }
  }
}

/*---------------------------- XML Helpers -----------------------------------*/

template <typename MPTraits>
void
MPLibraryTests<MPTraits>::
ReadXMLFile(const std::string& _filename) {
  // Open the XML and get the root node.
  XMLNode mpNode(_filename, "MotionPlanning");

  // Find the 'MPLibrary' node.
  XMLNode* planningLibrary = nullptr;
  for(auto& child : mpNode)
    if(child.Name() == "Library")
      planningLibrary = &child;

  // Throw exception if we can't find it.
  if(!planningLibrary)
    throw ParseException(WHERE) << "Cannot find MPLibrary node in XML file '"
                                << _filename << "'.";

  // Parse the library node to set algorithms and parameters.
  for(auto& child : *planningLibrary)
    ParseChild(child);

}


template <typename MPTraits>
bool
MPLibraryTests<MPTraits>::
ParseChild(XMLNode& _node) {
  if(_node.Name() == "DistanceMetrics") {
    m_distanceMetricTests->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "ValidityCheckers") {
    m_validityCheckerTests->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "NeighborhoodFinders") {
    m_neighborhoodFinderTests->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Samplers") {
    m_samplerTests->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "LocalPlanners") {
    m_localPlannerTests->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Extenders") {
    m_extenderTests->ParseXML(_node);
    return true;
  }
  //else if(_node.Name() == "PathModifiers") {
  //  m_pathModifierTests->ParseXML(_node);
  //  return true;
  //}
  else if(_node.Name() == "Connectors") {
    m_connectorTests->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Metrics") {
    m_metricTests->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "MapEvaluators") {
    m_mapEvaluatorTests->ParseXML(_node);
    return true;
  }
  else
    return false;
}

/*-------------------------------- Debugging ---------------------------------*/
/*--------------------------------------------------------------------*/
#endif
