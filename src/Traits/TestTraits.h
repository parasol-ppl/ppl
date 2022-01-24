#ifndef PPL_TEST_TRAITS_H_
#define PPL_TEST_TRAITS_H_

#include "MPLibrary/GoalTracker.h"
#include "MPLibrary/MPSolution.h"
#include "MPLibrary/MPTools/MPTools.h"

#include "Testing/MPLibrary/MPLibraryTests.h"

#include "ConfigurationSpace/LocalObstacleMap.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupLocalPlan.h"
#include "ConfigurationSpace/GroupPath.h"
#include "ConfigurationSpace/GroupRoadmap.h"
#include "ConfigurationSpace/Path.h"
#include "ConfigurationSpace/RoadmapGraph.h"
#include "ConfigurationSpace/Weight.h"

//distance metric includes

//validity checker includes
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#include "Testing/MPLibrary/ValidityCheckers/AlwaysTrueValidityTest.h"
#include "Testing/MPLibrary/ValidityCheckers/CollisionDetection/BoundingSpheresCollisionDetectionTest.h"
#include "Testing/MPLibrary/ValidityCheckers/CollisionDetection/InsideSpheresCollisionDetectionTest.h"
//neighborhood finder includes

//sampler includes
#include "Testing/MPLibrary/Samplers/UniformRandomSamplerTest.h"

//local planner includes

//extenders includes

//path smoothing includes

//connector includes

//metric includes

//map evaluator includes
#include "Testing/MPLibrary/MapEvaluators/LazyQueryTest.h"

//mp strategies includes
#include "MPLibrary/MPStrategies/ValidationStrategy.h"

////////////////////////////////////////////////////////////////////////////////
/// @ingroup MotionPlanningUniverse
/// @brief Defines available methods in the Motion Planning Universe for Cfg
/// @tparam C Cfg type
/// @tparam W Weight type
///
///TODO::Update this description
/// MPTraits is a type class which defines the motion planning universe. We
/// construct our methods through a factory design pattern, and thus this states
/// all available classes within an abstraction that you can use in the system.
/// Essentially, the important types are the CfgType or the @cspace abstraction
/// class, the WeightType or the edge type of the graph, and a type list for
/// each algorithm abstraction --- here you only need to define what you need,
/// as extraneous methods in the type class imply longer compile times.
///
/// All methods should have "Test" at the end to specify that they are using the
/// test version, and the test version header file should be included at the
/// top of this file.
////////////////////////////////////////////////////////////////////////////////
template <typename C, typename W = DefaultWeight<C>>
struct MPTraits {

  typedef C                               CfgType;
  typedef W                               WeightType;
  typedef RoadmapGraph<C, W>              RoadmapType;
  typedef PathType<MPTraits>              Path;
  typedef MPLibraryTests<MPTraits>         MPLibrary;
  typedef MPSolutionType<MPTraits>        MPSolution;
  typedef MPToolsType<MPTraits>           MPTools;
  typedef LocalObstacleMapType<MPTraits>  LocalObstacleMap;
  typedef GoalTrackerType<MPTraits>       GoalTracker;

  typedef GroupLocalPlan<CfgType>                    GroupWeightType;
  typedef GroupRoadmap<GroupCfg, GroupWeightType>    GroupRoadmapType;
  typedef GroupPath<MPTraits>                        GroupPathType;
  typedef GroupCfg                                   GroupCfgType;

  //types of distance metrics available in our world
  typedef boost::mpl::list<
      > DistanceMetricMethodList;

  //types of validity checkers available in our world
  typedef boost::mpl::list<
      AlwaysTrueValidityTest<MPTraits>
      > ValidityCheckerMethodList;

  //types of neighborhood finders available in our world
  typedef boost::mpl::list<
      > NeighborhoodFinderMethodList;

  //types of samplers available in our world
  typedef boost::mpl::list<
      UniformRandomSamplerTest<MPTraits>
      > SamplerMethodList;

  //types of local planners available in our world
  typedef boost::mpl::list<
      > LocalPlannerMethodList;

  //types of extenders avaible in our world
  typedef boost::mpl::list<
      > ExtenderMethodList;

  //types of path smoothing available in our world
  typedef boost::mpl::list<
      > PathModifierMethodList;


  //types of connectors available in our world
  typedef boost::mpl::list<
      > ConnectorMethodList;

  //types of metrics available in our world
  typedef boost::mpl::list<
      > MetricMethodList;


  //types of map evaluators available in our world
  typedef boost::mpl::list<
    LazyQueryTest<MPTraits>
      > MapEvaluatorMethodList;

  //types of motion planning strategies available in our world
  typedef boost::mpl::list<
    ValidationStrategy<MPTraits>
      > MPStrategyMethodList;
};

#endif
