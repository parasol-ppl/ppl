#ifndef PPL_TMP_TEST_TRAITS_H_
#define PPL_TMP_TEST_TRAITS_H_

// TMPStrategyMethods to include

//#include "TMPLibrary/TMPStrategies/SimpleMotionMethod.h"

// PoIPlacementMethods to include

// TaskEvaluators to include

#include "Testing/TMPLibrary/TaskEvaluators/SimpleMotionEvaluatorTest.h"

// TaskDecomposers to include

// TaskAllocators to include 

// StateGraphs to include

////////////////////////////////////////////////////////////////////////////////
/// @ingroup TaskAndMotionPlanningUniverse
/// @brief Defines available methods in the Task and Motion Planning Universe
/// 
///
/// TMPTraits is a type class which defines the task and motion planning universe. We
/// construct our methods through a factory design pattern, and thus this states
/// all available classes within an abstraction that you can use in the system.
////////////////////////////////////////////////////////////////////////////////
struct TMPTraits {

  //types of tmp strategy methods available in our world
  typedef boost::mpl::list<
      > TMPStrategyMethodList;

  //types of points of interest placement methods available in our world
  typedef boost::mpl::list<
      > PoIPlacementMethodList;

  //types of task evaluators available in our world
  typedef boost::mpl::list<
		SimpleMotionEvaluatorTest
      > TaskEvaluatorMethodList;

  //types of task decomposers available in our world
  typedef boost::mpl::list<
      > TaskDecomposerMethodList;

  //types of task allocators available in our world
  typedef boost::mpl::list<
      > TaskAllocatorMethodList;
};

#endif
