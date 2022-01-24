#ifndef PMPL_H_
#define PMPL_H_

// Define the configuration-space model (traits) based on the compile options.
#ifdef PMPCfg

#include "ConfigurationSpace/Cfg.h"
#ifndef PPL_TEST_TRAITS_H_
  #include "Traits/CfgTraits.h"
#else
  #include "Traits/TestTraits.h"
#endif

typedef MPTraits<Cfg> PMPLTraits;

#else
#error "Error, must define a RobotType for PMPL application"
#endif


// Mark the corresponding templates as extern to avoid recompiling them
// constantly.
extern template class MPLibraryType<PMPLTraits>;
extern template class MPSolutionType<PMPLTraits>;
extern template class RoadmapGraph<PMPLTraits::CfgType, PMPLTraits::WeightType>;
extern template class PathType<PMPLTraits>;


// Set the templated types using the chosen traits.
typedef PMPLTraits::CfgType     CfgType;
typedef PMPLTraits::WeightType  WeightType;
typedef PMPLTraits::RoadmapType RoadmapType;
typedef PMPLTraits::Path        Path;
typedef PMPLTraits::MPLibrary   MPLibrary;
typedef PMPLTraits::MPSolution  MPSolution;

#endif
