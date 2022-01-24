#ifndef PMPL_MP_LIBRARY_H_
#define PMPL_MP_LIBRARY_H_

#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/GroupTask.h"
#include "MPProblem/Robot/Robot.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

#include "Utilities/MetricUtils.h"
#include "Utilities/MPUtils.h"
#include "Utilities/XMLNode.h"

#include "MPLibrary/Connectors/ConnectorMethod.h"
#include "MPLibrary/DistanceMetrics/DistanceMetricMethod.h"
#include "MPLibrary/Extenders/ExtenderMethod.h"
#include "MPLibrary/LocalPlanners/LocalPlannerMethod.h"
#include "MPLibrary/MapEvaluators/MapEvaluatorMethod.h"
#include "MPLibrary/Metrics/MetricMethod.h"
#include "MPLibrary/MPStrategies/MPStrategyMethod.h"
#include "MPLibrary/MPTools/MPTools.h"
#include "MPLibrary/NeighborhoodFinders/NeighborhoodFinderMethod.h"
#include "MPLibrary/PathModifiers/PathModifierMethod.h"
#include "MPLibrary/Samplers/SamplerMethod.h"
#include "MPLibrary/ValidityCheckers/CollisionDetectionValidity.h"
#include "MPLibrary/ValidityCheckers/ValidityCheckerMethod.h"

#include <algorithm>
#include <atomic>
#include <unordered_map>


////////////////////////////////////////////////////////////////////////////////
/// A collection of planning algorithms that can operate on a specific
/// MPProblem and MPTask.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
#ifdef _PARALLEL
class MPLibraryType : public stapl::p_object
#else
class MPLibraryType
#endif
{

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::MPSolution       MPSolution;
    typedef typename MPTraits::RoadmapType      RoadmapType;
    typedef typename RoadmapType::VID           VID;
    typedef typename MPTraits::CfgType          CfgType;
    typedef typename MPTraits::GroupRoadmapType GroupRoadmapType;
    typedef typename MPTraits::Path             Path;
    typedef typename MPTraits::GroupPathType    GroupPath;
    typedef typename MPTraits::MPTools          MPTools;
    typedef typename MPTraits::LocalObstacleMap LocalObstacleMap;
    typedef typename MPTraits::GoalTracker      GoalTracker;

    ///@}
    ///@name Local Types
    ///@{

    /// Solver represents an input set to MPLibraryType. It includes an
    /// MPStrategy label, seed, base file name, and vizmo debug option.
    struct Solver {
      std::string label;         ///< The XML label for the strategy to use.
      long seed;                 ///< The seed.
      std::string baseFilename;  ///< The base name for output files.
      bool vizmoDebug;           ///< Save vizmo debug info?
    };

    ///@}
    ///@name Method Set Types
    ///@{

    typedef MethodSet<MPTraits, DistanceMetricMethod<MPTraits>> DistanceMetricSet;
    typedef MethodSet<MPTraits, ValidityCheckerMethod<MPTraits>>
                                                            ValidityCheckerSet;
    typedef MethodSet<MPTraits, NeighborhoodFinderMethod<MPTraits>>
                                                            NeighborhoodFinderSet;
    typedef MethodSet<MPTraits, SamplerMethod<MPTraits>>        SamplerSet;
    typedef MethodSet<MPTraits, LocalPlannerMethod<MPTraits>>   LocalPlannerSet;
    typedef MethodSet<MPTraits, ExtenderMethod<MPTraits>>       ExtenderSet;
    typedef MethodSet<MPTraits, PathModifierMethod<MPTraits>>   PathModifierSet;
    typedef MethodSet<MPTraits, ConnectorMethod<MPTraits>>      ConnectorSet;
    typedef MethodSet<MPTraits, MetricMethod<MPTraits>>         MetricSet;
    typedef MethodSet<MPTraits, MapEvaluatorMethod<MPTraits>>   MapEvaluatorSet;
    typedef MethodSet<MPTraits, MPStrategyMethod<MPTraits>>     MPStrategySet;

    ///@}
    ///@name Method Pointer Types
    ///@{

    typedef typename ValidityCheckerSet::MethodPointer ValidityCheckerPointer;
    typedef typename DistanceMetricSet::MethodPointer  DistanceMetricPointer;
    typedef typename NeighborhoodFinderSet::MethodPointer
                                                       NeighborhoodFinderPointer;
    typedef typename SamplerSet::MethodPointer         SamplerPointer;
    typedef typename LocalPlannerSet::MethodPointer    LocalPlannerPointer;
    typedef typename ExtenderSet::MethodPointer        ExtenderPointer;
    typedef typename PathModifierSet::MethodPointer    PathModifierPointer;
    typedef typename ConnectorSet::MethodPointer       ConnectorPointer;
    typedef typename MetricSet::MethodPointer          MetricPointer;
    typedef typename MapEvaluatorSet::MethodPointer    MapEvaluatorPointer;
    typedef typename MPStrategySet::MethodPointer      MPStrategyPointer;

    ///@}
    ///@name Construction
    ///@{

    MPLibraryType();

    MPLibraryType(const std::string& _filename);

    virtual ~MPLibraryType();

    ///@}
    ///@name Configuration
    ///@{

    /// Read an XML file to set the algorithms and parameters in this instance.
    /// @param _filename The XML file name.
    void ReadXMLFile(const std::string& _filename);

    ///@}
    ///@name Distance Metric Accessors
    ///@{

    DistanceMetricPointer GetDistanceMetric(const std::string& _l) {
      return m_distanceMetrics->GetMethod(_l);
    }
    void AddDistanceMetric(DistanceMetricPointer _dm, const std::string& _l) {
      m_distanceMetrics->AddMethod(_dm, _l);
    }

    ///@}
    ///@name Validity Checker Accessors
    ///@{

    ValidityCheckerPointer GetValidityChecker(const std::string& _l) {
      return m_validityCheckers->GetMethod(_l);
    }
    void AddValidityChecker(ValidityCheckerPointer _vc, const std::string& _l) {
      m_validityCheckers->AddMethod(_vc, _l);
    }

    /// Toggle (negate) the validity output for ALL validity checkers.
    void ToggleValidity() {
      for(auto& vc : *m_validityCheckers)
        vc.second->ToggleValidity();
    }

    ///@}
    ///@name Neighborhood Finder Accessors
    ///@{

    NeighborhoodFinderPointer GetNeighborhoodFinder(const std::string& _l) {
      return m_neighborhoodFinders->GetMethod(_l);
    }
    void AddNeighborhoodFinder(NeighborhoodFinderPointer _nf, const std::string& _l) {
      m_neighborhoodFinders->AddMethod(_nf, _l);
    }

    ///@}
    ///@name Sampler Accessors
    ///@{

    const SamplerSet* const GetSamplers() const {return m_samplers;}
    SamplerPointer GetSampler(const std::string& _l) {
      return m_samplers->GetMethod(_l);
    }
    void AddSampler(SamplerPointer _s, const std::string& _l) {
      m_samplers->AddMethod(_s, _l);
    }

    ///@}
    ///@name Local Planner Accessors
    ///@{

    LocalPlannerPointer GetLocalPlanner(const std::string& _l) {
      return m_localPlanners->GetMethod(_l);
    }
    void AddLocalPlanner(LocalPlannerPointer _lp, const std::string& _l) {
      m_localPlanners->AddMethod(_lp, _l);
    }

    ///@}
    ///@name Extender Accessors
    ///@{

    ExtenderPointer GetExtender(const std::string& _l) {
      return m_extenders->GetMethod(_l);
    }
    void AddExtender(ExtenderPointer _mps, const std::string& _l) {
      m_extenders->AddMethod(_mps, _l);
    }

    ///@}
    ///@name Path Modifier Accessors
    ///@{

    PathModifierPointer GetPathModifier(const std::string& _l) {
      return m_pathModifiers->GetMethod(_l);
    }
    void AddPathModifier(PathModifierPointer _ps, const std::string& _l) {
      m_pathModifiers->AddMethod(_ps, _l);
    }

    ///@}
    ///@name Connector Accessors
    ///@{

    ConnectorPointer GetConnector(const std::string& _l) {
      return m_connectors->GetMethod(_l);
    }
    void AddConnector(ConnectorPointer _c, const std::string& _l) {
      m_connectors->AddMethod(_c, _l);
    }

    ///@}
    ///@name Metric Accessors
    ///@{

    MetricPointer GetMetric(const std::string& _l) {return m_metrics->GetMethod(_l);}
    void AddMetric(MetricPointer _m, const std::string& _l) {
      m_metrics->AddMethod(_m, _l);
    }

    ///@}
    ///@name Map Evaluator Accessors
    ///@{

    MapEvaluatorPointer GetMapEvaluator(const std::string& _l) {
      return m_mapEvaluators->GetMethod(_l);
    }
    void AddMapEvaluator(MapEvaluatorPointer _me, const std::string& _l) {
      m_mapEvaluators->AddMethod(_me, _l);
    }

    /// For cases where we need to reset all instances of TimeEvaluator.
/*    void ResetTimeEvaluators() {
      for(auto& labelPtr : *m_mapEvaluators) {
        auto t = dynamic_cast<TimeEvaluator<MPTraits>*>(labelPtr.second.get());
        if(t)
          t->Initialize();
      }
    }
*/
    ///@}
    ///@name MPStrategy Accessors
    ///@{

    const MPStrategySet* const GetMPStrategies() const {return m_mpStrategies;}
    MPStrategyPointer GetMPStrategy(const std::string& _l) {
      return m_mpStrategies->GetMethod(_l);
    }
    void AddMPStrategy(MPStrategyPointer _mps, const std::string& _l) {
      m_mpStrategies->AddMethod(_mps, _l);
    }

    ///@}
    ///@name MPTools Accessors
    ///@{

    MPTools* GetMPTools() {return m_mpTools;}

    ///@}
    ///@name Input Accessors
    ///@{

    MPProblem* GetMPProblem() const noexcept;
    void SetMPProblem(MPProblem* const _problem) noexcept;

    MPTask* GetTask() const noexcept;
    void SetTask(MPTask* const _task) noexcept;

    GroupTask* GetGroupTask() const noexcept;
    void SetGroupTask(GroupTask* const _task) noexcept;

    const std::string& GetBaseFilename() const noexcept;
    void SetBaseFilename(const std::string& _s) noexcept;

    ///@}
    ///@name Solution Accessors
    ///@{

    MPSolution* GetMPSolution() const noexcept;
    void SetMPSolution(MPSolution* _sol) noexcept;

    RoadmapType*      GetRoadmap(Robot* const _r = nullptr) const noexcept;
    GroupRoadmapType* GetGroupRoadmap(RobotGroup* const _g = nullptr) const noexcept;
    RoadmapType*      GetBlockRoadmap(Robot* const _r = nullptr) const noexcept;
    Path*             GetPath(Robot* const _r = nullptr) const noexcept;
    GroupPath*        GetGroupPath(RobotGroup* const _g = nullptr) const noexcept;
    LocalObstacleMap* GetLocalObstacleMap(Robot* const _r = nullptr) const noexcept;

    GoalTracker*      GetGoalTracker() const noexcept;
    StatClass*        GetStatClass() const noexcept;

    ///@}
    ///@name Edge Reconstruction
    ///@{

    /// Recompute a roadmap edge path at full resolution (source and target cfgs
    /// are not included).
    /// @param _roadmap The roadmap pointer.
    /// @param _source The source VID.
    /// @param _target The target VID.
    /// @param _posRes The position resolution to use.
    /// @param _oriRes The orientation resolution to use.
    /// @return A vector of cfgs along the edge, spaced at resolution intervals.
    std::vector<typename RoadmapType::VP> ReconstructEdge(
        RoadmapType* const _roadmap,
        const VID _source, const VID _target,
        const double _posRes, const double _oriRes);

    /// @overload This version is for groups.
    std::vector<typename GroupRoadmapType::VP> ReconstructEdge(
        GroupRoadmapType* const _roadmap,
        const VID _source, const VID _target,
        const double _posRes, const double _oriRes);

    /// @overload This version uses the Environment's resolutions.
    template <typename AbstractRoadmapType>
    std::vector<typename AbstractRoadmapType::VP> ReconstructEdge(
        AbstractRoadmapType* const _roadmap,
        const VID _source, const VID _target);

    ///@}
    ///@name Execution Interface
    ///@{

    /// Halt the current strategy and move on to finalization. Useful for
    /// controlling the library from an external client object.
    void Halt();

    /// Check to see if the strategy should continue.
    bool IsRunning() const noexcept;

    /// Set the current seed to match the first solver node.
    void SetSeed() const noexcept;

    /// Set the current seed.
    /// @param _seed The seed value.
    void SetSeed(const long _seed) const noexcept;

    /// Add an input set to this MPLibrary.
    /// @param _label        The MPStrategy label to use.
    /// @param _seed         The random seed to use.
    /// @param _baseFileName The base name of the XML file to use.
    /// @param _vizmoDebug   Enable/disable vizmo debug.
    void AddSolver(const std::string& _label, long _seed,
        const std::string& _baseFileName, bool _vizmoDebug = false) {
      m_solvers.push_back(Solver{_label, _seed, _baseFileName, _vizmoDebug});
    }

    std::vector<std::string> GetSolverLabels() {
      std::vector<std::string> labels;
      for (auto& solver : m_solvers)
        labels.push_back(solver.label);
      return labels;
    }

    /// Run a single input (solver) and get back its solution.
    /// @param _problem The problem representation.
    /// @param _task The task representation.
    /// @param _solution The solution container, which may or may not
    ///                  already be populated. Existing solutions should
    ///                  be extended, not overwritten.
    void Solve(MPProblem* _problem, MPTask* _task, MPSolution* _solution);
    ///@example MPLibrary_UseCase.cpp
    /// This is an example of how to use the MPLibrary methods.

    /// Run each input (solver) in sequence.
    /// @param _problem The problem representation.
    /// @param _task The task representation.
    void Solve(MPProblem* _problem, MPTask* _task);

    /// Group overload:
    void Solve(MPProblem* _problem, GroupTask* _task);

    void Solve(MPProblem* _problem, GroupTask* _task, MPSolution* _solution);

    /// Run a specific MPStrategy from the XML file with a designated seed and
    /// base output file name. This is intended for use with the simulator where
    /// agents may need to call various strategies within a single execution.
    /// @param _problem The problem representation.
    /// @param _task The task representation.
    /// @param _solution The solution container, which may or may not
    ///                  already be populated. Existing solutions should
    ///                  be extended, not overwritten.
    /// @param _label The label of the MPStrategy to call.
    /// @param _seed The seed to use.
    /// @param _baseFilename The base name for output files.
    void Solve(MPProblem* _problem, MPTask* _task, MPSolution* _solution,
        const std::string& _label, const long _seed,
        const std::string& _baseFilename);

    ///@}
    ///@name Debugging
    ///@{

    void Print(ostream& _os) const; ///< Print each method set.

    ///@}

  private:

    ///@name Execution Helpers
    ///@{

    /// Execute a solver with the current problem, task, and solution.
    /// @param _s The solver to execute.
    void RunSolver(const Solver& _s);

    ///@}
    ///@name Construction Helpers
    ///@{

    /// Initialize all algorithms in each method set.
    void Initialize();

    /// Clear temporary states and variables.
    void Uninitialize();

    /// Helper for parsing XML nodes.
    /// @param _node The child node to be parsed.
    bool ParseChild(XMLNode& _node);

    ///@}
    ///@name Inputs
    ///@{

    MPProblem*          m_problem{nullptr};  ///< The current MPProblem.
    MPTask*             m_task{nullptr};     ///< The current task.
    GroupTask*          m_groupTask{nullptr};///< The current group task.
    MPSolution*         m_solution{nullptr}; ///< The current solution.
    std::vector<Solver> m_solvers;           ///< The set of inputs to execute.

    ///@}
    ///@name Other Library Objects
    ///@{
    /// These do not use method sets because the sub-objects are not expected to
    /// share a common interface.

    std::unique_ptr<GoalTracker> m_goalTracker;
    MPTools* m_mpTools{nullptr};

    ///@}
    ///@name Internal State
    ///@{

    std::atomic<bool> m_running{true};  ///< Keep running the strategy?

    ///@}

  protected:

    ///@name Method Sets
    ///@{
    /// Method sets hold and offer access to the motion planning objects of the
    /// corresponding type.

    DistanceMetricSet*     m_distanceMetrics{nullptr};
    ValidityCheckerSet*    m_validityCheckers{nullptr};
    NeighborhoodFinderSet* m_neighborhoodFinders{nullptr};
    SamplerSet*            m_samplers{nullptr};
    LocalPlannerSet*       m_localPlanners{nullptr};
    ExtenderSet*           m_extenders{nullptr};
    PathModifierSet*       m_pathModifiers{nullptr};
    ConnectorSet*          m_connectors{nullptr};
    MetricSet*             m_metrics{nullptr};
    MapEvaluatorSet*       m_mapEvaluators{nullptr};
    MPStrategySet*         m_mpStrategies{nullptr};

    ///@}
};

/*---------------------------- Construction ----------------------------------*/

template <typename MPTraits>
MPLibraryType<MPTraits>::
MPLibraryType() {
  m_distanceMetrics = new DistanceMetricSet(this,
      typename MPTraits::DistanceMetricMethodList(), "DistanceMetrics");
  m_validityCheckers = new ValidityCheckerSet(this,
      typename MPTraits::ValidityCheckerMethodList(), "ValidityCheckers");
  m_neighborhoodFinders = new NeighborhoodFinderSet(this,
      typename MPTraits::NeighborhoodFinderMethodList(), "NeighborhoodFinders");
  m_samplers = new SamplerSet(this,
      typename MPTraits::SamplerMethodList(), "Samplers");
  m_localPlanners = new LocalPlannerSet(this,
      typename MPTraits::LocalPlannerMethodList(), "LocalPlanners");
  m_extenders = new ExtenderSet(this,
      typename MPTraits::ExtenderMethodList(), "Extenders");
  m_pathModifiers = new PathModifierSet(this,
      typename MPTraits::PathModifierMethodList(), "PathModifiers");
  m_connectors = new ConnectorSet(this,
      typename MPTraits::ConnectorMethodList(), "Connectors");
  m_metrics = new MetricSet(this,
      typename MPTraits::MetricMethodList(), "Metrics");
  m_mapEvaluators = new MapEvaluatorSet(this,
      typename MPTraits::MapEvaluatorMethodList(), "MapEvaluators");
  m_mpStrategies = new MPStrategySet(this,
      typename MPTraits::MPStrategyMethodList(), "MPStrategies");
  m_mpTools = new MPTools(this);
  m_goalTracker.reset(new GoalTracker(this));
}


template <typename MPTraits>
MPLibraryType<MPTraits>::
MPLibraryType(const std::string& _filename) : MPLibraryType() {
  ReadXMLFile(_filename);
}


template <typename MPTraits>
MPLibraryType<MPTraits>::
~MPLibraryType() {
  delete m_distanceMetrics;
  delete m_validityCheckers;
  delete m_neighborhoodFinders;
  delete m_samplers;
  delete m_localPlanners;
  delete m_extenders;
  delete m_pathModifiers;
  delete m_connectors;
  delete m_metrics;
  delete m_mapEvaluators;
  delete m_mpStrategies;
  delete m_mpTools;
}


template <typename MPTraits>
void
MPLibraryType<MPTraits>::
Initialize() {
  MethodTimer mt(this->GetStatClass(), "MPLibrary::Initialize");

  // Set up the goal tracker.
  m_goalTracker->Clear();
  if(this->GetTask())
    m_goalTracker->AddMap(GetRoadmap(), GetTask());
  else if(this->GetGroupTask())
    m_goalTracker->AddMap(GetGroupRoadmap(), GetGroupTask());
  else
    throw RunTimeException(WHERE) << "No current task was set.";

  m_distanceMetrics->Initialize();
  m_validityCheckers->Initialize();
  m_neighborhoodFinders->Initialize();
  m_samplers->Initialize();
  m_localPlanners->Initialize();
  m_extenders->Initialize();
  m_pathModifiers->Initialize();
  m_connectors->Initialize();
  m_metrics->Initialize();
  m_mapEvaluators->Initialize();
  m_mpTools->Initialize();

  m_running = true;
}


template <typename MPTraits>
void
MPLibraryType<MPTraits>::
Uninitialize() {
  // Clear goal tracker.
  m_goalTracker->Clear();

  // Clear group hooks.
  GroupRoadmapType* const groupMap = this->GetGroupRoadmap();
  if(groupMap)
    groupMap->ClearHooks();

  // Also clear hooks for the individual robot if it exists:
  if(m_solution->GetRobot())
    this->GetRoadmap()->ClearHooks();
}

/*---------------------------- XML Helpers -----------------------------------*/

template <typename MPTraits>
void
MPLibraryType<MPTraits>::
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

  // Ensure we have at least one solver.
  if(m_solvers.empty())
    throw ParseException(WHERE) << "Cannot find Solver node in XML file '"
                                << _filename << "'.";

  // Print XML details if requested.
  bool print = mpNode.Read("print", false, false, "Print all XML input");
  if(print)
    Print(cout);

  // Handle XML warnings/errors.
  bool warnings = mpNode.Read("warnings", false, false, "Report warnings");
  if(warnings) {
    bool warningsAsErrors = mpNode.Read("warningsAsErrors", false, false,
        "XML warnings considered errors");
    planningLibrary->WarnAll(warningsAsErrors);
  }
}


template <typename MPTraits>
bool
MPLibraryType<MPTraits>::
ParseChild(XMLNode& _node) {
  if(_node.Name() == "DistanceMetrics") {
    m_distanceMetrics->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "ValidityCheckers") {
    m_validityCheckers->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "NeighborhoodFinders") {
    m_neighborhoodFinders->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Samplers") {
    m_samplers->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "LocalPlanners") {
    m_localPlanners->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Extenders") {
    m_extenders->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "PathModifiers") {
    m_pathModifiers->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Connectors") {
    m_connectors->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Metrics") {
    m_metrics->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "MapEvaluators") {
    m_mapEvaluators->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "MPStrategies") {
    m_mpStrategies->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "MPTools") {
    m_mpTools->ParseXML(_node);
    return true;
  }
  else if(_node.Name() == "Solver") {
    const std::string label = _node.Read("mpStrategyLabel", true, "",
        "The strategy to use.");

    const long seed = _node.Read("seed", true, size_t(1), size_t(0),
        std::numeric_limits<size_t>::max(),
        "The random number generator seed.");

    const std::string baseFilename = _node.Read("baseFilename", true, "",
        "BaseFilename for the solver.") + "." + std::to_string(seed);

    const bool vdOutput = _node.Read("vizmoDebug", false, false,
        "True yields VizmoDebug output for the solver.");

    m_solvers.emplace_back(Solver{label, seed, baseFilename, vdOutput});
    return true;
  }
  else
    return false;
}

/*-------------------------------- Debugging ---------------------------------*/

template <typename MPTraits>
void
MPLibraryType<MPTraits>::
Print(ostream& _os) const {
  _os << "MPLibrary" << std::endl;
  m_distanceMetrics->Print(_os);
  m_validityCheckers->Print(_os);
  m_neighborhoodFinders->Print(_os);
  m_samplers->Print(_os);
  m_localPlanners->Print(_os);
  m_extenders->Print(_os);
  m_pathModifiers->Print(_os);
  m_connectors->Print(_os);
  m_metrics->Print(_os);
  m_mapEvaluators->Print(_os);
  m_mpStrategies->Print(_os);
}

/*----------------------------- Input Accessors ------------------------------*/

template <typename MPTraits>
inline
MPProblem*
MPLibraryType<MPTraits>::
GetMPProblem() const noexcept {
  return m_problem;
}


template <typename MPTraits>
void
MPLibraryType<MPTraits>::
SetMPProblem(MPProblem* const _problem) noexcept
{
  m_problem = _problem;
}


template <typename MPTraits>
inline
MPTask*
MPLibraryType<MPTraits>::
GetTask() const noexcept {
  return m_task;
}


template <typename MPTraits>
inline
void
MPLibraryType<MPTraits>::
SetTask(MPTask* const _task) noexcept {
  m_task = _task;
}


template <typename MPTraits>
inline
GroupTask*
MPLibraryType<MPTraits>::
GetGroupTask() const noexcept {
  return m_groupTask;
}


template <typename MPTraits>
inline
void
MPLibraryType<MPTraits>::
SetGroupTask(GroupTask* const _task) noexcept {
  m_groupTask = _task;
}


template <typename MPTraits>
inline
const std::string&
MPLibraryType<MPTraits>::
GetBaseFilename() const noexcept {
  return m_problem->GetBaseFilename();
}


template <typename MPTraits>
inline
void
MPLibraryType<MPTraits>::
SetBaseFilename(const std::string& _s) noexcept {
  m_problem->SetBaseFilename(_s);
}

/*---------------------------- Solution Accessors ----------------------------*/

template <typename MPTraits>
inline
typename MPTraits::MPSolution*
MPLibraryType<MPTraits>::
GetMPSolution() const noexcept {
  return m_solution;
}


template <typename MPTraits>
inline
void
MPLibraryType<MPTraits>::
SetMPSolution(MPSolution* const _sol) noexcept {
  m_solution = _sol;
}


template <typename MPTraits>
inline
typename MPTraits::RoadmapType*
MPLibraryType<MPTraits>::
GetRoadmap(Robot* const _r) const noexcept {
  if(!_r and !GetTask())
    return nullptr;
  return m_solution->GetRoadmap(_r ? _r : GetTask()->GetRobot());
}


template <typename MPTraits>
inline
typename MPTraits::GroupRoadmapType*
MPLibraryType<MPTraits>::
GetGroupRoadmap(RobotGroup* const _g) const noexcept {
  if(!_g and !GetGroupTask())
    return nullptr;
  return m_solution->GetGroupRoadmap(_g ? _g : GetGroupTask()->GetRobotGroup());
}


template <typename MPTraits>
inline
typename MPTraits::RoadmapType*
MPLibraryType<MPTraits>::
GetBlockRoadmap(Robot* const _r) const noexcept {
  if(!_r and !GetTask())
    return nullptr;
  return m_solution->GetBlockRoadmap(_r ? _r : GetTask()->GetRobot());
}


template <typename MPTraits>
typename MPTraits::Path*
MPLibraryType<MPTraits>::
GetPath(Robot* const _r) const noexcept {
  if(!_r and !GetTask())
    return nullptr;
  return m_solution->GetPath(_r ? _r : GetTask()->GetRobot());
}


template <typename MPTraits>
typename MPTraits::GroupPathType*
MPLibraryType<MPTraits>::
GetGroupPath(RobotGroup* const _g) const noexcept {
  if(!_g and !GetGroupTask())
    return nullptr;
  return m_solution->GetGroupPath(_g ? _g : GetGroupTask()->GetRobotGroup());
}


template <typename MPTraits>
inline
typename MPTraits::LocalObstacleMap*
MPLibraryType<MPTraits>::
GetLocalObstacleMap(Robot* const _r) const noexcept {
  if(!_r and !GetTask())
    return nullptr;
  return m_solution->GetLocalObstacleMap(_r ? _r : GetTask()->GetRobot());
}


template <typename MPTraits>
inline
typename MPTraits::GoalTracker*
MPLibraryType<MPTraits>::
GetGoalTracker() const noexcept {
  return m_goalTracker.get();
}


template <typename MPTraits>
inline
StatClass*
MPLibraryType<MPTraits>::
GetStatClass() const noexcept {
  return m_solution->GetStatClass();
}

/*--------------------------- Edge Reconstruction ----------------------------*/

template <typename MPTraits>
std::vector<typename MPTraits::RoadmapType::VP>
MPLibraryType<MPTraits>::
ReconstructEdge(RoadmapType* const _roadmap, const VID _source,
    const VID _target, const double _posRes,
    const double _oriRes) {
  const auto& start         = _roadmap->GetVertex(_source);
  const auto& end           = _roadmap->GetVertex(_target);
  const auto& edge          = _roadmap->GetEdge(_source, _target);
  const auto& intermediates = edge.GetIntermediates();

  // Construct the set of start, intermediates, and end waypoints as needed.
  auto waypoints = intermediates;
  waypoints.insert(waypoints.begin(), start);
  waypoints.push_back(end);

  // Check for intermediates. If there are any, we will use straight-line to
  // recompute the path. Otherwise use the lp label if available, and fall back
  // to straight-line if not (this will always happen with extenders).
  const std::string lpLabel = !intermediates.size()
                            and !edge.GetLPLabel().empty()
                            ? edge.GetLPLabel()
                            : "sl";

  // Construct a resolution-level path along the recreated edge.
  auto lp = this->GetLocalPlanner(lpLabel);
  return lp->BlindPath(waypoints, _posRes, _oriRes);
}


template <typename MPTraits>
std::vector<typename MPTraits::GroupRoadmapType::VP>
MPLibraryType<MPTraits>::
ReconstructEdge(GroupRoadmapType* const _roadmap, const VID _source,
    const VID _target, const double _posRes,
    const double _oriRes) {
  const auto& start         = _roadmap->GetVertex(_source);
  const auto& end           = _roadmap->GetVertex(_target);
  const auto& edge          = _roadmap->GetEdge(_source, _target);
  const auto& intermediates = edge.GetIntermediates();

  // Construct the set of start, intermediates, and end waypoints as needed.
  auto waypoints = intermediates;
  waypoints.insert(waypoints.begin(), start);
  waypoints.push_back(end);

  // Check for intermediates. If there are any, we will use straight-line to
  // recompute the path. Otherwise use the lp label if available, and fall back
  // to straight-line if not (this will always happen with extenders).
  const std::string lpLabel = !intermediates.size()
                            and !edge.GetLPLabel().empty()
                            ? edge.GetLPLabel()
                            : "sl";

  // Construct a resolution-level path along the recreated edge.
  auto lp = this->GetLocalPlanner(lpLabel);
  return lp->BlindPath(waypoints, _posRes, _oriRes, edge.GetActiveRobots());
}


template< typename MPTraits>
template <typename AbstractRoadmapType>
std::vector<typename AbstractRoadmapType::VP>
MPLibraryType<MPTraits>::
ReconstructEdge(AbstractRoadmapType* const _roadmap, const VID _source,
    const VID _target) {
  auto env = this->GetMPProblem()->GetEnvironment();
  return this->ReconstructEdge(_roadmap, _source, _target,
                               env->GetPositionRes(), env->GetOrientationRes());
}

/*--------------------------- Execution Interface ----------------------------*/

template <typename MPTraits>
void
MPLibraryType<MPTraits>::
Halt() {
  m_running = false;
}


template <typename MPTraits>
bool
MPLibraryType<MPTraits>::
IsRunning() const noexcept {
  return m_running;
}


template <typename MPTraits>
void
MPLibraryType<MPTraits>::
SetSeed() const noexcept {
  if(m_solvers.empty())
    throw RunTimeException(WHERE) << "No solver nodes.";
  SetSeed(m_solvers[0].seed);
}


template <typename MPTraits>
void
MPLibraryType<MPTraits>::
SetSeed(const long _seed) const noexcept {
#ifdef _PARALLEL
  SRand(_seed + get_location_id());
#else
  SRand(_seed);
#endif
}


template <typename MPTraits>
void
MPLibraryType<MPTraits>::
Solve(MPProblem* _problem, MPTask* _task, MPSolution* _solution) {
  m_problem = _problem;
  m_task = _task;
  m_solution = _solution;

  for(auto& solver : m_solvers)
    RunSolver(solver);
}


template <typename MPTraits>
void
MPLibraryType<MPTraits>::
Solve(MPProblem* _problem, MPTask* _task) {
  m_problem = _problem;
  m_task = _task;

  for(auto& solver : m_solvers) {
    // Create storage for the solution.
    m_solution = new MPSolution(m_task->GetRobot());

    RunSolver(solver);

    delete m_solution;
  }

  m_solution = nullptr;
}


template <typename MPTraits>
void
MPLibraryType<MPTraits>::
Solve(MPProblem* _problem, GroupTask* _task) {
  m_problem = _problem;
  m_groupTask = _task;

  for(auto& solver : m_solvers) {
    // Create storage for the solution.
    m_solution = new MPSolution(m_groupTask->GetRobotGroup());

    RunSolver(solver);

    delete m_solution;
  }

  m_solution = nullptr;
}


template <typename MPTraits>
void
MPLibraryType<MPTraits>::
Solve(MPProblem* _problem, GroupTask* _task, MPSolution* _solution) {
  m_problem = _problem;
  m_groupTask = _task;
  m_solution = _solution;

  for(auto& solver : m_solvers) {
    RunSolver(solver);
  }

}


template <typename MPTraits>
void
MPLibraryType<MPTraits>::
Solve(MPProblem* _problem, MPTask* _task, MPSolution* _solution,
    const std::string& _label, const long _seed,
    const std::string& _baseFilename) {
  m_problem = _problem;
  m_task = _task;
  m_solution = _solution;

  Solver s{_label, _seed, _baseFilename, false};
  RunSolver(s);
}


template <typename MPTraits>
void
MPLibraryType<MPTraits>::
RunSolver(const Solver& _solver) {
  // Announce the method label and seed.
  std::cout << "\n\nMPLibrary is solving with MPStrategyMethod labeled "
            << _solver.label << " using seed " << _solver.seed << "."
            << std::endl;

  // Save the original prefix for output files (base name).
  const std::string originalBaseFilename = m_problem->GetBaseFilename();

  // Use the solver node to set the base name for output files.
  std::string baseFilename = _solver.baseFilename;

  // If this task has a label, append it to the solver's output file name.
  if(m_groupTask) {
    if(!m_groupTask->GetLabel().empty())
      baseFilename += "." + m_groupTask->GetLabel();
  }
  else if(!m_task->GetLabel().empty())
    baseFilename += "." + m_task->GetLabel();

  // Remove spaces to keep file names nice.
  {
    auto newEnd = std::remove_if(baseFilename.begin(), baseFilename.end(),
        ::isspace);
    baseFilename.erase(newEnd, baseFilename.end());
  }

  // Set the output file locations.
  SetBaseFilename(GetMPProblem()->GetPath(baseFilename));
  GetStatClass()->SetAuxDest(GetBaseFilename());

  // Initialize vizmo debug if requested.
  if(_solver.vizmoDebug) {
    VDInit(GetBaseFilename() + ".vd");
    if(m_groupTask)
      VDTrackRoadmap(this->GetGroupRoadmap());
    else
      VDTrackRoadmap(this->GetRoadmap());
  }

  // Initialize the library's algorithms.
  SetSeed(_solver.seed);
  Initialize();

  // Reset the seed
  SetSeed(_solver.seed);

  GetMPStrategy(_solver.label)->operator()();

  // Close vizmo debug if necessary
  if(_solver.vizmoDebug)
    VDClose();

  this->GetStatClass()->PrintClock("MPLibrary::Initialize", std::cout);
  Uninitialize();
  m_problem->SetBaseFilename(originalBaseFilename);
}

/*----------------------------------------------------------------------------*/

#endif
