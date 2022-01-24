#ifndef PPL_SIPP_METHOD_H_
#define PPL_SIPP_METHOD_H_

#include "MapEvaluatorMethod.h"

#include "ConfigurationSpace/Path.h"
#include "ConfigurationSpace/RoadmapGraph.h"
#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPLibrary/LocalPlanners/StraightLine.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/MPTask.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/MetricUtils.h"
#include "Utilities/SSSP.h"
#include "Geometry/Boundaries/Range.h"
#include "nonstd/io.h"
#include "MPLibrary/MPTools/SafeIntervalTool.h"

#include <unordered_map>
#include <vector>
#include <iostream>
#include <unordered_set>

////////////////////////////////////////////////////////////////////////////////
/// Map evaluator for Safe Interval Path Planning (SIPP)
/// Utilizes MPTools/SafeIntervalTools to help calculate Safe intervals
/// for vertices and edges
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class SIPPMethod : public MapEvaluatorMethod<MPTraits> {

  public:
    ///@name Motion Planning Types
    ///@{
    typedef typename MPTraits::GoalTracker GoalTracker;
    typedef typename GoalTracker::VIDSet VIDSet;

    typedef std::unordered_map<size_t,
                std::unordered_map<size_t,
                    std::vector<Range<double>>>> EdgeIntervals;

    ///@}
    ///@name Classes and Structures
    ///@{
    struct State {
      size_t vd;
      Cfg cfg;
      std::vector<Range<double>> intervals;

      /// TODO: May have to iterate through intervals to check instead.
      bool operator==(const State& _state) const {
        return vd == _state.vd &&
               cfg == _state.cfg &&
               intervals == _state.intervals;
      }

      /// TODO:
      friend ostream& operator <<(ostream& _os, const State& _state) {
        return _os;
      }

      /// TODO:
      friend istream& operator >>(istream& _is, State& _state) {
        return _is;
      }

    };
    typedef RoadmapGraph<Cfg, DefaultWeight<Cfg>> RoadmapGraphType;
    typedef RoadmapGraph<State, DefaultWeight<Cfg>> SIPPGraph;

    ///@}
    ///@name Construction
    ///@{
    SIPPMethod();

    SIPPMethod(XMLNode& _node);

    virtual ~SIPPMethod() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    virtual void Initialize() override;

    ///@}
    ///@name MapEvaluator Interface
    ///@{

    virtual bool operator()() override;

    ///@}
    ///@name Query Interface
    ///@{

    /// Generate a path from a start point to a goal point
    /// @param _start The start coordinate.
    /// @param _goal The goal coordinate.
    std::vector<size_t> GeneratePath(const size_t _start, const size_t _goal);

    /// Set the distance metric to use
    void SetDMLabel(const std::string& _dmLabel);

    /// Set the start time of the query
    void SetStartTime(double _start);

    /// Set the end time of the query
    void SetEndTime(double _end);

    /// Set the edge intervals to use to generate a path
    virtual void SetEdgeIntervals(EdgeIntervals _edgeIntervals) override;

    /// Set the minimum end time of a path
    virtual void SetMinEndtime(double _minEndtime) override;

    /// Check if the path satisfies all constraints
    bool SatisfyConstraints();

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Reset the path and list of undiscovered goals.
    /// Also resets wait times and cached safe intervals.
    virtual void Reset(RoadmapGraphType* const _r);

    /// TODO: Remove this
    /// Calculates the safe intervals for each vertex in the roadmap
    void computeIntervals();

    /// Check whether a path connecting a start to one of several goals exists
    /// in the roadmap using safe intervals.
    /// @param _start The start VID to use.
    /// @param _goals The goal VIDs to use.
    /// @return True if a path from _start to one of _goals was generated.
    virtual bool PerformSubQuery(const size_t _start, const size_t _goal);

    /// Define a function for computing path weights w.r.t. dynamic obstacles.
    /// Here the metric is the number of time steps, and we return the distance
    /// with a wait time if taking an edge would result in a collision
    /// with a dynamic obstacle. If waiting cannot fix, return infinity.
    /// @param _ei An iterator to the edge we are checking.
    /// @param _sourceDistance The shortest time to the source node.
    /// @param _targetDistance The best known time to the target node.
    /// @return The time to the target node via this edge including waiting,
    ///         or infinity if taking this edge would result in a collision
    ///         with dynamic obstacles.
    virtual double DynamicPathWeight(typename SIPPGraph::adj_edge_iterator& _ei,
        const double _sourceDistance, const double _targetDistance);

    /// Heuristic function for Safe interval path planning.
    /// Calls Dijkstra's from goal node to find shortest path to all
    /// nodes in roadmap.
    /// @param _g
    /// @param _source
    /// @param _target
    /// @return
    double SIPPHeuristic(const SIPPGraph* _g,
                         typename SIPPGraph::vertex_descriptor _source,
                         typename SIPPGraph::vertex_descriptor _target);

    /// Neighbors function for Safe interval path planning.
    void SIPPNeighbors(SIPPGraph* _g,
                       typename SIPPGraph::vertex_descriptor _vd);

    /// Initialize the cost to go from the start point to the goal
    void InitializeCostToGo(size_t _goal);

    /// Calculate the overlap between a safe interval of one configuration
    /// and the safe intervals of another configuration
    Range<double> IntervalOverlaps(std::vector<Range<double>> _intervals,
                                   Range<double> _interval);

    /// Check if any of the safe intervals in _intervals contains timestep
    Range<double> IntervalContains(std::vector<Range<double>> _intervals,
                                   double timestep);

    /// Calculates the overlapping safe intervals between _intervalsA and _intervalsB
    std::vector<Range<double>> OverlappingIntervals(std::vector<Range<double>> _intervalsA, std::vector<Range<double>> _intervalsB);

    void cleanup();
    ///@}
    ///@name Internal State
    ///@{

    RoadmapGraphType* m_roadmap{nullptr};
    SIPPGraph* m_sippGraph{nullptr};
    MPTask* m_task{nullptr};

    std::string m_safeIntervalLabel;
    std::string m_dmLabel;

    std::unordered_map<size_t,double> m_costToGoMap;

    std::unordered_map<size_t, std::vector<Range<double>>> m_roadmap_intervals;

    std::unordered_map<size_t,std::unordered_set<typename SIPPGraph::
                            vertex_descriptor>> m_sipp_mappings;

    std::unordered_map<typename RoadmapGraphType::vertex_descriptor, bool> m_changed;

    std::unordered_map<size_t,
                   unordered_map<size_t,
                   std::vector<Range<double>>>> m_edgeIntervals;

    std::vector<size_t> m_wait_timesteps;
    std::unordered_map<size_t,
                       std::unordered_map<size_t, double>> m_transitionWait;

    size_t m_goalIndex{0};
    double m_startTime{0};
    double m_endTime{0};
    double m_minEndtime{0};
    double m_pathCost{0};
    size_t m_currentGoalVID{0};

    bool m_initialized{false};
    bool m_invervalsSet{false};
    bool m_sippSet{false};

    ///@}
};

template <typename MPTraits>
std::ostream& operator<<(std::ostream& _os,
    const typename SIPPMethod<MPTraits>::State& _state);

template <typename MPTraits>
std::istream& operator>>(std::istream& _is,
    typename SIPPMethod<MPTraits>::State& _state);

template <typename MPTraits>
std::ostream& operator<<(std::ostream& _os,
    const typename SIPPMethod<MPTraits>::State& _state) {
  //_os << "VID: " << _state.vd;
  //_os << ", Intervals: " << _state.intervals;
  //_os << ", Cfg: ";
  //_state.cfg.Write(_os);
  return _os;
}

template <typename MPTraits>
std::istream& operator>>(std::istream& _is,
    typename SIPPMethod<MPTraits>::State& _state) {
  //std::string label;
  //_is >> label;
  //_is >> _state.vd;

  //_state.cfg.Read(_is);
  //_is >> _state.intervals;
  //TODO: Read in Intervals
  return _is;
}

template <typename MPTraits>
SIPPMethod<MPTraits>::
SIPPMethod() : MapEvaluatorMethod<MPTraits>() {
  this->SetName("SIPPMethod");
}

template <typename MPTraits>
SIPPMethod<MPTraits>::
SIPPMethod(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("SIPPMethod");

  // Parse options
  m_safeIntervalLabel = _node.Read("safeIntervalToolLabel", true, "",
      "Label of the SafeIntervalTool");
  m_dmLabel = _node.Read("dmLabel", false, "",
      "Alternate distance metric to replace edge weight during search.");
}



template <typename MPTraits>
void
SIPPMethod<MPTraits>::
Print(std::ostream& _os) const {
  _os << this->GetNameAndLabel() << "::"
      << "\n\tSearch Alg: astar"
      << "\n\tSI Tool Label: " << m_safeIntervalLabel
      << "\n\tAlternative DM: " << m_dmLabel
      << std::endl;
}

template <typename MPTraits>
void
SIPPMethod<MPTraits>::
Initialize() {
  if(m_initialized)
    return;
  // Clear previous state
  m_dmLabel.clear();
  Reset(nullptr);

  this->GetStatClass()->SetStat(
      "SIPPMethod::" + this->GetLabel() + "::FoundPath", 0);

  m_initialized = true;
}



template <typename MPTraits>
bool
SIPPMethod<MPTraits>::
operator()() {
  auto si = this->GetMPTools()->GetSafeIntervalTool(m_safeIntervalLabel);
  //std::cout << "Using SI Tool: "
  //          << si->GetLabel()
  //          << std::endl;
  //std::cout << m_sippGraph->Size() << std::endl;
  // Removed cached egdges if appropriate. Probably don't want to always do this.
  si->Initialize();

  auto goalTracker = this->GetGoalTracker();
  const std::vector<size_t> unreachedGoals = goalTracker->UnreachedGoalIndexes();
  auto task = this->GetTask();
  const size_t numGoals = task->GetNumGoals();

  if(goalTracker->GetStartVIDs().empty()) {
    std::cout << "No start VIDs, query cannot succeed."
              << std::endl;
    return false;
  }
  //std::cout << "here" << std::endl;
  auto r = this->GetRoadmap();
  if(unreachedGoals.empty() or r != m_roadmap or task != m_task)
    Reset(r);

  /*std::cout << "Querying roadmap for a path satisfying task '"
              << task->GetLabel()
              << "', " << unreachedGoals.size() << " / " << numGoals
              << " goals not reached."
              << "\n\tTrying to connect goal: " << m_goalIndex
              << "\n\tUnreached goals: " << unreachedGoals
              << std::endl;*/

  // Search for a sequential path through each task constraint in order.
  auto path = this->GetPath();
  for(; m_goalIndex < numGoals; ++m_goalIndex) {
    // If this goal constraint is unreached, the query cannot succeed.
    auto iter = std::find(unreachedGoals.begin(), unreachedGoals.end(),
        m_goalIndex);
    const bool unreached = iter != unreachedGoals.end();
    if(unreached) {
      std::cout << "\tGoal " << m_goalIndex << " has no satisfying VIDs."
                  << std::endl;
      return false;
    }

    // Get the start VID for this subquery.
    const size_t start = path->Empty() ? *goalTracker->GetStartVIDs().begin()
                                    : path->VIDs().back();

    // Get the goal VIDs for this subquery.
    const VIDSet& goals = goalTracker->GetGoalVIDs(m_goalIndex);
    if(goals.empty())
      throw RunTimeException(WHERE) << "No VIDs located for reached goal "
                                    << m_goalIndex << ".";

    std::cout << "\tEvaluating sub-query from " << start << " to " << goals
                << "." << std::endl;

    // Warn users if multiple goals are found.
    if(goals.size() > 1 and numGoals > 1)
      std::cerr << "\tWarning: subquery has " << goals.size() << " possible VIDs "
                << "for goal " << m_goalIndex << "/" << numGoals
                << ". The algorithm will try its best but isn't complete for "
                << "this case." << std::endl;

    //std::cout << "m_roadmap Size: " << m_roadmap->Size() << std::endl;

    computeIntervals();

    auto startCfg = m_roadmap->GetVertex(start);

    State state;
    state.vd = start;
    state.cfg = m_roadmap->GetVertex(start);
    state.intervals = {m_roadmap_intervals[start][0]};
    //auto siTool = this->GetMPTools()->GetSafeIntervalTool(m_safeIntervalLabel);
    //state.intervals = siTool->ComputeIntervals(startCfg);

    //std::cout << "Intervals: " << state.intervals[0] << std::endl;
    m_sippGraph = new SIPPGraph(state.cfg.GetRobot());
    //std::cout << "m_sippGraph Size: " << m_sippGraph->Size() << std::endl;

    auto newStart = m_sippGraph->AddVertex(state);
    //std::cout << "m_sippGraph Size: " << m_sippGraph->Size() << std::endl;
    //std::cout << "New VID: " << newVID << std::endl;
    // Perform this subquery. If it fails, there is no path.
    std::vector<size_t> g(goals.begin(),goals.end());
    const bool success = PerformSubQuery(newStart, g[0]);
    if(!success)
      return false;

  }


  // We generated a path successfully: track the path length history.
  const double timeRes = this->GetEnvironment()->GetTimeRes();
  this->GetStatClass()->AddToHistory("pathlength", this->GetPath()->Length());
  std::cout << this->GetPath()->Length() << std::endl;
  std::cout << this->GetPath()->TimeSteps() * timeRes << std::endl;
  this->GetStatClass()->SetStat(
      "QueryMethod::" + this->GetLabel() + "::FoundPath", 1);

  std::cout << "\tConnected all goals!" << std::endl;

  return true;
}

template <typename MPTraits>
std::vector<size_t>
SIPPMethod<MPTraits>::
GeneratePath(const size_t _start, const size_t _goal) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, "SIPPMethod::GeneratePath");

  // Check for trivial path.
  // if(_goal == _start && m_startTime >= m_endTime)
  //   return {_start};

  stats->IncStat("Graph Search");
  m_currentGoalVID = _goal;
  // Set up termination criterion to exit early if we find goal node.
  SSSPTerminationCriterion<SIPPGraph> termination(
      [_goal,this](typename SIPPGraph::vertex_iterator& _vi,
        const SSSPOutput<SIPPGraph>& _sssp) {
        auto sipp_state = _vi->property();
       if(_goal == sipp_state.vd && this->SatisfyConstraints())
        std::cout << "pathCost: " << this->m_pathCost << ", minEndtime: " <<  m_minEndtime << std::endl;
        return  (_goal == sipp_state.vd && this->SatisfyConstraints()) ? SSSPTermination::EndSearch
                                          : SSSPTermination::Continue;
      }
  );

  SSSPPathWeightFunction<SIPPGraph> weight = [this](
      typename SIPPGraph::adj_edge_iterator& _ei,
      const double _sourceDistance,
      const double _targetDistance) {
    return this->DynamicPathWeight(_ei, _sourceDistance, _targetDistance);
  };

  SSSPHeuristicFunction<SIPPGraph> heuristic = [this](
      const SIPPGraph* _g,
      typename SIPPGraph::vertex_descriptor _source,
      typename SIPPGraph::vertex_descriptor _target) {
    return this->SIPPHeuristic(_g, _source, _target);
  };

  SSSPNeighborsFunction<SIPPGraph> neighbors = [this](
      SIPPGraph* _g, typename SIPPGraph::vertex_descriptor _vd) {
    this->SIPPNeighbors(_g, _vd);
  };

  SSSPOutput<SIPPGraph> output;
  std::vector<size_t> starts = {_start};
  std::vector<size_t> goals = {};


  std::cout << starts << goals << std::endl;
  output = AStarSSSP(m_sippGraph,
                          starts,
                          goals,
                          weight,
                          heuristic,
                          neighbors,
                          termination);

  auto current = output.ordering.back();
  auto lastState = m_sippGraph->GetVertex(current);
  if(_goal != lastState.vd) {
    std::cout << "Failed to find a path." <<std::endl;
    return std::vector<size_t>();
  }

  //std::cout << "Printing wait times" << std::endl;
  State state = m_sippGraph->GetVertex(current);
  m_wait_timesteps.push_back(0);
  State prev = state;
  std::vector<size_t> path;
  path.push_back(state.vd);
  auto previous = current;

  while(current != _start) {
    previous = current;
    current = output.parent[current];
    state = m_sippGraph->GetVertex(current);
    m_wait_timesteps.push_back(ceil(m_transitionWait[current][previous]));
    //prev = state;
    path.push_back(state.vd);
  }

  std::reverse(path.begin(), path.end());
  std::reverse(m_wait_timesteps.begin(), m_wait_timesteps.end());

  cleanup();

  //std::cout << "Path : " << path << std::endl;
  //std::cout << "Wait: " << m_wait_timesteps << std::endl;

  return path;
}

template <typename MPTraits>
void
SIPPMethod<MPTraits>::
SetDMLabel(const std::string& _dmLabel) {
  m_dmLabel = _dmLabel;
}



template <typename MPTraits>
void
SIPPMethod<MPTraits>::
Reset(RoadmapGraphType* const _r) {
  m_roadmap = _r;
  m_task = this->GetTask();
  // TODO Do we need this
  //m_sippGraph = nullptr;

  m_goalIndex = 0;
  m_transitionWait.clear();
  m_wait_timesteps.clear();

  if(this->GetPath())
    this->GetPath()->Clear();
}

template <typename MPTraits>
bool
SIPPMethod<MPTraits>::
PerformSubQuery(const size_t _start, const size_t _goal) {
  InitializeCostToGo(_goal);
  auto path = this->GeneratePath(_start, _goal);

  // check if path is empty
  if(path.empty())
    return false;

  // add segment of path to full solution path
  *this->GetPath() += path;
  this->GetPath()->SetWaitTimes(m_wait_timesteps);
  return true;
}

template <typename MPTraits>
double
SIPPMethod<MPTraits>::
DynamicPathWeight(typename SIPPGraph::adj_edge_iterator& _ei,
    const double _sourceDistance, const double _targetDistance) {
    //std::cout << "start dynamicpathweight 1" << std::endl;
  const double timeRes = this->GetEnvironment()->GetTimeRes();

  State sourceState = m_sippGraph->GetVertex(_ei->source());
  double sourceDistance = _sourceDistance;
  if(sourceDistance > 0) {
    sourceDistance = sourceDistance - m_costToGoMap[sourceState.vd];
  }

  // Compute new distance which is the number of timesteps at which
  // the robot would reach the target node.
  const double edgeWeight  = _ei->property().GetTimeSteps(),
               newDistance = sourceDistance + edgeWeight;

  // Will not use edge if new distance is worse than pervious
  //if(newDistance >= _targetDistance) {
     // std::cout << "end dynamicpathweight 2" << std::endl;

  //  return newDistance;
  //}

  auto siTool = this->GetMPTools()->GetSafeIntervalTool(m_safeIntervalLabel);

  double waitTime = 0;
  // Ensure that target vertex is contained withing a SafeInterval when
  // arriving.

  auto t = _ei->target();
  State targetState = m_sippGraph->GetVertex(t);
  const auto& target = m_roadmap->GetVertex(targetState.vd);
  //auto vertexIntervals = siTool->ComputeIntervals(target);
  auto vertexInterval = targetState.intervals[0];
  if(!(siTool->ContainsTimestep({vertexInterval}, newDistance*timeRes))) {
    // TODO Check if its possible to wait
    // If not return inf
    // else add waiting time to new distance
    const auto srcCfg = m_roadmap->GetVertex(sourceState.vd);
    auto srcIntervals = siTool->ComputeIntervals(srcCfg);
    // Get current Interval
    double dist = sourceDistance * timeRes;
    Range<double> interval = IntervalContains(srcIntervals, dist);

    if(interval.max - interval.min >= 0) {
      // Working interval is from current timestep to final timestep
      // in current interval
      Range<double> workingInterval = Range<double>(sourceDistance,interval.max);

      // Translate interval by edgeweight to obtain new interval to check for
      // overlaps.
      workingInterval.Translate(edgeWeight);

      // Check if working interval overlaps targets intervals
      Range<double> overlap = IntervalOverlaps({vertexInterval}, workingInterval);

      if(overlap.max - overlap.min >= 0) {
        waitTime = (overlap.min - edgeWeight) - sourceDistance;
      } else {
          //std::cout << "end dynamicpathweight 3" << std::endl;

        return std::numeric_limits<double>::infinity();
      }

    } else {
        //std::cout << "end dynamicpathweight 4" << std::endl;

      return std::numeric_limits<double>::infinity();
    }
  }

  // Ensure that the edge is contained within a SafeInterval if leaving.
  auto u = _ei->source();
  State src = m_sippGraph->GetVertex(u);

  // source dist + waiting time
  bool isSafe = false;

  std::vector<Range<double>> edgeIntervals;
 // std::vector<Range<double>> cachedEdgeIntervals;
  {
    MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::RetrieveIntervals");
    //cachedEdgeIntervals = m_edgeIntervals[src.vd][targetState.vd];
    edgeIntervals = m_edgeIntervals[src.vd][targetState.vd];
    //std::cout << "edge(" << src.vd << "," << targetState.vd << ")[cached]: " << m_edgeIntervals[src.vd][targetState.vd] << std::endl;
  }
  if(edgeIntervals.empty()) {
    MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::ComputeIntervals");
    edgeIntervals = siTool->ComputeIntervals(_ei->property(), src.vd, targetState.vd, m_roadmap);
    //std::cout << "EdgeInterval were not cached for this edge. " << std::endl;
  }

  
  if(edgeIntervals.empty())
    throw RunTimeException(WHERE) << "Should always be at least one edge interval." << std::endl;

  // Look at edge intervals and get overlap with source interval
  //const auto srcCfg = m_roadmap->GetVertex(src.vd);
  //auto sourceIntervals = siTool->ComputeIntervals(srcCfg);
  auto sourceInterval = src.intervals[0];

  // iterate through in order
  std::vector<Range<double>> overlappingIntervals = OverlappingIntervals({sourceInterval}, edgeIntervals);
  //std::cout << "Overlapping Intervals size: " << overlappingIntervals.size() << std::endl;

  for(auto interval : overlappingIntervals) {
    // TODO::Change computation of overlapping intervals to avoid needing this
    if(sourceDistance*timeRes > interval.max)
      continue;

    // check if beginning of interval + edge lays in target interval
    auto startTime = std::max(sourceDistance*timeRes, interval.min);
    if((siTool->ContainsTimestep(m_roadmap_intervals[m_roadmap->GetVID(target)], (startTime+edgeWeight)))) {
    //if((siTool->ContainsTimestep(m_roadmap_intervals[m_roadmap->GetVID(target)], interval.min+edgeWeight))) {
      waitTime = std::lround(startTime/timeRes) - sourceDistance;
      isSafe = true;
      break;
    }

    // Row subtract edge from beginning of target and see if that lays in edge interval (if it does you are done and wait time is that time)
    //for(auto& r : vertexIntervals) {
    auto& r = vertexInterval;
      if(startTime <= r.min-edgeWeight && interval.max >= r.min-edgeWeight) {
      //if(interval.min <= r.min-edgeWeight && interval.max >= r.min-edgeWeight) {
        waitTime = r.min - edgeWeight - sourceDistance;
        isSafe = true;
        break;
      }
    //}
    if(isSafe) {
      break;
    }
    // set to true if ^
  }

  if(!isSafe) {
      //std::cout << "end dynamicpathweight 5" << std::endl;

    return std::numeric_limits<double>::infinity();
  }

  //m_roadmap_intervals.emplace(st.vd, vertexIntervals);
  // Edge is okay
  //m_transitionWait[src.vd][targetState.vd] = waitTime;
  m_transitionWait[_ei->source()][_ei->target()] = waitTime;
 // std::cout << "Wait: " << m_transitionWait[src.vd][targetState.vd] << std::endl;
  //std::cout << "end dynamicpathweight 6" << std::endl;

  if(waitTime < 0)
    throw RunTimeException(WHERE) << "VERY BAD. SHOULD NEVER HAPPEN." << std::endl;
  if(targetState.vd == m_currentGoalVID)
    m_pathCost = newDistance + waitTime;
  return newDistance + waitTime;
}

template <typename MPTraits>
std::vector<Range<double>>
SIPPMethod<MPTraits>::
OverlappingIntervals(std::vector<Range<double>> _intervalsA, std::vector<Range<double>> _intervalsB) {
  std::vector<Range<double>> overlap;
  for(auto& interval : _intervalsB) {
    auto ovlp = IntervalOverlaps(_intervalsA, interval);
    if(ovlp.max > ovlp.min) {
      overlap.push_back(ovlp);
    }
  }
  return overlap;
}



template <typename MPTraits>
double
SIPPMethod<MPTraits>::
SIPPHeuristic(const  SIPPGraph* _g,
              typename SIPPGraph::vertex_descriptor _source,
              typename SIPPGraph::vertex_descriptor _target) {
  State state = _g->GetVertex(_target);

  double cost = m_costToGoMap[state.vd];
  return cost;
}

template <typename MPTraits>
void
SIPPMethod<MPTraits>::
SIPPNeighbors(SIPPGraph* _g,
    typename SIPPGraph::vertex_descriptor _vd) {
  auto siTool = this->GetMPTools()->GetSafeIntervalTool(m_safeIntervalLabel);
  //std::cout << "start sippneighbors" << std::endl;
  auto vi = _g->find_vertex(_vd);
  //std::cout << _vd << std::endl;
  //std::cout << _g->Size() << std::endl;

  auto state = vi->property();
  //std::cout << "here2" << std::endl;

  auto rit = m_roadmap->find_vertex(state.vd);
  //std::cout << "sipp start: " << _vd << std::endl;
  //std::cout << "roadmap start: " << state.vd << "\n" << std::endl;

  //auto safe_intervals = m_roadmap_intervals[state.vd];
  //auto cfg = m_roadmap->GetVertex(state.vd);
  //auto safe_intervals = siTool->ComputeIntervals(cfg);;
  auto safeInterval = state.intervals[0];

  // Iterate thorugh neighbors of current state
  for(auto ei = rit->begin(); ei != rit->end(); ++ei) {
    // Check if neighbor VID in SIPP Graph
    auto target = ei->target();
    //std::cout << "roadmap target: " << target << std::endl;
    // compute SI  here
    //if(!m_changed[target]) {
      // Iterate through Intervals
      //auto t_intervals = m_roadmap_intervals[target];
      auto tcfg = m_roadmap->GetVertex(target);
      auto t_intervals = siTool->ComputeIntervals(tcfg);

      // Check if overlapping, if so
      // Create new state for neighbor with that
      // interval in SIPP Graph
      for(auto interval : t_intervals) {
        auto intvl = IntervalOverlaps({safeInterval}, interval);

        if(intvl.max > intvl.min) {
          // Calculate earliest arrival time
          //auto edgeCost = ei->property().GetWeight();
          State new_state;
          new_state.vd = target;
          //new_state.cfg = m_roadmap->GetVertex(target);
          //new_state.intervals = m_roadmap_intervals[target];
          new_state.cfg = tcfg;
          new_state.intervals = {interval};
          auto sipp_target = _g->AddVertex(new_state);
          _g->AddEdge(_vd, sipp_target, ei->property());
          //std::cout << "_g size: " << _g->Size() << std::endl;

        }
      }
    //}
  }
 // std::cout << "end sippneighbors" << std::endl;
}

template<typename MPTraits>
void
SIPPMethod<MPTraits>::
InitializeCostToGo(size_t _goal) {

  if(_goal == std::numeric_limits<size_t>::max()) {
    throw RunTimeException(WHERE) << "Goal does not exist in current \
            roadmap" << std::endl;
  }

  std::vector<size_t> starts = {_goal};

  SSSPTerminationCriterion<RoadmapGraphType> termination(
      [](typename RoadmapGraphType::vertex_iterator& _vi,
        const SSSPOutput<RoadmapGraphType>& _sssp) {
        return SSSPTermination::Continue;
      }
  );

  SSSPPathWeightFunction<RoadmapGraphType> weight = [this](
      typename RoadmapGraphType::adj_edge_iterator& _ei,
      const double _sourceDistance,
      const double _targetDistance) {
    return _sourceDistance + _ei->property().GetTimeSteps();
  };
  auto output = DijkstraSSSP(m_roadmap, starts, weight, termination);
  m_costToGoMap = output.distance;
}

template <typename MPTraits>
void
SIPPMethod<MPTraits>::
computeIntervals() {
  auto si = this->GetMPTools()->GetSafeIntervalTool(m_safeIntervalLabel);
  //std::cout << "Using SI Tool: "
  //          << si->GetLabel()
  //          << std::endl;
  //std::cout << "Computing Intervals" << std::endl;
  for(auto vi = m_roadmap->begin(); vi != m_roadmap->end(); ++vi) {
    auto vid = m_roadmap->GetVID(vi);
    auto cfg = m_roadmap->GetVertex(vi);
    auto vertexIntervals = si->ComputeIntervals(cfg);
    m_roadmap_intervals[vid] = vertexIntervals;
  }
  m_invervalsSet = true;
}

template <typename MPTraits>
Range<double>
SIPPMethod<MPTraits>::
IntervalOverlaps(std::vector<Range<double>> _intervals,
    Range<double> _interval) {
  for(auto& r : _intervals){
    double min, max;
    if(_interval.min > r.min)
      min = _interval.min;
    else
      min = r.min;

    if(_interval.max < r.max)
      max = _interval.max;
    else
      max = r.max;

    if(r.max < _interval.min || r.min > _interval.max ||
     _interval.max < r.min || _interval.min > r.max) {
       continue;
    } else {
      return Range<double>(min,max);
    }
  }
  return Range<double>(0,-1);
}
template <typename MPTraits>
Range<double>
SIPPMethod<MPTraits>::
IntervalContains(std::vector<Range<double>> _intervals,
    double timestep) {
  for(auto& range : _intervals) {
    if(range.min <= timestep && range.max >= timestep) {
      return range;
    }
  }
  return Range<double>(0,-1);
}

template <typename MPTraits>
void
SIPPMethod<MPTraits>::
cleanup() {

}


template <typename MPTraits>
void
SIPPMethod<MPTraits>::
SetEdgeIntervals(typename SIPPMethod<MPTraits>::EdgeIntervals _edgeIntervals){

  m_edgeIntervals = _edgeIntervals;
}


template <typename MPTraits>
void
SIPPMethod<MPTraits>::
SetMinEndtime(double _minEndtime){

  m_minEndtime = _minEndtime;
}


template <typename MPTraits>
bool
SIPPMethod<MPTraits>::
SatisfyConstraints(){

  return m_pathCost > m_minEndtime;
}

#endif