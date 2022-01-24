#ifndef PMPL_SSSP_H_
#define PMPL_SSSP_H_

#include <functional>
#include <iomanip>
#include <iostream>
#include <limits>
#include <list>
#include <memory>
#include <queue>
#include <set>
#include <unordered_map>
#include <unordered_set>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// A descriptor-based adjacency map. Can model alternative adjacency mappings
/// and successors from an SSSP run.
////////////////////////////////////////////////////////////////////////////////
template <typename GraphType>
using SSSPAdjacencyMap =
      std::unordered_map<typename GraphType::vertex_descriptor,
                         std::vector<typename GraphType::vertex_descriptor>>;


////////////////////////////////////////////////////////////////////////////
/// The output of an SSSP run.
////////////////////////////////////////////////////////////////////////////
template <typename GraphType>
struct SSSPOutput {

  typedef typename GraphType::vertex_descriptor VD;
  typedef std::unordered_map<VD, double>        DistanceMap;
  typedef std::vector<VD>                       Ordering;
  typedef SSSPAdjacencyMap<GraphType>           Adjacency;
  typedef std::unordered_map<VD, VD>            ParentMap;

  DistanceMap distance;  ///< Distance to each cell from start.
  Ordering ordering;     ///< Cell discovery ordering.
  Adjacency successors;  ///< Maps predecessor -> successors.
  ParentMap parent;      ///< Maps successor -> parent.

};


////////////////////////////////////////////////////////////////////////////////
/// The possible early-termination conditions for an SSSP run.
////////////////////////////////////////////////////////////////////////////////
enum class SSSPTermination {
  Continue,   ///< Proceed as usual.
  EndBranch,  ///< End the branch at this vertex and do not relax its neighbors.
  EndSearch   ///< End the entire search process.
};


/// Debug output for termination criteria.
std::ostream& operator<<(std::ostream& _os, const SSSPTermination& _t);


////////////////////////////////////////////////////////////////////////////////
/// Define an early termination criterion as a function which takes a vertex
/// iterator and the current output object.
////////////////////////////////////////////////////////////////////////////////
template <typename GraphType>
using SSSPTerminationCriterion =
      std::function<SSSPTermination(typename GraphType::vertex_iterator&,
                                    const SSSPOutput<GraphType>& _sssp)>;


/// Create a standard SSSP stop criterion.
/// @return A termination criterion which never stops early.
template <typename GraphType>
SSSPTerminationCriterion<GraphType>
SSSPDefaultTermination() {
  return [](typename GraphType::vertex_iterator&, const SSSPOutput<GraphType>&)
         {
           return SSSPTermination::Continue;
         };
}


////////////////////////////////////////////////////////////////////////////////
/// Define a path weight function for SSSP. This takes an edge iterator and the
/// best path distances to the source and target nodes found so far. It should
/// return the new target path distance that would be achieved by using this
/// edge instead of the previous best. Use infinity to represent an
/// untraversable edge or max double to represent a worst-case (but still
/// traversable) edge.
////////////////////////////////////////////////////////////////////////////////
template <typename GraphType>
using SSSPPathWeightFunction =
      std::function<double(typename GraphType::adj_edge_iterator&, const double,
                           const double)>;


/// Create a standard SSSP weight function.
/// @return A path weight function which adds the edge property weight to the
///         source distance.
template <typename GraphType>
SSSPPathWeightFunction<GraphType>
SSSPDefaultPathWeight() {
  return [](typename GraphType::adj_edge_iterator& _ei,
            const double _sourceDistance,
            const double _targetDistance)
         {
           const double edgeWeight  = _ei->property().GetWeight(),
                        newDistance = _sourceDistance + edgeWeight;
           return newDistance;
         };
}

////////////////////////////////////////////////////////////////////////////////
/// Define heuristic function for SSSP. This takes the graph, source node and
/// target node. It should return a value representing some estimate of the cost
/// left to finish the search.
////////////////////////////////////////////////////////////////////////////////
template <typename GraphType>
using SSSPHeuristicFunction =
      std::function<double(
          const GraphType* g,
          typename GraphType::vertex_descriptor source,
          typename GraphType::vertex_descriptor target)>;

/// Create a standard SSSP heuristic function.
/// @return A heuristic function of 0 for every node
template <typename GraphType>
SSSPHeuristicFunction<GraphType>
SSSPDefaultHeuristic() {
  return [](const GraphType* _g,
            typename GraphType::vertex_descriptor _source,
            typename GraphType::vertex_descriptor _target)
  {
    return 0.0;
  };
}


////////////////////////////////////////////////////////////////////////////////
/// Define neighbors function for SSSP. This takes in the graph and the current
/// node to get the neighbors of. This should return a vertex iterator of the
/// neighbors.
////////////////////////////////////////////////////////////////////////////////
template <typename GraphType>
using SSSPNeighborsFunction =
      std::function<void(
                GraphType* g,
                typename GraphType::vertex_descriptor vd)>;

/// Create a standard SSSP neighbors function.
/// @return
template <typename GraphType>
SSSPNeighborsFunction<GraphType>
SSSPDefaultNeighbors() {
  return [](GraphType* _g,
            typename GraphType::vertex_descriptor _vd){};
}



/// Compute the SSSP through a graph from a start node to a goal node with
/// A* algorithm
/// @param _g The graph pointer.
/// @param _starts The vertex descriptors to start from.
/// @param _goals The vertex descriptors to find shortest path to.
/// @param _weight The function for determining total path weight.
/// @param _heuristic The function for determining heuristic cost.
/// @param _neighbors The function for getting neighbors of a given node.
/// @return  An output object which contains discovered information.
template<typename GraphType>
SSSPOutput<GraphType>
AStarSSSP(
    GraphType* const _g,
    const std::vector<typename GraphType::vertex_descriptor>& _starts,
    std::vector<typename GraphType::vertex_descriptor>& _goals,
    SSSPPathWeightFunction<GraphType>& _weight,
    SSSPHeuristicFunction<GraphType>& _heuristic,
    SSSPNeighborsFunction<GraphType>& _neighbors,
    SSSPTerminationCriterion<GraphType>& _earlyStop,
    const double _startDistance = 0)
{
  //static constexpr bool debug = true;
  //if(debug)
  //  std::cout << "AStarSSSP" << std::endl;

  using VD = typename GraphType::vertex_descriptor;
  using EI = typename GraphType::adj_edge_iterator;

  /// An element in the priority queue for A*, representing one instance of
  /// discovering a cell. Cells may be discovered multiple times from different,
  /// parents with different distances.
  struct element {
    VD parent;      ///< The parent cell descriptor.
    VD vd;          ///< This cell descriptor.
    double g;       ///< Computed distance to this cell at time of insertion.
    double h;       ///< Computed heuristic at time of insertion.

    /// Construct an element for the search queue.
    /// @param _parent The parent node descriptor.
    /// @param _target The target node descriptor.
    /// @param _g The distance from starting node to this node.
    /// @param _h The heuristic value for this node.
    element(const VD _parent, const VD _target, const double _g,
            const double _h) : parent(_parent), vd(_target), g(_g), h(_h) {}

    /// Total ordering by decreasing f = g + h
    bool operator>(const element& _e) const noexcept {
      return g + h > _e.g + _e.h;
    }
  };

  std::priority_queue<element,
                      std::vector<element>,
                      std::greater<element>> frontier;

  std::unordered_set<VD> seen;
  std::unordered_map<VD, double> cost;

  SSSPOutput <GraphType> output;

  auto relax = [&_g, &cost, &frontier, &_weight, &_heuristic](EI& _ei) {
    const VD source = _ei->source(),
             target = _ei->target();

    const double sourceCost = cost[source],
                 targetCost = cost.count(target)
                            ? cost[target]
                            : std::numeric_limits<double>::infinity(),
                 newG       = _weight(_ei, sourceCost, targetCost),
                 newH       = _heuristic(_g, source, target);

    if(newG + newH >= targetCost)
      return;

    cost[target] = newG + newH;
    frontier.emplace(source, target, newG, newH);
  };

  // Initialize each starting node
  for(const auto start : _starts) {
    cost[start] = _startDistance;
    // TODO should heuristic be 0 at start? may need to change later
    frontier.emplace(start, start, 0, 0);
  }

  // A*
  while(!frontier.empty()) {
    // Get the next element
    element current = frontier.top();
    frontier.pop();

    // If seen this node, it is stale. Discard.
    if(seen.count(current.vd))
      continue;
    seen.insert(current.vd);

    output.ordering.push_back(current.vd);
    output.distance[current.vd] = cost[current.vd];
    output.successors[current.parent].push_back(current.vd);
    output.parent[current.vd] = current.parent;

    // Get vertex iterator for current vertex
    _neighbors(_g, current.vd);
    auto vi = _g->find_vertex(current.vd);
    // Check for early termination
    auto stop = _earlyStop(vi, output);

    /*if(debug)
      std::cout << "\tVertex: " << current.vd
                << ", parent: " << current.parent
                << ", score: " << std::setprecision(4) << cost[current.vd]
                << ", stop: " << stop
                << std::endl;*/
    if(stop == SSSPTermination::Continue)
      // Continue search as usual
      ;
    else if(stop == SSSPTermination::EndBranch)
      // End this branch (do not relax neighbors)
      continue;
    else if(stop == SSSPTermination::EndSearch)
      // End the entire search
      break;

    // Relax each outgoing edge
    for(auto ei = vi->begin(); ei != vi->end(); ++ei) {
      relax(ei);
    }
  }

  // The _starts were added to their own successor map - fix that now.
  for(const auto start: _starts) {
    auto& startMap = output.successors[start];
    startMap.erase(std::find(startMap.begin(), startMap.end(), start));
  }

  return output;
}

/// Compute the SSSP through a graph from a start node to a goal node with
/// A* algorithm
/// @param _g The graph pointer.
/// @param _starts The vertex descriptors to start from.
/// @param _goals The vertex descriptors to find shortest path to.
/// @param _weight The function for determining total path weight.
/// @param _heuristic The function for determining heuristic cost.
/// @param _neighbors The function for getting neighbors of a given node.
/// @return  An output object which contains discovered information.
template<typename GraphType>
SSSPOutput<GraphType>
AStarSSSP(
    GraphType* const _g,
    const std::vector<typename GraphType::vertex_descriptor>& _starts,
    std::vector<typename GraphType::vertex_descriptor>& _goals,
    SSSPPathWeightFunction<GraphType>& _weight,
    SSSPTerminationCriterion<GraphType>& _earlyStop) {
  auto _neighbors = SSSPDefaultNeighbors<GraphType>();
  auto _heuristic = SSSPDefaultHeuristic<GraphType>();

  return AStarSSSP(_g, _starts, _goals, _weight, _heuristic, _neighbors, _earlyStop);
}

/// Compute the SSSP through a graph from a start node with Dijkstra's algorithm.
/// @param _g The graph pointer.
/// @param _starts The vertex descriptors to start from.
/// @param _weight The function for determining total path weight.
/// @param _earlyStop The early termination criterion.
/// @return An output object which contains the discovered information.
template <typename GraphType>
SSSPOutput<GraphType>
DijkstraSSSP(
    GraphType* const _g,
    const std::vector<typename GraphType::vertex_descriptor>& _starts,
    SSSPPathWeightFunction<GraphType>& _weight,
    SSSPTerminationCriterion<GraphType>& _earlyStop,
    const SSSPAdjacencyMap<GraphType>& _adjacencyMap = {},
		const double _startDistance = 0)
{
  static constexpr bool debug = false;
  const bool customAdjacency = !_adjacencyMap.empty();
  if(debug)
    std::cout << "DijkstraSSSP" << std::endl;

  using VD = typename GraphType::vertex_descriptor;
  using EI = typename GraphType::adj_edge_iterator;

  /// An element in the PQ for dijkstra's, representing one instance of
  /// discovering a cell. Cells may be discovered multiple times from different
  /// parents with different distances.
  struct element {

    VD parent;         ///< The parent cell descriptor.
    VD vd;             ///< This cell descriptor.
    double distance;   ///< Computed distance at the time of insertion.

    /// Construct an element for the search queue.
    /// @param _parent The parent node descriptor.
    /// @param _target The target node descriptor.
    /// @param _distance The distance from parent to target.
    element(const VD _parent, const VD _target, const double _distance)
      : parent(_parent), vd(_target), distance(_distance) {}

    /// Total ordering by decreasing distance.
    bool operator>(const element& _e) const noexcept {
      return distance > _e.distance;
    }

  };

  // Define a min priority queue for dijkstras. We will not update elements when
  // better distances are found - instead we will track the most up-to-date
  // distance and ignore elements with different values. This is effectively a
  // lazy delete of stale elements.
  std::priority_queue<element,
                      std::vector<element>,
                      std::greater<element>> pq;

  // Initialize visited and temporary distance maps. The later holds an *exact*
  // copy of the most up-to-date distance for each node. The absence of an entry
  // will be taken as the initial state.
  std::unordered_set<VD> visited;
  std::unordered_map<VD, double> distance;

  // Initialize the output object.
  SSSPOutput<GraphType> output;

  // Define a relax edge function.
  auto relax = [&distance, &pq, &_weight](EI& _ei) {
    const VD source = _ei->source(),
             target = _ei->target();

    const double sourceDistance = distance[source],
                 targetDistance = distance.count(target)
                                ? distance[target]
                                : std::numeric_limits<double>::infinity(),
                 newDistance    = _weight(_ei, sourceDistance, targetDistance);

    // If the new distance isn't better, quit.
    if(newDistance >= targetDistance)
      return;

    // Otherwise, update target distance and add the target to the queue.
    distance[target] = newDistance;
    pq.emplace(source, target, newDistance);
  };

  // Initialize each start node.
  for(const auto start : _starts) {
    distance[start] = _startDistance;
    pq.emplace(start, start, 0);
  }

  // Dijkstras.
  while(!pq.empty()) {
    // Get the next element.
    element current = pq.top();
    pq.pop();

    // If we are done with this node, the element is stale. Discard.
    if(visited.count(current.vd))
      continue;
    visited.insert(current.vd);

    // Save this score and successor relationship.
    output.ordering.push_back(current.vd);
    output.distance[current.vd] = distance[current.vd];
    output.successors[current.parent].push_back(current.vd);
    output.parent[current.vd] = current.parent;

    // Check for early termination.
    auto vi = _g->find_vertex(current.vd);
    auto stop = _earlyStop(vi, output);

    if(debug)
      std::cout << "\tVertex: " << current.vd
                << ", parent: " << current.parent
                << ", score: " << std::setprecision(4) << distance[current.vd]
                << ", stop: " << stop
                << std::endl;

    if(stop == SSSPTermination::Continue)
      // Continue the search as usual.
      ;
    else if(stop == SSSPTermination::EndBranch)
      // End this branch (do not relax neighbors).
      continue;
    else if(stop == SSSPTermination::EndSearch)
      // End the entire search.
      break;

    // Relax each outgoing edge.
    for(auto ei = vi->begin(); ei != vi->end(); ++ei) {
      // If we are not using custom adjacency, simply relax the edge.
      if(!customAdjacency)
        relax(ei);
      // Otherwise, only relax if this edge appears in _adjacencyMap.
      else if(_adjacencyMap.count(current.vd)) {
        const auto& neighbors = _adjacencyMap.at(current.vd);
        auto iter = std::find(neighbors.begin(), neighbors.end(), ei->target());
        if(iter != neighbors.end())
          relax(ei);
      }
    }
  }

  // The _starts were added to their own successor map - fix that now.
  for(const auto start : _starts) {
    auto& startMap = output.successors[start];
    startMap.erase(std::find(startMap.begin(), startMap.end(), start));
  }

  return output;
}


/// Compute the SSSP through a graph from a start node with Dijkstra's algorithm.
/// Use the standard termination criterion.
/// @param _g The graph pointer.
/// @param _starts The vertex descriptors to start from.
/// @param _weight The function for determining total path weight.
/// @return An output object which contains the discovered information.
template <typename GraphType>
SSSPOutput<GraphType>
DijkstraSSSP(
    GraphType* const _g,
    const std::vector<typename GraphType::vertex_descriptor>& _starts,
    SSSPPathWeightFunction<GraphType>& _weight,
    const SSSPAdjacencyMap<GraphType>& _adjacencyMap = {})
{
  auto stop = SSSPDefaultTermination<GraphType>();

  return DijkstraSSSP(_g, _starts, _weight, stop, _adjacencyMap);
}


/// Compute the SSSP through a graph from a start node with Dijkstra's algorithm.
/// Use the standard path weight.
/// @param _g The graph pointer.
/// @param _starts The vertex descriptors to start from.
/// @param _stop The termination criterion.
/// @return An output object which contains the discovered information.
template <typename GraphType>
SSSPOutput<GraphType>
DijkstraSSSP(
    GraphType* const _g,
    const std::vector<typename GraphType::vertex_descriptor>& _starts,
    SSSPTerminationCriterion<GraphType>& _stop,
    const SSSPAdjacencyMap<GraphType>& _adjacencyMap = {})
{
  auto weight = SSSPDefaultPathWeight<GraphType>();

  return DijkstraSSSP(_g, _starts, weight, _stop, _adjacencyMap);
}


/// Compute the SSSP through a graph from a start node with Dijkstra's algorithm.
/// Use the standard path weight and termination criterion.
/// @param _g The graph pointer.
/// @param _starts The vertex descriptors to start from.
/// @return An output object which contains the discovered information.
template <typename GraphType>
SSSPOutput<GraphType>
DijkstraSSSP(
    GraphType* const _g,
    const std::vector<typename GraphType::vertex_descriptor>& _starts,
    const SSSPAdjacencyMap<GraphType>& _adjacencyMap = {})
{
  auto weight = SSSPDefaultPathWeight<GraphType>();
  auto stop   = SSSPDefaultTermination<GraphType>();

  return DijkstraSSSP(_g, _starts, weight, stop, _adjacencyMap);
}



////////////////////////////////////////////////////////////////////////////////
/// The output of a two variable SSSP run. Iterate back through the list
/// nodes to construct the path.
////////////////////////////////////////////////////////////////////////////////

struct TwoVariableSSSPNode {
  size_t m_vid;
  double m_distance;
  std::shared_ptr<TwoVariableSSSPNode> m_parent;
  double m_waitTimeSteps{0};

  TwoVariableSSSPNode(size_t _vid, double _distance, std::shared_ptr<TwoVariableSSSPNode> _parent)
    : m_vid(_vid), m_distance(_distance), m_parent(_parent) {}

};

template <typename GraphType>
std::shared_ptr<TwoVariableSSSPNode>
TwoVariableDijkstraSSSP(
    GraphType* const _g,
    const std::vector<typename GraphType::vertex_descriptor>& _starts,
    std::unordered_set<size_t> _goals,
    const double _startTime,
    const double _minEndTime,
    const double _lastConstraint,
    const double _lastGoalConstraint,
    SSSPPathWeightFunction<GraphType>& _weight)
{
  using NodePtr = std::shared_ptr<TwoVariableSSSPNode>;

  // Set up a priority queue for the elements.
  auto compare = [](const NodePtr& _a, const NodePtr& _b) {
    return _a->m_distance > _b->m_distance;
  };
  std::priority_queue<NodePtr,
                      std::deque<NodePtr>,
                      decltype(compare)> pq(compare);

  // Set up structures to track visitation and distance.
  std::unordered_set<size_t> visitedPostConstraints;
  std::unordered_map<double, std::unordered_set<size_t>> discoveredVertices;

  for(const size_t vid : _starts)
  {
    pq.emplace(new TwoVariableSSSPNode(vid, _startTime, nullptr));
    discoveredVertices[_startTime].insert(vid);
  }

  NodePtr current;

  //while(!_goals.count(current->m_vid) or current->m_distance < _minEndTime)
  while(!pq.empty())
  {
    // Get the next node.
    current = pq.top();
    pq.pop();

    // Extract the current VID and distance.
    const size_t vid      = current->m_vid;
    const double distance = current->m_distance;

    // Check if we're past the last constraint on the goal location.
    const bool pastLastGoalConstraint = distance > _lastGoalConstraint;

    // If this is a goal and we're past the last constraint on the goal location, 
    // we're done (we need to continue searching otherwise to ensure we don't violate 
    // a constraint while sitting on the goal).
    //if(pastLastConstraint and _goals.count(vid))
    if(pastLastGoalConstraint and _goals.count(vid))
    {
      // Wait until the minimum end time if needed.
      current->m_waitTimeSteps = std::max(0., _minEndTime - distance);
      return current;
    }

    // Check if we're past the last constraint.
    const bool pastLastConstraint = distance > _lastConstraint;

    // If we're past the last constraint, we start tracking visited status since
    // we can no longer benefit from revisitation.
    if(pastLastConstraint)
    {
      // Skip this node if we've already visited after the last constraint.
      if(visitedPostConstraints.count(vid))
        continue;
      // Mark this node as visited.
      visitedPostConstraints.insert(vid);
    }

    /// @todo If we want to add waiting, it should be added as a self edge here
    // Add children of the current node to the queue.
    auto vit = _g->find_vertex(vid);
    for(auto eit = vit->begin(); eit != vit->end(); eit++)
    {
      const size_t target = eit->target();
      // TODO When we're past the last constraint, we might know the best
      //      distance to this target which could avoid extraneous conflict
      //      checking.
      // Use a dummy infinite distance to force revisiting this node regardless
      // of the prior best path.
      const double bestTargetDistance = std::numeric_limits<double>::infinity();

      // Compute the distance to the target through this path and edge.
      const double newDistance = _weight(eit, distance, bestTargetDistance);

      // If the new distance is better and we haven't tried it already.
      if(newDistance < bestTargetDistance and
          !discoveredVertices[newDistance].count(target))
      {
        discoveredVertices[newDistance].insert(target);
        pq.emplace(new TwoVariableSSSPNode(target, newDistance, current));
      }
    }
  }

  return nullptr;
}


#endif
