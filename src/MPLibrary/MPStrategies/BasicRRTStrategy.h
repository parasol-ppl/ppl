#ifndef PMPL_BASIC_RRT_STRATEGY_H_
#define PMPL_BASIC_RRT_STRATEGY_H_

#include "MPStrategyMethod.h"
#include "MPProblem/Constraints/Constraint.h"
#include "Utilities/XMLNode.h"

#ifdef PMPL_USE_MATLAB
// Short-term hack until I move this to a dedicated strategy.
#include "Simulator/MatlabMicroSimulator.h"
#endif

#include <iomanip>
#include <iterator>
#include <string>
#include <unordered_set>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// The RRT algorithm grows one or more trees from a set of root nodes to solve
/// a single-query planning problem.
///
/// References:
///   Original RRT:
///   LaValle, Steven M. "Rapidly-Exploring Random Trees: A New Tool for Path
///   Planning." TR 98-11, Computer Science Dept., Iowa State Univ., 1998.
///   RRT Connect (bi-directional):
///   James Kuffner and Steven LaValle. "RRT-Connect: An Efficient Approach to
///   Single-Query Path Planning". ICRA 2000.
///   Nonholonomic RRT:
///   Steven LaValle and James Kuffner. "Randomized Kinodynamic Planning." IJRR
///   2001.
///
/// This method supports both uni-directional and bi-directional variants.
/// Nonholonomic problems are supported with the appropriate extender and
/// uni-directional growth.
///
/// For uni-directional methods, we support an additional 'goal extension'
/// heuristic which attempts to connect configurations near the goal to the goal
/// region. This is necessary to get RRT to terminate - the alternative is to
/// rely on goal-biased sampling to eventually complete the problem, which is
/// not an efficient solution.
///
/// For bi-directional methods, the algorithm will attempt to connect trees
/// after each successful extension. The new node will be extended toward the
/// nearest neighbor in each other tree. If the extension reaches its
/// destination, the two trees will merge.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class BasicRRTStrategy : public MPStrategyMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;
    typedef typename RoadmapType::VertexSet VertexSet;

    ///@}
    ///@name Construction
    ///@{

    BasicRRTStrategy();

    BasicRRTStrategy(XMLNode& _node);

    virtual ~BasicRRTStrategy() = default;

    ///@}
    ///@name MPBaseObject overrides
    ///@{

    virtual void Print(std::ostream& _os) const;

    ///@}

  protected:

    ///@name MPStrategy Overrides
    ///@{

    virtual void Initialize() override;
    virtual void Iterate() override;

    ///@}
    ///@name Direction Helpers
    ///@{

    /// Get a random configuration to grow towards.
    virtual CfgType SelectTarget();

    /// Sample a target configuration to grow towards from an existing
    /// configuration. m_disperseTrials samples are attempted.
    /// @param _v The VID of the existing configuration.
    /// @return The sample who's growth direction yields the greatest separation
    ///         from the existing configuration's neighbors.
    /// @todo This functionality can probably be moved into a dispersed
    ///       extender, which we could call several times here.
    CfgType SelectDispersedTarget(const VID _v);

    ///@}
    ///@name Neighbor Helpers
    ///@{

    /// Find the nearest roadmap configuration to an arbitrary configuration.
    /// @param _cfg The query configuration.
    /// @param _candidates The candidate set to search, or null for whole
    ///                    roadmap.
    /// @return The VID of the roadmap configuration nearest to _cfg.
    virtual VID FindNearestNeighbor(const CfgType& _cfg,
        const VertexSet* const _candidates = nullptr);

    /// Select the best neighbor from the set of candidates returned by the NF.
    /// Default implementation selects the nearest.
    /// @param _cfg The query configuration.
    /// @param _neighbors The set of neighbors returned by the NF.
    /// @return The best of _neighbors to extend from for this method.
    virtual Neighbor SelectNeighbor(const CfgType& _cfg,
        const std::vector<Neighbor>& _neighbors);

    ///@}
    ///@name Growth Helpers
    ///@{

    /// Extend a new configuration from a nearby configuration towards a growth
    /// target.
    /// @param _nearVID The nearby configuration's VID.
    /// @param _target  The growth target.
    /// @param _lp An LPOutput for returning local planner info.
    /// @param _requireNew Require the extension to generate a new roadmap node
    ///                    (true unless connecting trees).
    /// @return The new node's VID.
    virtual VID Extend(const VID _nearVID, const CfgType& _target,
        LPOutput<MPTraits>& _lp, const bool _requireNew = true);

    /// @overload
    VID Extend(const VID _nearVID, const CfgType& _target,
        const bool _requireNew = true);

    /// Add a new configuration to the roadmap and current tree.
    /// @param _newCfg The new configuration to add.
    /// @return A pair with the added VID and a bool indicating whether the new
    ///         node was already in the map.
    virtual std::pair<VID, bool> AddNode(const CfgType& _newCfg);

    /// Add a new edge to the roadmap.
    /// @param _source The source node.
    /// @param _target The target node.
    /// @param _lpOutput The extender output.
    virtual void AddEdge(const VID _source, const VID _target,
        const LPOutput<MPTraits>& _lpOutput);

    /// Try to connect a configuration to its neighbors.
    /// @param _newVID The VID of the configuration to connect.
    void ConnectNeighbors(const VID _newVID);

    /// Try to extend a new configuration toward each goal region that is within
    /// the extender's range.
    /// @param _newVID The VID of a newly extended configuration.
    /// @note This only applies when not growing goals.
    void TryGoalExtension(const VID _newVID);

    /// Try to extend a new configuration toward a specific goal region. No-op
    /// if the goal is outside the extender's range.
    /// @param _newVID The VID of a newly extended configuration.
    /// @param _boundary The goal boundary.
    /// @note This only applies when not growing goals.
    void TryGoalExtension(const VID _newVID, const Boundary* const _boundary);

    ///@}
    ///@name Tree Helpers
    ///@{

    /// Attempt to expand the map by growing towards a target configuration from
    /// the nearest existing node.
    /// @param _target The target configuration.
    /// @return The VID of a newly created Cfg if successful, INVALID_VID
    ///         otherwise.
    VID ExpandTree(const CfgType& _target);

    /// Attempt to expand the map by growing towards a target configuration from
    /// an arbitrary existing node.
    /// @param _nearestVID The VID to grow from.
    /// @param _target The target configuration.
    /// @return The VID of a newly created Cfg if successful, INVALID_VID
    ///         otherwise.
    virtual VID ExpandTree(const VID _nearestVID, const CfgType& _target);

    /// If multiple trees exist, try to connect the current tree with the
    /// one that is nearest to a recently grown configuration.
    /// @param _recentlyGrown The VID of the recently grown configuration.
    void ConnectTrees(const VID _recentlyGrown);

    ///@}
    ///@name MP Object Labels
    ///@{

    std::string m_samplerLabel;  ///< The sampler label.
    std::string m_nfLabel;       ///< The neighborhood finder label.
    std::string m_ncLabel;       ///< The connector label (for RRG).
    std::string m_exLabel;       ///< The extender label.
    std::string m_goalDmLabel;   ///< Dm for checking goal extensions.

    std::string m_fallbackNfLabel; ///< NF for searching the active set, used if the main one fails.

    ///@}
    ///@name RRT Properties
    ///@{

    bool m_growGoals{false};      ///< Grow trees from goals.
    double m_growthFocus{0};      ///< The fraction of goal-biased expansions.
    double m_goalThreshold{0};    ///< Distance threshold for goal extension.
    size_t m_numDirections{1};    ///< Expansion directions per iteration.
    size_t m_disperseTrials{3};   ///< Sample attempts for disperse search.

    ///@}
    ///@name Internal State
    ///@{

    std::vector<VertexSet> m_trees;  ///< The current tree set.

    ///@}

};

/*----------------------------- Construction ---------------------------------*/

template <typename MPTraits>
BasicRRTStrategy<MPTraits>::
BasicRRTStrategy() {
  this->SetName("BasicRRTStrategy");
}


template <typename MPTraits>
BasicRRTStrategy<MPTraits>::
BasicRRTStrategy(XMLNode& _node) : MPStrategyMethod<MPTraits>(_node) {
  this->SetName("BasicRRTStrategy");

  // Parse RRT parameters
  m_growGoals = _node.Read("growGoals", false, m_growGoals,
      "Grow a tree each goal in addition to the start?");

  m_growthFocus = _node.Read("growthFocus", false,
      m_growthFocus, 0.0, 1.0,
      "Fraction of goal-biased iterations");
  m_numDirections = _node.Read("m", false,
      m_numDirections, size_t(1), size_t(1000),
      "Number of directions to extend");
  m_disperseTrials = _node.Read("trial", m_numDirections > 1,
      m_disperseTrials, size_t(1), size_t(1000),
      "Number of trials to get a dispersed direction");

  // Parse MP object labels
  m_samplerLabel = _node.Read("samplerLabel", true, "", "Sampler Label");
  m_nfLabel = _node.Read("nfLabel", true, "", "Neighborhood Finder");
  m_exLabel = _node.Read("extenderLabel", true, "", "Extender label");
  m_ncLabel = _node.Read("connectorLabel", false, "",
      "Connection Method for RRG-like behavior.");
  m_fallbackNfLabel = _node.Read("fallbackNfLabel", false, "",
      "Fall back NF in case the main one fails.");

  m_goalDmLabel = _node.Read("goalDmLabel", false, "",
      "Distance metric for checking goal extensions in uni-directional RRT.");
  m_goalThreshold = _node.Read("goalThreshold", false,
      m_goalThreshold, 0., std::numeric_limits<double>::max(),
      "For each extension that ends within this distance of the goal (according "
      "to the goal DM), attempt to extend towards the goal. If the value is 0 or"
      " not set, the extender's max range will be used instead.");


  // Some options only apply when growing goals
  if(m_growGoals) {
    const std::string when(" with bi-directional growth.");

    if(!m_goalDmLabel.empty())
      throw ParseException(_node.Where()) << "Cannot use goal DM" << when;
    if(m_goalThreshold)
      throw ParseException(_node.Where()) << "Cannot use goal threshold" << when;
    if(m_growthFocus)
      throw ParseException(_node.Where()) << "Cannot use growth focus" << when;
  }
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template <typename MPTraits>
void
BasicRRTStrategy<MPTraits>::
Print(std::ostream& _os) const {
  MPStrategyMethod<MPTraits>::Print(_os);
  _os << "\tSampler: " << m_samplerLabel
      << "\n\tNeighborhood Finder: " << m_nfLabel
      << "\n\tExtender: " << m_exLabel
      << "\n\tConnection Method: " << m_ncLabel
      << "\n\tGoal check DM: " << m_goalDmLabel
      << "\n\tGrow Goals: " << m_growGoals
      << "\n\tGrowth Focus: " << m_growthFocus
      << "\n\tExpansion directions / trials: " << m_numDirections
      << " / " << m_disperseTrials
      << std::endl;
}

/*-------------------------- MPStrategy overrides ----------------------------*/

template <typename MPTraits>
void
BasicRRTStrategy<MPTraits>::
Initialize() {
  // Sanity checks on grow goals option.
  if(m_growGoals) {
    // Assert that we are not using a nonholonomic robot.
    if(this->GetTask()->GetRobot()->IsNonholonomic())
      throw RunTimeException(WHERE) << "Bi-directional growth with nonholonomic "
                                    << "robots is not supported (requires a "
                                    << "steering function).";

    // Assert that we are not using a rewiring connector.
    const bool rewiring = !m_ncLabel.empty()
                      and this->GetConnector(m_ncLabel)->IsRewiring();
    if(rewiring)
      throw RunTimeException(WHERE) << "Bi-directional growth is not supported "
                                    << "with rewiring connectors (rewiring "
                                    << "connectors need to follow the parent "
                                    << "trail which doesn't make sense for a "
                                    << "non-tree roadmap.";
  }

  // Try to generate a start configuration if we have start constraints.
  const VID start = this->GenerateStart(m_samplerLabel);
  const bool noStart = start == INVALID_VID;
  #ifdef PMPL_USE_MATLAB
  if(noStart)
    throw RunTimeException(WHERE) << "A start configuration is required for "
                                  << "jointed needle planning."
                                  << std::endl;
  /// @todo Hard-coded setting of initial needle state for now, to be cleaned up
  ///       later by making a dedicated strategy for that which takes the
  ///       initial parameters via xml.
  auto g = this->GetRoadmap();
  CfgType& cfg = g->GetVertex(start);
  cfg.SetStat("insertion", .005);
  cfg.SetStat("c1-4", 0);
  cfg.SetStat("c1-3", 0);
  cfg.SetStat("c1-2", 0);
  cfg.SetStat("c1-1", 0);
  cfg.SetStat("c1-0", 0);
  cfg.SetStat("c2-4", 0);
  cfg.SetStat("c2-3", 0);
  cfg.SetStat("c2-2", 0);
  cfg.SetStat("c2-1", 0);
  cfg.SetStat("c2-0", 0);
  cfg.SetStat("c3-4", 0);
  cfg.SetStat("c3-3", 0);
  cfg.SetStat("c3-2", 0);
  cfg.SetStat("c3-1", 0);
  cfg.SetStat("c3-0", 0);
  cfg.GetRobot()->GetMatlabMicroSimulator()->SetInsertionCfg(cfg);
  #endif

  // If we are growing goals, try to generate goal configurations.
  if(m_growGoals)
    this->GenerateGoals(m_samplerLabel);

  // If we have neither start nor goals, generate a random root. Try up to 100
  // times.
  if(noStart and !m_growGoals)
  {
    // Determine which sampler to use.
    const auto& samplerLabel = this->m_querySampler.empty()
                             ? m_samplerLabel
                             : this->m_querySampler;

    auto s = this->GetSampler(m_samplerLabel);

    std::vector<CfgType> samples;
    s->Sample(1, 100, this->GetEnvironment()->GetBoundary(),
        std::back_inserter(samples));

    // If we made no samples, throw an error.
    if(samples.empty())
      throw RunTimeException(WHERE) << "Failed to generate a random root with "
                                    << "sampler '" << samplerLabel << "'.";

    // Add the configuration to the graph.
    auto g = this->GetRoadmap();
    const auto& cfg = samples[0];
    g->AddVertex(cfg);
  }
}


template <typename MPTraits>
void
BasicRRTStrategy<MPTraits>::
Iterate() {
  // Find growth target.
  const CfgType target = this->SelectTarget();

  // Expand the tree from nearest neigbor to target.
  const VID newVID = this->ExpandTree(target);
  if(newVID == INVALID_VID)
    return;

  // If growing goals, try to connect other trees to the new node. Otherwise
  // check for a goal extension.
  if(m_growGoals)
    ConnectTrees(newVID);
  else
    TryGoalExtension(newVID);
}

/*--------------------------- Direction Helpers ------------------------------*/

template <typename MPTraits>
typename MPTraits::CfgType
BasicRRTStrategy<MPTraits>::
SelectTarget() {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::SelectTarget");

  CfgType target(this->GetTask()->GetRobot());

  // Get the sampler and boundary.
  const Boundary* samplingBoundary = this->GetEnvironment()->GetBoundary();
  const std::string* samplerLabel = &m_samplerLabel;

  // Select goal growth with probability m_growthFocus.
  auto goalTracker = this->GetGoalTracker();
  const std::vector<size_t> unreachedGoals = goalTracker->UnreachedGoalIndexes();

  if(unreachedGoals.size() and DRand() < m_growthFocus) {
    // Randomly select a goal constraint boundary.
    const auto& goalConstraints = this->GetTask()->GetGoalConstraints();
    const size_t index = unreachedGoals[LRand() % unreachedGoals.size()];
    const Boundary* const b = goalConstraints[index]->GetBoundary();

    // We may eventually support constraints that cannot be described in terms
    // of a boundary, but that is outside the scope of the present
    // implementation.
    if(!b)
      throw NotImplementedException(WHERE) << "Non-boundary constraints are not "
                                           << "yet supported.";

    // If there is a query sampler, use that for goal sampling.
    if(!this->m_querySampler.empty())
      samplerLabel = &this->m_querySampler;

    if(this->m_debug)
      std::cout << "Sampling growth target from goal " << index
                << " (sampler '" << *samplerLabel << "'):"
                << std::endl;
  }
  // Otherwise, use the designated sampler with the environment boundary.
  else if(this->m_debug)
    std::cout << "Random growth target selected (sampler '" << *samplerLabel
              << "'):" << std::endl;

  std::vector<CfgType> samples;
  auto s = this->GetSampler(*samplerLabel);
  while(samples.empty())
    s->Sample(1, 100, samplingBoundary, std::back_inserter(samples));
  target = samples.front();

  if(this->m_debug)
    std::cout << "\t" << target.PrettyPrint() << std::endl;

  return target;
}


template <typename MPTraits>
typename MPTraits::CfgType
BasicRRTStrategy<MPTraits>::
SelectDispersedTarget(const VID _v) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::SelectDispersedTarget");

  // Get original cfg with vid _v and its neighbors
  auto g = this->GetRoadmap();
  const std::vector<VID> neighbors = g->GetChildren(_v);
  const CfgType& originalCfg = g->GetVertex(_v);

  // Look for the best extension direction, which is the direction with the
  // largest angular separation from any neighbor.
  CfgType bestCfg(this->GetTask()->GetRobot());
  double bestAngle = -MAX_DBL;
  for(size_t i = 0; i < m_disperseTrials; ++i) {
    // Get a random configuration
    CfgType randCfg(this->GetTask()->GetRobot());
    randCfg.GetRandomCfg(this->GetEnvironment());

    // Get the unit direction toward randCfg
    CfgType randDir = randCfg - originalCfg;
    randDir /= randDir.Magnitude();

    // Calculate the minimum angular separation between randDir and the
    // unit directions to originalCfg's neighbors
    double minAngle = MAX_DBL;
    for(auto& vid : neighbors) {
      const CfgType& neighbor = g->GetVertex(vid);

      // Get the unit direction toward neighbor
      CfgType neighborDir = neighbor - originalCfg;
      neighborDir /= neighborDir.Magnitude();

      // Compute the angle between randDir and neighborDir
      double sum{0};
      for(size_t j = 0; j < originalCfg.DOF(); ++j)
        sum += randDir[j] * neighborDir[j];
      const double angle = std::acos(sum);

      // Update minimum angle
      minAngle = std::min(minAngle, angle);
    }

    // Now minAngle is the smallest angle between randDir and any neighborDir.
    // Keep the randDir that produces the largest minAngle.
    if(bestAngle < minAngle) {
      bestAngle = minAngle;
      bestCfg = randCfg;
    }
  }

  return bestCfg;
}

/*---------------------------- Neighbor Helpers ------------------------------*/

template <typename MPTraits>
typename BasicRRTStrategy<MPTraits>::VID
BasicRRTStrategy<MPTraits>::
FindNearestNeighbor(const CfgType& _cfg, const VertexSet* const _candidates) {
  auto stats = this->GetStatClass();
  MethodTimer mt(stats, this->GetNameAndLabel() + "::FindNearestNeighbor");

  if(this->m_debug)
    std::cout << "Searching for nearest neighbors to " << _cfg.PrettyPrint()
              << " with '" << m_nfLabel << "' from "
              << (_candidates
                  ? "a set of size " + std::to_string(_candidates->size())
                  : "the full roadmap")
              << "."
              << std::endl;

  // Search for the nearest neighbors according to the NF.
  std::vector<Neighbor> neighbors;
  auto g = this->GetRoadmap();
  auto nf = this->GetNeighborhoodFinder(m_nfLabel);
  if(_candidates)
    nf->FindNeighbors(g, _cfg, *_candidates, std::back_inserter(neighbors));
  else
    nf->FindNeighbors(g, _cfg, std::back_inserter(neighbors));

  // If we found no neighbors, try the fallback NF if we have one.
  if(neighbors.empty() and !m_fallbackNfLabel.empty()) {
    auto nf = this->GetNeighborhoodFinder(m_fallbackNfLabel);
    if(_candidates)
      nf->FindNeighbors(g, _cfg, *_candidates, std::back_inserter(neighbors));
    else
      nf->FindNeighbors(g, _cfg, std::back_inserter(neighbors));
  }

  // Check for no neighbors. We really don't want this to happen - if you see
  // high numbers for this, you likely have problems with parameter or algorithm
  // selection.
  if(neighbors.empty()) {
    stats->IncStat(this->GetNameAndLabel() + "::FailedNF");
    if(this->m_debug)
      std::cout << "\tFailed to find a nearest neighbor."
                << std::endl;
    return INVALID_VID;
  }

  const Neighbor best = SelectNeighbor(_cfg, neighbors);

  if(this->m_debug) {
    const Neighbor& nearest = neighbors[0];
    std::cout << "\tFound " << neighbors.size() << " candidate neighbors."
              << std::endl
              << "\tNearest is VID " << nearest.target << " at distance "
              << std::setprecision(4) << nearest.distance << "."
              << std::endl
              << "\tBest is VID " << best.target << " at distance "
              << std::setprecision(4) << best.distance << "."
              << std::endl;
  }

  return best.target;
}


template <typename MPTraits>
Neighbor
BasicRRTStrategy<MPTraits>::
SelectNeighbor(const CfgType& _cfg, const std::vector<Neighbor>& _neighbors) {
  // Return the nearest (first).
  return _neighbors[0];
}

/*----------------------------- Growth Helpers -------------------------------*/

template <typename MPTraits>
typename BasicRRTStrategy<MPTraits>::VID
BasicRRTStrategy<MPTraits>::
Extend(const VID _nearVID, const CfgType& _target, LPOutput<MPTraits>& _lp,
    const bool _requireNew) {
  auto stats = this->GetStatClass();
  const std::string id = this->GetNameAndLabel() + "::Extend";
  MethodTimer mt(stats, id);
  stats->IncStat(id);

  auto e = this->GetExtender(m_exLabel);
  const CfgType& qNear = this->GetRoadmap()->GetVertex(_nearVID);
  CfgType qNew(this->GetTask()->GetRobot());

  const bool success = e->Extend(qNear, _target, qNew, _lp);
  if(this->m_debug)
    std::cout << "Extending from VID " << _nearVID
              << "\n\tqNear: " << qNear.PrettyPrint()
              << "\n\tExtended "
              << std::setprecision(4) << _lp.m_edge.first.GetWeight()
              << " units."
              << std::endl;

  if(!success) {
    // The extension failed to exceed the minimum distance.
    if(this->m_debug)
      std::cout << "\tNode too close, not adding." << std::endl;
    return INVALID_VID;
  }

  // The extension succeeded. Try to add the node.
  const auto extension = AddNode(qNew);

  const VID& newVID = extension.first;
  const bool nodeIsNew = extension.second;
  if(!nodeIsNew) {
    // The extension reproduced an existing node.
    if(_requireNew) {
      if(this->m_debug)
        std::cout << "\tNode already exists (" << newVID
                  << "), not adding." << std::endl;
      return INVALID_VID;
    }
    else if(this->m_debug)
      std::cout << "\tConnected to existing node " << newVID << "."
                << std::endl;
  }

  // The node was ok. Add the edge.
  AddEdge(_nearVID, newVID, _lp);

  return newVID;
}


template <typename MPTraits>
typename BasicRRTStrategy<MPTraits>::VID
BasicRRTStrategy<MPTraits>::
Extend(const VID _nearVID, const CfgType& _target, const bool _requireNew) {
  LPOutput<MPTraits> dummyLP;
  return this->Extend(_nearVID, _target, dummyLP, _requireNew);
}


template <typename MPTraits>
std::pair<typename BasicRRTStrategy<MPTraits>::VID, bool>
BasicRRTStrategy<MPTraits>::
AddNode(const CfgType& _newCfg) {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::AddNode");

  auto g = this->GetRoadmap();

  const VID lastVID = g->GetLastVID();
  const VID newVID  = g->AddVertex(_newCfg);

  const bool nodeIsNew = lastVID != g->GetLastVID();
  if(nodeIsNew) {
    if(this->m_debug)
      std::cout << "\tAdding VID " << newVID << "."
                << std::endl;
  }

  return {newVID, nodeIsNew};
}


template <typename MPTraits>
void
BasicRRTStrategy<MPTraits>::
AddEdge(const VID _source, const VID _target,
    const LPOutput<MPTraits>& _lpOutput) {
  MethodTimer mt(this->GetStatClass(), this->GetNameAndLabel() + "::AddEdge");

  if(this->m_debug)
    std::cout << "\tAdding Edge (" << _source << ", " << _target << ")."
              << std::endl;

  // Add the edge.
  auto g = this->GetRoadmap();

  // If we are growing goals, we need to add bi-directional edges for the query
  // to work (otherwise the trees join at fruitless junctions with no strong
  // connectivity between them). This also applies if we are using a
  // non-rewiring connector as in with RRG.
  const bool biDirectionalEdges = m_growGoals or
      (!m_ncLabel.empty() and !this->GetConnector(m_ncLabel)->IsRewiring());
  if(biDirectionalEdges)
    g->AddEdge(_source, _target, _lpOutput.m_edge);
  // Use a one-way edge for uni-directional RRT because superfluous back edges
  // serve no useful purpose and increase query time. This is also required for
  // problems with one-way extenders such as KinodynamicExtender.
  else
    g->AddEdge(_source, _target, _lpOutput.m_edge.first);
}


template <typename MPTraits>
void
BasicRRTStrategy<MPTraits>::
ConnectNeighbors(const VID _newVID) {
  // Make sure _newVID is valid and we have a connector.
  if(_newVID == INVALID_VID or m_ncLabel.empty())
    return;

  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::ConnectNeighbors");

  // Try to connect _newVID to its neighbors using the connector.
  this->GetConnector(m_ncLabel)->Connect(this->GetRoadmap(), _newVID);
}


template <typename MPTraits>
void
BasicRRTStrategy<MPTraits>::
TryGoalExtension(const VID _newVID) {
  // Make sure _newVID is valid.
  if(m_growGoals or m_goalDmLabel.empty() or _newVID == INVALID_VID)
    return;

  // Make sure we have goals.
  const auto& goalConstraints = this->GetTask()->GetGoalConstraints();
  if(goalConstraints.empty())
    return;

  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::TryGoalExtension");

  if(this->m_debug)
    std::cout << "Checking goal extension for new node " << _newVID << " at "
              << this->GetRoadmap()->GetVertex(_newVID).PrettyPrint()
              << std::endl;

  for(const auto& constraint : goalConstraints)
    TryGoalExtension(_newVID, constraint->GetBoundary());
}


template <typename MPTraits>
void
BasicRRTStrategy<MPTraits>::
TryGoalExtension(const VID _newVID, const Boundary* const _boundary) {
  if(!_boundary)
    throw RunTimeException(WHERE) << "Constraints which do not produce a "
                                  << "boundary are not supported.";

  // First check if _newVID is already in the goal region.
  auto g = this->GetRoadmap();
  const CfgType& cfg = g->GetVertex(_newVID);
  const bool inGoal = _boundary->InBoundary(cfg);
  if(inGoal) {
    if(this->m_debug)
      std::cout << "\tNode is already in this goal boundary." << std::endl;
    return;
  }

  // Get the nearest point to _newVID within this goal region.
  std::vector<double> data = cfg.GetData();
  _boundary->PushInside(data);
  CfgType target(cfg.GetRobot());
  target.SetData(data);

  // Check the nearest point to _newVID in each goal region. If it lies within
  // the goal threshold, try to extend towards it.
  auto dm = this->GetDistanceMetric(m_goalDmLabel);
  const double distance = dm->Distance(cfg, target),
               range = m_goalThreshold == 0.
                     ? this->GetExtender(m_exLabel)->GetMaxDistance()
                     : m_goalThreshold;


  if(this->m_debug)
    std::cout << "\tNearest goal configuration is " << distance << " / "
              << range << " units away at " << target.PrettyPrint()
              << "."
              << std::endl;

  // If we are out of range, do not attempt to extend.
  if(distance > range) {
    if(this->m_debug)
      std::cout << "\tNot attempting goal extension." << std::endl;
    return;
  }

  // Try to extend towards the target.
  const VID extended = this->Extend(_newVID, target);
  if(extended == INVALID_VID)
    return;

  // Check if we reached the goal boundary.
  const CfgType& extendedCfg = g->GetVertex(extended);
  const bool reached = _boundary->InBoundary(extendedCfg);
  if(reached) {
    if(this->m_debug)
      std::cout << "\tExtension reached goal boundary." << std::endl;
    return;
  }

  // Some extenders use a variable distance, so retry if we got part-way there.
  // Note that the original Cfg reference may have been invalidated by the graph
  // expanding!
  const double extendedDistance = dm->Distance(g->GetVertex(_newVID),
      extendedCfg);
  if(extendedDistance < distance) {
    if(this->m_debug)
      std::cout << "\tExtension made progress but did not reach goal, retrying."
                << std::endl;
    TryGoalExtension(extended, _boundary);
  }
  else if(this->m_debug)
    std::cout << "\tExtension did not make progress." << std::endl;
}

/*------------------------------ Tree Helpers --------------------------------*/

template <typename MPTraits>
typename BasicRRTStrategy<MPTraits>::VID
BasicRRTStrategy<MPTraits>::
ExpandTree(const CfgType& _target) {
  const VID nearestVID = FindNearestNeighbor(_target);
  if(nearestVID == INVALID_VID)
    return INVALID_VID;

  return this->ExpandTree(nearestVID, _target);
}


template <typename MPTraits>
typename BasicRRTStrategy<MPTraits>::VID
BasicRRTStrategy<MPTraits>::
ExpandTree(const VID _nearestVID, const CfgType& _target) {
  if(this->m_debug)
    std::cout << "Trying expansion from node " << _nearestVID << " "
         << this->GetRoadmap()->GetVertex(_nearestVID).PrettyPrint()
         << std::endl;

  // Try to extend from the _nearestVID to _target
  const VID newVID = this->Extend(_nearestVID, _target);
  if(newVID == INVALID_VID)
    return INVALID_VID;

  // Connect neighbors if we are using a connector.
  ConnectNeighbors(newVID);

  // Expand to other directions
  for(size_t i = 1; i < m_numDirections; ++i) {
    if(this->m_debug)
      std::cout << "Expanding to other directions (" << i << "/"
                << m_numDirections - 1 << "):: ";

    // Select a dispersed target and expand towards it.
    const CfgType randCfg = SelectDispersedTarget(_nearestVID);
    const VID additionalNewVID = this->Extend(_nearestVID, randCfg);

    // If we failed, move on to the next attempt.
    if(additionalNewVID == INVALID_VID)
      continue;

    // Connect neighbors if we are using a connector.
    ConnectNeighbors(additionalNewVID);
  }

  return newVID;
}


template <typename MPTraits>
void
BasicRRTStrategy<MPTraits>::
ConnectTrees(const VID _recentlyGrown) {
  auto g = this->GetRoadmap();
  auto ccTracker = g->GetCCTracker();
  const size_t ccCount = ccTracker->GetNumCCs();

  if(!m_growGoals or _recentlyGrown == INVALID_VID or ccCount == 1)
    return;

  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::ConnectTrees");

  // Get the configuration by value in case the graph's vertex vector gets
  // re-allocated.
  const CfgType qNew = g->GetVertex(_recentlyGrown);

  if(this->m_debug)
    std::cout << "Trying to connect " << ccCount - 1 << " other trees "
              << "to node " << _recentlyGrown << "."
              << std::endl;

  // Try to connect qNew to each other CC.
  VertexSet representatives = ccTracker->GetRepresentatives();
  while(representatives.size())
  {
    // Get the next representative and remove it from the set.
    const VID representative = *representatives.begin();
    representatives.erase(representatives.begin());

    // If the new vertex and this are in the same CC, there is nothing to do.
    const bool sameCC = ccTracker->InSameCC(_recentlyGrown, representative);
    if(sameCC)
      continue;

    // Get the CC associated with this representative.
    const VertexSet* const cc = ccTracker->GetCC(representative);

    // Find nearest neighbor to qNew in the other tree.
    const VID nearestVID = FindNearestNeighbor(qNew, cc);
    if(nearestVID == INVALID_VID)
      continue;

    // Try to extend from the other tree to qNew.
    this->Extend(nearestVID, qNew, false);
  }
}

/*----------------------------------------------------------------------------*/

#endif
