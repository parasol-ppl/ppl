#ifndef ADAPTIVE_RRT_H_
#define ADAPTIVE_RRT_H_

#include <stdint.h>
#include "BasicRRTStrategy.h"

// assembly code to measure cpu cycles
static inline uint64_t GetCycles(){
  uint64_t n;
  __asm__ __volatile__ ("rdtsc" : "=A"(n));
  return n;
}

////////////////////////////////////////////////////////////////////////////////
/// Adaptively selects growth methods in RRT.
///
/// References:
///   Denny, Jory, et al. "Adapting RRT growth for heterogeneous environments." 2013
///   IEEE/RSJ International Conference on Intelligent Robots and Systems. IEEE,
///   2013.
///
/// AdaptiveRRT employs structural filtering to the RRT paradigm by providing a
/// two-level cost-adaptive strategy to select the RRT growth method. First,
/// it uses the "visibility" of a node the method selects a set of RRT methods
/// to choose from based on some probability distribution. This probability
/// distribution is updated based upon the success/fail of the growth and its
/// cost.
///
/// @ingroup MotionPlanningStrategies
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class AdaptiveRRT : public BasicRRTStrategy<MPTraits> {
  public:
    ///@name Motion Planning Types
    ///@{

    // GrowthSet: map<Growth Method, pair<Weight Tuple, Cost>>
    typedef std::map<std::string, std::pair<std::pair<double, long>, double>> GrowthSet;
    // GrowthSets: map<Visibility Threshold, GrowthSet>
    typedef std::map<double, GrowthSet> GrowthSets;

    typedef typename MPTraits::CfgType      CfgType;
    typedef typename MPTraits::WeightType   WeightType;
    typedef typename MPTraits::RoadmapType  RoadmapType;
    typedef typename RoadmapType::VID       VID;

    ///@}
    ///@name Local Types
    ///@{

    //cost calculation method for AdaptiveRRT
    enum CostMethod {FIXED, REWARD, CYCLES};

    ///@}
    ///@ Construction
    ///@{

    AdaptiveRRT(double _wallPenalty = 0.5, double _gamma = 0.5,
        const GrowthSets& _growthSets = GrowthSets(), CostMethod _c = FIXED);

    AdaptiveRRT(XMLNode& _node);

    //AdaptiveRRT();

    virtual ~AdaptiveRRT() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const;

    ///@}


  protected:
    ///@name MPStrategy Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name Tree Helpers
    ///@{

    /// Expand tree in the direction of a given configuration
    /// @param _dir Direction of random configuration
    /// @return VID of newly created Cfg if successful, otherwise INVALID_VID
    virtual VID ExpandTree(CfgType& _dir);

    /// Select growth method from a set of growth methods
    /// @param _gs The growth set containing the growth method we want to access
    /// @return The string indicator for the chosen growth method
    std::string SelectGrowthMethod(GrowthSet& _gs);

    /// Update average cost of growth method
    /// @param _cost Average cost used to update cost of growth method
    /// @param _s String indicator for growth method
    /// @param _gs The growth set containing the growth method we want to access
    void UpdateCost(double _cost, std::string _s, GrowthSet& _gs);

    /// Update weight of growth method given a reward value
    /// @param _r Reward value
    /// @param _s String indicator for growth method
    /// @param _gs The growth set containing the growth method we want to access
    void RewardGrowthMethod(double _r, std::string _s, GrowthSet& _gs);


    /// Updates tree based on expansion type
    /// @param _nearest Vertex ID of nearest configuration
    /// @param _new New configuration to be added to tree
    /// @param _dir Direction of random configuration
    /// @param _delta Expansion distance between _nearest and _new
    /// @return The VID of the newly added configuration
    VID UpdateTree(VID _nearest, CfgType& _new, CfgType& _dir, double _delta);

    /// Adds node to tree and updates visibility
    /// @param _newCfg New configuration to add to tree
    /// @param _nearVID Vertex ID of nearest tree configuration
    /// @param _againstWall Bool flag indicating if _newCfg is against a wall.
    /// @param _ratio Reward ratio
    /// @return The VID of the configuration that was added to the tree
    VID AddNode(CfgType& _newCfg, VID _nearVID, bool _againstWall,
        double _ratio);

    ///@}


  private:

    ///@name Helpers
    ///@{
    /// Updates configuration's running average of visibility with a given value
    /// @param _cfg Configuration who's visibility we want to update
    /// @param _val The value we want to incorporate into the running average of
    ///             visibility
    void AvgVisibility(CfgType& _cfg, double _val);

    /// Calculates cost insensitive probability for a growth method
    /// @param _s String indicator for growth method
    /// @param _gs The growth set containing the growth method we want to access
    /// @return Cost insensitive probability calculated for growth method
    double CostInsensitiveProb(std::string _s, GrowthSet& _gs);

    ///@}
    ///@name Accessors
    ///@{

    /// Gets the visibility of a configuration
    /// @param _cfg Configuration that we want to look up visibility for
    /// @return Visibility value of configuration
    double GetVisibility(CfgType& _cfg);

    /// Gets the cost of a growth method
    /// @param _gs The growth set containing the growth method we want to access
    /// @param _s String indicator for growth method
    /// @return Cost of growth method
    double GetCost(std::string _s, GrowthSet& _gs);

    ///@}
    ///@name Adaptive RRT Properties
    ///@{

    double m_wallPenalty;      ///< Penalty for hitting C-obst, initial visibility for nodes which extend into C-obst

    double m_gamma;            ///< Weighting factor on probability

    GrowthSets m_growthSets;   ///< Growth Strategy set pairs

    CostMethod m_costMethod;   ///< Cost calculation method

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
AdaptiveRRT<MPTraits>::
AdaptiveRRT(double _wallPenalty, double _gamma, const GrowthSets& _growthSets,
    CostMethod _c) : BasicRRTStrategy<MPTraits>(),
  m_wallPenalty(_wallPenalty), m_gamma(_gamma), m_growthSets(_growthSets),
  m_costMethod(_c) {
    this->SetName("AdaptiveRRT");
  }

template <typename MPTraits>
AdaptiveRRT<MPTraits>::
AdaptiveRRT(XMLNode& _node) : BasicRRTStrategy<MPTraits>(_node){
  this->SetName("AdaptiveRRT");

  // Parse growth methods and store into growth sets
  for(auto& child : _node) {
    if(child.Name() == "GrowthSet"){
      double threshold = child.Read("threshold", true, 0.0, 0.0,
          1.0, "Threshold of visibility for selecting GrowthSet");
      GrowthSet growthSet;
      for(auto& child2 : child) {
        if(child2.Name() == "Extender"){
          std::string label = child2.Read("label", true, "", "Extender strategy");
          growthSet[label] = std::make_pair(std::make_pair(0, 0), 1.0);
        }
      }
      m_growthSets[threshold] = growthSet;
    }
  }

  // Parse Adaptive RRT parameters
  m_wallPenalty = _node.Read("wallPenalty", false, 0.5, 0.0, 1.0,
      "Initial visibility for nodes agains C-obst");
  m_gamma = _node.Read("gamma", true, 0.5, 0.0, 1.0,
      "Gamma for adaptivity formulas");
  std::string costMethod = _node.Read("cost", true, "fixed", "Cost method");
  transform(costMethod.begin(), costMethod.end(),
      costMethod.begin(), ::tolower);

  if(costMethod == "fixed")
    m_costMethod = FIXED;
  else if(costMethod == "reward")
    m_costMethod = REWARD;
  else if(costMethod == "cycles")
    m_costMethod = CYCLES;
  else
    throw ParseException(_node.Where(), "Unknown cost method '" + costMethod +
        "'. Choices are 'fixed', 'reward', or 'cycles'.");
}
/*
template <typename MPTraits>
AdaptiveRRT<MPTraits>::
AdaptiveRRT() {
  this->SetName("AdaptiveRRT");
}
*/

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
AdaptiveRRT<MPTraits>::Print(std::ostream& _os) const {
  BasicRRTStrategy<MPTraits>::Print(_os);
  _os << "\tWallPenalty::" << m_wallPenalty << std::endl;
  _os << "\tGamma::" << m_gamma << std::endl;
  _os << "\tGrowthSets::" << std::endl;
  typedef typename GrowthSets::const_iterator GSIT;
  typedef typename GrowthSet::const_iterator GIT;
  for(GSIT gsit = m_growthSets.begin(); gsit!=m_growthSets.end(); ++gsit){
    _os << "\t\tGrowthSet::" << gsit->first << std::endl;
    for(GIT git = gsit->second.begin(); git!=gsit->second.end(); ++git){
      _os << "\t\t\t" << git->first << std::endl;
    }
  }
  _os << "\tCostMethod::";
  if(m_costMethod == FIXED) _os << "fixed";
  else if(m_costMethod == REWARD) _os << "reward";
  else _os << "cycles";
  _os << std::endl;
}

/*---------------------------- MPStrategy Overrides --------------------------*/

template <typename MPTraits>
void
AdaptiveRRT<MPTraits>::Initialize(){
  BasicRRTStrategy<MPTraits>::Initialize();

  // Initizlie variables for each root of the graph
  auto rdmp = this->GetRoadmap();
  for(auto v = rdmp->begin(); v!=rdmp->end(); v++){
    v->property().SetStat("Success", 1);
    v->property().SetStat("Fail", 0);
    v->property().SetStat("Visibility", 1);
  }
}

/*-------------------------------- Tree Helpers ------------------------------*/

template <typename MPTraits>
typename AdaptiveRRT<MPTraits>::VID
AdaptiveRRT<MPTraits>::ExpandTree(CfgType& _dir){
  // Setup MP Variables
  //TODO: auto dm = this->GetDistanceMetric(this->m_dmLabel);
  auto dm = this->GetDistanceMetric(this->m_goalDmLabel);
  VID recentVID = INVALID_VID;

  // get the nearest expand node from the roadmap
  auto nf = this->GetNeighborhoodFinder(this->m_nfLabel);
  //TODO: std::vector<std::pair<VID, double> > kClosest;

  std::vector<Neighbor> kClosest;
  nf->FindNeighbors(this->GetRoadmap(), _dir, std::back_inserter(kClosest));
  /*
  TODO:
  nf->FindNeighbors(this->GetRoadmap(),
      this->m_trees.begin(), this->m_trees.end(),
      this->m_trees.size() ==
      this->GetRoadmap()->get_num_vertices(),
      _dir, std::back_inserter(kClosest));
  */
  CfgType& nearest = this->GetRoadmap()->GetVertex(kClosest[0].source);

  // get visiblity of nearest node to decide growth method
  double visibility = GetVisibility(nearest);

  if(this->m_debug)
    std::cout << "nearest:: " << nearest << "\tvisibility:: " << visibility << std::endl;

  double minDist = this->GetExtender(this->m_exLabel)->GetMinDistance();
  if(dm->Distance(_dir, nearest) < minDist){
    // found a random configuration that was too close.
    nearest.IncrementStat("Fail");
    // visiblity of nearest configuration is penalized with 0
    AvgVisibility(nearest, 0);
    return recentVID;
  }

  // select the growth set based upon the visibility of the nearest node
  GrowthSets::reverse_iterator rgsit = m_growthSets.rbegin();
  for(; rgsit!=m_growthSets.rend(); ++rgsit){
    if(visibility > rgsit->first){
      break;
    }
  }

  // select the growth method from the selected growth set
  std::string gm = SelectGrowthMethod(rgsit->second);

  // start timing from cycles
  uint64_t start = GetCycles();

  // grow the RRT using gm growth method
  CfgType newCfg(this->GetTask()->GetRobot());
  LPOutput<MPTraits> lpOutput;
  auto e = this->GetExtender(gm);
  bool verifiedValid = e->Extend(nearest, _dir, newCfg, lpOutput);
  double delta = e->GetMaxDistance();

  // end timing from cycles
  uint64_t end = GetCycles();
  uint64_t cost = end - start;

  //update cost based upon cycles
  if(m_costMethod == CYCLES)
    UpdateCost(cost, gm, rgsit->second);

  // if expansion fails, penalize nearest configuration with 0
  if(!verifiedValid) {
    if(this->m_debug)
      std::cout << "Growth Failed on::" << nearest << ", with visibility::"
        << visibility << std::endl;
    nearest.IncrementStat("Fail");
    AvgVisibility(nearest, 0);

    if(m_costMethod == REWARD)
      UpdateCost(delta, gm, rgsit->second);

    // reward growth method based on expanded distance in proportion to delta_q
    RewardGrowthMethod(-minDist, gm, rgsit->second);

    return recentVID;
  }

  // Distance between nearest configuration and new configuration
  double dist = dm->Distance(newCfg, nearest);

  // update weight of growth method
  if(m_costMethod == REWARD)
    UpdateCost(max(delta - dist, 0.0) + 1E-6, gm, rgsit->second);

  if(dist >= minDist) {
    // Tree expansion was successful
    nearest.IncrementStat("Success");
    // update the tree with the new configuration
    recentVID = UpdateTree(kClosest[0].source, newCfg, _dir, delta);
    //TODO:: if(recentVID > this->m_trees.back()) {
    if(recentVID > this->GetRoadmap()->GetLastVID()) {
      // if node does not exist in roadmap, add to tree.
      this->m_trees[0].insert(recentVID);
      // reward growth method based on expanded distance in proportion to delta_q
      RewardGrowthMethod(dist/delta, gm, rgsit->second);
    }
    else {
      // if node already exists in the roadmap. decrement reward
      RewardGrowthMethod(-minDist, gm, rgsit->second);
    }
  }
  else{
    // could not expand tree because distance was smaller than minDist.
    nearest.IncrementStat("Fail");

    // penalize nearest configuration's visibility with 0
    AvgVisibility(nearest, 0);
    RewardGrowthMethod(-minDist, gm, rgsit->second);
  }

  return recentVID;
}

template <typename MPTraits>
std::string
AdaptiveRRT<MPTraits>::SelectGrowthMethod(GrowthSet& _gs){
  if(this->m_debug)
    std::cout << ":::::Selecting Growth Method:::::" << std::endl;

  std::map<std::string, double> pistars, pis;
  double spc = 0.0;

  // compute p_i*/c_i and the sum over all growth methods
  for(GrowthSet::const_iterator gsit = _gs.begin(); gsit!=_gs.end(); ++gsit){
    if(this->m_debug)
      std::cout << "Method::" << gsit->first
        << "::Cost::" << GetCost(gsit->first, _gs)
        << "::Weight::" << gsit->second.second << std::endl;
    double pistar = CostInsensitiveProb(gsit->first, _gs)/GetCost(gsit->first,
        _gs);
    pistars[gsit->first] = pistar;
    spc += pistar;
  }

  // compute p_i for each method
  for(GrowthSet::const_iterator gsit = _gs.begin(); gsit!=_gs.end(); ++gsit){
    pis[gsit->first] = pistars[gsit->first]/spc;
    if(this->m_debug)
      std::cout << "Method::" << gsit->first
        << "::Prob::" << pis[gsit->first] << std::endl;
  }

  if(this->m_debug)
    std::cout << std::endl << std::endl;

  // select method based upon probability
  double r = DRand(), cumm = 0.0;
  for(std::map<std::string, double>::const_iterator mit = pis.begin(); mit!=pis.end();
      ++mit){
    cumm += mit->second;
    if(r <= cumm) {
      if(this->m_debug) {
        std::cout << "r::" << r << std::endl;
        std::cout << "MethodSelected::" << mit->first << std::endl;
      }
      return mit->first;
    }
  }

  std::cerr << "Error::Growth method not selected." << std::endl;
  std::exit(1);
}

template <typename MPTraits>
void
AdaptiveRRT<MPTraits>::UpdateCost(double _cost, std::string _s, GrowthSet& _gs){
  // update the average cost of the growth method
  if(_gs[_s].first.second == 0){
    _gs[_s].first.second = 1;
    _gs[_s].first.first = _cost;
  }
  else{
    _gs[_s].first.first += _cost;
    _gs[_s].first.second++;
  }
}

template <typename MPTraits>
void
AdaptiveRRT<MPTraits>::RewardGrowthMethod(double _r, std::string _s, GrowthSet& _gs){
  if(this->m_debug)
    std::cout << "Reward::" << _s << "::" << _r << std::endl;
  // factor is e^(g * x_i' / K) where g is gamma, x_i' = x_i/p_i* where x_i is
  // the reward r, and K is the number of growth methods
  double factor = std::exp(m_gamma * (_r/CostInsensitiveProb(_s, _gs)) /
      double(_gs.size()));

  // update the weight on growth method _s
  _gs[_s].second *= factor;
}

template <typename MPTraits>
typename AdaptiveRRT<MPTraits>::VID
AdaptiveRRT<MPTraits>::UpdateTree(VID _expandNode, CfgType& _newCfg,
    CfgType& _dir, double _delta){
  auto dm = this->GetDistanceMetric(this->m_goalDmLabel);

  CfgType& nearest = this->GetRoadmap()->GetVertex(_expandNode);

  double visibility = GetVisibility(nearest);
  double distToNear = dm->Distance(nearest, _newCfg);
  // if expansion did not reach at least delta * visibility and q_new is not
  // q_rand. Then it will be a partial expansion
  bool partial = distToNear < _delta && _newCfg != _dir;
  double ratio = distToNear / _delta;

  VID returnVID = AddNode(_newCfg, _expandNode, partial, ratio);

  if(this->m_debug){
    std::cout << "near vid::" << _expandNode << "\tvisibility::" << visibility
      << std::endl;
    std::cout << "new vid::" << returnVID << "\tvisibility::"
      << GetVisibility(_newCfg) << std::endl;
    std::cout << "expansionRatio::" << ratio << std::endl;
  }

  return returnVID;
}

template <typename MPTraits>
typename AdaptiveRRT<MPTraits>::VID
AdaptiveRRT<MPTraits>::AddNode(CfgType& _newCfg, VID _nearVID,
    bool _againstWall, double _ratio){
  auto rdmp = this->GetRoadmap();

  // add the vertex to the graph
  VID newVID = rdmp->AddVertex(_newCfg);

  // calculate edge weight and add edge
  CfgType& parentcfg = rdmp->GetVertex(_nearVID);
  if(this->m_debug)
    std::cout << "parentcfg::" << parentcfg << std::endl;
  int weight;
  CfgType incr(this->GetTask()->GetRobot());
  Environment* env = this->GetEnvironment();
  incr.FindIncrement(parentcfg, _newCfg, &weight, env->GetPositionRes(),
      env->GetOrientationRes());
  std::pair<WeightType, WeightType> weights = std::make_pair(WeightType("",
        weight), WeightType("", weight));
  rdmp->AddEdge(_nearVID, newVID, weights);

  // update the visibility at the new node and near node
  CfgType& newcfg = rdmp->GetVertex(newVID);

  // The new node is against a wall. Set visibility of new node to wall penalty.
  // Update parent with expansion ratio
  if(_againstWall){
    newcfg.SetStat("Visibility", m_wallPenalty);
    AvgVisibility(parentcfg, _ratio);
  }
  // newCfg inherits half of parents visibility and half of the ratio currently
  // parent is updated with ratio (mostly 1 in this case)
  else{
    newcfg.SetStat("Visibility", (_ratio + parentcfg.GetStat("Visibility"))/
        2.0);
    AvgVisibility(parentcfg, _ratio);
  }

  // initialize success and fail
  newcfg.SetStat("Success", 1);
  newcfg.SetStat("Fail", 0);
  VDAddTempEdge(parentcfg, _newCfg);

  return newVID;
}


/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
void
AdaptiveRRT<MPTraits>::AvgVisibility(CfgType& _cfg, double _val){
  // note: this function assumes that the total attempts has already been updated
  // which is why we multiply by (total-1) and divide by (total) instead of (total)
  // and (total+1)
  double success = _cfg.GetStat("Success");
  double fail = _cfg.GetStat("Fail");
  double total = success+fail;
  _cfg.SetStat("Visibility", (_cfg.GetStat("Visibility")*(total-1) + _val)/
      total);
}


template <typename MPTraits>
double
AdaptiveRRT<MPTraits>::CostInsensitiveProb(std::string _s, GrowthSet& _gs){
  // calculate cost insensitive probability of growth method
  double sw = 0.0;
  for(GrowthSet::const_iterator gsit = _gs.begin(); gsit!=_gs.end(); ++gsit){
    sw+=log(gsit->second.second+1);
  }
  return (1.-m_gamma)*log(_gs[_s].second+1)/sw + m_gamma*double(_gs.size());
}

/*--------------------------------- Accessors --------------------------------*/
template <typename MPTraits>
double
AdaptiveRRT<MPTraits>::GetCost(std::string _s, GrowthSet& _gs){
  if(m_costMethod == FIXED)
    return 1;
  if(_gs[_s].first.second == 0)
    return 1;
  else
    return _gs[_s].first.first/(double)_gs[_s].first.second;
}

template <typename MPTraits>
double
AdaptiveRRT<MPTraits>::GetVisibility(CfgType& _cfg){
  // Visibility is stored with the cfg. Isolate definition of visibility to this
  // function.
  if(!_cfg.IsStat("Visibility")){
    std::cerr << "Warning::Visibility not a statistic of node::" << _cfg << std::endl;
    std::cerr << "Setting and Returning 1 as its visibility." << std::endl;
    _cfg.SetStat("Success", 1);
    _cfg.SetStat("Fail", 0);
    _cfg.SetStat("Visibility", 1);
  }
  return _cfg.GetStat("Visibility");
}

#endif
