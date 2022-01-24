#ifndef PMPL_GROUP_LOCAL_PLAN_H_
#define PMPL_GROUP_LOCAL_PLAN_H_

#include "ConfigurationSpace/Cfg.h"
#include "ConfigurationSpace/GroupCfg.h"
#include "ConfigurationSpace/GroupRoadmap.h"
#include "ConfigurationSpace/Weight.h"
#include "MPProblem/RobotGroup/RobotGroup.h"

#include "containers/sequential/graph/graph_util.h"

#include <algorithm>
#include <iostream>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// A local plan for multiple robots. Each robot will be executing this motion
/// simultaneously, and the total time is the same for each robot.
///
/// @todo Remove the 'active robots'. All robots are always active in a group
///       edge. We do need to retain formation data for reconstructing edges.
/// @todo Remove the 'skip edge' stuff. There is no such thing as a skipped
///       edge: the disassembly code needs to be adjusted to only use the edges
///       they want.
/// @todo Rework so that we only need a robot group to construct this, in which
///       case it will have all local edges. It should only get tied to a
///       roadmap with SetGroupRoadmap after it has been added to one.
////////////////////////////////////////////////////////////////////////////////
template <typename CfgType>
class GroupLocalPlan final {

  public:

    ///@name Local Types
    ///@{

    typedef double                                  EdgeWeight;
    typedef DefaultWeight<CfgType>                  IndividualEdge;
    typedef GroupRoadmap<GroupCfg, GroupLocalPlan>  GroupRoadmapType;
    typedef std::vector<GroupCfg>                   GroupCfgPath;
    typedef std::vector<size_t>                     Formation;
    typedef size_t                                  GroupVID;

    typedef stapl::edge_descriptor_impl<size_t>     ED;

    ///@}
    ///@name Construction
    ///@{

    /// Constructs a GroupLocalPlan.
    /// @param _g The group of robots to follow this edge. Defaults to nullptr.
    /// @param _lpLabel The string label to assign to this plan. Defaults to empty string.
    /// @param _w The weight of the plan. Defaults to 0.0.
    /// @param _path The path to be given by the plan. Defaults to GroupCfgPath().
    GroupLocalPlan(GroupRoadmapType* const _g = nullptr,
        const std::string& _lpLabel = "",
        const double _w = 0.0, const GroupCfgPath& _path = GroupCfgPath());

    ///@}
    ///@name Ordering and Equality
    ///@{

    /// Check if the given plan is equal to the current.
    /// @param _w The given plan.
    virtual bool operator==(const GroupLocalPlan& _w) const noexcept;

    /// Check if the given plan is unequal to the current.
    /// @param _w The given plan.
    virtual bool operator!=(const GroupLocalPlan& _w) const noexcept;

    /// Check if the given plan is less than the current.
    /// @param _other The given plan.
    virtual bool operator<(const GroupLocalPlan& _other) const noexcept;

    ///@}
    ///@name Roadmap Edge Weight
    ///@{
    /// Get/set the numeric weight for this local plan as used when querying a
    /// roadmap.

    EdgeWeight GetWeight() const noexcept;

    void SetWeight(const EdgeWeight _w) noexcept;

    ///@}
    ///@name Misc. Interface Functions
    ///@{

    // There is no current use case where these should ever get reset to false.
    void SetSkipEdge() noexcept;
    bool SkipEdge() const noexcept;

    /// Set the list of active robots to the vector of robot indices given.
    /// @param The vector of robo indices given.
    void SetActiveRobots(const std::vector<size_t>& _indices);
    /// Get the vector of active robots indices.
    /// @return The vector of active robot indices.
    const std::vector<size_t>& GetActiveRobots() const noexcept;
    
    /// Reset the states of this object.
    void Clear() noexcept;

    /// Get the group configuration intermediates.
    GroupCfgPath& GetIntermediates() noexcept;
    /// Get the group configuration intermediates.
    const GroupCfgPath& GetIntermediates() const noexcept;

    /// Set the group configuration intermediates.
    void SetIntermediates(const GroupCfgPath& _cfgs);

    /// Get the string label of this current local plan.
    const std::string& GetLPLabel() const noexcept;
    /// Set the string label of this current local plan.
    /// @param The desired string label.
    void SetLPLabel(const std::string _label) noexcept;

    ///@}
    ///@name Individual Local Plans
    ///@{

    /// Set the individual edge for a robot to a local copy of an edge.
    /// @param _robot The robot which the edge refers to.
    /// @param _edge The edge.
    void SetEdge(const size_t _robot, IndividualEdge&& _edge);

    /// overload for Robot pointer
    void SetEdge(Robot* const _robot, IndividualEdge&& _edge);

    /// Set the individual edge for a robot to a roadmap copy of an edge.
    /// @param _robot The robot which the edge refers to.
    /// @param _ed The edge descriptor.
    void SetEdge(Robot* const _robot, const ED _ed);

    /// Get the individual edge for a robot.
    /// @param _robot The robot which the edge refers to.
    /// @return A pointer to the edge for _robot. It will be null if it has not
    ///         yet been set.
    IndividualEdge* GetEdge(Robot* const _robot);

    /// Get the individual edge for a robot.
    /// @param _robot The robot which the edge refers to.
    /// @return A pointer to the edge for _robot. It will be null if it has not
    ///         yet been set.
    const IndividualEdge* GetEdge(Robot* const _robot) const;

    ///Overloads for using index instead of robot pointer.
    IndividualEdge* GetEdge(const size_t _robotIndex);
    const IndividualEdge* GetEdge(const size_t _robotIndex) const;

    /// Get a vector of local edges in the plan.
    std::vector<IndividualEdge>& GetLocalEdges() noexcept;

    /// Get a vector of local edges' descriptors.
    std::vector<ED>& GetEdgeDescriptors() noexcept;

    /// Clear all local edges in the plan.
    void ClearLocalEdges() noexcept;

    /// Get the number of robots given in this group local plan.
    size_t GetNumRobots() const noexcept;

    ///@}
    ///@name Stapl graph interface
    ///@{

    /// This only adds weights, it doesn't take intermediates into account.
    virtual GroupLocalPlan operator+(const GroupLocalPlan& _other) const ;

    /// Get the weight of the plan.
    double Weight() const noexcept;

    ///@}
    ///@name Iteration
    ///@{
    /// Iterate over the EDs in this edge.

    typedef typename std::vector<ED>::iterator       iterator;
    typedef typename std::vector<ED>::const_iterator const_iterator;

    iterator begin() noexcept;
    iterator end() noexcept;

    const_iterator begin() const noexcept;
    const_iterator end() const noexcept;

    ///@}

  private:

    ///@name Internal State
    ///@{

    GroupRoadmapType* m_groupMap{nullptr};  ///< The robot group which follows this edge.

    std::string m_lpLabel;   ///< Label of local planner that built this edge.

    /// The edge weight.
    double m_weight{std::numeric_limits<double>::infinity()};

    GroupCfgPath m_intermediates; ///< Group cfg intermediates.

    // The ordered formation for this local plan with respect to the robots
    // in m_groupMap. The first robot in the list is assumed to be the leader.
    std::vector<size_t> m_activeRobots;

    /// Note that any edges added to m_localEdges must be valid and complete.
    std::vector<IndividualEdge> m_localEdges; ///< Edges which are not in a map.
    std::vector<ED> m_edges;           ///< Descriptors of the individual edges.

    bool m_skipEdge{false}; ///< Flag to skip full recreation in GroupPath::FullCfgs.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename CfgType>
GroupLocalPlan<CfgType>::
GroupLocalPlan(GroupRoadmapType* const _g, const std::string& _lpLabel,
    const double _w, const GroupCfgPath& _intermediates)
    : m_groupMap(_g), m_lpLabel(_lpLabel), m_weight(_w),
      m_intermediates(_intermediates)  {
  if(m_groupMap)
    m_edges.resize(m_groupMap->GetGroup()->Size(), INVALID_ED);
  else
    std::cout << "Warning: no group map provided in group LP!" << std::endl;
}

/*--------------------------- Ordering and Equality --------------------------*/

template <typename CfgType>
bool
GroupLocalPlan<CfgType>::
operator==(const GroupLocalPlan& _other) const noexcept {
  // Ensure the edges belong to the same group.
  if(m_groupMap->GetGroup() != _other.m_groupMap->GetGroup())
    return false;

  // Ensure the edges are equal.
//  for(size_t i = 0; i < m_groupMap->GetGroup()->Size(); ++i) {
  for(const size_t i : m_activeRobots) {
    // If both descriptors are valid and equal, these edges are equal.
    const auto& ed1 = m_edges[i],
              & ed2 = _other.m_edges[i];
    if(ed1 != INVALID_ED and ed2 != INVALID_ED and ed1 != ed2)
      return false;

    // If either descriptor is invalid, one of the edges must be a local edge.
    // In that case, these edges must compare value-equal.
    const auto robot1 = m_groupMap->GetGroup()->GetRobot(i),
               robot2 = _other.m_groupMap->GetGroup()->GetRobot(i);
    if(*GetEdge(robot1) != *_other.GetEdge(robot2))
      return false;
  }

  return true;
}


template <typename CfgType>
bool
GroupLocalPlan<CfgType>::
operator!=(const GroupLocalPlan& _other) const noexcept {
  return !(*this == _other);
}


template <typename CfgType>
inline
bool
GroupLocalPlan<CfgType>::
operator<(const GroupLocalPlan& _other) const noexcept {
  return GetWeight() < _other.GetWeight();
}

/*---------------------------- Roadmap Edge Weights --------------------------*/

template <typename CfgType>
typename GroupLocalPlan<CfgType>::EdgeWeight
GroupLocalPlan<CfgType>::
GetWeight() const noexcept {
  return m_weight;
}


template <typename CfgType>
void
GroupLocalPlan<CfgType>::
SetWeight(const EdgeWeight _w) noexcept {
  m_weight = _w;
}

/*------------------------- Misc Interface Functions -------------------------*/

template <typename CfgType>
void
GroupLocalPlan<CfgType>::
SetSkipEdge() noexcept {
  m_skipEdge = true;
}


template <typename CfgType>
bool
GroupLocalPlan<CfgType>::
SkipEdge() const noexcept {
  return m_skipEdge;
}


template <typename CfgType>
void
GroupLocalPlan<CfgType>::
SetActiveRobots(const std::vector<size_t>& _indices) {
  m_activeRobots = _indices;
}


template <typename CfgType>
const std::vector<size_t>&
GroupLocalPlan<CfgType>::
GetActiveRobots() const noexcept {
  return m_activeRobots;
}


template <typename CfgType>
void
GroupLocalPlan<CfgType>::
Clear() noexcept {
  // Reset the initial state variables of this object:
  m_lpLabel.clear();
  m_weight = 0.;
  m_intermediates.clear();
}


template <typename CfgType>
typename GroupLocalPlan<CfgType>::GroupCfgPath&
GroupLocalPlan<CfgType>::
GetIntermediates() noexcept {
  return m_intermediates;
}


template <typename CfgType>
const typename GroupLocalPlan<CfgType>::GroupCfgPath&
GroupLocalPlan<CfgType>::
GetIntermediates() const noexcept {
  return m_intermediates;
}


template <typename CfgType>
void
GroupLocalPlan<CfgType>::
SetIntermediates(const GroupCfgPath& _cfgs) {
  m_intermediates = _cfgs;
}


template <typename CfgType>
const std::string&
GroupLocalPlan<CfgType>::
GetLPLabel() const noexcept {
  return m_lpLabel;
}


template <typename CfgType>
void
GroupLocalPlan<CfgType>::
SetLPLabel(const std::string _label) noexcept {
  m_lpLabel = _label;
}

/*-------------------------- Individual Local Plans --------------------------*/

template <typename CfgType>
void
GroupLocalPlan<CfgType>::
SetEdge(Robot* const _robot, IndividualEdge&& _edge) {
  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);
  SetEdge(index, std::move(_edge));
}


template <typename CfgType>
void
GroupLocalPlan<CfgType>::
SetEdge(const size_t robotIndex, IndividualEdge&& _edge) {
  // Allocate space for local edges if not already done.
  m_localEdges.resize(m_groupMap->GetGroup()->Size());

  m_localEdges[robotIndex] = std::move(_edge);
  m_edges[robotIndex] = INVALID_ED;
}


template <typename CfgType>
void
GroupLocalPlan<CfgType>::
SetEdge(Robot* const _robot, const ED _ed) {
  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);

  m_edges[index] = _ed;
}


template <typename CfgType>
typename GroupLocalPlan<CfgType>::IndividualEdge*
GroupLocalPlan<CfgType>::
GetEdge(const size_t _robotIndex) {
  return const_cast<IndividualEdge*>(GetEdge(
                          this->m_groupMap->GetGroup()->GetRobot(_robotIndex)));
}


template <typename CfgType>
const typename GroupLocalPlan<CfgType>::IndividualEdge*
GroupLocalPlan<CfgType>::
GetEdge(const size_t _robotIndex) const {
  return GetEdge(this->m_groupMap->GetGroup()->GetRobot(_robotIndex));
}


template <typename CfgType>
typename GroupLocalPlan<CfgType>::IndividualEdge*
GroupLocalPlan<CfgType>::
GetEdge(Robot* const _robot) {
  return const_cast<IndividualEdge*>(GetEdge(_robot));
}


template <typename CfgType>
const typename GroupLocalPlan<CfgType>::IndividualEdge*
GroupLocalPlan<CfgType>::
GetEdge(Robot* const _robot) const {
  const size_t index = m_groupMap->GetGroup()->GetGroupIndex(_robot);

  const ED& descriptor = m_edges.at(index);
  if(descriptor != INVALID_ED)
    return &m_groupMap->GetRoadmap(index)->GetEdge(descriptor.source(),
                                                   descriptor.target());

  try {
    return &m_localEdges.at(index);
  }
  catch(const std::out_of_range&) {
    throw RunTimeException(WHERE) << "Requested individual edge for robot "
                                  << index << " (" << _robot << "), which is"
                                  << " either stationary for this LP or not "
                                  << "in the group.";
  }
}


template <typename CfgType>
std::vector<typename GroupLocalPlan<CfgType>::IndividualEdge>&
GroupLocalPlan<CfgType>::
GetLocalEdges() noexcept {
  return m_localEdges;
}


template <typename CfgType>
std::vector<typename GroupLocalPlan<CfgType>::ED>&
GroupLocalPlan<CfgType>::
GetEdgeDescriptors() noexcept {
  return m_edges;
}


template <typename CfgType>
void
GroupLocalPlan<CfgType>::
ClearLocalEdges() noexcept {
  m_localEdges.clear();
}


template <typename CfgType>
size_t
GroupLocalPlan<CfgType>::
GetNumRobots() const noexcept {
  return m_groupMap->GetGroup()->Size();
}

/*---------------------- stapl graph interface helpers -----------------------*/

template <typename CfgType>
GroupLocalPlan<CfgType>
GroupLocalPlan<CfgType>::
operator+(const GroupLocalPlan& _other) const {
  return GroupLocalPlan(m_groupMap, m_lpLabel,
                        m_weight + _other.m_weight);
}


template <typename CfgType>
double
GroupLocalPlan<CfgType>::
Weight() const noexcept {
  return GetWeight();
}

/*-------------------------------- Iteration ---------------------------------*/

template <typename CfgType>
typename GroupLocalPlan<CfgType>::iterator
GroupLocalPlan<CfgType>::
begin() noexcept {
  return m_edges.begin();
}


template <typename CfgType>
typename GroupLocalPlan<CfgType>::iterator
GroupLocalPlan<CfgType>::
end() noexcept {
  return m_edges.end();
}


template <typename CfgType>
typename GroupLocalPlan<CfgType>::const_iterator
GroupLocalPlan<CfgType>::
begin() const noexcept {
  return m_edges.begin();
}


template <typename CfgType>
typename GroupLocalPlan<CfgType>::const_iterator
GroupLocalPlan<CfgType>::
end() const noexcept {
  return m_edges.end();
}


/*------------------------------ Input/Output --------------------------------*/

template<typename CfgType>
std::ostream&
operator<<(std::ostream& _os, const GroupLocalPlan<CfgType>& _groupLP) {
  //For the group edges, the only caveat is that the intermediates need to line up.
  // Each individual edge within a GroupLocalPlan should have the same number of
  // intermediates (for now) so that we can do this easily. Then for a
  // GroupLocalPlan with n individual edges for i robots, you would print all i
  // of the nth intermediates, then the (n+1)th, etc.

  // Make a vector of edges corresponding to each robot's edge to prevent from
  // repeatedly retrieving each vector of cfgs.
#if 0
  // TODO: when group intermediates are needed, use this code (but it's untested
  //       right now). Also it should really just populate m_intermediates and
  //       then print that vector (see DefaultWeight::Write() for analogous code).
  std::vector< std::vector<CfgType> > edgeIntermediates;
  const size_t numRobots = _groupLP.GetNumRobots();
  size_t numIntermediates = 0;
  const std::vector<size_t>& activeRobots = _groupLP.GetActiveRobots();
  for(size_t i = 0; i < numRobots; ++i) {
    // Check if the robot is inactive, if so, just duplicate the start cfg:
    if(std::find(activeRobots.begin(), activeRobots.end(), i) ==
                                                           activeRobots.end()) {
      // Get the cfg that the robot is stationary at. Will be resized later to
      // account for correct number of intermediates.
      edgeIntermediates.push_back({_groupLP.GetRobotStartCfg(i)});
    }
    else {
      edgeIntermediates.push_back(_groupLP.GetEdge(i)->GetIntermediates());
      numIntermediates = edgeIntermediates.back().size();
    }
  }

  if(numIntermediates == 0)
    throw RunTimeException(WHERE, "No active robots were detected in an edge!");

  // Now all intermediate vectors of size 1 need to have their cfgs duplicated
  for(std::vector<CfgType>& intermediateVec : edgeIntermediates)
    if(intermediateVec.size() == 1)
      intermediateVec.resize(numIntermediates, intermediateVec[0]);
  // TODO: this could be optimized by not duplicating but won't be for now.

  // Now loop through all intermediates so we construct each intermediate's
  // composite cfg preserving the order of the robots in the group.
  // Note: assuming the same number of intermediates in each edge.
  for(size_t i = 0; i < numIntermediates; ++i) {
    for(size_t robot = 0; i < numRobots; ++i) {
      _os << edgeIntermediates[robot][i] << " ";
    }
  }

#endif

  _os << 0 << " "; // 0 intermediates for now.
  _os << _groupLP.Weight(); // Print out the weight.

  return _os;
}


#endif
