#ifndef PMPL_GROUP_PATH_H_
#define PMPL_GROUP_PATH_H_

#include "MPLibrary/MPLibrary.h"
#include "Utilities/PMPLExceptions.h"

#include <algorithm>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// A path of connected configurations from a given roadmap.
///
/// The implementation uses a vector of VID's as the primary representation.
/// The corresponding configurations are computed lazily upon request.
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class GroupPath final {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::GroupCfgType       GroupCfg;
    typedef typename MPTraits::GroupRoadmapType   GroupRoadmapType;
    typedef typename GroupRoadmapType::VID        VID;
    typedef typename GroupCfg::Formation          Formation;

    ///@}
    ///@name Construction
    ///@{

    /// Construct an empty path.
    /// @param _r The roadmap used by this path.
    GroupPath(GroupRoadmapType* const _r = nullptr);

    ///@}
    ///@name Path Interface
    ///@{

    /// Get the roadmap used by this path.
    GroupRoadmapType* GetRoadmap() const noexcept;

    /// Get the number of cfgs in the path.
    size_t Size() const noexcept;

    /// Check if the path is empty.
    bool Empty() const noexcept;

    /// Get the total edge weight.
    double Length() const;

    /// Get the VIDs in the path.
    const std::vector<VID>& VIDs() const noexcept;

    /// Get a copy of the Cfgs in the path.
    /// @warning If the cfgs in the roadmap are later altered (i.e., if the DOF
    ///          values or labels are edited), this copy will be out-of-date.
    const std::vector<GroupCfg>& Cfgs() const;

    /// Get the current full Cfg path with steps spaced one environment
    ///        resolution apart. This is not cached due to its size and
    ///        infrequent usage.
    /// @param _lib The planning library to use.
    /// @return The full path of configurations, including local-plan
    ///         intermediates between the roadmap nodes.
    template <typename MPLibrary>
    const std::vector<GroupCfg> FullCfgs(MPLibrary* const _lib) const;

    /// Append another path to the end of this one.
    /// @param _p The path to append.
    GroupPath& operator+=(const GroupPath& _p);

    /// Add another path to the end of this one and return the result.
    /// @param _p The path to add.
    GroupPath operator+(const GroupPath& _p) const;

    /// Append a new set of VIDs to the end of this path.
    /// @param _vids The VIDs to append.
    GroupPath& operator+=(const std::vector<VID>& _vids);

    /// Add a new set of VIDs to the end of this path and return the result.
    /// @param _vids The VIDs to add.
    GroupPath operator+(const std::vector<VID>& _vids) const;

    /// Copy assignment operator.
    GroupPath& operator=(const GroupPath& _p);

    /// Clear all data in the path.
    void Clear();

    /// Clear cached data, but leave the VIDs.
    void FlushCache();

    ///@}

  private:

    ///@name Helpers
    ///@{

    void AssertSameMap(const GroupPath& _p) const;

    ///@}
    ///@name Internal State
    ///@{

    GroupRoadmapType* const m_roadmap;     ///< The roadmap.
    std::vector<VID> m_vids;               ///< The vids of the path configurations.

    mutable std::vector<GroupCfg> m_cfgs;  ///< The path configurations.
    mutable bool m_cfgsCached{false};      ///< Are the current cfgs correct?

    mutable double m_length{0};            ///< The path length.
    mutable bool m_lengthCached{false};    ///< Is the current path length correct?

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
GroupPath<MPTraits>::
GroupPath(GroupRoadmapType* const _r) : m_roadmap(_r) { }

/*------------------------------ Path Interface ------------------------------*/

template <typename MPTraits>
typename MPTraits::GroupRoadmapType*
GroupPath<MPTraits>::
GetRoadmap() const noexcept {
  return m_roadmap;
}


template <typename MPTraits>
size_t
GroupPath<MPTraits>::
Size() const noexcept {
  return m_vids.size();
}


template <typename MPTraits>
bool
GroupPath<MPTraits>::
Empty() const noexcept {
  return m_vids.empty();
}


template <typename MPTraits>
double
GroupPath<MPTraits>::
Length() const {
  // If the length is cached, we don't need to recompute.
  if(m_lengthCached)
    return m_length;
  m_lengthCached = true;

  // Recompute the length by summing the edge weights.
  m_length = 0;
  for(auto start = m_vids.begin(); start + 1 < m_vids.end(); ++start) {
    // Skip repeated vertices.
    /// @todo This will be an error if we allow self-edges.
    if(*start == *(start + 1))
      continue;

    // Add this edge's weight to the sum.
    const auto& edge = m_roadmap->GetEdge(*start, *(start + 1));
    m_length += edge.GetWeight();
  }

  return m_length;
}


template <typename MPTraits>
const std::vector<typename GroupPath<MPTraits>::VID>&
GroupPath<MPTraits>::
VIDs() const noexcept {
  return m_vids;
}


template <typename MPTraits>
const std::vector<typename MPTraits::GroupCfgType>&
GroupPath<MPTraits>::
Cfgs() const {
  // If the cfgs are cached, we don't need to recompute.
  if(m_cfgsCached)
    return m_cfgs;
  m_cfgsCached = true;

  m_cfgs.clear();
  m_cfgs.reserve(m_vids.size());
  for(const auto& vid : m_vids)
    m_cfgs.push_back(m_roadmap->GetVertex(vid));

  return m_cfgs;
}


template <typename MPTraits>
template <typename MPLibrary>
const std::vector<typename MPTraits::GroupCfgType>
GroupPath<MPTraits>::
FullCfgs(MPLibrary* const _lib) const {
  if(m_vids.empty())
    return std::vector<GroupCfg>();

  // Insert the first vertex.
  std::vector<GroupCfg> out = {m_roadmap->GetVertex(m_vids.front())};

  for(auto it = m_vids.begin(); it + 1 < m_vids.end(); ++it) {
    const VID source = *it,
              target = *(it + 1);
    const auto& edge = m_roadmap->GetEdge(source, target);

    // Insert intermediates between vertices. For assembly planning (skip edge):
    // don't reconstruct the edge when it's for a part that has been placed off
    // to the side, just use the two cfgs. This edge will just be (start, end).
    if(!edge.SkipEdge()) {
      std::vector<GroupCfg> edge = _lib->ReconstructEdge(m_roadmap, source, target);
      out.insert(out.end(), edge.begin(), edge.end());
    }

    // Insert the next vertex.
    out.push_back(m_roadmap->GetVertex(target));
  }
  return out;
}


template <typename MPTraits>
GroupPath<MPTraits>&
GroupPath<MPTraits>::
operator+=(const GroupPath& _p) {
  AssertSameMap(_p);
  return *this += _p.m_vids;
}


template <typename MPTraits>
GroupPath<MPTraits>
GroupPath<MPTraits>::
operator+(const GroupPath& _p) const {
  AssertSameMap(_p);
  return *this + _p.m_vids;
}


template <typename MPTraits>
GroupPath<MPTraits>&
GroupPath<MPTraits>::
operator+=(const std::vector<VID>& _vids) {
  if(_vids.size()) {
    FlushCache();
    std::copy(_vids.begin(), _vids.end(), std::back_inserter(m_vids));
  }
  return *this;
}


template <typename MPTraits>
GroupPath<MPTraits>
GroupPath<MPTraits>::
operator+(const std::vector<VID>& _vids) const {
  GroupPath out(*this);
  out += _vids;
  return out;
}


template <typename MPTraits>
GroupPath<MPTraits>&
GroupPath<MPTraits>::
operator=(const GroupPath& _p) {
  if(m_roadmap != _p.m_roadmap)
    throw RunTimeException(WHERE) << "Can't assign path from another roadmap";

  m_vids         = _p.m_vids;
  m_cfgs         = _p.m_cfgs;
  m_cfgsCached   = _p.m_cfgsCached;
  m_length       = _p.m_length;
  m_lengthCached = _p.m_lengthCached;

  return *this;
}


template <typename MPTraits>
void
GroupPath<MPTraits>::
Clear() {
  FlushCache();
  m_vids.clear();
}


template <typename MPTraits>
void
GroupPath<MPTraits>::
FlushCache() {
  m_lengthCached = false;
  m_cfgsCached = false;
  m_cfgs.clear();
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
void
GroupPath<MPTraits>::
AssertSameMap(const GroupPath& _p) const {
  if(m_roadmap != _p.m_roadmap)
    throw RunTimeException(WHERE) << "Can't add paths from different roadmaps "
                                  << "(source = " << _p.m_roadmap << ","
                                  << " target = " << m_roadmap << ").";
}

/*----------------------------------------------------------------------------*/

#endif
