#ifndef LOCAL_OBSTACLE_MAP_H_
#define LOCAL_OBSTACLE_MAP_H_

#ifndef INVALID_VID
#define INVALID_VID (std::numeric_limits<size_t>::max())
#endif

#ifndef INVALID_EID
#define INVALID_EID (std::numeric_limits<size_t>::max())
#endif

#include "Utilities/MetricUtils.h"
#include "Utilities/PMPLExceptions.h"

#include <unordered_map>
#include <unordered_set>


////////////////////////////////////////////////////////////////////////////////
/// Maintains an associative mapping between free and obstacle configurations
/// to avoid running a neighborhood finder on the obstacle map (which is
/// typically huge).
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class LocalObstacleMapType final {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::RoadmapType RoadmapType;
    typedef typename RoadmapType::VID      VID;

    ///@}
    ///@name Local Types
    ///@{

    typedef std::unordered_set<VID>            VertexSet;
    typedef std::unordered_map<VID, VertexSet> MapType;

    ///@}
    ///@name Construction
    ///@{

    LocalObstacleMapType(StatClass* const _stats);

    ///@}
    ///@name Accessors
    ///@{

    /// Get the local obstacle map for a given free vertex.
    /// @param _vid A free vertex.
    /// @return The local obstacle map for _vid.
    const VertexSet& Get(const VID& _vid) const;

    /// Get the aggregate local obstacle map for a set of free vertices.
    /// @param _c A container of free vertices.
    /// @return The aggregate local obstacle map for all vertices in _c.
    template <typename ContainerType>
    const VertexSet Get(const ContainerType& _c) const;

    /// Get the inverse local obstacle map for a given obstacle vertex.
    /// @param _vid An obstacle vertex.
    /// @return The inverse obstacle map for _vid.
    const VertexSet& Inverse(const VID& _vid) const;

    /// Get the aggregate inverse local obstacle map for a set of obstacle
    /// vertices.
    /// @param _c A set of obstacle vertices.
    /// @return The aggregate inverse obstacle map for all vertices in _c.
    template <typename ContainerType>
    const VertexSet Inverse(const ContainerType& _c) const;

    ///@}
    ///@name Addition
    ///@{
    /// Add a new obstacle configuration to the local obstacle map of a given
    /// free node.
    /// @param _obst The new obstacle node to add.
    /// @param _free The free node to add it to.

    void Add(const VID& _obst, const VID& _free);

    template <typename ContainerType>
    void Add(const ContainerType& _obst, const VID& _free);

    template <typename ContainerType>
    void Add(const VID& _obst, const ContainerType& _free);

    template <typename ContainerType>
    void Add(const ContainerType& _obst, const ContainerType& _free);

    ///@}
    ///@name Deletion
    ///@{
    /// Delete an obstacle configuration from one or more local obstacle maps.
    /// @param _obst The obstacle VID(s) to delete, or INVALID_VID to delete all.
    /// @param _free The free VID(s) to delete from, or INVALID_VID to delete
    ///              from all nodes.

    void Delete(const VID& _obst = INVALID_VID, const VID& _free = INVALID_VID);

    template <typename ContainerType>
    void Delete(const VID& _obst, const ContainerType& _free);

    template <typename ContainerType>
    void Delete(const ContainerType& _obst, const VID& _free = INVALID_VID);

    template <typename ContainerType>
    void Delete(const ContainerType& _obst, const ContainerType& _free);

    ///@}

  private:

    ///@name Internal State
    ///@{

    MapType m_map;     ///< Maps free to obstacle cfgs.
    MapType m_inverse; ///< Maps obstacle to free cfgs.

    StatClass* m_stats{nullptr};          ///< The stat class.

    static constexpr bool s_debug{false}; ///< Enable debugging messages?

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
LocalObstacleMapType<MPTraits>::
LocalObstacleMapType(StatClass* const _stats) : m_stats(_stats) { }

/*-------------------------------- Accessors ---------------------------------*/

template <typename MPTraits>
const typename LocalObstacleMapType<MPTraits>::VertexSet&
LocalObstacleMapType<MPTraits>::
Get(const VID& _vid) const {
  // If _vid isn't in the LOM, this call initializes its empty map.
  auto& map = const_cast<MapType&>(m_map);
  return map[_vid];
}


template <typename MPTraits>
template <typename ContainerType>
const typename LocalObstacleMapType<MPTraits>::VertexSet
LocalObstacleMapType<MPTraits>::
Get(const ContainerType& _c) const {
  VertexSet aggregate;
  for(const auto& vid : _c) {
    const auto& lom = Get(vid);
    aggregate.insert(lom.begin(), lom.end());
  }
  return aggregate;
}


template <typename MPTraits>
const typename LocalObstacleMapType<MPTraits>::VertexSet&
LocalObstacleMapType<MPTraits>::
Inverse(const VID& _vid) const {
  // If _vid isn't in the inverse LOM, this call initializes its empty map.
  auto& map = const_cast<MapType&>(m_inverse);
  return map[_vid];
}


template <typename MPTraits>
template <typename ContainerType>
const typename LocalObstacleMapType<MPTraits>::VertexSet
LocalObstacleMapType<MPTraits>::
Inverse(const ContainerType& _c) const {
  VertexSet aggregate;
  for(const auto& vid : _c) {
    const auto& inv = Inverse(vid);
    aggregate.insert(inv.begin(), inv.end());
  }
  return aggregate;
}

/*-------------------------------- Addition ----------------------------------*/

template <typename MPTraits>
void
LocalObstacleMapType<MPTraits>::
Add(const VID& _obst, const VID& _free) {
  MethodTimer mt(m_stats, "LocalObstacleMapType::Add");

  // Try to add the obstacle node to the map
  auto result = m_map[_free].insert(_obst);

  // If success, update the inverse map as well.
  if(result.second) {
    m_inverse[_obst].insert(_free);
    if(s_debug)
      std::cout << "\tAdding obstacle node " << _obst
                << " to the LOM of free node " << _free << "."
                << std::endl;
  }
  else if(s_debug)
    std::cout << "\tObstacle node " << _obst << " already exists in the LOM of "
              << "free node " << _free << ", not adding."
              << std::endl;
}


template <typename MPTraits>
template <typename ContainerType>
void
LocalObstacleMapType<MPTraits>::
Add(const ContainerType& _obst, const VID& _free) {
  for(const auto& vid : _obst)
    Add(vid, _free);
}


template <typename MPTraits>
template <typename ContainerType>
void
LocalObstacleMapType<MPTraits>::
Add(const VID& _obst, const ContainerType& _free) {
  for(const auto& vid : _free)
    Add(_obst, vid);
}


template <typename MPTraits>
template <typename ContainerType>
void
LocalObstacleMapType<MPTraits>::
Add(const ContainerType& _obst, const ContainerType& _free) {
  for(const auto& vid : _free)
    Add(_obst, vid);
}

/*------------------------------- Deletion -----------------------------------*/

template <typename MPTraits>
void
LocalObstacleMapType<MPTraits>::
Delete(const VID& _obst, const VID& _free) {
  MethodTimer mt(m_stats, "LocalObstacleMapType::Delete");

  const bool clearAll = _obst == INVALID_VID,
             fromAll  = _free == INVALID_VID;

  if(s_debug)
    std::cout << "\tAttempting to delete "
              << (clearAll ? "all obstacle nodes" :
                             "obstacle node " + std::to_string(_obst))
              << " from "
              << (fromAll ? "all local obstacle maps... " :
                            std::to_string(_free) + "'s local obstacle map...")
              << std::endl;

  // If not clearing all, check that the obstacle node exists
  if(!clearAll && !m_inverse.count(_obst))
    throw RunTimeException(WHERE,
        "The obstacle VID " + std::to_string(_obst) + " was not found!.");

  // If not clearing from all, check that the free node exists
  if(!fromAll && !m_map.count(_free))
    throw RunTimeException(WHERE,
        "The free VID " + std::to_string(_free) + " was not found!.");

  const unsigned short test = clearAll * 10 + fromAll;
  switch(test) {
    case 0:  // clear one obst VID from one free node
      m_map[_free].erase(_obst);
      m_inverse[_obst].erase(_free);
      break;
    case 1:  // clear one obst VID from all free nodes
      {
        auto iter = m_inverse.find(_obst);
        for(auto& free : iter->second)
          m_map[free].erase(_obst);
        m_inverse.erase(iter);
      }
      break;
    case 10: // clear all obst VIDs from one free node
      m_map.erase(_free);
      break;
    case 11: // clear all obst VIDs from all free nodes
      m_map.clear();
      m_inverse.clear();
  }

  if(s_debug)
    std::cout << "\t\tSuccess!" << std::endl;
}


template <typename MPTraits>
template <typename ContainerType>
void
LocalObstacleMapType<MPTraits>::
Delete(const VID& _obst, const ContainerType& _free) {
  for(const auto& vid : _free)
    Delete(_obst, vid);
}


template <typename MPTraits>
template <typename ContainerType>
void
LocalObstacleMapType<MPTraits>::
Delete(const ContainerType& _obst, const VID& _free) {
  for(const auto& vid : _obst)
    Delete(vid, _free);
}


template <typename MPTraits>
template <typename ContainerType>
void
LocalObstacleMapType<MPTraits>::
Delete(const ContainerType& _obst, const ContainerType& _free) {
  for(const auto& vid : _free)
    Delete(_obst, vid);
}

/*----------------------------------------------------------------------------*/

#endif
