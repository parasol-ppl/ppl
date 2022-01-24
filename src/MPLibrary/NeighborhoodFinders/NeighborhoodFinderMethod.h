#ifndef PMPL_NEIGHBORHOOD_FINDER_METHOD_H_
#define PMPL_NEIGHBORHOOD_FINDER_METHOD_H_

#include <boost/mpl/list.hpp>
#include <boost/mpl/for_each.hpp>

#include "Neighbors.h"
#include "Utilities/MPUtils.h"
#include "ConfigurationSpace/RoadmapGraph.h"


////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for NeighborhoodFinders, which are methods that
/// solve nearest-neighbor queries against configurations in a roadmap.
///
/// The primary function 'FindNeighbors' takes an input configuration and
/// optionally a set of candidate neighbors. It returns the computed set of
/// "nearest" neighbors and their distances (through the 'Neighbor' structure).
/// @usage
/// @code
/// NeighborhoodFinderPointer nf = this->GetNeighborhoodFinder(m_nfLabel);
/// CfgType queryCfg;
/// VertexSet candidates;
/// std::vector<Neighbor> neighbors;
/// nf->FindNeighbors(this->GetRoadmap(), queryCfg, candidates,
///                   std::back_inserter(neighbors));
/// @endcode
///
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class NeighborhoodFinderMethod : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::RoadmapType            RoadmapType;
    typedef typename RoadmapType::VID                 VID;
    typedef typename RoadmapType::VertexSet           VertexSet;
    typedef typename MPTraits::CfgType                CfgType;
    typedef typename MPTraits::GroupRoadmapType       GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType           GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    /// The type of neighbors found.
    enum class Type {
      K,       ///< k-closest neighbors
      RADIUS,  ///< All neighbors within a radius
      APPROX,  ///< Approximate nearest neighbors
      OTHER    ///< Something else
    };

    /// Output iterator for writing discovered neighbors to a container.
    typedef typename std::back_insert_iterator<std::vector<Neighbor>>
        OutputIterator;

    ///@}
    ///@name Construction
    ///@{

    NeighborhoodFinderMethod(const Type _type = Type::OTHER);

    NeighborhoodFinderMethod(XMLNode& _node, const Type _type = Type::OTHER,
        const bool _requireDM = true);

    virtual ~NeighborhoodFinderMethod() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name Accessors
    ///@{

    /// @return Type of neighborhood finder
    Type GetType() const noexcept;

    /// @return Number of closest neighbors to find
    virtual size_t& GetK() noexcept;

    /// @return Distance of farthest potential neighbor
    virtual double& GetRadius() noexcept;

    /// Set the distance metric label.
    /// @param _label The new DM label to use.
    virtual void SetDMLabel(const std::string& _label) noexcept;

    /// Get the distance metric label.
    /// @return The label for the current DM.
    virtual const std::string& GetDMLabel() const noexcept;

    ///@}
    ///@name Nearest-Neighbor Queries
    ///@{

    /// Some methods can be implemented more efficiently if the candidates are
    /// provided in a hash set. This function is to support that; the default
    /// implementation forwards to the iterator version.
    /// @param _r The roadmap.
    /// @param _cfg The query configuration.
    /// @param _candidates The set of candidate VIDs.
    /// @param _out Output iterator.
    virtual void FindNeighbors(RoadmapType* const _r, const CfgType& _cfg,
        const VertexSet& _candidates, OutputIterator _out) = 0;
    ///@example NeighborhoodFinder_UseCase.cpp
    /// This is an example of how to use the neighborhood finder methods.

    /// @overload This version is for group roadmaps.
    virtual void FindNeighbors(GroupRoadmapType* const _r,
        const GroupCfgType& _cfg, const VertexSet& _candidates,
        OutputIterator _out) = 0;

    /// @overload This verion uses the full roadmap as the candidate set.
    template <typename AbstractRoadmapType>
    void FindNeighbors(AbstractRoadmapType* const _r,
        const typename AbstractRoadmapType::CfgType& _cfg, OutputIterator _out);

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Checks if there is a direct edge to potential neighbor.
    /// @param _g The roadmap graph we are searching.
    /// @param _c The query configuration.
    /// @param _v A potential neighbor for _c.
    /// @return True if there is already a direct edge from _c to _v.
    /// @note Takes linear time in |g|.
    template <typename AbstractRoadmapType>
    bool DirectEdge(const AbstractRoadmapType* _g,
        const typename AbstractRoadmapType::CfgType& _c,
        const typename AbstractRoadmapType::VID _v) const noexcept;

    ///@}
    ///@name Internal State
    ///@{
    /// @todo Remove m_k and m_radius - these don't apply to all NFs so it
    ///       doesn't make sense to have them here. Fix design errors in
    ///       SRT method which require this.

    Type m_nfType{Type::OTHER}; ///< Type of neighborhood finder.
    size_t m_k{0};              ///< How many closest neighbors to find?
    double m_radius{0};         ///< Maximum distance of closest neighbors.

    std::string m_dmLabel;      ///< The distance metric to use.
    bool m_unconnected{false};  ///< Require neighbors with no direct edge.

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
NeighborhoodFinderMethod<MPTraits>::
NeighborhoodFinderMethod(const Type _type) : MPBaseObject<MPTraits>(),
    m_nfType(_type) {
}


template <typename MPTraits>
NeighborhoodFinderMethod<MPTraits>::
NeighborhoodFinderMethod(XMLNode& _node, const Type _type,
    const bool _requireDM) : MPBaseObject<MPTraits>(_node) {
  if(_requireDM)
    m_dmLabel = _node.Read("dmLabel", true, "", "Distance Metric Method");

  m_unconnected = _node.Read("unconnected", false, m_unconnected,
      "Require neighbors to be non-adjacent to the query configuration");

  m_nfType = _type;
  switch(_type) {
    case Type::K:
    {
      m_k = _node.Read("k", true,
          m_k, size_t(0), std::numeric_limits<size_t>::max(),
          "The number of neighbors to find. Zero for all.");
      break;
    }
    case Type::RADIUS:
    {
      m_radius = _node.Read("radius", true,
          m_radius, 0., std::numeric_limits<double>::max(),
          "Include all neighbors within this metric radius.");
      break;
    }
    default:
      break;
  }
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
NeighborhoodFinderMethod<MPTraits>::
Print(std::ostream& _os) const {
  MPBaseObject<MPTraits>::Print(_os);
  _os << "\tdmLabel: " << m_dmLabel
      << "\n\tunconnected: " << m_unconnected
      << std::endl;
}

/*-------------------------------- Accessors ---------------------------------*/

template <typename MPTraits>
inline
typename NeighborhoodFinderMethod<MPTraits>::Type
NeighborhoodFinderMethod<MPTraits>::
GetType() const noexcept {
  return m_nfType;
}


template <typename MPTraits>
inline
size_t&
NeighborhoodFinderMethod<MPTraits>::
GetK() noexcept {
  return m_k;
}


template <typename MPTraits>
inline
double&
NeighborhoodFinderMethod<MPTraits>::
GetRadius() noexcept {
  return m_radius;
}


template <typename MPTraits>
inline
void
NeighborhoodFinderMethod<MPTraits>::
SetDMLabel(const std::string& _label) noexcept {
  m_dmLabel = _label;
}


template <typename MPTraits>
inline
const std::string&
NeighborhoodFinderMethod<MPTraits>::
GetDMLabel() const noexcept {
  return m_dmLabel;
}

/*------------------------ Nearest-Neighbor Queries --------------------------*/

template <typename MPTraits>
template <typename AbstractRoadmapType>
void
NeighborhoodFinderMethod<MPTraits>::
FindNeighbors(AbstractRoadmapType* const _r,
    const typename AbstractRoadmapType::CfgType& _cfg,
    OutputIterator _out) {
  this->FindNeighbors(_r, _cfg, _r->GetAllVIDs(), _out);
}

/*-------------------------------- Helpers -----------------------------------*/

template <typename MPTraits>
template <typename AbstractRoadmapType>
inline
bool
NeighborhoodFinderMethod<MPTraits>::
DirectEdge(const AbstractRoadmapType* _g,
    const typename AbstractRoadmapType::CfgType& _c,
    const typename AbstractRoadmapType::VID _v) const noexcept {
  // The nodes are neighbors if _c is in the graph and the edge (_c, _v) exists.
  const typename AbstractRoadmapType::VID vid = _g->GetVID(_c);
  return vid != INVALID_VID and _g->IsEdge(vid, _v);
}

/*----------------------------------------------------------------------------*/

#endif
