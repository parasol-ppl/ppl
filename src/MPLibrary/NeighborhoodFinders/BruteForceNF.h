#ifndef PMPL_BRUTE_FORCE_NF_H_
#define PMPL_BRUTE_FORCE_NF_H_

#include "NeighborhoodFinderMethod.h"

#include <queue>


////////////////////////////////////////////////////////////////////////////////
/// Determine the nearest neighbors with a brute force search.
///
/// This method does a direct distance check between the query configuration and
/// each input candidate.
///
/// @ingroup NeighborhoodFinders
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class BruteForceNF : public NeighborhoodFinderMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::RoadmapType       RoadmapType;
    typedef typename MPTraits::CfgType           CfgType;
    typedef typename RoadmapType::VID            VID;
    typedef typename RoadmapType::VertexSet      VertexSet;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;
    typedef typename MPTraits::GroupCfgType      GroupCfgType;

    ///@}
    ///@name Local Types
    ///@{

    using typename NeighborhoodFinderMethod<MPTraits>::Type;
    using typename NeighborhoodFinderMethod<MPTraits>::OutputIterator;

    ///@}
    ///@name Construction
    ///@{

    BruteForceNF();

    BruteForceNF(XMLNode& _node);

    virtual ~BruteForceNF() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name NeighborhoodFinderMethod Overrides
    ///@{

    virtual void FindNeighbors(RoadmapType* const _r, const CfgType& _cfg,
        const VertexSet& _candidates, OutputIterator _out) override;

    virtual void FindNeighbors(GroupRoadmapType* const _r,
        const GroupCfgType& _cfg, const VertexSet& _candidates,
        OutputIterator _out) override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    /// Templated implementation for both individual and group versions.
    template <typename AbstractRoadmapType>
    void FindNeighborsImpl(AbstractRoadmapType* const _r,
        const typename AbstractRoadmapType::CfgType& _cfg,
        const VertexSet& _candidates, OutputIterator _out);

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
BruteForceNF<MPTraits>::
BruteForceNF() : NeighborhoodFinderMethod<MPTraits>(Type::K) {
  this->SetName("BruteForceNF");
}


template <typename MPTraits>
BruteForceNF<MPTraits>::
BruteForceNF(XMLNode& _node) :
    NeighborhoodFinderMethod<MPTraits>(_node, Type::K) {
  this->SetName("BruteForceNF");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
BruteForceNF<MPTraits>::
Print(std::ostream& _os) const {
  NeighborhoodFinderMethod<MPTraits>::Print(_os);
  _os << "\tk: " << (this->m_k ? std::to_string(this->m_k) : "all")
      << std::endl;
}

/*-------------------- NeighborhoodFinderMethod Overrides --------------------*/

template <typename MPTraits>
void
BruteForceNF<MPTraits>::
FindNeighbors(RoadmapType* const _r, const CfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::FindNeighbors");

  this->FindNeighborsImpl(_r, _cfg, _candidates, _out);
}


template <typename MPTraits>
void
BruteForceNF<MPTraits>::
FindNeighbors(GroupRoadmapType* const _r, const GroupCfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  MethodTimer mt(this->GetStatClass(),
      this->GetNameAndLabel() + "::FindNeighbors");

  this->FindNeighborsImpl(_r, _cfg, _candidates, _out);
}

/*--------------------------------- Helpers ----------------------------------*/

template <typename MPTraits>
template <typename AbstractRoadmapType>
void
BruteForceNF<MPTraits>::
FindNeighborsImpl(AbstractRoadmapType* const _r,
    const typename AbstractRoadmapType::CfgType& _cfg,
    const VertexSet& _candidates, OutputIterator _out) {
  auto dm = this->GetDistanceMetric(this->m_dmLabel);

  if(this->m_debug)
    std::cout << "Checking for nearest "
              << (this->m_k ? std::to_string(this->m_k) : "all")
              << " neighbors with dm '" << this->m_dmLabel << "'."
              << "\n\tQuery cfg: " << _cfg.PrettyPrint()
              << std::endl;

  // Keep a max pq of the k best so far (so that we can quickly test and remove
  // the farthest).
  std::priority_queue<Neighbor> pq;

  for(const VID vid : _candidates) {
    // Check for prior connection.
    if(this->m_unconnected and this->DirectEdge(_r, _cfg, vid))
      continue;

    // Get the candidate Cfg and check against connection to self.
    const auto& node = _r->GetVertex(vid);
    if(node == _cfg)
      continue;

    // Get the distance from the query cfg to the candidate. If it is infinite,
    // these configurations are not connectable.
    const double distance = dm->Distance(node, _cfg);
    if(std::isinf(distance))
      continue;

    // Track the closest m_k neighbors.
    if(!this->m_k or pq.size() < this->m_k)
      pq.emplace(vid, distance);
    else if(distance < pq.top().distance) {
      pq.pop();
      pq.emplace(vid, distance);
    }
  }

  if(this->m_debug)
    std::cout << "\tFound " << pq.size() << " neighbors." << std::endl;

  // Write k closest to vector, sorted greatest to least distance.
  std::vector<Neighbor> closest;
  closest.reserve(pq.size());
  while(!pq.empty()) {
    closest.push_back(pq.top());
    pq.pop();
  }

  // Write to output iterator in reverse order.
  std::copy(closest.rbegin(), closest.rend(), _out);
}

/*----------------------------------------------------------------------------*/

#endif
