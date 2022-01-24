#ifndef PMPL_NEIGHBORHOOD_CONNECTOR_H_
#define PMPL_NEIGHBORHOOD_CONNECTOR_H_

#include "ConnectorMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Connect nearby neighbors together. In this method, the 'second set' of
/// vertices referred to by ConnectorMethod is determined by a nearest neighbors
/// method.
///
/// Connect nodes in map to their neighbors. The following algorithm is used:
/// - for each node, cfg1, in roadmap
///     - find neighbors N for cfg1
///     - lp is a local planner
///     - for each node cfg2 in N and numFailures < m_fail
///         - test lp.IsConnected(cfg1, cfg2)
///         - if connected, add this edge to map, _r.
/// @ingroup Connectors
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class NeighborhoodConnector: public ConnectorMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType           CfgType;
    typedef typename MPTraits::RoadmapType       RoadmapType;
    typedef typename MPTraits::GroupCfgType      GroupCfgType;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;
    typedef typename RoadmapType::VID            VID;
    typedef typename RoadmapType::VertexSet      VertexSet;

    ///@}
    ///@name Local Types
    ///@{

    template <typename AbstractRoadmapType>
    using OutputIterator = typename ConnectorMethod<MPTraits>::template
                           OutputIterator<AbstractRoadmapType>;

    ///@}
    ///@name Construction
    ///@{

    NeighborhoodConnector();

    NeighborhoodConnector(XMLNode& _node);

    virtual ~NeighborhoodConnector() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}

  protected:

    ///@name ConnectorMethod Overrides
    ///@{

    virtual void ConnectImpl(RoadmapType* const _r, const VID _source,
        const VertexSet* const _targetSet = nullptr,
        OutputIterator<RoadmapType>* const _collision = nullptr) override;

    virtual void ConnectImpl(GroupRoadmapType* const _r, const VID _source,
        const VertexSet* const _targetSet = nullptr,
        OutputIterator<GroupRoadmapType>* const _collision = nullptr) override;

    using ConnectorMethod<MPTraits>::m_neighborBuffer;

    ///@}
    ///@name Helpers
    ///@{

    template <typename AbstractRoadmapType>
    void ConnectImplImpl(AbstractRoadmapType* const _r, const VID _source,
        const VertexSet* const _targetSet = nullptr,
        OutputIterator<AbstractRoadmapType>* const _collision = nullptr);

    ///@}
    ///@name Internal State
    ///@{

    std::string m_nfLabel;   ///< NeighborhoodFinder for selecting connections.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
NeighborhoodConnector<MPTraits>::
NeighborhoodConnector() {
  this->SetName("NeighborhoodConnector");
}


template <typename MPTraits>
NeighborhoodConnector<MPTraits>::
NeighborhoodConnector(XMLNode& _node) : ConnectorMethod<MPTraits>(_node) {
  this->SetName("NeighborhoodConnector");

  m_nfLabel = _node.Read("nfLabel", true, "",
      "The neighborhood finder for identifying connections to attempt.");
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template <typename MPTraits>
void
NeighborhoodConnector<MPTraits>::
Print(std::ostream& _os) const {
  ConnectorMethod<MPTraits>::Print(_os);
  _os << "\tnfLabel: " << m_nfLabel
      << std::endl;
}

/*------------------------ ConnectorMethod Interface -------------------------*/

template <typename MPTraits>
void
NeighborhoodConnector<MPTraits>::
ConnectImpl(RoadmapType* const _r, const VID _source,
    const VertexSet* const _targetSet,
    OutputIterator<RoadmapType>* const _collision) {
  ConnectImplImpl(_r, _source, _targetSet, _collision);
}


template <typename MPTraits>
void
NeighborhoodConnector<MPTraits>::
ConnectImpl(GroupRoadmapType* const _r, const VID _source,
    const VertexSet* const _targetSet,
    OutputIterator<GroupRoadmapType>* const _collision) {
  ConnectImplImpl(_r, _source, _targetSet, _collision);
}

/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
template <typename AbstractRoadmapType>
void
NeighborhoodConnector<MPTraits>::
ConnectImplImpl(AbstractRoadmapType* const _r, const VID _source,
    const VertexSet* const _targetSet,
    OutputIterator<AbstractRoadmapType>* const _collision) {
  auto nf = this->GetNeighborhoodFinder(this->m_nfLabel);
  const auto& cfg = _r->GetVertex(_source);

  // Determine nearest neighbors.
  m_neighborBuffer.clear();
  if(_targetSet)
    nf->FindNeighbors(_r, cfg, *_targetSet,
        std::back_inserter(m_neighborBuffer));
  else
    nf->FindNeighbors(_r, cfg, std::back_inserter(m_neighborBuffer));

  // Attempt connections.
  this->ConnectNeighbors(_r, _source, m_neighborBuffer, _collision);
}

/*----------------------------------------------------------------------------*/

#endif
