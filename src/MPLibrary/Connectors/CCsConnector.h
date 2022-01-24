#ifndef PMPL_CCS_CONNECTOR_H_
#define PMPL_CCS_CONNECTOR_H_

#include "ConnectorMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Tries to connect the source vertices to targets in other CCs.
///
/// @note If the skip same CC option is set, it will forgo checking the
///       remainder of the targets in a target CC after the first success.
///
/// @ingroup Connectors
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class CCsConnector: public ConnectorMethod<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType           CfgType;
    typedef typename MPTraits::RoadmapType       RoadmapType;
    typedef typename RoadmapType::VID            VID;
    typedef typename RoadmapType::VertexSet      VertexSet;
    typedef typename MPTraits::GroupRoadmapType  GroupRoadmapType;

    ///@}
    ///@name Local Types
    ///@{

    template <typename AbstractRoadmapType>
    using OutputIterator = typename ConnectorMethod<MPTraits>::template
                           OutputIterator<AbstractRoadmapType>;

    ///@}
    ///@name Construction
    ///@{

    CCsConnector();

    CCsConnector(XMLNode& _node);

    virtual ~CCsConnector() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}

  protected:

    ///@name ConnectorMethod Overrides
    ///@{

    /// @note If a target set is provided, we only attempt to connect to CCs
    ///       containing at least one of its members.
    
    /// Generate edges with single vertex as source
    /// @param _r Roadmap to connect
    /// @param _source Source vertex to connect
    /// @param _targetSet The set of target vertices, set to null for full roadmap
    /// @param _collision Output iterator for collisons
    virtual void ConnectImpl(RoadmapType* const _r, const VID _source,
        const VertexSet* const _targetSet = nullptr,
        OutputIterator<RoadmapType>* const _collision = nullptr) override;

    using ConnectorMethod<MPTraits>::m_neighborBuffer;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::string m_nfLabel;  ///< The neighborhood finder for nearest nodes.

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
CCsConnector<MPTraits>::
CCsConnector() {
  this->SetName("CCsConnector");
}


template <typename MPTraits>
CCsConnector<MPTraits>::
CCsConnector(XMLNode& _node) : ConnectorMethod<MPTraits>(_node) {
  this->SetName("CCsConnector");

  m_nfLabel = _node.Read("nfLabel", true, "",
      "The neighborhood finder for identifying connections to attempt.");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
CCsConnector<MPTraits>::
Print(std::ostream& _os) const {
  ConnectorMethod<MPTraits>::Print(_os);
  _os << "\tNeighborhood Finder: " << m_nfLabel
      << std::endl;
}

/*------------------------ ConnectorMethod Overrides -------------------------*/

template <typename MPTraits>
void
CCsConnector<MPTraits>::
ConnectImpl(RoadmapType* const _r, const VID _source,
    const VertexSet* const _targetSet,
    OutputIterator<RoadmapType>* const _collision) {
  // Find a representative for each CC.
  auto ccTracker = _r->GetCCTracker();
  const VertexSet representatives = ccTracker->GetRepresentatives();

  if(this->m_debug)
    std::cout << "\tThere are " << ccTracker->GetNumCCs() << " CCs."
              << std::endl;

  // Try to connect _source to each CC.
  auto nf = this->GetNeighborhoodFinder(this->m_nfLabel);
  const auto& cfg = _r->GetVertex(_source);
  for(const VID rep : representatives) {
    // Skip representatives in the same cc.
    if(ccTracker->InSameCC(_source, rep)) {
      if(this->m_debug)
        std::cout << "\t\tNodes " << _source << ", " << rep << " are already "
                  << "in the same CC."
                  << std::endl;
      continue;
    }

    // Get the representative's CC.
    const VertexSet* const cc = ccTracker->GetCC(rep);
    if(this->m_debug)
      std::cout << "\t\tAttempting connection to CC with node " << rep
                << " of size " << cc->size() << "."
                << std::endl;

    // Determine nearest neighbors in the representative's CC.
    m_neighborBuffer.clear();
    if(_targetSet)
      nf->FindNeighbors(_r, cfg, VertexSetIntersection(*cc, *_targetSet),
          std::back_inserter(m_neighborBuffer));
    else
      nf->FindNeighbors(_r, cfg, *cc, std::back_inserter(m_neighborBuffer));

    // Attempt connections.
    this->ConnectNeighbors(_r, _source, m_neighborBuffer, _collision,
        this->m_skipIfSameCC);
  }

  if(this->m_debug)
    std::cout << "\tThere are now " << ccTracker->GetNumCCs() << " CCs."
              << std::endl;
}

/*----------------------------------------------------------------------------*/

#endif
