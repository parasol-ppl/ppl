#ifndef PMPL_LP_OUTPUT_H_
#define PMPL_LP_OUTPUT_H_

#include <string>
#include <utility>
#include <vector>

////////////////////////////////////////////////////////////////////////////////
/// Computed information from a local plan.
///
/// Stores all information available from local plan computations, including
/// intermediates along edges (not straight line), the path
/// generated, and the edge weights to be added to the RoadmapGraph.
///
/// @todo Destroy this object and have LPs/Extenders work directly with a
///       LocalPlan object, which should replace the DefaultWeight class as our
///       roadmap edge.
///
/// @ingroup LocalPlanners
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
struct LPOutput {

  ///@name Motion Planning Types
  ///@{

  typedef typename MPTraits::CfgType    CfgType;
  typedef typename MPTraits::WeightType WeightType;

  ///@}
  ///@name Local Types
  ///@{

  typedef std::pair<WeightType, WeightType> LPEdge;

  ///@}
  ///@name Internal State
  ///@{

  /// The resolution-level path computed by a local planner. Does not include
  /// the start or goal configurations.
  std::vector<CfgType> m_path;

  /// The set of 'intermediate' configurations, between each of which is a
  /// straight-line path in c-space (i.e. these are the vertices of a polygonal
  /// chain). Does not include the start or goal configurations.
  std::vector<CfgType> m_intermediates;

  /// A pair of weight objects. The first is in the forward direction and the
  /// second is its reverse.
  LPEdge m_edge;

  ///@}
  ///@name Construction
  ///@{

  LPOutput();

  ///@}
  ///@name Interface
  ///@{

  void Clear();

  void SetLPLabel(const std::string& _label);

  void AddIntermediatesToWeights(const bool _saveIntermediates);

  ///@}

};

/*----------------------------------------------------------------------------*/

template <typename MPTraits>
LPOutput<MPTraits>::
LPOutput() {
  Clear();
}


template <typename MPTraits>
void
LPOutput<MPTraits>::
Clear() {
  m_path.clear();
  m_intermediates.clear();
  m_edge.first.Clear();
  m_edge.second.Clear();
}


template <typename MPTraits>
void
LPOutput<MPTraits>::
SetLPLabel(const std::string& _label) {
  m_edge.first.SetLPLabel(_label);
  m_edge.second.SetLPLabel(_label);
}


template <typename MPTraits>
void
LPOutput<MPTraits>::
AddIntermediatesToWeights(const bool _saveIntermediates) {
  if(!_saveIntermediates)
    return;

  // Make a copy of the intermediates in reverse order for the backward edge.
  std::vector<CfgType> tmp;
  tmp.reserve(m_intermediates.size());
  std::copy(m_intermediates.rbegin(), m_intermediates.rend(),
            std::back_inserter(tmp));

  // Set both edges.
  m_edge.first.SetIntermediates(m_intermediates);
  m_edge.second.SetIntermediates(tmp);
}


/*----------------------------------------------------------------------------*/

#endif
