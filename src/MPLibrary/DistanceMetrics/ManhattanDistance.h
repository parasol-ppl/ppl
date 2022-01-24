#ifndef PMPL_MANHATTAN_DISTANCE_H_
#define PMPL_MANHATTAN_DISTANCE_H_

#include "MinkowskiDistance.h"


////////////////////////////////////////////////////////////////////////////////
/// Compute the Manhattan distance between two configurations.
///
/// The Manhattan distance requires that each basis of the configuration space
/// be traversed separately. Its name is analogous to how one moves around the
/// city of Manhattan by following the streets rather than flying over the
/// buildings.
///
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ManhattanDistance : public MinkowskiDistance<MPTraits> {

  public:

    ///@name Construction
    ///@{

    ManhattanDistance(bool _normalize = false);
    ManhattanDistance(XMLNode& _node);
    virtual ~ManhattanDistance() = default;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
ManhattanDistance<MPTraits>::
ManhattanDistance(bool _normalize) :
    MinkowskiDistance<MPTraits>(1, 1, 1, _normalize) {
  this->SetName("Manhattan");
}


template <typename MPTraits>
ManhattanDistance<MPTraits>::
ManhattanDistance(XMLNode& _node) : MinkowskiDistance<MPTraits>(_node) {
  this->SetName("Manhattan");

  this->m_r1 = 1;
  this->m_r2 = 1;
  this->m_r3 = 1;
  this->m_normalize = _node.Read("normalize", false, false,
      "flag if position dof should be normalized by environment diagonal");
}

/*----------------------------------------------------------------------------*/

#endif