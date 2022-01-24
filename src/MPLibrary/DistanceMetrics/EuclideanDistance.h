#ifndef PMPL_EUCLIDEAN_DISTANCE_H_
#define PMPL_EUCLIDEAN_DISTANCE_H_

#include "MinkowskiDistance.h"


////////////////////////////////////////////////////////////////////////////////
/// Measure the standard Euclidean distance between two configurations.
///
/// Euclidean Distance is Minkowski Distance where r1=r2=2, r3=0.5.
///
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class EuclideanDistance : public MinkowskiDistance<MPTraits> {

  public:

    ///@name Construction
    ///@{

    EuclideanDistance(bool _normalize = false);
    EuclideanDistance(XMLNode& _node);
    virtual ~EuclideanDistance() = default;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
EuclideanDistance<MPTraits>::
EuclideanDistance(bool _normalize) :
    MinkowskiDistance<MPTraits>(2, 2, 1. / 2, _normalize) {
  this->SetName("Euclidean");
}


template <typename MPTraits>
EuclideanDistance<MPTraits>::
EuclideanDistance(XMLNode& _node) : MinkowskiDistance<MPTraits>(_node) {
  this->SetName("Euclidean");

  this->m_r1 = 2;
  this->m_r2 = 2;
  this->m_r3 = 1.0/2;
  this->m_normalize = _node.Read("normalize", false, false,
      "flag if position dof should be normalized by environment diagonal");
}

/*----------------------------------------------------------------------------*/

#endif
