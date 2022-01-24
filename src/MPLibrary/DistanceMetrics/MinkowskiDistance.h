#ifndef PMPL_MINKOWSKI_DISTANCE_H_
#define PMPL_MINKOWSKI_DISTANCE_H_

#include "DistanceMetricMethod.h"

#include "MPProblem/Environment/Environment.h"


////////////////////////////////////////////////////////////////////////////////
/// Minkowski distance is a generalized L-p norm where the exponents p and 1/p
/// each have separate values.
///
/// @todo Remove m_r2, which isn't part of the Minkowski difference and doesn't
///       have any valid use.
///
/// @todo Move the normalization option up to the base class.
///
/// @todo Separate the computation of orientation and joint distances.
///
/// @ingroup DistanceMetrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class MinkowskiDistance : public DistanceMetricMethod<MPTraits> {

  public:

    ///@name Local Types
    ///@{

    typedef typename MPTraits::CfgType       CfgType;
    typedef typename MPTraits::GroupCfgType  GroupCfgType;

    ///@}
    ///@name Construction
    ///@{

    MinkowskiDistance(double _r1 = 3, double _r2 = 3, double _r3 = 1. / 3,
        bool _normalize = false);

    MinkowskiDistance(XMLNode& _node);

    virtual ~MinkowskiDistance() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const;

    ///@}
    ///@name DistanceMetricMethod Overrides
    ///@{

    virtual double Distance(const CfgType& _c1, const CfgType& _c2) override;

    virtual void ScaleCfg(double _length, CfgType& _c, const CfgType& _o)
        override;

    ///@}

  protected:

    ///@name Helpers
    ///@{

    double PositionDistance(const CfgType& _c);
    double OrientationDistance(const CfgType& _c);

    ///@}
    ///@name Internal State
    ///@{

    double m_r1{3};          ///< For position part.
    double m_r2{3};          ///< For rotation part.
    double m_r3{1. / 3.};    ///< For calculating root.
    bool m_normalize{false}; ///< Normalize distance w.r.t. environment?

    ///@}
};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
MinkowskiDistance<MPTraits>::
MinkowskiDistance(double _r1, double _r2, double _r3, bool _normalize) :
    DistanceMetricMethod<MPTraits>(), m_r1(_r1), m_r2(_r2), m_r3(_r3),
    m_normalize(_normalize) {
  this->SetName("Minkowski");
}


template <typename MPTraits>
MinkowskiDistance<MPTraits>::
MinkowskiDistance(XMLNode& _node) : DistanceMetricMethod<MPTraits>(_node),
    m_r1(3), m_r2(3), m_r3(1./3.), m_normalize(false) {
  this->SetName("Minkowski");

  m_r1 = _node.Read("r1", false, 3., 0., MAX_DBL, "r1");
  m_r2 = _node.Read("r2", false, 3., 0., MAX_DBL, "r2");
  m_r3 = _node.Read("r3", false, 1. / 3., 0., MAX_DBL, "r3");
  m_normalize = _node.Read("normalize", false, false, "flag if position dof "
      "should be normalized by environment diagonal");
}

/*--------------------------- MPBaseObject Overrides -------------------------*/

template <typename MPTraits>
void
MinkowskiDistance<MPTraits>::
Print(std::ostream& _os) const {
  DistanceMetricMethod<MPTraits>::Print(_os);
  _os << "\tr1 = " << m_r1 << endl
      << "\tr2 = " << m_r2 << endl
      << "\tr3 = " << m_r3 << endl
      << "\tnormalize = " << m_normalize << endl;
}

/*---------------------- DistanceMetricMethod Overrides ----------------------*/

template <typename MPTraits>
double
MinkowskiDistance<MPTraits>::
Distance(const CfgType& _c1, const CfgType& _c2) {
  const CfgType diff = _c2 - _c1;
  const double pos = PositionDistance(diff),
               ori = OrientationDistance(diff);
  return std::pow(pos + ori, m_r3);
}


template <typename MPTraits>
void
MinkowskiDistance<MPTraits>::
ScaleCfg(double _length, CfgType& _c, const CfgType& _o) {
  /// @todo This implementation is very poor. Scaling should be a
  ///       constant-time operation - complexity should not depend on the
  ///       length of the input vector.
  double originalLength = this->Distance(_o, _c);
  double diff = _length - originalLength;
  do {
    _c = (_c - _o) * (_length / originalLength) + _o;
    originalLength = this->Distance(_o, _c);
    diff = _length - originalLength;
  } while((diff > 0.1) || (diff < -0.1));
}

/*---------------------------------- Helpers ---------------------------------*/

template <typename MPTraits>
double
MinkowskiDistance<MPTraits>::
PositionDistance(const CfgType& _c) {
  const std::vector<double> p = _c.GetPosition();
  double distance = 0;

  if(m_normalize) {
    const double diagonal = this->GetEnvironment()->GetBoundary()->GetMaxDist(
        m_r1, m_r3);
    for(size_t i = 0; i < p.size(); ++i)
      distance += std::pow(fabs(p[i]) / diagonal, m_r1);
  }
  else
    for(size_t i = 0; i < p.size(); ++i)
      distance += std::pow(fabs(p[i]), m_r1);

  return distance;
}


template <typename MPTraits>
double
MinkowskiDistance<MPTraits>::
OrientationDistance(const CfgType& _c) {
  const std::vector<double> o = _c.GetOrientation();
  double distance = 0;
  for(size_t i = 0; i < o.size(); ++i)
    distance += std::pow(fabs(o[i]), m_r2);
  return distance;
}

/*----------------------------------------------------------------------------*/

#endif
