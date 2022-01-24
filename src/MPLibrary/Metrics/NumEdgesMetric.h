#ifndef PMPL_NUM_EDGES_METRIC_H_
#define PMPL_NUM_EDGES_METRIC_H_

#include "MetricMethod.h"


////////////////////////////////////////////////////////////////////////////////
/// Count the number of edges in roadmap.
/// @ingroup Metrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class NumEdgesMetric : public MetricMethod<MPTraits> {

  public:

    ///@name Construction
    ///@{

    NumEdgesMetric();

    NumEdgesMetric(XMLNode& _node);

    virtual ~NumEdgesMetric() = default;

    ///@}
    ///@name MetricMethod Interface
    ///@{

    virtual double operator()() override;

    ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename MPTraits>
NumEdgesMetric<MPTraits>::
NumEdgesMetric() {
  this->SetName("NumEdgesMetric");
}


template <typename MPTraits>
NumEdgesMetric<MPTraits>::
NumEdgesMetric(XMLNode& _node) : MetricMethod<MPTraits>(_node) {
  this->SetName("NumEdgesMetric");
}

/*--------------------------- MetricMethod Interface -------------------------*/

template <typename MPTraits>
double
NumEdgesMetric<MPTraits>::
operator()() {
  if(this->GetGroupTask())
    return this->GetGroupRoadmap()->get_num_edges();
  else
    return this->GetRoadmap()->get_num_edges();
}

/*----------------------------------------------------------------------------*/

#endif
