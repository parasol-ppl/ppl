#ifndef PMPL_NUM_NODES_METRIC_H
#define PMPL_NUM_NODES_METRIC_H

#include "MetricMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Evaluates the number of nodes in the current roadmap.
/// @ingroup Metrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class NumNodesMetric : public MetricMethod<MPTraits> {

  public:

    ///@name Construction
    ///@{

    NumNodesMetric();

    NumNodesMetric(XMLNode& _node);

    virtual ~NumNodesMetric() = default;

    ///@}
    ///@name Metric Interface
    ///@{

    virtual double operator()() override;

    ///@}
};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
NumNodesMetric<MPTraits>::
NumNodesMetric() {
  this->SetName("NumNodesMetric");
}


template <typename MPTraits>
NumNodesMetric<MPTraits>::
NumNodesMetric(XMLNode& _node) : MetricMethod<MPTraits>(_node){
  this->SetName("NumNodesMetric");
}

/*---------------------------- Metric Interface ------------------------------*/

template <typename MPTraits>
double
NumNodesMetric<MPTraits>::
operator()() {
  if(this->GetGroupRoadmap())
    return this->GetGroupRoadmap()->Size();
  else
    return this->GetRoadmap()->Size();
}

/*----------------------------------------------------------------------------*/

#endif
