#ifndef PMPL_TIME_METRIC_H_
#define PMPL_TIME_METRIC_H_

#include "MetricMethod.h"
#include "Utilities/MetricUtils.h"


////////////////////////////////////////////////////////////////////////////////
/// This metric measures the r-usage time for the entire MPLibrary run.
/// @ingroup Metrics
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class TimeMetric : public MetricMethod<MPTraits> {

  public:

    ///@name Construction
    ///@{

    TimeMetric();

    TimeMetric(XMLNode& _node);

    virtual ~TimeMetric() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Initialize() override;

    ///@}
    ///@name MetricMethod Overrides
    ///@{

    virtual double operator()() override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    static std::string s_clockName; ///< The clock name.

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
std::string
TimeMetric<MPTraits>::
s_clockName("Total Running Time");


template <typename MPTraits>
TimeMetric<MPTraits>::
TimeMetric() {
  this->SetName("TimeMetric");
}


template <typename MPTraits>
TimeMetric<MPTraits>::
TimeMetric(XMLNode& _node) : MetricMethod<MPTraits>(_node) {
  this->SetName("TimeMetric");
}

/*-------------------------- MPBaseObject Overrides --------------------------*/

template <typename MPTraits>
void
TimeMetric<MPTraits>::
Initialize() {
  // Clear the previous clock (if any) and start again.
  auto stats =  this->GetStatClass();
  stats->ClearClock(s_clockName);
  stats->StartClock(s_clockName);
}

/*-------------------------- MetricMethod Overrides --------------------------*/

template <typename MPTraits>
double
TimeMetric<MPTraits>::
operator()() {
  auto stats = this->GetStatClass();

  // Report the elapsed time.
  stats->StopClock(s_clockName);
  stats->StartClock(s_clockName);

  return (double)stats->GetSeconds(s_clockName);
}

/*----------------------------------------------------------------------------*/

#endif
