#ifndef PMPL_MAP_EVALUATOR_METHOD_H_
#define PMPL_MAP_EVALUATOR_METHOD_H_

#include "MPLibrary/MPBaseObject.h"
#include "Utilities/MPUtils.h"


////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for \ref MapEvaluators.
///
/// All \ref MapEvaluators have one main function, @c operator(), which
/// evaluates the current roadmap and returns true iff it satisfies a specific
/// set of criteria.
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////

template <typename MPTraits>
class MapEvaluatorMethod : public MPBaseObject<MPTraits> {
  public:

    ///@name Construction
    ///@{

    MapEvaluatorMethod() = default;

    // MapEvaluatorMethod(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {}
    MapEvaluatorMethod(XMLNode& _node);

    virtual ~MapEvaluatorMethod() = default;

    typedef std::unordered_map<size_t,
            std::unordered_map<size_t,
            std::vector<Range<double>>>> EdgeIntervals;

    ///@}
    ///@name MapEvaluator Interface
    ///@{

    /// Evaluate a roadmap.
    /// @return True if this roadmap meets the evaluation criteria.
    virtual bool operator()() = 0;
    ///@example MapEvaluator_UseCase.cpp
    /// This is an example of how to use the map evaluator method.

    /// Set the edge intervals of a roadmap
    virtual void SetEdgeIntervals(EdgeIntervals _edgeIntervals) {};

    /// Set the minimum end time of a path
    virtual void SetMinEndtime(double _minEndtime) {};

    ///@}
    ///@name Active Robots
    ///@{
    /// @todo This is an artifact of developing robot groups. It should be
    ///       replaced by using a proper subgroup where each robot within is
    ///       active.

    /// Set the active robots.
    void SetActiveRobots(const std::vector<size_t>& _activeRobots);

    /// Get the active robots.
    std::vector<size_t> GetActiveRobots() const;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    /// The active robots, used only by group map evaluators. Depending on the
    /// evaluator, the usage could be different, but the current use case is to
    /// set the robot(s) that are being moved, then a clearance check is done
    /// only considering those specified bodies (see MinimumClearanceEvaluator).
    /// @todo This is an artifact of developing robot groups. It should be
    ///       replaced by using a proper subgroup where each robot within is
    ///       active.
    std::vector<size_t> m_activeRobots;

    ///@}

};


template<typename MPTraits>
MapEvaluatorMethod<MPTraits>::
MapEvaluatorMethod(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
}

/*----------------------------- Active Robots --------------------------------*/

template<typename MPTraits>
void
MapEvaluatorMethod<MPTraits>::
SetActiveRobots(const std::vector<size_t>& _activeRobots) {
  m_activeRobots = _activeRobots;
}


template<typename MPTraits>
std::vector<size_t>
MapEvaluatorMethod<MPTraits>::
GetActiveRobots() const {
  return m_activeRobots;
}

/*----------------------------------------------------------------------------*/


#endif
