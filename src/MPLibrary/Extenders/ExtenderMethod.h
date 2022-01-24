#ifndef PMPL_EXTENDER_METHOD_H_
#define PMPL_EXTENDER_METHOD_H_

#include "MPLibrary/LocalPlanners/LPOutput.h"
#include "MPLibrary/LocalPlanners/GroupLPOutput.h"
#include "MPLibrary/ValidityCheckers/CollisionDetection/CDInfo.h"
#include <limits>

////////////////////////////////////////////////////////////////////////////////
/// Base algorithm abstraction for \ref Extenders.
///
/// ExtenderMethod has one main method, @c Extend, to grow a simple path from a
/// starting node in some input direction - note that not all expansion
/// methods go in straight lines through @cspace.
///
/// @usage
/// @code
/// ExtenderPointer e = this->GetExtender(m_exLabel);
/// CfgType start, goal, new;
/// LPOutput<MPTraits> lp;
/// bool pass = e->Extend(start, goal, new, lp);
/// @endcode
///
/// @todo Local planners and Extenders represent the same concepts and should be
///       merged into a single class with both an Extend and LocalPlan function.
///       This will help simplify several other objects within PMPL as well,
///       such as bi-directional RRT.
///
/// @ingroup Extenders
////////////////////////////////////////////////////////////////////////////////
template <class MPTraits>
class ExtenderMethod : public MPBaseObject<MPTraits> {

  public:

    ///@name Motion Planning Types
    ///@{

    typedef typename MPTraits::CfgType CfgType;
    typedef typename MPTraits::GroupCfgType GroupCfgType;
    typedef typename MPTraits::GroupWeightType GroupWeightType;

    ///@}
    ///@name Construction
    ///@{

    ExtenderMethod() = default;

    ExtenderMethod(XMLNode& _node);

    virtual ~ExtenderMethod() = default;

    ///@}
    ///@name MPBaseObject Overrides
    ///@{

    virtual void Print(std::ostream& _os) const override;

    ///@}
    ///@name Required Interface
    ///@{

    /// Get the minimum extension distance.
    virtual double GetMinDistance() const;

    /// Get the maximum extension distance.
    virtual double GetMaxDistance() const;

    /// Extends a local plan from a starting configuration towards a target
    /// configuration.
    /// @param _start Initial configuration to grow from.
    /// @param _end   Target configuration to grow towards.
    /// @param _new   Placeholder for resulting configuration.
    /// @param _lp    Placeholder for polygonal chain configurations for
    ///               non-straight-line extention operations and associated
    ///               weight.
    /// @return True if the extension produced a valid configuration that was
    ///         at least the minimum distance away from the starting point.
    virtual bool Extend(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp) = 0;
    ///@example Extenders_UseCase.cpp
    /// This is an example of how to use the extender methods.

    /// An optional version if CDInfo is desired. Not required to implement.
    virtual bool Extend(const CfgType& _start, const CfgType& _end,
        CfgType& _new, LPOutput<MPTraits>& _lp, CDInfo& _cdInfo);

    /// For groups.
    /// @override
    /// @param _robotIndexes The indexes of the robots which should move (empty
    ///                      for all).
    virtual bool Extend(const GroupCfgType& _start, const GroupCfgType& _end,
        GroupCfgType& _new, GroupLPOutput<MPTraits>& _lp,
        const std::vector<size_t>& _robotIndexes = {});

    /// For groups, with CD info.
    /// @override
    /// @param _robotIndexes The indexes of the robots which should move (empty
    ///                      for all).
    virtual bool Extend(const GroupCfgType& _start, const GroupCfgType& _end,
        GroupCfgType& _new, GroupLPOutput<MPTraits>& _lp, CDInfo& _cdInfo,
        const std::vector<size_t>& _robotIndexes = {});

    ///@}

  protected:

    ///@name Extender Properties
    ///@{

    double m_minDist{.1};  ///< The minimum valid extension distance.
    double m_maxDist{1.};  ///< The maximum valid extension distance.

    ///@}

};

/*------------------------------ Construction --------------------------------*/

template <typename MPTraits>
ExtenderMethod<MPTraits>::
ExtenderMethod(XMLNode& _node) : MPBaseObject<MPTraits>(_node) {
  // We do not require these to be specified because some extenders don't use
  // them (like KinodynamicExtender).
  m_maxDist = _node.Read("maxDist", false, m_maxDist,
      std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
      "The maximum allowed distance to expand from the starting node to the "
      "target node.");

  m_minDist = _node.Read("minDist", false, m_minDist,
      std::numeric_limits<double>::min(), std::numeric_limits<double>::max(),
      "The minimum valid distance when expanding from the starting node to the "
      "target node (shorter extensions are considered invalid).");
}

/*------------------------- MPBaseObject Overrides ---------------------------*/

template <typename MPTraits>
void
ExtenderMethod<MPTraits>::
Print(std::ostream& _os) const {
  MPBaseObject<MPTraits>::Print(_os);
  _os << "\tMin distance: " << m_minDist
      << "\n\tMax distance: " << m_maxDist
      << std::endl;
}

/*------------------------- ExtenderMethod Interface -------------------------*/

template <typename MPTraits>
double
ExtenderMethod<MPTraits>::
GetMinDistance() const {
  return m_minDist;
}


template <typename MPTraits>
double
ExtenderMethod<MPTraits>::
GetMaxDistance() const {
  return m_maxDist;
}


template <typename MPTraits>
bool
ExtenderMethod<MPTraits>::
Extend(const CfgType& _start, const CfgType& _end, CfgType& _new,
    LPOutput<MPTraits>& _lp, CDInfo& _cdInfo) {
  throw NotImplementedException(WHERE);
}


template <typename MPTraits>
bool
ExtenderMethod<MPTraits>::
Extend(const GroupCfgType& _start, const GroupCfgType& _end,
    GroupCfgType& _new, GroupLPOutput<MPTraits>& _lp,
    const std::vector<size_t>& _robotIndexes) {
  throw NotImplementedException(WHERE);
}


template <typename MPTraits>
bool
ExtenderMethod<MPTraits>::
Extend(const GroupCfgType& _start, const GroupCfgType& _end,
    GroupCfgType& _new, GroupLPOutput<MPTraits>& _lp, CDInfo& _cdInfo,
    const std::vector<size_t>& _robotIndexes) {
  throw NotImplementedException(WHERE);
}

/*----------------------------------------------------------------------------*/

#endif
