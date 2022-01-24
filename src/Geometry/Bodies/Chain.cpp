#include "Chain.h"

#include "Geometry/Bodies/Connection.h"
#include "Geometry/Bodies/Body.h"
#include "Geometry/Bodies/MultiBody.h"

#include "Utilities/PMPLExceptions.h"

#include <algorithm>
#include <queue>
#include <set>


/*-------------------------------- Generation --------------------------------*/

std::vector<Chain>
Chain::
Decompose(const MultiBody* const _mb) {
  /// @todo Expand to support joints with more than one forward or backward
  ///       connection. For now we only support chain robots.

  // Composite multibodies are not supported.
  if(_mb->IsComposite())
    throw RunTimeException(WHERE) << "Composite multibodies are not supported.";

  // Single-body multibodies are not supported.
  if(_mb->GetNumBodies() == 1)
    throw RunTimeException(WHERE) << "Cannot make a chain for a single-body "
                                  << "multibody.";

  // Initialize lists of end effectors and joints.
  std::vector<const Body*> endEffectors = _mb->GetEndEffectors();
  std::set<const Connection*> unusedJoints;
  bool baseUsed = false;
  {
    const auto& joints = _mb->GetJoints();
    for(const auto& joint : joints)
      unusedJoints.insert(joint.get());
  }

  // Initialize the chain list. We will have at least as many chains as
  // end-effectors.
  std::vector<Chain> chains;
  chains.reserve(endEffectors.size());

  // Start from the end-effectors and scan inward. Repeat until all EEs have
  // been used.
  while(!endEffectors.empty()) {
    // Get the next end effector.
    const Body* current = endEffectors.back();

    // Start a joint list for this chain.
    JointList joints;

    // Trace backward from the end-effector along unused joints until we cannot
    // continue.
    while(true)
    {
      // Get the number of backward connections from this body.
      const size_t numConnections = current->BackwardConnectionCount();

      // If there are no backward connections, we have arrived at the base.
      // Create the corresponding chain.
      if(numConnections == 0) {
        // Ensure we haven't already used the base.
        if(baseUsed)
          throw RunTimeException(WHERE) << "Identified two base bodies.";
        baseUsed = true;

        chains.push_back(Chain(_mb, std::move(joints), endEffectors.back(),
            _mb->GetBase(), false));

        // Orient the chain forward.
        chains.back().Reverse();

        // Break out of the search from this EE.
        break;
      }

      // We are still here, so this is not the base body. Find one of its
      // backward joints to add to the list.
      bool foundOne = false;
      for(size_t i = 0; i < numConnections; ++i) {
        const Connection* const joint = &current->GetBackwardConnection(i);

        // If we have already used this joint, skip it.
        if(!unusedJoints.count(joint))
          continue;

        // Add this joint to the current joint list and mark it as used.
        joints.push_back(joint);
        unusedJoints.erase(joint);
        foundOne = true;
        break;
      }

      // If we did not find an unused joint, then we are finished with this
      // chain.
      if(!foundOne) {
        // Add an arbitrary joint as the splitting joint.
        joints.push_back(&current->GetBackwardConnection(0));

        chains.emplace_back(Chain(_mb, std::move(joints), nullptr,
            endEffectors.back(), false));

        // Orient the chain forward.
        chains.back().Reverse();

        // Break out of the search from this EE.
        break;
      }

      // If we found an unused joint, move on to its parent body.
      current = joints.back()->GetPreviousBody();
    }

    // Remove this end-effector from the list.
    endEffectors.pop_back();
  }

  // If the robot has closed chains, we have missed some joints.
  if(!unusedJoints.empty())
    throw NotImplementedException(WHERE) << "Closed chain robot detected, "
                                         << "support not yet implemented.";

  return chains;
}


std::pair<Chain, Chain>
Chain::
Bisect() const noexcept {
  // Ensure that this chain is longer than a single link.
  if(IsSingleLink())
    throw RunTimeException(WHERE) << "Cannot bisect a single-link chain.";

  //This method is called by ReachableVolumeSampler::SampleInternal 
  //which reverses chain result if m_forward is false creating the assumption
  //that if only one of m_frontBody or m_backBody not nullptr then it is m_frontBody.
  //(It does not happen that m_backBody is defined while m_frontBody is not.)
  //Thus splitting can either take the ceiling of m_joints or the floor of Size()
  //which counts bodies implied by m_frontBody and m_backBody.
  //No other adjustments need to be made to loop bounds.
  //Ceiling splits on the first of the possibly two middle joints while
  //floor splits on the second of the possibly two middle joints.
  //Either is correct.
  const size_t halfSize = std::ceil(m_joints.size() / 2.); // break splitting ties on first joint
  //const size_t halfSize = std::floor(Size() / 2.);       // break splittin gties on second joint

  JointList joints1, joints2;

  for(auto iter = m_joints.begin(), end = m_joints.begin() + halfSize;
      iter != end; ++iter) 
    joints1.push_back(*iter);

  for(auto iter = m_joints.begin() + halfSize - 1; iter != m_joints.end(); ++iter) 
    joints2.push_back(*iter);

  return {
    Chain(m_multibody, std::move(joints1), m_frontBody, nullptr, m_forward),
    Chain(m_multibody, std::move(joints2), nullptr,  m_backBody, m_forward)
  };
}

/*-------------------------------- Modifiers ---------------------------------*/

Chain&
Chain::
Reverse() noexcept {
  std::reverse(m_joints.begin(), m_joints.end());
  std::swap(m_frontBody, m_backBody);
  m_forward = !m_forward;

  return *this;
}


Chain&
Chain::
Append(const Chain& _other) noexcept {
  // Assert that the two chains have the same traversal direction.
  if(IsForward() != _other.IsForward())
    throw RunTimeException(WHERE) << "Cannot append two chains with opposite "
                                  << "traversal orders."
                                  << "\n\tChain 1: " << IsForward()
                                  << "\n\tChain 2: " << _other.IsForward();

  // Assert that this chain doesn't have a back body and _other doesn't have a
  // front body (we can only join chains at a joint).
  if(GetBackBody() or _other.GetFrontBody())
    throw RunTimeException(WHERE) << "Cannot join chains with bodies in between.";

  // Assert that the end of this chain is connected to the root of _other.
  if(GetLastJoint() != _other.GetFirstJoint())
    throw RunTimeException(WHERE) << "Can only join chains at a common joint.";

  // We're still here, so the chains are compatible for joining.
  // Copy the joints in _other into this.
  std::copy(_other.begin() + 1, _other.end(), std::back_inserter(m_joints));

  m_backBody = _other.m_backBody;

  return *this;
}


bool
Chain::
IsSingleLink() const noexcept {
  return Size() == 2;
}

/*-------------------------------- Iteration ---------------------------------*/

Chain::iterator
Chain::
begin() const noexcept {
  return m_joints.begin();
}


Chain::iterator
Chain::
end() const noexcept {
  return m_joints.end();
}

/*--------------------------------- Queries ----------------------------------*/

const Connection*
Chain::
GetFirstJoint() const noexcept {
  return m_joints.front();
}


const Connection*
Chain::
GetLastJoint() const noexcept {
  return m_joints.back();
}


bool
Chain::
IsForward() const noexcept {
  return m_forward;
}


size_t
Chain::
GetNumJoints() const noexcept {
  return m_joints.size();
}


size_t
Chain::
Size() const noexcept {
  return m_joints.size() + bool(m_frontBody) + bool(m_backBody);
}


const Body*
Chain::
GetFrontBody() const noexcept {
  return m_frontBody;
}


const Body*
Chain::
GetBackBody() const noexcept {
  return m_backBody;
}

/*------------------------------- Construction -------------------------------*/

Chain::
Chain(const MultiBody* const _mb, JointList&& _joints, const Body* const _front,
    const Body* const _back, const bool _forward)
  : m_multibody(_mb), m_joints(std::move(_joints)), m_frontBody(_front),
    m_backBody(_back), m_forward(_forward)
{
  // A chain must contain at least one joint.
  if(m_joints.empty())
    throw RunTimeException(WHERE) << "A chain must contain at least one joint.";

  // A chain must contain at least one link.
  if(Size() == 1)
    throw RunTimeException(WHERE) << "A chain must contain at least one link.";
}

/*---------------------------------- Debug -----------------------------------*/

std::ostream&
operator<<(std::ostream& _os, const Chain& _c) {
  _os << "{";

  if(_c.GetFrontBody())
    _os << "b" << _c.GetFrontBody()->GetIndex() << ", ";

  for(auto joint : _c) {
    if(joint != _c.GetFirstJoint())
      _os << ", ";
    _os << "j" << joint->GetPreviousBodyIndex()
        << "-" << joint->GetNextBodyIndex();
  }

  if(_c.GetBackBody())
    _os << ", b" << _c.GetBackBody()->GetIndex();

  return _os << " (" << (_c.IsForward() ? "forward" : "backward") << ")}";
}

/*----------------------------------------------------------------------------*/
