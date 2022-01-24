#ifndef PMPL_CHAIN_H_
#define PMPL_CHAIN_H_

#include <deque>
#include <iostream>
#include <utility>
#include <vector>

class Connection;
class Body;
class MultiBody;


////////////////////////////////////////////////////////////////////////////////
/// A chain represents some subset of a linked multibody. It defines a set of
/// joints, traversal ordering (relative to the multibody base), and possibly a
/// front and/or back body. It must contain at least one link between the
/// front/back bodies and joints.
///
/// For multibodies with branchings and/or closures, Decompose will return a
/// Chain for each linear segment. Splitting joints (where the multibody has
/// been broken into chains) are repeated so that it is clear where chains
/// connect (also true for bisect).
///
/// This object assumes that the represented multibody will not change
/// structure. If it does, any chains built from the previous version will
/// likely be invalid.
////////////////////////////////////////////////////////////////////////////////
class Chain final {

  public:

    ///@name Local Types
    ///@{

    typedef std::deque<const Connection*> JointList;
    typedef JointList::const_iterator iterator;

    ///@}
    ///@name Generation
    ///@{
    /// Methods for generating chains of a multibody.

    /// Decompose a multibody into a set of linear chains. Splitting joints
    /// will be present in each chain that contains them.
    /// @param _mb The multibody to decompose.
    /// @return A set of Chains representing linear subsets of _mb.
    static std::vector<Chain> Decompose(const MultiBody* const _mb);

    /// Bisect this chain to produce two subchains which both contain the
    /// splitting joint. Their order will be the same as this. If this chain has
    /// an odd number of joints, the first subchain will take the extra one.
    std::pair<Chain, Chain> Bisect() const noexcept;

    ///@}
    ///@name Modifiers
    ///@{
    /// All modifiers return a self-reference for easy function chaining (bad
    /// pun intended).

    /// Reverse the traversal order of this chain.
    Chain& Reverse() noexcept;

    /// Merge another chain into this one. The two chains must have the same
    /// traversal order, and the other chain's root must be connected to this
    /// chain's end.
    /// @param _other The chain to append to this one.
    Chain& Append(const Chain& _other) noexcept;

    ///@}
    ///@name Iteration
    ///@{
    /// Iterate over the bodies in the chain in traversal order.

    iterator begin() const noexcept;
    iterator end() const noexcept;

    ///@}
    ///@name Queries
    ///@{

    /// Get the body at the front of the chain (w.r.t. traversal order). Will be
    /// null if this chain begins with a joint.
    const Body* GetFrontBody() const noexcept;

    /// Get the body at the back of the chain (w.r.t. traversal order). Will be
    /// null if this chain ends with a joint.
    const Body* GetBackBody() const noexcept;

    /// Get the first joint.
    const Connection* GetFirstJoint() const noexcept;

    /// Get the last joint.
    const Connection* GetLastJoint() const noexcept;

    /// Is this chain oriented in the same way as the multibody?
    bool IsForward() const noexcept;

    /// Get the number of joints in the chain.
    size_t GetNumJoints() const noexcept;

    /// Get the number of objects in the chain (joints and bodies).
    size_t Size() const noexcept;

    /// Check if the current chain has only one link
    bool IsSingleLink() const noexcept;

    ///@}

  private:

    ///@}
    ///@name Construction
    ///@{

    /// Construct a chain.
    /// @param _mb The multibody.
    /// @param _bodies The joints in this chain, from root to end.
    /// @param _front The front body w.r.t. traversal order if present, nullptr
    ///               otherwise.
    /// @param _back The back body w.r.t. traversal order if present, nullptr
    ///              otherwise.
    /// @param _forward True if _end is further away from the base than _root.
    Chain(const MultiBody* const _mb, JointList&& _joints,
        const Body* const _front, const Body* const _back, const bool _forward);

    ///@}
    ///@name Internal State
    ///@{

    const MultiBody* const m_multibody; ///< The multibody.

    JointList m_joints;                 ///< The joints in this chain.
    const Body* m_frontBody{nullptr};   ///< The front body, if present.
    const Body* m_backBody{nullptr};    ///< The back body, if present.
    bool m_forward{true};               ///< Is the chain in forward order?

    ///@}

};


/// Output operator for debugging chains.
std::ostream& operator<<(std::ostream&, const Chain&);

#endif
