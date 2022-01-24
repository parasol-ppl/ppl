#ifndef PMPL_CONSTRAINT_H_
#define PMPL_CONSTRAINT_H_

#include <memory>
#include <vector>

class Boundary;
class Cfg;
class Robot;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// An abstract base class representing the required interface for a constraint
/// on the state of a robot. The Constraint interface requires plain jane Cfg's
/// to ensure that they are usable by reactive agents.
///
/// Constraints may apply to specific robots, or they may be more general
/// criterion on any robot. In the later case, the robot pointer for the
/// constraint will be null.
////////////////////////////////////////////////////////////////////////////////
class Constraint {

  public:

    ///@name Construction
    ///@{

    /// Create a constraint for a robot.
    /// @param _r The robot to constrain.
    explicit Constraint(Robot* const _r);

    virtual ~Constraint();

    /// Construct a constraint of the appropriate type from an XML node.
    /// @param _r The robot to which the constraint applies.
    /// @param _node The XML node to parse.
    /// @return A constraint for _r of the type/parameters described by _node.
    static std::unique_ptr<Constraint> Factory(Robot* const _r, XMLNode& _node);

    /// Copy this constraint.
    /// @return A copy of this constraint.
    virtual std::unique_ptr<Constraint> Clone() const = 0;

    ///@}
    ///@name Constraint Interface
    ///@{

    /// Change the subject of this constraint.
    /// @param _r The new robot to constrain, or null to represent any robot.
    virtual void SetRobot(Robot* const _r);

    /// Get a sampling boundary that describes the subset of CSpace allowed by
    /// this constraint.
    /// @note Many constraints can be described in terms of boundaries, but some
    ///       cannot (such as constraints with ordering requirements). If the
    ///       constraint can't be represented as a boundary, this will return a
    ///       null pointer.
    virtual const Boundary* GetBoundary() const = 0;

    /// Determine whether a given configuration of the robot satisfies this
    /// constraint.
    /// @param _c The configuration to check.
    /// @return   True if _c satisfies this constraint.
    virtual bool Satisfied(const Cfg& _c) const = 0;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    Robot* m_robot{nullptr}; ///< The subject of this constraint.

    ///@}

};

#endif
