#ifndef PMPL_BOUNDARY_CONSTRAINT_H_
#define PMPL_BOUNDARY_CONSTRAINT_H_

#include "Constraint.h"

#include <iostream>

class Boundary;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// A robot satisfies this constraint if it passes the 'is inside' boundary
/// check. Various boundary types may be considered.
////////////////////////////////////////////////////////////////////////////////
class BoundaryConstraint : public Constraint {

  public:

    ///@name Construction
    ///@{

    /// Construct a constraint from a specific boundary.
    /// @param _r The robot to constrain.
    /// @param _b The boundary.
    explicit BoundaryConstraint(Robot* const _r, std::unique_ptr<Boundary>&& _b);

    /// Construct a constraint from an XML node.
    /// @param _r The robot to constrain.
    /// @param _node The node to parse.
    explicit BoundaryConstraint(Robot* const _r, XMLNode& _node);

    BoundaryConstraint(const BoundaryConstraint& _other);

    virtual ~BoundaryConstraint();

    virtual std::unique_ptr<Constraint> Clone() const override;

    ///@}
    ///@name Constraint Interface
    ///@{

    virtual const Boundary* GetBoundary() const override;

    virtual bool Satisfied(const Cfg& _c) const override;

    ///@}

  protected:

    ///@name Internal State
    ///@{

    std::unique_ptr<Boundary> m_boundary; ///< The boundary for this constraint.

    ///@}

};


std::ostream& operator<<(std::ostream&, const BoundaryConstraint&);

#endif
