#ifndef PMPL_WORKSPACE_BOUNDING_SPHERICAL_SHELL_H_
#define PMPL_WORKSPACE_BOUNDING_SPHERICAL_SHELL_H_

#include "AbstractBoundingSphericalShell.h"


////////////////////////////////////////////////////////////////////////////////
/// A 2 or 3 dimensional bounding spherical shell in workspace.
////////////////////////////////////////////////////////////////////////////////
class WorkspaceBoundingSphericalShell : public AbstractBoundingSphericalShell {

  public:

    ///@name Construction
    ///@{

    explicit WorkspaceBoundingSphericalShell(const size_t _n,
        const double _outer = std::numeric_limits<double>::max(),
        const double _inner = 0);

    explicit WorkspaceBoundingSphericalShell(const std::vector<double>& _center,
        const double _outer = std::numeric_limits<double>::max(),
        const double _inner = 0);

    explicit WorkspaceBoundingSphericalShell(const Vector3d& _center,
        const double _outer = std::numeric_limits<double>::max(),
        const double _inner = 0);

    WorkspaceBoundingSphericalShell(XMLNode& _node);

    virtual ~WorkspaceBoundingSphericalShell() noexcept;

    virtual std::unique_ptr<Boundary> Clone() const override;

    ///@}
    ///@name Property Accessors
    ///@{

    virtual Boundary::Space Type() const noexcept override;

    virtual std::string Name() const noexcept override;

    ///@}
    ///@name Containment Testing
    ///@{

    using Boundary::InBoundary;

    virtual bool InBoundary(const Cfg& _cfg) const override;

    ///@}
    ///@name Polyhedron Representations
    ///@{

    /// This will be an approximate spherical shell with 960 faces.
    virtual GMSPolyhedron MakePolyhedron() const override;

    ///@}
};

#endif
