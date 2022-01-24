#ifndef PMPL_WORKSPACE_BOUNDING_BOX_H_
#define PMPL_WORKSPACE_BOUNDING_BOX_H_

#include "AbstractBoundingBox.h"


////////////////////////////////////////////////////////////////////////////////
/// A two or three dimensional bounding box in workspace.
////////////////////////////////////////////////////////////////////////////////
class WorkspaceBoundingBox : public AbstractBoundingBox {

  public:

    ///@name Construction
    ///@{

    explicit WorkspaceBoundingBox(const size_t _n);

    explicit WorkspaceBoundingBox(const std::vector<double>& _center);

    WorkspaceBoundingBox(XMLNode& _node);

    virtual ~WorkspaceBoundingBox() noexcept;

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

    virtual bool InBoundary(const Cfg& _c) const override;

    ///@}
    ///@name Polyhedron Representations
    ///@{

    using Boundary::CGALPolyhedron;

    virtual CGALPolyhedron CGAL() const override;

    virtual GMSPolyhedron MakePolyhedron() const override;

    ///@}

};

#endif
