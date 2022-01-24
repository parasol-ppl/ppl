#ifndef CSPACE_BOUNDING_BOX_H_
#define CSPACE_BOUNDING_BOX_H_

#include "AbstractBoundingBox.h"


////////////////////////////////////////////////////////////////////////////////
/// An n-dimensional bounding box in c-space.
////////////////////////////////////////////////////////////////////////////////
class CSpaceBoundingBox : public AbstractBoundingBox {

  public:

    ///@name Construction
    ///@{

    explicit CSpaceBoundingBox(const size_t _n);

    explicit CSpaceBoundingBox(const std::vector<double>& _center);

    CSpaceBoundingBox(XMLNode& _node);

    virtual ~CSpaceBoundingBox() noexcept;

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
    ///@name Special Modifiers
    ///@{

    /// Shrink the box to a single point in C-Space.
    void ShrinkToPoint(const Cfg& _c) noexcept;

    ///@}

};

#endif
