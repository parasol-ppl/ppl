#ifndef C_SPACE_BOUNDING_SPHERE_H_
#define C_SPACE_BOUNDING_SPHERE_H_

#include "AbstractBoundingSphere.h"


////////////////////////////////////////////////////////////////////////////////
/// An n-dimensional bounding sphere in c-space.
////////////////////////////////////////////////////////////////////////////////
class CSpaceBoundingSphere : public AbstractBoundingSphere {

  public:

    ///@name Construction
    ///@{

    explicit CSpaceBoundingSphere(const size_t _n,
        const double _radius = std::numeric_limits<double>::max());

    explicit CSpaceBoundingSphere(const std::vector<double>& _center,
        const double _radius = std::numeric_limits<double>::max());

    CSpaceBoundingSphere(XMLNode& _node);

    virtual ~CSpaceBoundingSphere() noexcept;

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
};

#endif
