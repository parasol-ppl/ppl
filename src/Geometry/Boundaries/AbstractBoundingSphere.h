#ifndef ABSTRACT_BOUNDING_SPHERE_H_
#define ABSTRACT_BOUNDING_SPHERE_H_

#include "Boundary.h"
#include "Geometry/Shapes/NSphere.h"


////////////////////////////////////////////////////////////////////////////////
/// An n-dimensional bounding sphere.
/// @ingroup Geometry
////////////////////////////////////////////////////////////////////////////////
class AbstractBoundingSphere : public Boundary, public NSphere {

  public:

    ///@name Construction
    ///@{

    /// Construct a bounding sphere at the origin with a fixed number of
    /// dimensions and radius.
    /// @param _n The number of dimensions to use.
    /// @param _radius The radius (infinite by default).
    explicit AbstractBoundingSphere(const size_t _n,
        const double _radius = std::numeric_limits<double>::max());

    /// Construct a bounding sphere with a given center point and radius.
    /// @param _center The center point, which is assumed to be of full
    ///                dimension.
    /// @param _radius The radius (infinite by default).
    explicit AbstractBoundingSphere(const std::vector<double>& _center,
        const double _radius = std::numeric_limits<double>::max());

    /// Construct a bounding sphere from an XML node.
    /// @param _node The XML node to parse.
    AbstractBoundingSphere(XMLNode& _node);

    virtual ~AbstractBoundingSphere() noexcept;

    ///@}
    ///@name Boundary Properties
    ///@{

    virtual size_t GetDimension() const noexcept override;

    virtual double GetMaxDist(const double _r1 = 2.0, const double _r2 = 0.5)
        const override;

    virtual const Range<double>& GetRange(const size_t _i) const override;

    virtual const std::vector<double>& GetCenter() const noexcept override;

    virtual double GetVolume() const noexcept override;

    ///@}
    ///@name Sampling
    ///@{

    virtual std::vector<double> GetRandomPoint() const override;

    virtual void PushInside(std::vector<double>& _sample) const noexcept override;

    ///@}
    ///@name Containment Testing
    ///@{

    using Boundary::InBoundary;

    virtual bool InBoundary(const std::vector<double>& _p) const override;

    ///@}
    ///@name Clearance Testing
    ///@{

    virtual double GetClearance(const Vector3d& _p) const override;

    virtual Vector3d GetClearancePoint(const Vector3d& _p) const override;

    ///@}
    ///@name Modifiers
    ///@{

    virtual void SetCenter(const std::vector<double>& _c) noexcept override;

    virtual void Translate(const Vector3d& _v) override;

    virtual void Translate(const std::vector<double>& _t) override;

    virtual void ResetBoundary(const std::vector<std::pair<double, double>>& _bbx,
        const double _margin) override;

    ///@}
    ///@name I/O
    ///@{

    virtual void Read(std::istream& _is, CountingStreamBuffer& _cbs) override;

    virtual void Write(std::ostream& _os) const override;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Compute the ranges.
    std::vector<Range<double>> ComputeRange() const;

    ///@}
    ///@name Internal State
    ///@{

    std::vector<Range<double>> m_range;

    ///@}

};

/*----------------------------------- I/O ------------------------------------*/

std::ostream& operator<<(std::ostream& _os, const AbstractBoundingSphere& _b);

/// @TODO Move impl from environment to here.
//std::istream& operator>>(std::istream& _is, const Boundary& _b);

/*----------------------------------------------------------------------------*/

#endif
