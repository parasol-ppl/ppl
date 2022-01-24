#ifndef PMPL_ABSTRACT_BOUNDING_SPHERICAL_SHELL_H_
#define PMPL_ABSTRACT_BOUNDING_SPHERICAL_SHELL_H_

#include "Boundary.h"
#include "Geometry/Shapes/NSphericalShell.h"


////////////////////////////////////////////////////////////////////////////////
/// An n-dimensional bounding spherical shell.
/// @ingroup Geometry
////////////////////////////////////////////////////////////////////////////////
class AbstractBoundingSphericalShell : public Boundary, public NSphericalShell {

  public:

    ///@name Construction
    ///@{

    /// Construct a bounding spherical shell at the origin with a fixed number of
    /// dimensions and radius.
    /// @param _n The number of dimensions to use.
    /// @param _outer The outer radius (infinite by default).
    /// @param _inner The inner radius (0 by default).
    explicit AbstractBoundingSphericalShell(const size_t _n,
        const double _outer = std::numeric_limits<double>::max(),
        const double _inner = 0);

    /// Construct a bounding spherical shell with a given center point and radius.
    /// @param _center The center point, which is assumed to be of full
    ///                dimension.
    /// @param _outer The outer radius (infinite by default).
    /// @param _inner The inner radius (0 by default).
    explicit AbstractBoundingSphericalShell(const std::vector<double>& _center,
        const double _outer = std::numeric_limits<double>::max(),
        const double _inner = 0);

    /// Construct a bounding spherical shell from an XML node.
    /// @param _node The XML node to parse.
    AbstractBoundingSphericalShell(XMLNode& _node);

    virtual ~AbstractBoundingSphericalShell() noexcept;

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

std::ostream& operator<<(std::ostream& _os,
    const AbstractBoundingSphericalShell& _b);

/// @TODO Move impl from environment to here.
//std::istream& operator>>(std::istream& _is, const Boundary& _b);

/*----------------------------------------------------------------------------*/

#endif
