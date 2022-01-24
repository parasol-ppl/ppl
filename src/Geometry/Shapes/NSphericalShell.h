#ifndef PMPL_N_SPHERICAL_SHELL_H_
#define PMPL_N_SPHERICAL_SHELL_H_

#include <cstddef>
#include <iostream>
#include <limits>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// A general representation of a spherical shell volume in N dimensions (i.e.,
/// a spherical volume with an outer and inner radius).
///
/// Like NSphere, this object also models the inscribed hyper-volume.
////////////////////////////////////////////////////////////////////////////////
class NSphericalShell {

  public:

    ///@name Construction
    ///@{

    /// Construct an n-spherical shell at the origin with a given dimension and
    /// radius.
    /// @param _n The dimension.
    /// @param _outer The outer radius (infinite by default).
    /// @param _inner The inner radius (0 by default).
    explicit NSphericalShell(const size_t _n,
        const double _outer = std::numeric_limits<double>::max(),
        const double _inner = 0);

    /// Construct an n-spherical shell with a given center point and radius.
    /// @param _c The center point, which is assumed to be of full dimension.
    /// @param _outer The outer radius (infinite by default).
    /// @param _inner The inner radius (0 by default).
    explicit NSphericalShell(const std::vector<double>& _center,
        const double _outer = std::numeric_limits<double>::max(),
        const double _inner = 0);

    virtual ~NSphericalShell() noexcept;

    ///@}
    ///@name Accessors
    ///@{

    /// Get the dimension of this sphere.
    size_t GetDimension() const noexcept;

    /// Set the center point.
    /// @param _c The new center point.
    void SetCenter(const std::vector<double>& _c) noexcept;

    /// Get the center point.
    const std::vector<double>& GetCenter() const noexcept;

    /// Get the outer radius.
    double GetOuterRadius() const noexcept;

    /// Get the inner radius.
    double GetInnerRadius() const noexcept;

    /// Set the outer radius.
    void SetOuterRadius(const double _r) noexcept;

    /// Set the inner radius.
    void SetInnerRadius(const double _r) noexcept;

    /// Translate the entire n-spherical shell.
    /// @param _v The translation vector to apply.
    void Translate(const std::vector<double>& _v) noexcept;

    /// Compute the (hyper)volume or Lebesgue measure.
    double GetVolume() const noexcept;

    ///@}
    ///@name Point Testing
    ///@{
    /// If the sphere and point have different dimensions, the missing values
    /// will be assumed to be 0.

    /// Test if a given point lies within the n-spherical shell.
    /// @param _p The point to test.
    /// @return True if _p lies within the n-spherical shell.
    bool Contains(const std::vector<double>& _p) const noexcept;

    /// Compute the minimum distance to the sphere's surface from a given point.
    /// This is bounding-sphere style, so clearance is positive if the point is
    /// inside the sphere and negative if it is outside.
    /// @param _p The point of interest.
    /// @return The minimum distance from _p to the sphere's surface.
    double Clearance(std::vector<double> _p) const noexcept;

    /// Find the point on the sphere that is nearest to a given reference point.
    /// @param _p The reference point.
    /// @return The point on the surface that is nearest to _p.
    std::vector<double> ClearancePoint(std::vector<double> _p) const noexcept;

    ///@}
    ///@name Sampling
    ///@{

    /// Sample a random point in the n-spherical shell with uniform probability
    /// via the Muller/Marsaglia method.
    std::vector<double> Sample() const;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::vector<double> m_center; ///< The center point.
    double m_outerRadius;         ///< The outer radius.
    double m_innerRadius;         ///< The inner radius.

    ///@}

    friend std::istream& operator>>(std::istream& _is, NSphericalShell& _sphere);
};

/*----------------------------------- I/O ------------------------------------*/

std::istream&
operator>>(std::istream& _is, NSphericalShell& _sphere);

std::ostream&
operator<<(std::ostream& _os, const NSphericalShell& _sphere);

/*----------------------------------------------------------------------------*/

#endif
