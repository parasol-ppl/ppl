#ifndef N_SPHERE_H_
#define N_SPHERE_H_

#include <cstddef>
#include <iostream>
#include <limits>
#include <vector>


////////////////////////////////////////////////////////////////////////////////
/// A general representation of a spherical volume in N dimensions.
///
/// This object also models the inscribed hyper-volume, so the proper formal
/// name for it is an 'N-ball'. I.e., setting N = 3 creates a (two-)sphere in R^3
/// plus the inscribed volume. See wikipedia's article on n-sphere for further
/// clarification.
////////////////////////////////////////////////////////////////////////////////
class NSphere {

  public:

    ///@name Construction
    ///@{

    /// Construct an n-sphere at the origin with a given dimension and radius.
    /// @param _n The dimension of the n-sphere.
    /// @param _r The sphere radius (infinite by default).
    explicit NSphere(const size_t _n,
        const double _r = std::numeric_limits<double>::max());

    /// Construct an n-sphere with a given center point and radius.
    /// @param _c The center point of the n-sphere, which is assumed to be of
    ///           full dimension.
    /// @param _r The sphere radius (infinite by default).
    explicit NSphere(const std::vector<double>& _center,
        const double _r = std::numeric_limits<double>::max());

    virtual ~NSphere() noexcept;

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

    /// Get the radius.
    double GetRadius() const noexcept;

    /// Set the radius.
    void SetRadius(const double _r) noexcept;

    /// Translate the entire n-sphere.
    /// @param _v The translation vector to apply.
    void Translate(const std::vector<double>& _v) noexcept;

    /// Compute the (hyper)volume or Lebesgue measure.
    double GetVolume() const noexcept;

    ///@}
    ///@name Point Testing
    ///@{
    /// If the sphere and point have different dimensions, the missing values
    /// will be assumed to be 0.

    /// Test if a given point lies within the n-sphere.
    /// @param _p The point to test.
    /// @return True if _p lies within the n-sphere.
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

    /// Sample a random point in the n-sphere with uniform probability via the
    /// Muller/Marsaglia method.
    std::vector<double> Sample() const;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::vector<double> m_center; ///< The center point.
    double m_radius;              ///< The radius.

    ///@}

    friend std::istream& operator>>(std::istream& _is, NSphere& _sphere);
};

/*----------------------------------- I/O ------------------------------------*/

std::istream&
operator>>(std::istream& _is, NSphere& _sphere);

std::ostream&
operator<<(std::ostream& _os, const NSphere& _sphere);

/*----------------------------------------------------------------------------*/

#endif
