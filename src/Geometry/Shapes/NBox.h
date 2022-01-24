#ifndef PMPL_N_BOX_H_
#define PMPL_N_BOX_H_

#include <cstddef>
#include <iostream>
#include <vector>

#include "Geometry/Boundaries/Range.h"


////////////////////////////////////////////////////////////////////////////////
/// An axis-aligned rectangular prism in n dimensions.
////////////////////////////////////////////////////////////////////////////////
class NBox {

  public:

    ///@name Construction
    ///@{

    /// Construct an infinite box centered at the origin in _n dimensions.
    /// @param _n The number of dimensions.
    explicit NBox(const size_t _n);

    /// Construct an infinite box with a designated center point.
    /// @param _center The center point, which is assumed to be of full
    ///                dimension.
    explicit NBox(const std::vector<double>& _center);

    virtual ~NBox() noexcept;

    ///@}
    ///@name Accessors
    ///@{

    /// Get the dimension of this box.
    size_t GetDimension() const noexcept;

    /// Set the center point.
    void SetCenter(const std::vector<double>& _c) noexcept;

    /// Get the center point.
    const std::vector<double>& GetCenter() const noexcept;

    /// Get a range in a given dimension.
    /// @param _i The dimension index.
    /// @return The range of this volume in dimension _i.
    const Range<double>& GetRange(const size_t _i) const noexcept;

    /// Get all of the ranges.
    const std::vector<Range<double>>& GetRanges() const noexcept;

    /// Set the range in a given dimension.
    /// @param _i The dimension index.
    /// @param _r The new range to use.
    void SetRange(const size_t _i, const Range<double>& _r) noexcept;
    void SetRange(const size_t _i, Range<double>&& _r) noexcept;

    /// Set the range in a given dimension.
    /// @param _i The dimension index.
    /// @param _min The new lower bound.
    /// @param _max The new upper bound.
    void SetRange(const size_t _i, const double _min, const double _max) noexcept;

    /// Translate the entire n-box.
    /// @param _v The translation vector to apply. If it has fewer dimensions
    ///           than the NBox, it will be applied to the first (_v.size())
    ///           dimensions of this.
    void Translate(const std::vector<double>& _v) noexcept;

    /// Compute the (hyper)volume or Lebesgue measure.
    double GetVolume() const noexcept;

    ///@}
    ///@name Point Testing
    ///@{
    /// If the box and point have different dimensions, the missing values will
    /// be assumed to be 0.

    /// Test if a given point lies within the box.
    /// @param _p The point to test.
    /// @return True if _p lies within the box.
    bool Contains(const std::vector<double>& _p) const noexcept;

    /// Compute the minimum distance to the box's surface from a given point.
    /// This is bounding-box style, so clearance is positive if the point is
    /// inside the box and negative if it is outside.
    /// @param _p The point of interest.
    /// @return The minimum distance from _p to the box's surface.
    double Clearance(const std::vector<double>& _p) const noexcept;

    /// Find the point on the box that is nearest to a given reference point.
    /// @param _p The reference point.
    /// @return The point on the surface that is nearest to _p.
    std::vector<double> ClearancePoint(std::vector<double> _p) const noexcept;

    ///@}
    ///@name Sampling
    ///@{

    /// Sample a random point in the box with uniform probability.
    std::vector<double> Sample() const;

    ///@}

  private:

    ///@name Internal State
    ///@{

    std::vector<double> m_center;        ///< The center point of the box.
    std::vector<Range<double>> m_range;  ///< The range in each dimension.

    ///@}

    friend std::istream& operator>>(std::istream& _is, NBox& _box);
};

/*----------------------------------- I/O ------------------------------------*/

std::istream&
operator>>(std::istream& _is, NBox& _box);

std::ostream&
operator<<(std::ostream& _os, const NBox& _box);

/*----------------------------------------------------------------------------*/

#endif
