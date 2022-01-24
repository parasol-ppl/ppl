#ifndef RANGE_TYPE_H_
#define RANGE_TYPE_H_

#include <cctype>
#include <iostream>
#include <limits>
#include <type_traits>
#include <utility>

#include "Utilities/MPUtils.h"


////////////////////////////////////////////////////////////////////////////////
/// A range of numeric values.
////////////////////////////////////////////////////////////////////////////////
template <typename T>
struct Range final {

  ///@}
  ///@name Internal State
  ///@{

  T min;  ///< The lower bound on this range.
  T max;  ///< The upper bound on this range.

  ///@}
  ///@name Construction
  ///@{

  /// Construct a range over all values of T.
  Range() noexcept;

  /// Construct a bounded range.
  /// @param _min The lower bound.
  /// @param _max The upper bound.
  Range(const T _min, const T _max) noexcept;

  /// Construct a bounded range.
  /// @param _bounds A pair of min, max values.
  Range(const std::pair<T, T>& _bounds) noexcept;

  ///@}
  ///@name Queries
  ///@{

  /// Compute the length of this range.
  T Length() const noexcept;

  /// Compute the center of this range.
  /// @warning This will not be exact for integral types.
  T Center() const noexcept;

  /// Test if a value is inside this range.
  /// @param _val The value to test.
  /// @param _tolerance Tolerance to accommodate floating-point error, as a
  ///                   fraction of the min/max.
  /// @return True if the test value lies inside the range (or on the boundary
  ///         for inclusive test).
  template <typename U>
  bool Contains(const U& _val, const double _tolerance = 1e-8)
      const noexcept;

  /// Test the clearance of a value. Clearance is defined as the minimum
  /// distance to a boundary value, and will be negative if the test value falls
  /// outside the range.
  /// @param _val The value to test.
  /// @return The distance of _val from the nearest endpoint.
  template <typename U>
  T Clearance(const U& _val) const noexcept;

  /// Find the range endpoint that is closest to a target value.
  /// @param _val The target value.
  /// @return The endpoint of this that is closest to _val.
  template <typename U>
  T ClearancePoint(const U& _val) const noexcept;

  /// Sample the range for a random contained value with uniform probability.
  T Sample() const noexcept;

  /// Checks for equality with another Range
  bool operator==(const Range<T>& _other) const {
    return min == _other.min && max == _other.max;
  }

  ///@}
  ///@name Modifiers
  ///@{

  /// Resize the range.
  /// @param _min The new lower bound.
  /// @param _max The new upper bound.
  void Resize(const T _min, const T _max) noexcept;

  /// Translate the range.
  /// @param _t The translation magnitude.
  void Translate(const T _t) noexcept;

  /// Translate the range to a new center.
  /// @param _t The new center value.
  void SetCenter(const T _t) noexcept;

  /// Expand the range to include a value.
  /// @param _t The new value to include.
  void ExpandToInclude(const T _t) noexcept;

  ///@}

};

/*------------------------------- Construction -------------------------------*/

template <typename T>
inline
Range<T>::
Range() noexcept :
    min(T(0)), max(T(0)) { }


template <typename T>
inline
Range<T>::
Range(const T _min, const T _max) noexcept :
    min(_min), max(_max) { }


template <typename T>
inline
Range<T>::
Range(const std::pair<T, T>& _bounds) noexcept :
    min(_bounds.first), max(_bounds.second) { }

/*-------------------------------- Queries -----------------------------------*/

template <typename T>
inline
T
Range<T>::
Length() const noexcept {
  return max -  min;
}


template <typename T>
inline
T
Range<T>::
Center() const noexcept {
  return (max + min) / T(2);
}


template <typename T>
template <typename U>
inline
bool
Range<T>::
Contains(const U& _val, const double _tolerance) const noexcept {
  return (min - std::abs(min) * _tolerance) <= _val
     and _val <= (max + std::abs(max) * _tolerance);
}


template <typename T>
template <typename U>
inline
T
Range<T>::
Clearance(const U& _val) const noexcept {
  static_assert(!std::is_unsigned<T>::value, "Can't compute clearance for "
      "unsigned ranges as the return would be negative if the test value "
      "lies outside the range.");
  return std::min(_val - min, max - _val);
}


template <typename T>
template <typename U>
inline
T
Range<T>::
ClearancePoint(const U& _val) const noexcept {
  static_assert(!std::is_unsigned<T>::value, "Can't compute clearance for "
      "unsigned ranges as the return would be negative if the test value "
      "lies outside the range.");
  const T distToMin = std::abs(min - _val),
          distToMax = std::abs(max - _val);
  return distToMin < distToMax ? min : max;
}


template <typename T>
inline
T
Range<T>::
Sample() const noexcept {
  return min + (max - min) * DRand();
}

/*-------------------------------- Modifiers ---------------------------------*/

template <typename T>
inline
void
Range<T>::
Resize(const T _min, const T _max) noexcept {
  min = _min;
  max = _max;
}


template <typename T>
inline
void
Range<T>::
Translate(const T _t) noexcept {
  min += _t;
  max += _t;
}


template <typename T>
inline
void
Range<T>::
SetCenter(const T _t) noexcept {
  Translate(_t - Center());
}


template <typename T>
inline
void
Range<T>::
ExpandToInclude(const T _t) noexcept {
  min = std::min(min, _t);
  max = std::max(max, _t);
}

/*---------------------------------- I/O -------------------------------------*/

template <typename T>
std::ostream&
operator<<(std::ostream& _os, const Range<T>& _r) {
  return _os << _r.min << ":" << _r.max;
}


template <typename T>
std::istream&
operator>>(std::istream& _is, Range<T>& _r) {
  char delim;
  return _is >> _r.min >> std::skipws >> delim >> _r.max;
}

/*----------------------------------------------------------------------------*/

#endif
