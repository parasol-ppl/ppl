#include "NSphere.h"

#include "nonstd/container_ops.h"

#include "Utilities/PMPLExceptions.h"
#include "Utilities/MPUtils.h"


/*------------------------------ Construction --------------------------------*/

NSphere::
NSphere(const size_t _n, const double _r) : m_center(_n, 0), m_radius(_r) { }


NSphere::
NSphere(const std::vector<double>& _c, const double _r) : m_center(_c),
    m_radius(_r) { }


NSphere::
~NSphere() noexcept = default;

/*------------------------------- Accessors ----------------------------------*/

size_t
NSphere::
GetDimension() const noexcept {
  return m_center.size();
}


void
NSphere::
SetCenter(const std::vector<double>& _c) noexcept {
  const size_t maxIndex = std::min(_c.size(), GetDimension());
  for(size_t i = 0; i < maxIndex; ++i)
    m_center[i] = _c[i];
}


const std::vector<double>&
NSphere::
GetCenter() const noexcept {
  return m_center;
}


double
NSphere::
GetRadius() const noexcept {
  return m_radius;
}


void
NSphere::
SetRadius(const double _r) noexcept {
  m_radius = _r;
}


void
NSphere::
Translate(const std::vector<double>& _v) noexcept {
  const size_t maxIndex = std::min(_v.size(), GetDimension());
  for(size_t i = 0; i < maxIndex; ++i)
    m_center[i] += _v[i];
}


double
NSphere::
GetVolume() const noexcept {
  // Ref: https://en.wikipedia.org/wiki/Volume_of_an_n-ball
  const double d     = GetDimension(),
               halfD = d / 2.;
  return std::pow(m_radius, d) * std::pow(PI, halfD) / std::tgamma(halfD + 1);
}

/*--------------------------------- Testing ----------------------------------*/

bool
NSphere::
Contains(const std::vector<double>& _p) const noexcept {
  return Clearance(_p) >= 0;
}


double
NSphere::
Clearance(std::vector<double> _p) const noexcept {
  // Only consider dimensions that are in both _p and this.
  const size_t maxIndex = std::min(_p.size(), GetDimension());
  _p.resize(maxIndex);

  // Transform _p to the coordinate frame at the sphere's center.
  for(size_t i = 0; i < maxIndex; ++i)
    _p[i] -= m_center[i];

  return m_radius - nonstd::magnitude<double>(_p);
}


std::vector<double>
NSphere::
ClearancePoint(std::vector<double> _p) const noexcept {
  // Only consider dimensions that are in both _p and this.
  const size_t maxIndex = std::min(_p.size(), GetDimension());
  _p.resize(maxIndex, 0.);

  // Transform _p to the coordinate frame at the sphere's center.
  for(size_t i = 0; i < maxIndex; ++i)
    _p[i] -= m_center[i];

  // Find the scaling factor that places _p on the sphere's boundary.
  const double scale = m_radius / nonstd::magnitude<double>(_p);

  // Scale the values in point and return to original coordinate frame.
  for(size_t i = 0; i < maxIndex; ++i) {
    _p[i] *= scale;
    _p[i] += m_center[i];
  }

  return _p;
}

/*--------------------------------- Sampling ---------------------------------*/

std::vector<double>
NSphere::
Sample() const {
  // Generate a direction with uniform probability by creating a random value
  // for each dimension in the range [-1,1] with gaussian probability (mean 0,
  // var 1).
  std::vector<double> point(GetDimension());
  for(auto& value : point)
    value = GRand();

  // Generate a distance-from-center. The random fraction of the radius d must be
  // adjusted to d^(1/N) to get uniform sampling within the N-sphere's volume.
  const double scale = m_radius * std::pow(DRand(), 1. / GetDimension())
                     / nonstd::magnitude<double>(point);

  // Scale and translate the point.
  for(size_t i = 0; i < point.size(); ++i) {
    point[i] *= scale;
    point[i] += m_center[i];
  }

  return point;
}

/*----------------------------------- I/O ------------------------------------*/

std::istream&
operator>>(std::istream& _is, NSphere& _sphere) {
  _sphere.m_center.clear();

  // Read opening bracket.
  char c;
  if(!(_is >> c && c == '['))
    throw ParseException(WHERE) << "Failed reading NSphere bounds. Missing '['.";

  // Read the center point values.
  double temp;
  while(true) {
    // Eat white space.
    _is >> std::ws;

    // If the next character is not digit, we are done reading center point
    // values.
    c = _is.peek();
    if(!isdigit(c) and c != '-' and c != '.')
      break;

    // Otherwise, read the next center point value.
    if(!(_is >> temp))
      throw ParseException(WHERE) << "Failed reading center point value "
                                  << _sphere.m_center.size() << ".";
    _sphere.m_center.push_back(temp);
  }

  // Read separator.
  if(!(_is >> c and c == ';'))
    throw ParseException(WHERE) << "Failed reading NSphere bounds. Read '"
                                << c << "' instead of ';'.";

  // Read the radius.
  if(!(_is >> temp))
    throw ParseException(WHERE) << "Failed reading NSphere radius.";
  _sphere.m_radius = temp;

  // Read the last separator.
  _is >> std::ws;
  if(!(_is >> c and c == ']'))
    throw ParseException(WHERE) << "Failed reading NSphere bounds. Missing ']'.";

  return _is;
}


std::ostream&
operator<<(std::ostream& _os, const NSphere& _sphere) {
  _os << "[ ";
  for(size_t i = 0; i < _sphere.GetDimension(); ++i)
    _os << _sphere.GetCenter()[i] << " ";
  return _os << "; " << _sphere.GetRadius() << " ]";
}

/*----------------------------------------------------------------------------*/
