#include "NSphericalShell.h"

#include "nonstd/container_ops.h"

#include "Utilities/PMPLExceptions.h"
#include "Utilities/MPUtils.h"


/*------------------------------ Construction --------------------------------*/

NSphericalShell::
NSphericalShell(const size_t _n, const double _outer, const double _inner)
  : m_center(_n, 0), m_outerRadius(_outer), m_innerRadius(_inner) { }


NSphericalShell::
NSphericalShell(const std::vector<double>& _c, const double _outer,
    const double _inner)
  : m_center(_c), m_outerRadius(_outer), m_innerRadius(_inner) { }


NSphericalShell::
~NSphericalShell() noexcept = default;

/*------------------------------- Accessors ----------------------------------*/

size_t
NSphericalShell::
GetDimension() const noexcept {
  return m_center.size();
}


void
NSphericalShell::
SetCenter(const std::vector<double>& _c) noexcept {
  const size_t maxIndex = std::min(_c.size(), GetDimension());
  for(size_t i = 0; i < maxIndex; ++i)
    m_center[i] = _c[i];
}


const std::vector<double>&
NSphericalShell::
GetCenter() const noexcept {
  return m_center;
}


double
NSphericalShell::
GetOuterRadius() const noexcept {
  return m_outerRadius;
}


double
NSphericalShell::
GetInnerRadius() const noexcept {
  return m_innerRadius;
}


void
NSphericalShell::
SetOuterRadius(const double _r) noexcept {
  m_outerRadius = _r;
}


void
NSphericalShell::
SetInnerRadius(const double _r) noexcept {
  m_innerRadius = _r;
}


void
NSphericalShell::
Translate(const std::vector<double>& _v) noexcept {
  const size_t maxIndex = std::min(_v.size(), GetDimension());
  for(size_t i = 0; i < maxIndex; ++i)
    m_center[i] += _v[i];
}


double
NSphericalShell::
GetVolume() const noexcept {
  // Ref: https://en.wikipedia.org/wiki/Volume_of_an_n-ball
  const double d     = GetDimension(),
               halfD = d / 2.,
               a     = std::pow(PI, halfD),
               b     = std::tgamma(halfD + 1);
  return (std::pow(m_outerRadius, d) - std::pow(m_innerRadius, d)) * a / b;
}

/*--------------------------------- Testing ----------------------------------*/

bool
NSphericalShell::
Contains(const std::vector<double>& _p) const noexcept {
  return Clearance(_p) >= 0;
}


double
NSphericalShell::
Clearance(std::vector<double> _p) const noexcept {
  // Only consider dimensions that are in both _p and this.
  const size_t maxIndex = std::min(_p.size(), GetDimension());
  _p.resize(maxIndex);

  // Transform _p to the coordinate frame at the sphere's center.
  for(size_t i = 0; i < maxIndex; ++i)
    _p[i] -= m_center[i];

  const double magnitude = nonstd::magnitude<double>(_p);

  return std::min(m_outerRadius - magnitude, magnitude - m_innerRadius);
}


std::vector<double>
NSphericalShell::
ClearancePoint(std::vector<double> _p) const noexcept {
  // Only consider dimensions that are in both _p and this.
  const size_t maxIndex = std::min(_p.size(), GetDimension());
  _p.resize(maxIndex, 0.);

  // Transform _p to the coordinate frame at the sphere's center.
  for(size_t i = 0; i < maxIndex; ++i)
    _p[i] -= m_center[i];

  const double magnitude = nonstd::magnitude<double>(_p);

  // Find the scaling factors that place _p on the sphere's boundaries.
  const double outerScale = m_outerRadius / magnitude,
               innerScale = m_innerRadius / magnitude;

  // Choose the least extreme scaling factor (closest to 1).
  const double scale = std::abs(outerScale - 1) < std::abs(innerScale - 1)
                     ? outerScale : innerScale;

  // Scale the values in point and return to original coordinate frame.
  for(size_t i = 0; i < maxIndex; ++i) {
    _p[i] *= scale;
    _p[i] += m_center[i];
  }

  return _p;
}

/*--------------------------------- Sampling ---------------------------------*/

std::vector<double>
NSphericalShell::
Sample() const {
  // Generate a direction with uniform probability by creating a random value
  // for each dimension in the range [-1,1] with gaussian probability (mean 0,
  // var 1).
  std::vector<double> point(GetDimension());
  for(auto& value : point)
    value = GRand();

  // Generate a distance-from-center. The random fraction of the radius d must be
  // adjusted to d^(1/N) to get uniform sampling within the N-sphere's volume.
  // Since this is a shell instead of a whole sphere, we also adjust d so that
  // it initially produces values only within the shell.
  /// @todo This is likely close but not correct, need to verify.
  const double d0 = std::pow(DRand(), 1. / GetDimension()),
               fractionEmpty = m_innerRadius / m_outerRadius,
               d  = (1 - fractionEmpty) * d0 + fractionEmpty;

  const double scale = m_outerRadius * d
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
operator>>(std::istream& _is, NSphericalShell& _sphere) {
  _sphere.m_center.clear();

  // Read opening bracket.
  char c;
  if(!(_is >> c && c == '['))
    throw ParseException(WHERE, "Failed reading NSphericalShell bounds. "
        "Missing '['.");

  // Read the center point values.
  double temp;
  while(true) {
    // Eat white space.
    _is >> std::ws;

    // If the next character is not digit, we are done reading center point
    // values.
    if(!isdigit(_is.peek()))
      break;

    // Otherwise, read the next center point value.
    if(!(_is >> temp))
      throw ParseException(WHERE, "Failed reading center point value " +
          std::to_string(_sphere.m_center.size()) + ".");
    _sphere.m_center.push_back(temp);
  }

  // Read separator.
  if(!(_is >> c and c == ';'))
    throw ParseException(WHERE, "Failed reading NSphericalShell bounds. "
        "Missing ';'.");

  // Read the outer radius.
  if(!(_is >> temp))
    throw ParseException(WHERE, "Failed reading NSphericalShell outer radius.");
  _sphere.m_outerRadius = temp;

  // Read the inner radius.
  if(!(_is >> temp))
    throw ParseException(WHERE, "Failed reading NSphericalShell inner radius.");
  _sphere.m_innerRadius = temp;

  // Swap the radii if they are mixed up.
  if(_sphere.m_innerRadius > _sphere.m_outerRadius)
    std::swap(_sphere.m_innerRadius, _sphere.m_outerRadius);

  // Read the last separator.
  _is >> std::ws;
  if(!(_is >> c and c == ']'))
    throw ParseException(WHERE, "Failed reading NSphericalShell bounds. "
        "Missing ']'.");

  return _is;
}


std::ostream&
operator<<(std::ostream& _os, const NSphericalShell& _sphere) {
  _os << "[ ";

  for(size_t i = 0; i < _sphere.GetDimension() - 1; ++i)
    _os << _sphere.GetCenter()[i] << " ";

  return _os << "; " << _sphere.GetOuterRadius() << " "
             << _sphere.GetInnerRadius() << " ]";
}

/*----------------------------------------------------------------------------*/
