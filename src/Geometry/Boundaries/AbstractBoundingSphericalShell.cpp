#include "Geometry/Boundaries/AbstractBoundingSphericalShell.h"

#include "Geometry/Boundaries/Range.h"
#include "Utilities/MPUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"


/*------------------------------- Construction -------------------------------*/

AbstractBoundingSphericalShell::
AbstractBoundingSphericalShell(const size_t _n, const double _outer,
    const double _inner)
  : NSphericalShell(_n, _outer, _inner), m_range(ComputeRange()) { }


AbstractBoundingSphericalShell::
AbstractBoundingSphericalShell(const std::vector<double>& _center,
    const double _outer, const double _inner)
  : NSphericalShell(_center, _outer, _inner), m_range(ComputeRange()) { }


AbstractBoundingSphericalShell::
AbstractBoundingSphericalShell(XMLNode& _node) : NSphericalShell(0) {
  const std::string limits = _node.Read("limits", true, "",
      "The dimensions of the bounding sphere.");

  std::istringstream buffer(limits);

  // Try to read in limits using NSphericalShell.
  try {
    buffer >> static_cast<NSphericalShell&>(*this);
  }
  catch(PMPLException& _e) {
    throw ParseException(_node.Where(), _e.what());
  }

  m_range = ComputeRange();
}


AbstractBoundingSphericalShell::
~AbstractBoundingSphericalShell() noexcept = default;

/*---------------------------- Property Accessors ----------------------------*/

size_t
AbstractBoundingSphericalShell::
GetDimension() const noexcept {
  return NSphericalShell::GetDimension();
}


double
AbstractBoundingSphericalShell::
GetMaxDist(const double _r1, const double _r2) const {
  return std::pow(2. * NSphericalShell::GetOuterRadius(), _r1 * _r2);
}


const Range<double>&
AbstractBoundingSphericalShell::
GetRange(const size_t _i) const {
  if(_i > NSphericalShell::GetDimension())
    throw RunTimeException(WHERE,
        "Invalid access to dimension '" + std::to_string(_i) + "'.");
  return m_range[_i];
}


const std::vector<double>&
AbstractBoundingSphericalShell::
GetCenter() const noexcept {
  return NSphericalShell::GetCenter();
}


double
AbstractBoundingSphericalShell::
GetVolume() const noexcept {
  return NSphericalShell::GetVolume();
}

/*-------------------------------- Sampling ----------------------------------*/

std::vector<double>
AbstractBoundingSphericalShell::
GetRandomPoint() const {
  return NSphericalShell::Sample();
}


void
AbstractBoundingSphericalShell::
PushInside(std::vector<double>& _sample) const noexcept {
  if(NSphericalShell::Contains(_sample))
    return;

  auto clearancePoint = NSphericalShell::ClearancePoint(_sample);

  const size_t dimension = std::min(_sample.size(), clearancePoint.size());
  for(size_t i = 0; i < dimension; ++i)
    _sample[i] = clearancePoint[i];
}

/*----------------------------- Containment Testing --------------------------*/

bool
AbstractBoundingSphericalShell::
InBoundary(const std::vector<double>& _p) const {
  return NSphericalShell::Contains(_p);
}

/*------------------------------ Clearance Testing ---------------------------*/

double
AbstractBoundingSphericalShell::
GetClearance(const Vector3d& _p) const {
  return NSphericalShell::Clearance(std::vector<double>{_p[0], _p[1], _p[2]});
}


Vector3d
AbstractBoundingSphericalShell::
GetClearancePoint(const Vector3d& _p) const {
  auto v = NSphericalShell::ClearancePoint(
      std::vector<double>{_p[0], _p[1], _p[2]});
  return Vector3d(v[0], v[1], NSphericalShell::GetDimension() > 2 ? v[2] : _p[2]);
}

/*---------------------------------- Modifiers -------------------------------*/

void
AbstractBoundingSphericalShell::
SetCenter(const std::vector<double>& _c) noexcept {
  NSphericalShell::SetCenter(_c);

  const size_t maxIndex = std::min(_c.size(), NSphericalShell::GetDimension());

  for(size_t i = 0; i < maxIndex; ++i)
    m_range[i].SetCenter(_c[i]);
}


void
AbstractBoundingSphericalShell::
Translate(const Vector3d& _v) {
  AbstractBoundingSphericalShell::Translate(
      std::vector<double>{_v[0], _v[1], _v[2]});
}


void
AbstractBoundingSphericalShell::
Translate(const std::vector<double>& _t) {
  NSphericalShell::Translate(_t);

  const size_t maxIndex = std::min(_t.size(), NSphericalShell::GetDimension());

  for(size_t i = 0; i < maxIndex; ++i)
    m_range[i].Translate(_t[i]);
}


void
AbstractBoundingSphericalShell::
ResetBoundary(const std::vector<std::pair<double, double>>&, const double) {
  throw RunTimeException(WHERE, "This operation does not make sense for a "
      "spherical shell. Can't resize each dimension independently!");
}

/*--------------------------------- Helpers ----------------------------------*/

std::vector<Range<double>>
AbstractBoundingSphericalShell::
ComputeRange() const {
  std::vector<Range<double>> r(NSphericalShell::GetDimension());

  for(size_t i = 0; i < GetDimension(); ++i)
    r[i].Resize(NSphericalShell::GetCenter()[i] - NSphericalShell::GetOuterRadius(),
                NSphericalShell::GetCenter()[i] + NSphericalShell::GetOuterRadius());

  return r;
}

/*------------------------------------ I/O -----------------------------------*/

void
AbstractBoundingSphericalShell::
Read(std::istream& _is, CountingStreamBuffer& _cbs) {
  // Try to read in using NSphericalShell. Re-propogate any exceptions with the better
  // debug info from the CountingStreamBuffer.
  try {
    _is >> static_cast<NSphericalShell&>(*this);
  }
  catch(PMPLException& _e) {
    throw ParseException(_cbs.Where(), _e.what());
  }

  m_range = std::move(ComputeRange());
}


void
AbstractBoundingSphericalShell::
Write(std::ostream& _os) const {
  _os << static_cast<const NSphericalShell&>(*this);
}


std::ostream&
operator<<(std::ostream& _os, const AbstractBoundingSphericalShell& _b) {
  return _os << static_cast<const Boundary&>(_b);
}

/*----------------------------------------------------------------------------*/
