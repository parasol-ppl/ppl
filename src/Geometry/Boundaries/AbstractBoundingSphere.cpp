#include "Geometry/Boundaries/AbstractBoundingSphere.h"

#include "Geometry/Boundaries/Range.h"
#include "Utilities/MPUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"


/*------------------------------- Construction -------------------------------*/

AbstractBoundingSphere::
AbstractBoundingSphere(const size_t _n, const double _radius) :
    NSphere(_n, _radius), m_range(ComputeRange()) { }


AbstractBoundingSphere::
AbstractBoundingSphere(const std::vector<double>& _center, const double _radius) :
    NSphere(_center, _radius), m_range(ComputeRange()) { }


AbstractBoundingSphere::
AbstractBoundingSphere(XMLNode& _node) : NSphere(0) {
  const std::string limits = _node.Read("limits", true, "",
      "The dimensions of the bounding sphere.");

  std::istringstream buffer(limits);

  // Try to read in limits using NSphere.
  try {
    buffer >> static_cast<NSphere&>(*this);
  }
  catch(PMPLException& _e) {
    throw ParseException(_node.Where(), _e.what());
  }

  m_range = ComputeRange();
}


AbstractBoundingSphere::
~AbstractBoundingSphere() noexcept = default;

/*---------------------------- Property Accessors ----------------------------*/

size_t
AbstractBoundingSphere::
GetDimension() const noexcept {
  return NSphere::GetDimension();
}


double
AbstractBoundingSphere::
GetMaxDist(const double _r1, const double _r2) const {
  return std::pow(2. * NSphere::GetRadius(), _r1 * _r2);
}


const Range<double>&
AbstractBoundingSphere::
GetRange(const size_t _i) const {
  if(_i > NSphere::GetDimension())
    throw RunTimeException(WHERE,
        "Invalid access to dimension '" + std::to_string(_i) + "'.");
  return m_range[_i];
}


const std::vector<double>&
AbstractBoundingSphere::
GetCenter() const noexcept {
  return NSphere::GetCenter();
}


double
AbstractBoundingSphere::
GetVolume() const noexcept {
  return NSphere::GetVolume();
}

/*-------------------------------- Sampling ----------------------------------*/

std::vector<double>
AbstractBoundingSphere::
GetRandomPoint() const {
  return NSphere::Sample();
}


void
AbstractBoundingSphere::
PushInside(std::vector<double>& _sample) const noexcept {
  if(NSphere::Contains(_sample))
    return;

  auto clearancePoint = NSphere::ClearancePoint(_sample);

  const size_t dimension = std::min(_sample.size(), clearancePoint.size());
  for(size_t i = 0; i < dimension; ++i)
    _sample[i] = clearancePoint[i];
}

/*----------------------------- Containment Testing --------------------------*/

bool
AbstractBoundingSphere::
InBoundary(const std::vector<double>& _p) const {
  return NSphere::Contains(_p);
}

/*------------------------------ Clearance Testing ---------------------------*/

double
AbstractBoundingSphere::
GetClearance(const Vector3d& _p) const {
  return NSphere::Clearance(std::vector<double>{_p[0], _p[1], _p[2]});
}


Vector3d
AbstractBoundingSphere::
GetClearancePoint(const Vector3d& _p) const {
  auto v = NSphere::ClearancePoint(std::vector<double>{_p[0], _p[1], _p[2]});
  return Vector3d(v[0], v[1], NSphere::GetDimension() > 2 ? v[2] : _p[2]);
}

/*---------------------------------- Modifiers -------------------------------*/

void
AbstractBoundingSphere::
SetCenter(const std::vector<double>& _c) noexcept {
  NSphere::SetCenter(_c);

  const size_t maxIndex = std::min(_c.size(), NSphere::GetDimension());

  for(size_t i = 0; i < maxIndex; ++i)
    m_range[i].SetCenter(_c[i]);
}


void
AbstractBoundingSphere::
Translate(const Vector3d& _v) {
  AbstractBoundingSphere::Translate(std::vector<double>{_v[0], _v[1], _v[2]});
}


void
AbstractBoundingSphere::
Translate(const std::vector<double>& _t) {
  NSphere::Translate(_t);

  const size_t maxIndex = std::min(_t.size(), NSphere::GetDimension());

  for(size_t i = 0; i < maxIndex; ++i)
    m_range[i].Translate(_t[i]);
}


void
AbstractBoundingSphere::
ResetBoundary(const std::vector<std::pair<double, double>>&, const double) {
  throw RunTimeException(WHERE, "This operation does not make sense for a "
      "sphere. Can't resize each dimension independently!");
}

/*--------------------------------- Helpers ----------------------------------*/

std::vector<Range<double>>
AbstractBoundingSphere::
ComputeRange() const {
  std::vector<Range<double>> r(NSphere::GetDimension());

  for(size_t i = 0; i < GetDimension(); ++i)
    r[i].Resize(NSphere::GetCenter()[i] - NSphere::GetRadius(),
                NSphere::GetCenter()[i] + NSphere::GetRadius());

  return r;
}

/*------------------------------------ I/O -----------------------------------*/

void
AbstractBoundingSphere::
Read(std::istream& _is, CountingStreamBuffer& _cbs) {
  // Try to read in using NSphere. Re-propogate any exceptions with the better
  // debug info from the CountingStreamBuffer.
  try {
    _is >> static_cast<NSphere&>(*this);
  }
  catch(PMPLException& _e) {
    throw ParseException(_cbs.Where(), _e.what());
  }

  m_range = std::move(ComputeRange());
}


void
AbstractBoundingSphere::
Write(std::ostream& _os) const {
  _os << static_cast<const NSphere&>(*this);
}


std::ostream&
operator<<(std::ostream& _os, const AbstractBoundingSphere& _b) {
  return _os << static_cast<const Boundary&>(_b);
}

/*----------------------------------------------------------------------------*/
