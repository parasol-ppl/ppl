#include "Geometry/Boundaries/AbstractBoundingBox.h"

#include "ConfigurationSpace/Cfg.h"
#include "Utilities/MPUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#include "nonstd/numerics.h"

#include <sstream>


/*------------------------------- Construction -------------------------------*/

AbstractBoundingBox::
AbstractBoundingBox(const size_t _n) : NBox(_n) { }


AbstractBoundingBox::
AbstractBoundingBox(const std::vector<double>& _center) : NBox(_center) { }


AbstractBoundingBox::
AbstractBoundingBox(XMLNode& _node) : NBox(0) {
  const std::string limits = _node.Read("limits", true, "",
      "The dimensions of the bounding box.");

  std::istringstream buffer(limits);

  // Try to read in bounding box limits using NBox. Re-propogate any exceptions
  // with more precise 'where' info.
  try {
    buffer >> static_cast<NBox&>(*this);
  }
  catch(PMPLException& _e) {
    throw ParseException(_node.Where(), _e.what());
  }
}


AbstractBoundingBox::
~AbstractBoundingBox() noexcept = default;

/*---------------------------- Property Accessors ----------------------------*/

size_t
AbstractBoundingBox::
GetDimension() const noexcept {
  return NBox::GetDimension();
}


double
AbstractBoundingBox::
GetMaxDist(const double _r1, const double _r2) const {
  double maxdist = 0;

  const size_t maxIndex = std::min(size_t(3), NBox::GetDimension());
  for(size_t i = 0; i < maxIndex; ++i)
    maxdist += std::pow(NBox::GetRange(i).Length(), _r1);

  return std::pow(maxdist, _r2);
}


const Range<double>&
AbstractBoundingBox::
GetRange(const size_t _i) const {
  if(_i > NBox::GetDimension())
    throw RunTimeException(WHERE,
        "Invalid access to dimension '" + std::to_string(_i) + "'.");
  return NBox::GetRange(_i);
}


const std::vector<double>&
AbstractBoundingBox::
GetCenter() const noexcept {
  return NBox::GetCenter();
}


double
AbstractBoundingBox::
GetVolume() const noexcept {
  return NBox::GetVolume();
}

/*-------------------------------- Sampling ----------------------------------*/

std::vector<double>
AbstractBoundingBox::
GetRandomPoint() const {
  return NBox::Sample();
}


void
AbstractBoundingBox::
PushInside(std::vector<double>& _sample) const noexcept {
  if(NBox::Contains(_sample))
    return;

  auto clearancePoint = NBox::ClearancePoint(_sample);

  const size_t dimension = std::min(_sample.size(), clearancePoint.size());
  for(size_t i = 0; i < dimension; ++i)
    _sample[i] = clearancePoint[i];
}

/*----------------------------- Containment Testing --------------------------*/

bool
AbstractBoundingBox::
InBoundary(const std::vector<double>& _p) const {
  return NBox::Contains(_p);
}

/*------------------------------ Clearance Testing ---------------------------*/

double
AbstractBoundingBox::
GetClearance(const Vector3d& _p) const {
  return NBox::Clearance(std::vector<double>{_p[0], _p[1], _p[2]});
}


Vector3d
AbstractBoundingBox::
GetClearancePoint(const Vector3d& _p) const {
  const size_t maxIndex = std::min(size_t(3), NBox::GetDimension());

  std::vector<double> input(maxIndex);
  for(size_t i = 0; i < maxIndex; ++i)
    input[i] = _p[i];

  auto p = NBox::ClearancePoint(input);
  return Vector3d{p[0], p[1], NBox::GetDimension() > 2 ? p[2] : 0};
}


int
AbstractBoundingBox::
GetSideID(const std::vector<double>& _p) const {
  double minClearance = std::numeric_limits<double>::max();
  int id = 0;

  const size_t maxIndex = std::min(size_t(3), NBox::GetDimension());
  for(size_t i = 0; i < maxIndex; ++i) {
    const auto& r = NBox::GetRange(i);
    const double clearance = r.Clearance(_p[i]);
    if(clearance < minClearance) {
      minClearance = clearance;
      id = (_p[i] - r.min) < (r.max - _p[i]) ? i : i + 3;
    }
  }
  return id;
}

/*---------------------------------- Modifiers -------------------------------*/

void
AbstractBoundingBox::
SetCenter(const std::vector<double>& _c) noexcept {
  NBox::SetCenter(_c);
}


void
AbstractBoundingBox::
Translate(const Vector3d& _v) {
  AbstractBoundingBox::Translate(std::vector<double>{_v[0], _v[1], _v[2]});
}


void
AbstractBoundingBox::
Translate(const std::vector<double>& _t) {
  NBox::Translate(_t);
}


void
AbstractBoundingBox::
ResetBoundary(const std::vector<std::pair<double, double>>& _bbx,
    const double _margin) {
  const size_t maxIndex = std::min(_bbx.size(), NBox::GetDimension());
  for(size_t i = 0; i < maxIndex; ++i)
    NBox::SetRange(i, _bbx[i].first - _margin, _bbx[i].second + _margin);
}

/*------------------------------------ I/O -----------------------------------*/

void
AbstractBoundingBox::
Read(std::istream& _is, CountingStreamBuffer& _cbs) {
  // Try to read in using NBox. Re-propogate any exceptions with more precise
  // 'where' info.
  try {
    _is >> static_cast<NBox&>(*this);
  }
  catch(PMPLException& _e) {
    throw ParseException(_cbs.Where(), _e.what());
  }
}


void
AbstractBoundingBox::
Write(std::ostream& _os) const {
  _os << static_cast<const NBox&>(*this);
}


std::ostream&
operator<<(std::ostream& _os, const AbstractBoundingBox& _b) {
  return _os << static_cast<const Boundary&>(_b);
}

/*----------------------------------------------------------------------------*/
