#include "MPUtils.h"
#include "MetricUtils.h"
#include "ConfigurationSpace/Cfg.h"
#include "MPProblem/Robot/Robot.h"

/*------------------------- Random Number Generation -------------------------*/

double DRand() {return drand48();}
long LRand() {return lrand48();}
long MRand() {return mrand48();}

double
GRand(bool _reset) {
  static bool cached = false;
  static double cachedValue;

  // Clear the cache if reset was called.
  if(_reset) {
    cached = false;
    return 0.;
  }

  // If an output is cached, return it.
  if(cached) {
    cached = false;
    return cachedValue;
  }

  // Otherwise, compute the next two values. Store one, cache the other.
  double v1, v2, rsq;
  do {
    v1 = 2 * DRand() - 1.;
    v2 = 2 * DRand() - 1.;
    rsq = v1*v1 + v2*v2;
  } while(rsq >= 1. || rsq == 0.);
  double fac = sqrt(-2. * log(rsq) / rsq);
  cachedValue = v1 * fac;
  cached = true;
  return v2 * fac;
}


double
GaussianDistribution(double _mean, double _stdev) {
  return GRand(false) * _stdev + _mean;
}


void
SRand(const unsigned long _seed) {
  srand(_seed);
  srand48(_seed);
}

/*------------------------------ Geometry Utils ------------------------------*/

double
Normalize(const double& _a) {
  // Translate _a right by one so that mod 2 gives a normalized rotation.
  double a = std::fmod(_a + 1., 2.);
  // Untranslate _a to the [-1, 1] range. If it was negative we would also have
  // to add 2 (one whole rotation).
  return a < 0 ? a + 1 : a - 1;
}


double
TriangleHeight(const Point3d& _a, const Point3d& _b, const Point3d& _c) {
  //Using Heron's formula
  double ab = (_a - _b).norm();
  double bc = (_b - _c).norm();
  double ac = (_a - _c).norm();
  double p = (ab + bc + ac) / 2; //half of the perimeter
  double area = std::sqrt(p * (p - ab) * (p - bc) * (p - ac));
  double height = 2 * area / (std::max(std::max(ab, bc), ac)); //h = 2A/b
  return height;
}


bool
PtInTriangle(const Point2d& _a, const Point2d& _b, const Point2d& _c,
    const Point2d & _p) {
  // Check whether _p is inside the triangle by ensuring that it is in the
  // correct half-space of each edge segment.
  Vector2d ab = _b - _a,
           bc = _c - _b;
  // CW
  if(ab[0] * bc[1] - ab[1] * bc[0] < 0) {
    // ABxAP
    if(ab[0] * (_p[1] - _a[1]) >= ab[1] * (_p[0] - _a[0]))
      return false;
    // BCxBP
    if(bc[0] * (_p[1] - _b[1]) >= bc[1] * (_p[0] - _b[0]))
      return false;
    // CAxCP
    if((_a[0] - _c[0]) * (_p[1] - _c[1]) >= (_a[1] - _c[1]) * (_p[0] - _c[0]))
      return false;
  }
  // CCW
  else {
    // ABxAP
    if(ab[0] * (_p[1] - _a[1]) < ab[1] * (_p[0] - _a[0]))
      return false;
    // BCxBP
    if(bc[0] * (_p[1] - _b[1]) < bc[1] * (_p[0] - _b[0]))
      return false;
    // CAxCP
    if((_a[0] - _c[0]) * (_p[1] - _c[1]) < (_a[1] - _c[1]) * (_p[0] - _c[0]))
      return false;
  }
  return true;
}


bool
PtInTriangle(const Point2d& _a, const Point2d& _b, const Point2d& _c,
    const Point2d& _p, double& _u, double& _v) {

  double epsilon = 0.0000001;

  // Compute vectors
  Vector2d v0 = _c - _a;
  Vector2d v1 = _b - _a;
  Vector2d v2 = _p - _a;

  // Compute dot products
  double dot00 = v0 * v0;
  double dot01 = v0 * v1;
  double dot02 = v0 * v2;
  double dot11 = v1 * v1;
  double dot12 = v1 * v2;

  // Compute barycentric coordinates
  double invDenom = 1. / (dot00 * dot11 - dot01 * dot01);
  _u = (dot11 * dot02 - dot01 * dot12) * invDenom;
  _v = (dot00 * dot12 - dot01 * dot02) * invDenom;

  // Check if point is in triangle
  return (_u >= -epsilon) && (_v >= -epsilon) && (_u + _v < 1. + epsilon);
}


Point3d
GetPtFromBarycentricCoords(const Point3d& _a, const Point3d& _b,
    const Point3d& _c, double _u, double _v) {
  return _a + (_u * (_c - _a)) + (_v * (_b - _a));
}


double
NormalizeTheta(double _theta) {
  double val = _theta + PI;
  return val == 0 ? _theta : val - TWOPI * floor(val / TWOPI) - PI;
}

/*----------------------------------------------------------------------------*/

std::vector<Cfg>
LoadPath(const std::string &_filename, Robot* _robot) {
  std::vector<Cfg> result;
  if(!FileExists(_filename))
    throw ParseException(WHERE, "File '" + _filename + "' does not exist");

  std::ifstream pathfile(_filename);
  std::string line;
  Cfg cfg = Cfg(_robot);
  std::getline(pathfile, line);
  std::getline(pathfile, line);
  std::getline(pathfile, line);
  while(pathfile) {
    std::getline(pathfile, line);
    std::istringstream cfgInput(line);
    cfg.Read(cfgInput);
    result.push_back(cfg);
  }
  return result;
}
