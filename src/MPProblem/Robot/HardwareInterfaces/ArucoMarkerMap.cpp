#include "ArucoMarkerMap.h"

#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#include <algorithm>
#include <sstream>


/*------------------------------- Construction -------------------------------*/

ArucoMarkerMap::
ArucoMarkerMap(const std::string& _filename) {
  // Open the XML and get the root node.
  XMLNode root(_filename, "MarkerMap");

  // Read the version and units.
  const size_t version = root.Read<size_t>("version", false, 0, 1,
      std::numeric_limits<size_t>::max(), "The map file version number.");

  constexpr size_t maxVersion = 1;
  if(version > maxVersion)
    throw ParseException(WHERE) << "Unrecognized file version, up to "
                                << maxVersion << " is supported.";

  const double distanceFactor = GetDistanceFactor(root),
               angleFactor    = GetAngleFactor(root);

  for(auto& child : root) {
    // Ignore the description node, which is just for users.
    if(child.Name() == "description")
      child.Ignore();
    else if(child.Name() == "markers")
      ParseMarkers(child, distanceFactor, angleFactor);
  }
}

/*-------------------------------- Interface ---------------------------------*/

const mathtool::Transformation&
ArucoMarkerMap::
GetMarkerTransformation(const size_t _id) const {
  try {
    return m_map.at(_id);
  }
  catch(const std::runtime_error& _e) {
    // Re-propagate error with better info.
    throw RunTimeException(WHERE) << "Requested marker ID " << _id
                                  << " is not in the map.";
  }
}


mathtool::Transformation
ArucoMarkerMap::
GetCameraTransformation(const size_t _id, const double _x,
    const double _y, const double _t) const {
  return GetMarkerTransformation(_id) * CreateTransformation(_x, _y, _t);
}

/*--------------------------------- Helpers ----------------------------------*/

double
ArucoMarkerMap::
GetDistanceFactor(XMLNode& _node) {
  std::string units = _node.Read("distanceUnits", true, "",
      "The units used to measure distance. Choices are {in, m}.");
  std::transform(units.begin(), units.end(), units.begin(), ::tolower);

  if(units == "m")
    return 1.;
  else if(units == "in")
    return 1. / 39.3701;
  else
    throw ParseException(_node.Where()) << "Unrecognized distance unit "
                                        << units << ".";
  return 0;
}


double
ArucoMarkerMap::
GetAngleFactor(XMLNode& _node) {
  std::string units = _node.Read("angleUnits", true, "",
      "The units used to measure angles. Choices are {deg, rad}.");
  std::transform(units.begin(), units.end(), units.begin(), ::tolower);

  if(units == "rad")
    return 1.;
  else if(units == "deg")
    return PI / 180.;
  else
    throw ParseException(_node.Where()) << "Unrecognized angle unit " << units
                                        << ".";
  return 0;
}


mathtool::Transformation
ArucoMarkerMap::
CreateTransformation(const double _x, const double _y, const double _t) const {
  return mathtool::Transformation(mathtool::Vector3d(_x, _y, 0),
                                 {mathtool::EulerAngle(_t, 0, 0)});
}


void
ArucoMarkerMap::
ParseMarkers(XMLNode& _node, const double _distanceFactor,
    const double _angleFactor) {
  // Create an input stream from the marker data.
  std::istringstream data(_node.GetText());

  // Parse each line.
  size_t id;
  double x, y, t;
  while(data >> id >> x >> y >> t) {
    m_map[id] = CreateTransformation(x * _distanceFactor,
                                     y * _distanceFactor,
                                     t * _angleFactor);
  }
}

/*----------------------------------------------------------------------------*/
