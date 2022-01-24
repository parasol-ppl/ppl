#ifndef PMPL_ARUCO_MARKER_MAP_H_
#define PMPL_ARUCO_MARKER_MAP_H_

#include "Transformation.h"

#include <cstddef>
#include <string>
#include <unordered_map>

class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// A map of the position and orientation of several aruco markers.
////////////////////////////////////////////////////////////////////////////////
class ArucoMarkerMap final {

  ///@name Internal State
  ///@{

  /// A map from marker ID to the marker's global transformation.
  std::unordered_map<size_t, mathtool::Transformation> m_map;

  ///@}

  public:

    ///@name Construction
    ///@{

    /// Parse a marker map from an input XML file.
    /// @param _fileName File name of XML file for marker map
    ArucoMarkerMap(const std::string& _filename);

    ///@}
    ///@name Interface
    ///@{

    /// Get the global transformation of a given marker.
    /// @param _id The marker ID.
    /// @return The global transformation for this marker.
    const mathtool::Transformation& GetMarkerTransformation(const size_t _id)
        const;

    /// Get the global transformation of the camera relative to a marker.
    /// @param _id The marker ID.
    /// @param _x  The camera's X position in the marker's local frame.
    /// @param _y  The camera's Y position in the marker's local frame.
    /// @param _t  The camera's angle Theta in the marker's local frame.
    /// @return The camera's transformation in the global frame.
    mathtool::Transformation GetCameraTransformation(const size_t _id,
        const double _x, const double _y, const double _t) const;

    ///@}

  private:

    ///@name Helpers
    ///@{

    /// Get the conversion factor for the distance units.
    /// @param _node The XML node.
    /// @return The multiplier to convert map distances into meters.
    double GetDistanceFactor(XMLNode& _node);

    /// Get the conversion factor for the angle units.
    /// @param _node The XML node.
    /// @return The multiplier to convert map angles into radians.
    double GetAngleFactor(XMLNode& _node);

    /// Create a global transformation in the plane.
    /// @param _x  The X position.
    /// @param _y  The Y position.
    /// @param _t  The angle Theta in radians.
    /// @return The transformation.
    mathtool::Transformation CreateTransformation(const double _x,
        const double _y, const double _t) const;

    /// Parse the marker data child node to create the marker map.
    /// @param _node The child node.
    /// @param _distanceFactor The distance unit multiplier.
    /// @param _angleFactor The angle unit multiplier.
    void ParseMarkers(XMLNode& _node, const double _distanceFactor,
        const double _angleFactor);

    ///@}

};

#endif
