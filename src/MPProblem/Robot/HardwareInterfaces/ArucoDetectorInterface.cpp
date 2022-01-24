#include "ArucoDetectorInterface.h"

#include "ArucoMarkerMap.h"
#include "Commands.h"
#include "Simulator/Simulation.h"
#include "Utilities/XMLNode.h"

#include "detector_server/packet.h"
#include "nonstd/numerics.h"
#include "nonstd/tcp_socket.h"

#include <iostream>

/// The measured time that we need to send a message to the aruco detector.
static constexpr double arucoDetectorCommunicationTime = .3;


////////////////////////////////////////////////////////////////////////////////
/// An observation of the camera's position in a marker's local frame.
////////////////////////////////////////////////////////////////////////////////
struct ArucoObservation {

  size_t id{0};

  double x{0}, y{0}, t{0};

  ArucoObservation() = default;

  ArucoObservation(const packet& _p)
    : id(_p.id),
      x(_p.x / packet::s_factor),
      y(_p.y / packet::s_factor),
      t(_p.t / packet::s_factor)
  { }

};

/*------------------------------- Construction -------------------------------*/

ArucoDetectorInterface::
ArucoDetectorInterface(XMLNode& _node, const std::string& _ip,
    const unsigned short _port)
    : SensorInterface("ArucoDetector", _ip, _port, arucoDetectorCommunicationTime)
{
  // Parse the marker data to build a map.
  const std::string filename = _node.GetPath() + _node.Read("mapfile", true, "",
      "The marker map file.");
  m_markerMap = std::unique_ptr<ArucoMarkerMap>(new ArucoMarkerMap(filename));

  // Connect to the netbook's socket for aruco data.
  m_socket = std::unique_ptr<nonstd::tcp_socket>(
      new nonstd::tcp_socket(m_ip, std::to_string(m_port)));
  m_ready = true;
}


ArucoDetectorInterface::
~ArucoDetectorInterface() = default;

/*----------------------------- Sensor Interface -----------------------------*/

SensorInterface::SensorType
ArucoDetectorInterface::
GetType() const noexcept {
  return SensorInterface::SensorType::Transformation;
}


void
ArucoDetectorInterface::
SendCommand(const SensorCommand& _c) {
  std::lock_guard<std::mutex> guard(m_lock);
  m_ready = false;

  // Clear the previous measurement.
  m_observations.clear();

  // Tell the detector to take a measurement.
  bool go = true;
  *m_socket << go;

  // Get the number of markers observed.
  char c;
  *m_socket >> c;
  size_t count = c;

  // Receive a packet for each marker and save the observation.
  for(size_t i = 0; i < count; ++i) {
    packet p;
    *m_socket >> p;

    m_observations.emplace_back(p);
  }

  m_timestamp = Simulation::GetTimestamp();
  m_ready = true;
}


std::vector<mathtool::Transformation>
ArucoDetectorInterface::
GetLastTransformations() {
  std::lock_guard<std::mutex> guard(m_lock);

  std::vector<mathtool::Transformation> output;
  output.reserve(m_observations.size());

  for(const auto& o : m_observations)
    output.emplace_back(m_markerMap->GetCameraTransformation(o.id, o.x, o.y, o.t));

  return output;
}


std::vector<double>
ArucoDetectorInterface::
GetUncertainty() {
  /// @todo Determine our measurement uncertainty. This should be a vector of
  ///       three elements which describe the error in {x, y, t}.
  return {};
}

/*----------------------------------------------------------------------------*/
