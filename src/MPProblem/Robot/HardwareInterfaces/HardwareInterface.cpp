#include "HardwareInterface.h"

#ifdef PMPL_USE_ARUCO
#include "ArucoDetectorInterface.h"
#endif
#ifdef PMPL_USE_ICREATE
#include "ICreateInterface.h"
#endif

#include "Utilities/XMLNode.h"
#include "Utilities/PMPLExceptions.h"
#include "nonstd/io.h"
#include "nonstd/numerics.h"

#include <algorithm>
#include <sstream>


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~ Hardware Interface ~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*------------------------------- Construction -------------------------------*/

HardwareInterface::
HardwareInterface(const std::string& _name, const std::string& _ip,
    const unsigned short _port, const double _communicationTime) : m_name(_name),
    m_ip(_ip), m_port(_port), m_communicationTime(_communicationTime) {}


std::unique_ptr<HardwareInterface>
HardwareInterface::
Factory(XMLNode& _node) {
  // Get the IP and port.
  const std::string ip = _node.Read("ip", true, "",
      "The IPv4 address for the robot hardware.");

  unsigned short port = _node.Read<unsigned short>("port", false, 0, 0,
      std::numeric_limits<unsigned short>::max(),
      "The on-board controller port.");

  // For some reason I can't get GCC to ignore an unused variable warning here,
  // assigning to self as a hacky work-around.
  port = port;

  // Get the hardware type and downcase.
  std::string type = _node.Read("hardware", true, "", "The type of hardware.");
  std::transform(type.begin(), type.end(), type.begin(), ::tolower);

  std::unique_ptr<HardwareInterface> output;

  // Match the type string to known interfaces. Provide detailed errors here to
  // make sure we don't waste time on silly configuration problems.
  if(type == "icreate") {
#ifdef PMPL_USE_ICREATE
    output = std::unique_ptr<ICreateInterface>(
        port == 0 ? new ICreateInterface(ip)
                  : new ICreateInterface(ip, port)
    );
#else
    throw ParseException(_node.Where()) << "Requested interface for 'icreate', "
                                        << "but support for this was not "
                                        << "compiled. Re-make with 'icreate=1' "
                                        << "to enable.";
#endif
  }
  else if(type == "aruco") {
#ifdef PMPL_USE_ARUCO
    output = std::unique_ptr<ArucoDetectorInterface>(
        port == 0 ? new ArucoDetectorInterface(_node, ip)
                  : new ArucoDetectorInterface(_node, ip, port)
    );
#else
    throw ParseException(_node.Where()) << "Requested interface for 'aruco', "
                                        << "but support for this was not "
                                        << "compiled. Re-make with 'aruco=1' "
                                        << "to enable.";
#endif
  }
  else
    throw ParseException(_node.Where()) << "Unrecognized hardware '"
                                        << type << "'.";

  return output;
}

/*------------------------ Hardware Robot Properties -------------------------*/

const std::string&
HardwareInterface::
GetName() const noexcept {
  return m_name;
}


void
HardwareInterface::
SetName(const std::string& _name) noexcept {
  m_name = _name;
}


const std::string&
HardwareInterface::
GetIP() const noexcept {
  return m_ip;
}


void
HardwareInterface::
SetIP(const std::string& _ip) noexcept {
  m_ip = _ip;
}


unsigned short
HardwareInterface::
GetPort() const noexcept {
  return m_port;
}


void
HardwareInterface::
SetPort(const unsigned short _port) noexcept {
  m_port = _port;
}


double
HardwareInterface::
GetCommunicationTime() const noexcept {
  return m_communicationTime;
}


void
HardwareInterface::
SetCommunicationTime(const double _t) noexcept {
  m_communicationTime = _t;
}

/*----------------------------------------------------------------------------*/
