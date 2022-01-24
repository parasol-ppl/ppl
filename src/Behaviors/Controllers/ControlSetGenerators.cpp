#include "ControlSetGenerators.h"

#include "Geometry/Bodies/MultiBody.h"
#include "MPProblem/Robot/Actuator.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#include <algorithm>
#include <sstream>


ControlSet*
SimpleControlSetGenerator(Robot* const _r) {
  ControlSet* controls = new ControlSet;

  // Create coast action.
  Control c = {nullptr, Control::Signal(_r->GetMultiBody()->DOF(), 0)};
  controls->push_back(c);

  // Create forward and reverse controls for each actuator.
  for(auto& actuatorPair : _r->GetActuators()) {
    Actuator* actuator = actuatorPair.second.get();
    auto mask = actuator->ControlMask();
    c.actuator = actuator;

    // Create a min and max action for each controllable DOF.
    for(size_t i = 0; i < mask.size(); ++i) {
      // Skip DOFs that this actuator can't control.
      if(!mask[i])
        continue;

      // Make a control for the forward signal.
      c.signal[i] = 1;
      controls->push_back(c);

      // Make a control for the reverse signal.
      c.signal[i] = -1;
      controls->push_back(c);

      c.signal[i] = 0;
    }
  }

  return controls;
}


/// Internal helper for generating control sets from an enumerated list in an
/// XML file.
/// @param[in] _r The robot to control.
/// @param[in] _node The XML node for this control set.
/// @return The enumerated set of controls.
ControlSet*
EnumeratedControlSetGenerator(Robot* const _r, XMLNode& _node) {
  const size_t dof = _r->GetMultiBody()->DOF();

  ControlSet* controls = new ControlSet;

  // Parse each control in the enumerated set.
  for(auto& child : _node) {
    // Read the actuator label.
    const std::string& actuatorLabel = child.Read("actuator", true, "",
        "The label of the actuator that this control affects");

    // Check for coast control.
    if(actuatorLabel == "coast") {
      controls->emplace_back(nullptr, Control::Signal(dof, 0));
      continue;
    }

    // Parse non-coast control.

    // Read the control signal.
    const std::string signalString = child.Read("signal", true, "",
        "The control signal");

    // Parse the control signal.
    Control::Signal signal;
    std::istringstream signalStream(signalString);
    double temp;
    while(signalStream >> temp)
      signal.push_back(temp);

    // Assert that we read the right number of signal values.
    if(signal.size() != dof)
      throw ParseException(child.Where(), "Read control signal with " +
          std::to_string(signal.size()) + " DOFs, but Robot has " +
          std::to_string(dof) + " DOFs.");

    controls->emplace_back(_r->GetActuator(actuatorLabel), signal);
  }

  RemoveDuplicateControls(controls);
  return controls;
}


ControlSet*
XMLControlSetGenerator(Robot* const _r, XMLNode& _node) {
  // Read the type of control set to generate and downcase it for easier parsing.
  std::string type = _node.Read("type", true, "", "The control set type");
  std::transform(type.begin(), type.end(), type.begin(), ::tolower);

  // Setup the appropriate type of control.
  if(type == "simple")
    return SimpleControlSetGenerator(_r);
  else if(type == "enumerated")
    return EnumeratedControlSetGenerator(_r, _node);
  else
    throw ParseException(_node.Where(), "Unknown control set type.");
}


void
RemoveDuplicateControls(ControlSet* const _controls) {
  std::sort(_controls->begin(), _controls->end());
  auto iter = std::unique(_controls->begin(), _controls->end());
  _controls->erase(iter, _controls->end());
}
