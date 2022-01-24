#ifndef CONTROL_SET_GENERATORS_H_
#define CONTROL_SET_GENERATORS_H_

#include "MPProblem/Robot/Control.h"

class Robot;
class XMLNode;


/// Generate a discrete set of controls for each actuator, including full
/// forward, full reverse, and coast for each DOF (individually). Does not
/// include simultaneous actuation of multiple DOFs.
/// @param[in] _r The robot to control.
/// @return A simple control set for the robot.
ControlSet* SimpleControlSetGenerator(Robot* const _r);


/// Parse an XML ControlSet node.
/// @param[in] _r The robot to control.
/// @param[in] _node The XML node to parse.
/// @return The set of controls described by the XML node.
ControlSet* XMLControlSetGenerator(Robot* const _r, XMLNode& _node);


/// Ensure all of the controls in a set are unique by removing any duplicates.
/// @note The order of the controls will not be preserved.
/// @param[in] _controls The control set to modify.
void RemoveDuplicateControls(ControlSet* const _controls);

#endif
