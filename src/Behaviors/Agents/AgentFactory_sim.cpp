#include "Agent.h"

#include "Coordinator.h"
#include "ChildAgent.h"
#include "PathFollowingAgent.h"
#include "PlanningAgent.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"

#include <algorithm>
#include <string>

std::unique_ptr<Agent>
Agent::
Factory(Robot* const _r, XMLNode& _node) {
  // Read the node and mark it as visited.
  std::string type = _node.Read("type", true, "", "The Agent class name.");
  std::transform(type.begin(), type.end(), type.begin(), ::tolower);

  std::unique_ptr<Agent> output;

  if(type == "pathfollowing")
    output = std::unique_ptr<PathFollowingAgent>(
        new PathFollowingAgent(_r, _node)
    );
  else if(type == "planning")
    output = std::unique_ptr<PlanningAgent>(
        new PlanningAgent(_r, _node)
    );
  else if(type == "coordinator")
    output = std::unique_ptr<Coordinator>(
        new Coordinator(_r, _node)
    );
  else if(type == "child")
    output = std::unique_ptr<ChildAgent>(
        new ChildAgent(_r, _node)
    );
  else
    throw ParseException(_node.Where(), "Unknown agent type '" + type + "'.");

  // Read the debug flag.
  output->m_debug = _node.Read("debug", false, false, "Show debug messages.");

  return output;
}
