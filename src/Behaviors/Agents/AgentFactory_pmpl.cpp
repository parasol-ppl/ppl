#include "Agent.h"

#include "Utilities/XMLNode.h"

std::unique_ptr<Agent>
Agent::
Factory(Robot* const _r, XMLNode& _node) {
  // If we are not building the simulator, ignore the agent node and its
  // children.
  _node.Ignore();

  std::queue<XMLNode*> ignoredNodes;
  ignoredNodes.push(&_node);

  while(!ignoredNodes.empty()) {
    XMLNode* node = ignoredNodes.front();
    ignoredNodes.pop();

    node->Ignore();
    for(auto& child : *node)
      ignoredNodes.push(&child);
  }

  return {nullptr};
}
