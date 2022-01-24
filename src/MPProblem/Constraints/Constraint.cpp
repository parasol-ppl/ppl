#include "Constraint.h"
#include "BoundaryConstraint.h"
#include "CSpaceConstraint.h"

#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"


/*--------------------------------- Construction -----------------------------*/

Constraint::
Constraint(Robot* const _r) : m_robot(_r) { }


Constraint::
~Constraint() = default;


std::unique_ptr<Constraint>
Constraint::
Factory(Robot* const _r, XMLNode& _node) {
  std::unique_ptr<Constraint> output;

  if(_node.Name() == "CSpaceConstraint")
    output = std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(_r, _node));
  else if(_node.Name() == "BoundaryConstraint")
    output = std::unique_ptr<BoundaryConstraint>(
        new BoundaryConstraint(_r, _node));
  else
    throw RunTimeException(_node.Where()) << "Unrecognized constraint type '"
                                          << _node.Name() << "'.";

  return output;
}

/*---------------------------- Constraint Interface --------------------------*/

void
Constraint::
SetRobot(Robot* const _r) {
  m_robot = _r;
}

/*----------------------------------------------------------------------------*/
