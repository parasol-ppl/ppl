#include "CSpaceConstraint.h"

#include <limits>
#include <sstream>

#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/Boundaries/CSpaceBoundingBox.h"
#include "MPProblem/Robot/Robot.h"
#include "Utilities/IOUtils.h"
#include "Utilities/PMPLExceptions.h"
#include "Utilities/XMLNode.h"


/*------------------------------ Construction --------------------------------*/

CSpaceConstraint::
CSpaceConstraint(Robot* const _r, const Cfg& _cfg)
  : BoundaryConstraint(_r, nullptr)
{
  // Ensure that _cfg goes with m_robot.
  if(m_robot != _cfg.GetRobot())
    throw RunTimeException(WHERE) << "Cannot create point constraint with "
                                  << "configuration for a different robot.";

  // Create a boundary at _cfg.
  std::unique_ptr<CSpaceBoundingBox> bbx(new CSpaceBoundingBox(
      m_robot->GetMultiBody()->DOF() * (m_robot->IsNonholonomic() ? 2 : 1))
  );
  bbx->ShrinkToPoint(_cfg);

  m_boundary = std::move(bbx);
}


CSpaceConstraint::
CSpaceConstraint(Robot* const _r, XMLNode& _node)
  : BoundaryConstraint(_r, nullptr)
{
  /// @todo Verify that this works with constraints of lower dimension than the
  ///       robot's cspace (for partial constraint), or decide that we will not
  ///       support this and throw an error if requested.

  // The XML node will either describe a point or a bounding box.
  const std::string pointString = _node.Read("point", false, "", "The Cfg point"),
                    bbxString = _node.Read("bbx", false, "", "The Cfg box");

  if(pointString.empty() == bbxString.empty())
    throw ParseException(_node.Where()) << "A CSpaceConstraint should specify a "
                                        << "single configuration (point) or "
                                        << "bounding box (bbx), but not both.";

	ParseBoundaryString(_r,pointString,bbxString);

}

CSpaceConstraint::
CSpaceConstraint(Robot* const _r, std::string _pointString, std::string _bbxString) : BoundaryConstraint(_r,nullptr) {
	ParseBoundaryString(_r,_pointString,_bbxString);
}

CSpaceConstraint::
~CSpaceConstraint() = default;


std::unique_ptr<Constraint>
CSpaceConstraint::
Clone() const {
  return std::unique_ptr<CSpaceConstraint>(new CSpaceConstraint(*this));
}

/*------------------------------ Helpers --------------------------------*/

void 
CSpaceConstraint::
ParseBoundaryString(Robot* const _r, std::string _pointString, std::string _bbxString) {
  // Assert that we got exactly one of these.
  // Initialize the boundary.
  std::unique_ptr<CSpaceBoundingBox> bbx(new CSpaceBoundingBox(
      m_robot->GetMultiBody()->DOF() * (m_robot->IsNonholonomic() ? 2 : 1))
  );

  // Parse the boundary data.
  if(!_bbxString.empty()) {
    if(!_pointString.empty()) {
      throw RunTimeException(WHERE) << "Cannot parse both _pointString and _bbxString";
    }
    // This is a bounding box constraint.
    std::istringstream bbxStream(_bbxString);
    bbxStream >> *bbx;
  }
  else if(!_pointString.empty()) {
    // This is a point constraint.
    Cfg point(_r);
#ifdef VIZMO_MAP
    std::istringstream pointStream("0 " + _pointString);
#else
    std::istringstream pointStream(_pointString);
#endif
    point.Read(pointStream);

    bbx->ShrinkToPoint(point);
  }
  else {
    throw RunTimeException(WHERE) << "Cannot parse boundary string without _pointString or _bbxString";
  }

  m_boundary = std::move(bbx);
}
/*----------------------------------------------------------------------------*/
