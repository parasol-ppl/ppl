#ifndef PMPL_C_SPACE_CONSTRAINT_H_
#define PMPL_C_SPACE_CONSTRAINT_H_

#include <cstddef>
#include <iostream>
#include <map>

#include "BoundaryConstraint.h"

class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// A c-space boundary constraint, mainly provided as a convenience interface
/// for BoundaryConstraint.
////////////////////////////////////////////////////////////////////////////////
class CSpaceConstraint : public BoundaryConstraint {

  public:

    ///@name Construction
    ///@{

    /// Construct a constraint from a specific configuration.
    /// @param _r The robot to constrain.
    /// @param _c The single configuration for _r which satisfies this
    ///           constraint.
    explicit CSpaceConstraint(Robot* const _r, const Cfg& _c);

    /// Construct a constraint from an XML node.
    /// @param _r The robot to constrain.
    /// @param _node The node to parse.
    explicit CSpaceConstraint(Robot* const _r, XMLNode& _node);

    /// Construct a constraint from message strings.
    /// @param _r The robot to constrain.
    /// @param _pointString The string containing point constraint info.
    /// @param _bbxString The string containing bounding box constraint info.
    explicit CSpaceConstraint(Robot* const _r, std::string _pointString, std::string _bbxString);

    virtual ~CSpaceConstraint();

    virtual std::unique_ptr<Constraint> Clone() const override;

    ///@}

	private:
		///@name Helpers
		///@{

    /// Construct a constraint from a message string 
    /// (only one of _pointString or _bbxString can be non-empty).
    /// @param _r The robot to constrain.
    /// @param _pointString The string containing point constraint info.
    /// @param _bbxString The string containing bounding box constraint info.
		void ParseBoundaryString(Robot* const _r, std::string _pointString, std::string _bbxString);

		///@}

};

#endif
