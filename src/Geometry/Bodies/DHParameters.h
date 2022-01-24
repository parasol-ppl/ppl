#ifndef DH_PARAMETERS_H_
#define DH_PARAMETERS_H_

#include <iostream>
#include <fstream>
using namespace std;

#include <Transformation.h>
using namespace mathtool;

////////////////////////////////////////////////////////////////////////////////
/// Denavit-Hartenberg Parameters for describing joint connections.
///
/// We have changed from 'alternative' to 'standard' DH parameters on r5929.
/// This should only affect spherical joints, of which we had no working examples.
/// In the standard notation, a DH transform looks like two simpler transforms:
/// One:
/// - m_d is the offset along the parent z to the common normal (new x)
/// - m_theta is the rotation about the parent z
/// Two:
/// - m_a is the offset along the x axis after transform One
/// - m_alpha is the rotation about the x axis after transform One
/// Check wikipedia's article for further details.
///
/// @ingroup Geometry
////////////////////////////////////////////////////////////////////////////////
class DHParameters {

  public:

    ///@name Construction
    ///@{

    /// Construct a set of DH params from explicit values.
    /// @param _alpha Alpha
    /// @param _a A
    /// @param _d D
    /// @param _theta Theta
    DHParameters(double _alpha = 0.0, double _a = 0.0,
        double _d = 0.0, double _theta = 0.0);

    ///@}
    ///@name Conversion
    ///@{

    /// Convert the DH parameter representation into a standard transformation.
    Transformation GetTransformation() const;

    ///@}
    ///@name I/O
    ///@{

    /// Read a set of DH params from an instream.
    /// @param _is Input stream
    /// @param _d DHParameters
    friend istream& operator>>(istream& _is, DHParameters& _d);

    /// Write a set of DH params to an outstream.
    /// @param _os Output stream
    /// @param _d DHParameters
    friend ostream& operator<<(ostream& _os, const DHParameters& _d);

    ///@}
    ///@name Internal State
    ///@{

    double m_alpha;   ///< Angle between two x axis
    double m_a;       ///< Distance between two z axis
    double m_d;       ///< Algebraic distance along z axis
    double m_theta;   ///< Angle between two z axis

    ///@}

};

#endif
