#include "MatlabMicroSimulator.h"

#include "ConfigurationSpace/Cfg.h"
#include "MPProblem/Robot/Robot.h"
#include "Simulator/BulletEngine.h"
#include "Simulator/BulletModel.h"
#include "Utilities/PMPLExceptions.h"

#include "nonstd/timer.h"


// Ignore warnings induced by matlab's poor coding practices.
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "MatlabEngine.hpp"
#include "MatlabDataArray.hpp"
#pragma GCC diagnostic pop


/*
// Example for converting standard strings to utf16 because matlab frequently
// requires it. For API reference see here:
// https://en.cppreference.com/w/cpp/locale/codecvt_utf8_utf16
// Do not try to use matlab's convertUtf... function - it does not link.

const std::string std_string = "Some matlab statement";
const std::u16string converted = std::wstring_convert<
    std::codecvt_utf8_utf16<char16_t>, char16_t>{}.from_bytes(std_string);

// You can also declare a u16string directly like this:
const std::u16string example = u"Some matlab statement";
*/


/*------------------------------ Construction --------------------------------*/

MatlabMicroSimulator::
MatlabMicroSimulator(Robot* const _robot) :
    m_robot(_robot),
    m_matlabClock(new nonstd::timer)
{
  // We need to load several files, which must be either in the executable
  // directory or in the search path.
  const std::vector<std::u16string> options{
    u"-nojvm",             // Do not load the jvm
    u"-nosplash",          // Do not show the splash screen on startup
    u"-singleCompThread",  // Use a single thread for the matlab process
    u"arclength.m",
    u"ppdiff.m",
    u"ppint.m",
    u"splinefit.m",
    u"mtimesx.m"
    u"mtimesx_sparse.m"
    u"FlexibleNeedle.m",
    u"HingedFlexibleNeedle.m",
  };

  // Create a matlab engine handle with the provided options.
  // For documentation see here:
  // https://www.mathworks.com/help/matlab/apiref/matlab.engine.matlabengine.html
  m_engine = matlab::engine::startMATLAB(options);

  // Instantiate the hinged needle model.
  //m_engine->eval(u"hingedNeedle = HingedNeedle");
  m_engine->eval(u"hingedNeedle = HingedFlexibleNeedle()");
}


MatlabMicroSimulator::
~MatlabMicroSimulator() = default;

/*-------------------------------- Interface ---------------------------------*/

Cfg
MatlabMicroSimulator::
Test(const Cfg& _start, const Control& _control) const {
  // Sanity check to make sure we are testing on the right robot.
  if(_start.GetRobot() != m_robot)
    throw RunTimeException(WHERE) << "Can't test dynamics model on a "
                                  << "configuration for a different robot.";

  m_matlabClock->start();

  // Set up the internal model at _start.
  m_engine->setVariable(u"q", CfgToState(_start));

  std::cout << "Initial state:" << std::endl;
  m_engine->eval(u"q");

  // Convert _control to the right format.
  matlab::data::ArrayFactory factory;
  const matlab::data::TypedArray<double> control = factory.createArray<double>(
      {3, 1},
      {
      _control.signal[0],
      _control.signal[1],
      _control.signal[2],
      }
  );

  // Execute the control.
  m_engine->setVariable(u"control", control);
  //m_engine->eval(u"delta_q = hingedNeedle.M(q, control)");
  //m_engine->eval(u"q = q + delta_q");
  m_engine->eval(u"q = hingedNeedle.M(q, control)");

  // Return the resulting configuration.
  const Cfg output = StateToCfg(m_engine->getVariable(u"q"));
  m_matlabClock->stop();

  std::cout << "Matlab time: " << m_matlabClock->elapsed()
            << std::endl;

  return output;
}


ControlSpace
MatlabMicroSimulator::
GetControlSpace(const Cfg& _state) const {
  m_engine->setVariable(u"qcs", CfgToState(_state));
  m_engine->eval(u"c = hingedNeedle.C(qcs)");

  const matlab::data::TypedArray<double> c = m_engine->getVariable(u"c");

  // Assume a constant insertion rate for now.
  const double insertion = std::min<double>(.005, c[2]);

  ControlSpace controlSpace(3);

  controlSpace.SetRange(0, 0, c[0]);
  controlSpace.SetRange(1, 0, c[1]);
  controlSpace.SetRange(2, insertion, insertion);

  return controlSpace;
}


void
MatlabMicroSimulator::
SetInsertionCfg(const Cfg& _insertionCfg) {
  m_insertionCfg = _insertionCfg;
}


double
MatlabMicroSimulator::
GetMatlabTime() const {
  return m_matlabClock->elapsed();
}

/*--------------------------------- Helpers ----------------------------------*/

Cfg
MatlabMicroSimulator::
StateToCfg(const matlab::data::TypedArray<double>& _state) const {
  m_engine->eval(u"[x,z,t] = hingedNeedle.getHingeInformation(q)");
  const matlab::data::TypedArray<double> x = m_engine->getVariable(u"x"),
                                         z = m_engine->getVariable(u"z"),
                                         t = m_engine->getVariable(u"t");
  m_engine->eval(u"tipMoment = hingedNeedle.getTipLoading(q)");

  Cfg output(m_robot);
  output[0] = m_insertionCfg[0] + z[0] * s_scaling; // Z-coordinate of the hinge point.
  output[1] = m_insertionCfg[1] + x[0] * s_scaling; // X-coordinate of the hinge point.
  output[2] = t[0] / PI;
  output[3] = _state[0] / PI; // Joint angle.
  output.SetStat("insertion", _state[1]);
  output.SetStat("c1-4", _state[ 6]);
  output.SetStat("c1-3", _state[ 7]);
  output.SetStat("c1-2", _state[ 8]);
  output.SetStat("c1-1", _state[ 9]);
  output.SetStat("c1-0", _state[10]);
  output.SetStat("c2-4", _state[11]);
  output.SetStat("c2-3", _state[12]);
  output.SetStat("c2-2", _state[13]);
  output.SetStat("c2-1", _state[14]);
  output.SetStat("c2-0", _state[15]);
  output.SetStat("c3-4", _state[16]);
  output.SetStat("c3-3", _state[17]);
  output.SetStat("c3-2", _state[18]);
  output.SetStat("c3-1", _state[19]);
  output.SetStat("c3-0", _state[20]);

  return output;
}


matlab::data::TypedArray<double>
MatlabMicroSimulator::
CfgToState(const Cfg& _cfg) const {
  // Extract needed state data from _cfg.
  const double jointAngle = _cfg[3] * PI,
               insertion  = _cfg.GetStat("insertion");

  // The matlab model uses this format:
  // - Joint angle 1
  // - Joint angle 2
  // - ...
  // - Joint angle n
  // - Insertion depth of the needle, up to the hinge point. Does not include
  //   anything after the first hinge.
  // - Zmax is for vision processing. The Needle model works in the XZ plane
  //   where Z is the initial insertion direction.
  // - First 'break' for piecewise polynomial to describe needle shape. This is
  //   an arc length along the needle from proximal to distal end.
  // - Second break
  // - ...
  // - Nth break. The needle shape will be defined by N - 1 piecewise
  //   polynomials of order defined by the matlab model (see the file for this).
  //   I believe we will always evenly space the breaks if there are more than
  //   two.
  // - Coefficients for the piecewise polynomial terms, from highest to lowest
  //   order. If there are more than two breaks, then list the coefficients for
  //   each pp in order (break 0 to 1, 1 to 2, ... , n-1 to n).
  matlab::data::ArrayFactory factory;
  return factory.createArray<double>(
      {21, 1},
      {
        jointAngle, // Hinge joint angle.
        insertion,  // Insertion depth of hinge point.
        0.,         // First pp break at 0.
        insertion / 3.,
        insertion * 2. / 3.,
        insertion,  // Last pp break at full insertion length.
        _cfg.GetStat("c1-4"),
        _cfg.GetStat("c1-3"),
        _cfg.GetStat("c1-2"),
        _cfg.GetStat("c1-1"),
        _cfg.GetStat("c1-0"),
        _cfg.GetStat("c2-4"),
        _cfg.GetStat("c2-3"),
        _cfg.GetStat("c2-2"),
        _cfg.GetStat("c2-1"),
        _cfg.GetStat("c2-0"),
        _cfg.GetStat("c3-4"),
        _cfg.GetStat("c3-3"),
        _cfg.GetStat("c3-2"),
        _cfg.GetStat("c3-1"),
        _cfg.GetStat("c3-0"),
      }
  );
}

/*----------------------------------------------------------------------------*/
