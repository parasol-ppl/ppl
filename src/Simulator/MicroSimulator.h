#ifndef PMPL_MICRO_SIMULATOR_H_
#define PMPL_MICRO_SIMULATOR_H_

#include "MPProblem/Robot/Control.h"

#include <memory>

class BulletEngine;
class BulletModel;
class Cfg;
class MicroSimulator;
class Robot;


////////////////////////////////////////////////////////////////////////////////
/// A micro simulator for a single robot. It tests the result of applying a
/// control to a robot in a given configuration.
///
/// @details This object outsources forward dynamics computations to an isolated
///          bullet physics engine.
////////////////////////////////////////////////////////////////////////////////
class MicroSimulator final {

  ///@name Internal State
  ///@{

  Robot* const m_robot;           ///< The robot's pmpl model.
  BulletEngine* const m_engine;   ///< The local physics engine.
  BulletModel* const m_model;     ///< The robot's bullet model.

  ///@}

  public:

    ///@name Construction
    ///@{

    /// Construct a self-simulator for a robot.
    /// @param _robot The robot to simulate.
    MicroSimulator(Robot* const _robot);

    ~MicroSimulator();

    ///@}
    ///@name Interface
    ///@{

    /// Test the result of applying a control to the robot from a designated
    /// starting configuration.
    /// @param _start The starting configuration.
    /// @param _control The control to apply.
    /// @param _dt The length of time to apply the control.
    /// @return The result of applying control _c to the robot for _dt seconds,
    ///         starting from _start.
    Cfg Test(const Cfg& _start, const Control& _control, const double _dt);

    /// Test the result of applying a set of controls to the robot from a
    /// designated starting configuration.
    /// @param _start The starting configuration.
    /// @param _controlSet The set of controls to apply.
    /// @param _dt The length of time to apply the control.
    /// @return The result of applying control _c to the robot for _dt seconds,
    ///         starting from _start.
    Cfg Test(const Cfg& _start, const ControlSet& _controlSet, const double _dt);

    ///@}

};

#endif
