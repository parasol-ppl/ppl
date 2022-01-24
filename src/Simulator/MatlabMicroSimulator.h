#ifndef PMPL_MATLAB_MICRO_SIMULATOR_H_
#define PMPL_MATLAB_MICRO_SIMULATOR_H_

#include "ConfigurationSpace/Cfg.h"
#include "MPProblem/Robot/Control.h"

#include <memory>
#include <vector>

class Robot;

namespace matlab {
  namespace engine {
    class MATLABEngine;
  }
  namespace data {
    template <typename> class TypedArray;
  }
}

namespace nonstd {
  class timer;
}


////////////////////////////////////////////////////////////////////////////////
/// A micro simulator for a steerable needle using a matlab model. It can likely
/// be generalized later if we wish, but for now this is a dedicated object for
/// the needle steering project.
////////////////////////////////////////////////////////////////////////////////
class MatlabMicroSimulator final {

  ///@name Internal State
  ///@{

  Robot* const m_robot;           ///< The robot's pmpl model.
  std::unique_ptr<matlab::engine::MATLABEngine> m_engine; ///< The matlab engine.

  Cfg m_insertionCfg;   ///< The initial insertion configuration.

  /// Timer for matlab calls. Cannot use our ClockClass here because that is
  /// rusage based, and matlab is running as a separate process.
  std::unique_ptr<nonstd::timer> m_matlabClock;

  static constexpr const bool s_debug{true}; ///< Show debug messages?

  /// The environment's units are inflated by this factor.
  static constexpr const double s_scaling = 1000;

  ///@}

  public:

    ///@name Construction
    ///@{

    /// Construct a self-simulator for a robot.
    /// @param _robot The robot to simulate.
    MatlabMicroSimulator(Robot* const _robot);

    ~MatlabMicroSimulator();

    ///@}
    ///@name Interface
    ///@{

    /// Test the result of applying a control to the robot from a designated
    /// starting configuration.
    /// @param _start The starting configuration.
    /// @param _control The control to apply.
    /// @return The result of applying control _c to the robot, starting from
    ///         _start.
    Cfg Test(const Cfg& _start, const Control& _control) const;

    /// Get the allowed controls from a given state.
    /// @param _state The current needle state.
    /// @return The space of allowed controls from _state.
    ControlSpace GetControlSpace(const Cfg& _state) const;

    /// Set the insertion configuration.
    void SetInsertionCfg(const Cfg& _insertionCfg);

    /// Get the wall-clock time spent waiting on matlab.
    double GetMatlabTime() const;

    ///@}

  private:

    ///@name Helpers
    ///@{

    Cfg StateToCfg(const matlab::data::TypedArray<double>& _state) const;

    matlab::data::TypedArray<double> CfgToState(const Cfg& _cfg) const;

    ///@}

};

#endif
