#ifndef PMPL_STATE_ESTIMATOR_H_
#define PMPL_STATE_ESTIMATOR_H_

#include <vector>

#include "ConfigurationSpace/Cfg.h"
#include "MPProblem/Robot/Control.h"
#include "Transformation.h"

class Robot;
class SensorInterface;
class XMLNode;


////////////////////////////////////////////////////////////////////////////////
/// Base abstraction for a state estimator object, which combines the controls
/// applied to a hardware robot with sensor data to estimate the robot's true
/// state.
///
/// We assume that all process noise/uncertainty is iid and Gaussian-shaped.
////////////////////////////////////////////////////////////////////////////////
class StateEstimator {

  public:

    ///@name Construction
    ///@{

    StateEstimator(Robot* const _robot);
    
    /// Create a dynamically-allocated state estimator from an XML node.
    /// @param _robot The robot which this state estimator will work for.
    /// @param _node The XML node to parse.
    /// @return A state estimator of the type specified by _node.
    static std::unique_ptr<StateEstimator> Factory(Robot* const _robot, XMLNode& _node);

    virtual ~StateEstimator();

    ///@}
    ///@name Interface
    ///@{

    /// Set the current state and uncertainty.
    /// @param _cfg The state to set.
    /// @param _uncertainty The uncertainty in _cfg.
    void SetState(const Cfg& _cfg, const std::vector<double>& _uncertainty);

    /// Apply a set of controls to the current estimated state.
    /// @note The Actuator class should define the force/velocity uncertainty,
    ///       if any.
    /// @param _controls The set of controls to apply.
    /// @param _dt       The length of time to apply the controls.
    void ApplyControls(const ControlSet& _controls, const double _dt);

    /// Apply the most recent observations from a sensor to the current
    /// estimated state.
    /// @param _sensor The sensor to take observations from.
    virtual void ApplyObservations(SensorInterface* const _sensor) = 0;

    /// Get the current state estimate.
    const Cfg& GetEstimatedState() const;

    /// Get the uncertainty in the estimated state.
    const std::vector<double>& GetUncertainty() const;

    /// Get the covariance matrix for the state estimate.
    /// @todo Figure out how to represent this.

    ///@}

  protected:

    ///@name Internal State
    ///@{

    Robot* const m_robot;              ///< The owning robot.

    Cfg m_estimatedState;              ///< The current estimated state.
    std::vector<double> m_uncertainty; ///< Uncertainty in the current estimate.

    bool m_debug{false};               ///< The debug flag.

    ///@}

};

#endif
