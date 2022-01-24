#ifndef PMPL_SENSOR_INTERFACE_H
#define PMPL_SENSOR_INTERFACE_H

#include "HardwareInterface.h"

#include <atomic>

class SensorCommand;
namespace mathtool {
  class Transformation;
}


////////////////////////////////////////////////////////////////////////////////
/// An abstract interface for a hardware sensor that measures the robot's world
/// state (either base transformation and/or joint angles).
////////////////////////////////////////////////////////////////////////////////
class SensorInterface : public HardwareInterface {

  public:

    ///@name Local Types
    ///@{

    using HardwareInterface::HardwareType;

    /// The supported sensor types.
    enum SensorType {Transformation, JointAngle};

    ///@}
    ///@name Construction
    ///@{

    using HardwareInterface::HardwareInterface;

    virtual ~SensorInterface();

    ///@}
    ///@name Hardware Properties
    ///@{

    virtual HardwareType GetHardwareType() const noexcept override;

    ///@}
    ///@name Sensor Interface
    ///@{

    /// Indicates the type of sensor.
    /// @return The sensor type.
    virtual SensorType GetType() const noexcept = 0;

    /// Instruct the sensor to take a measurement.
    virtual void SendCommand(const SensorCommand& _c) = 0;

    /// Check if the sensor has completed its measurement.
    /// @return True if measurement ready to read
    bool IsReady() const noexcept;

    /// Get the timestep when the last measurement was completed.
    size_t GetLastTimestamp() const noexcept;

    /// Get the last transformation measurements.
    virtual std::vector<mathtool::Transformation> GetLastTransformations();

    /// Get the last joint angle measurements.
    virtual std::vector<std::vector<double>> GetLastJointAngles();

    /// Get the measurement uncertainty.
    /// @todo This should maybe be a covariance matrix instead of an uncertainty
    ///       in each measured dimension.
    virtual std::vector<double> GetUncertainty() = 0;

    /// Get the measurement transformation matrix H such that a configuration c
    /// produces a measurement y = Hc.
    /// @todo We need a dynamically sizable matrix for this, either dlib or
    ///       eigen will probably be the choice.

    ///@}

  protected:

    ///@name Internal State
    ///@{

    std::atomic<bool> m_ready{false};   ///< Is the measurement is ready to read?
    std::atomic<size_t> m_timestamp{0}; ///< Timestep of last completed measurement.

    ///@}

};

#endif
