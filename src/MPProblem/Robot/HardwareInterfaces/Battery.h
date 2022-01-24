#ifndef BATTERY_H_
#define BATTERY_H__

#include <iostream>


////////////////////////////////////////////////////////////////////////////////
/// Models a simulated battery with a finite amount of charge.
///
/// @TODO Unify with other hardware code. We want a general 'battery' base class
///       which could represent either a real or simulated battery. This should
///       be derived to describe either option.
////////////////////////////////////////////////////////////////////////////////
class Battery {

  public:

    ///@name Construction
    ///@{

    Battery();

    ///@}
    ///@name Battery Interface
    ///@{
    /// @TODO Document these functions.

    /// Set max and current values of the battery
    /// @param _max Max charge battery can hold
    /// @param _cur Current charge
    void SetValues(double _max, double _cur);

    /// Set current level to max
    void SetToMax();

    /// Print current and max battery levels
    void Print();

    /// Get current battery level
    double GetCurLevel();

    /// Get max charge battery can hold
    double GetMaxLevel();

    /// Increase charge of battery, stop if over max
    /// @param _increaseRate Amount to increase the current charge by
    void Charge(double _increaseRate);

    /// Reduce value of charge by sum of base and moving rates
    /// @param _depletionRateBase Base depletion rate of charge
    /// @param _depletionRateMoving Depletion rate of charge while moving
    void UpdateValue(double _depletionRateBase, double _depletionRateMoving);

    /// Reduce value of charge by base depletion rate
    /// @param _depletionRateBase Base depletion rate of charge
    void UpdateValue(double _depletionRateBase);

    ///@}

  protected:

    ///@name Internal State
    ///@{

    double m_maxLevel;  ///< Maximum charge the battery can hold.
    double m_curLevel;  ///< Current charge in the battery.

    ///@}

};

#endif
