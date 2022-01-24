#ifndef PMPL_CLOCK_CLASS_H_
#define PMPL_CLOCK_CLASS_H_

#include <iostream>
#include <string>


////////////////////////////////////////////////////////////////////////////////
/// This class is used to measure the processor time used between StartClock and
/// StopClock calls. This includes both user and system time.
///
/// @note Instances of this class are not thread-safe and should only be used by
///       a single thread. It is safe to run separate instances in different
///       threads.
///
/// @ingroup MetricUtils
////////////////////////////////////////////////////////////////////////////////
class ClockClass {

  public:

    ///@name Construction
    ///@{

    ClockClass();

    ///@}
    ///@name Interface
    ///@{

    /// Set the clock name.
    /// @param _name The name to use.
    void SetName(const std::string& _name);

    /// Reset the clock.
    void ClearClock();

    /// Start the clock and the name is identity of this clock.
    /// @throw If the clock is already started.
    void StartClock();

    /// Stop the clock and calculate the total running time.
    /// @throw If the clock is already stopped.
    void StopClock();

    /// Call StopClock and PrintClock.
    void StopPrintClock(std::ostream& _os);

    /// Print clock name and time in seconds to an outstream.
    /// @param _os The outstream to print to.
    void PrintClock(std::ostream& _os);

    /// Get the recorded time in seconds.
    double GetSeconds() const;

    /// Get the recorded time in microseconds.
    double GetUSeconds() const;

    ///@}

  private:

    ///@name Internal State
    ///@{

    struct timeval m_startTimeval; ///< The total usage at last start.
    struct timeval m_usedTimeval;  ///< The summed usage measured by this clock.

    bool m_running{false};         ///< Is the clock currently running?

    std::string m_clockName;       ///< The clock label.

    ///@}

};

#endif
