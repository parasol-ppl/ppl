#include "Utilities/ClockClass.h"

#include "Utilities/PMPLExceptions.h"

#include <sys/time.h>
#include <sys/resource.h>
#include <unistd.h>

#ifndef RUSAGE_THREAD
  #include <mach/mach_init.h>
  #include <mach/thread_act.h>
  #include <mach/mach_port.h>
#endif

/*---------------------------------- Helpers ---------------------------------*/

/// Get the total resource usage for this thread.
/// @return A timeval describing the current total resource usage for this
///         thread.
static
struct timeval
GetTotalUsage() {
  // Get the resource usage.
  struct rusage buf;
#ifdef RUSAGE_THREAD
  // Linux
  getrusage(RUSAGE_THREAD, &buf);
#else
  // Mac OSX
  thread_basic_info_data_t info = { 0 };
  mach_msg_type_number_t info_count = THREAD_BASIC_INFO_COUNT;
  thread_info(mach_thread_self(), THREAD_BASIC_INFO, (thread_info_t)&info, &info_count);

  buf.ru_utime.tv_sec  = info.user_time.seconds;
  buf.ru_utime.tv_usec = info.user_time.microseconds;
  buf.ru_stime.tv_sec  = info.system_time.seconds;
  buf.ru_stime.tv_usec = info.system_time.microseconds;
#endif

  // Add the user and system time to get the total usage.
  struct timeval& userTime   = buf.ru_utime,
                & systemTime = buf.ru_stime,
                  totalTime;
  timeradd(&userTime, &systemTime, &totalTime);

  return totalTime;
}

/*------------------------------- ClockClass ---------------------------------*/

ClockClass::
ClockClass() {
  ClearClock();
}


void
ClockClass::
SetName(const std::string& _name) {
  m_clockName = _name;
}


void
ClockClass::
ClearClock() {
  timerclear(&m_startTimeval);
  timerclear(&m_usedTimeval);
  m_running = false;
}


void
ClockClass::
StartClock() {
  if(m_running)
    throw RunTimeException(WHERE) << "Can't start a clock that's already running."
                                  << std::endl;
  m_running = true;

  // Set the current total usage as our start time val.
  m_startTimeval = GetTotalUsage();
}


void
ClockClass::
StopClock() {
  if(!m_running)
    throw RunTimeException(WHERE) << "Can't stop a clock that isn't running."
                                  << std::endl;
  m_running = false;

  // Subtract total usage from the start point to get the change in usage.
  struct timeval totalUsage = GetTotalUsage(),
                 deltaUsage;
  timersub(&totalUsage, &m_startTimeval, &deltaUsage);

  // Add delta usage to our measured usage.
  struct timeval buffer;
  timeradd(&deltaUsage, &m_usedTimeval, &buffer);
  m_usedTimeval = buffer;
}


void
ClockClass::
StopPrintClock(std::ostream& _os) {
  StopClock();
  PrintClock(_os);
}


void
ClockClass::
PrintClock(std::ostream& _os) {
  _os << m_clockName << ": " << GetSeconds() << " sec" << std::endl;
}


double
ClockClass::
GetSeconds() const {
  const auto& seconds  = m_usedTimeval.tv_sec;
  const auto& useconds = m_usedTimeval.tv_usec;
  return (double)useconds / 1e6 + seconds;
}


double
ClockClass::
GetUSeconds() const {
  const auto& seconds  = m_usedTimeval.tv_sec;
  const auto& useconds = m_usedTimeval.tv_usec;
  return (double)seconds * 1e6 + useconds;
}
/*----------------------------------------------------------------------------*/
