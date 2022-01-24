#include "PIDFeedback.h"

#include <algorithm>
#include <limits>

#include "MPProblem/Robot/Robot.h"
#include "Utilities/XMLNode.h"


/*------------------------------- Construction -------------------------------*/

PIDFeedback::
PIDFeedback(Robot* const _r, const double _p, const double _i, const double _d)
  : ControllerMethod(_r), m_p(_p), m_i(_i), m_d(_d) { }


PIDFeedback::
PIDFeedback(Robot* const _r, XMLNode& _node) : ControllerMethod(_r, _node) {
  m_p = _node.Read("proportional", true, 0.,
      std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(),
      "The proportional error gain");

  m_i = _node.Read("integral", true, 0.,
      std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(),
      "The integral error gain");

  m_d = _node.Read("derivative", true, 0.,
      std::numeric_limits<double>::lowest(), std::numeric_limits<double>::max(),
      "The derivative error gain");
}


PIDFeedback::
PIDFeedback(Robot* const _r, const PIDFeedback& _c)
  : ControllerMethod(_r, _c),
    m_p(_c.m_p),
    m_i(_c.m_i),
    m_d(_c.m_d),
    m_previousError(_r),
    m_integral(_r),
    m_target(_r)
{
  m_previousError.SetData(_c.m_previousError.GetData());
  m_integral.SetData(_c.m_integral.GetData());
  m_target.SetData(_c.m_target.GetData());
}


std::unique_ptr<ControllerMethod>
PIDFeedback::
Clone(Robot* const _r) const {
  return std::unique_ptr<PIDFeedback>(new PIDFeedback(_r, *this));
}


PIDFeedback::
~PIDFeedback() = default;

/*-------------------------------- Interface ---------------------------------*/

Control
PIDFeedback::
operator()(const Cfg& _current, const Cfg& _target, const double _dt) {
  // Re-initialize whenever we change targets.
  if(_target != m_target)
    Initialize(_target);

  return ControllerMethod::operator()(_current, _target, _dt);
}

/*--------------------------------- Helpers ----------------------------------*/

std::vector<double>
PIDFeedback::
ComputeDesiredForce(const Cfg& _current, const Cfg&, const double _dt) {
  /// @TODO This implementation is pretty gross. It should be cleaned up after
  ///       we develop a nice abstraction for C-Space directions. Until then we
  ///       need to work directly with the data vectors to avoid problems with
  ///       Cfg normalization.
  using data_vector = std::vector<double>;

  // Define some arithmetic functors for data vectors.

  // Mimic operator+=(data_vector&, const data_vector&).
  auto plus = [](const data_vector& _v1, const data_vector& _v2) {
    data_vector out;
    for(auto iter1 = _v1.begin(), iter2 = _v2.begin(); iter1 != _v1.end();
        ++iter1, ++iter2)
      out.push_back(*iter1 + *iter2);
    return out;
  };

  // Mimic operator*(data_vector&, double).
  auto times = [](const data_vector& _v, const double _d) {
    auto out = _v;
    std::for_each(out.begin(), out.end(), [_d](double& _val) {_val *= _d;});
    return out;
  };

  // Compute the error terms.
  const Cfg error = _current.FindDirectionTo(m_target);

  // Compute m_integral += error * _dt.
  auto data = plus(m_integral.GetData(), times(error.GetData(), _dt));
  m_integral.SetData(data);

  // Compute derivative = (error - m_previousError) / _dt.
  Cfg derivative = m_previousError.FindDirectionTo(error);
  data = times(derivative.GetData(), 1. / _dt);
  derivative.SetData(data);

  // Compute force = error * m_p +
  //                 m_integral * m_i +
  //                 derivative * m_d;
  auto force = plus(
                 times(error.GetData(), m_p),
                 plus(
                   times(m_integral.GetData(), m_i),
                   times(derivative.GetData(), m_d)
                 )
               );

  m_previousError = error;

  // Put the force into the robot's local frame.
  return _current.WorldDirToLocal(force);
}

/*--------------------------------- Helpers ----------------------------------*/

void
PIDFeedback::
Initialize(const Cfg& _target) {
  m_target        = _target;
  m_previousError = Cfg(m_robot);
  m_integral      = Cfg(m_robot);
}

/*----------------------------------------------------------------------------*/
