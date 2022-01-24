#include "Geometry/Boundaries/BoundaryIntersection.h"

#include "Geometry/Boundaries/Boundary.h"
#include "Utilities/PMPLExceptions.h"


std::vector<double>
RejectionSampleIntersection(const Boundary* const _b1, const Boundary* const _b2,
    const size_t _maxAttempts, size_t* const _attempts) noexcept {
  // For now, require that the boundaries have the same dimension and space
  // type.
  if(_b1->Type() != _b2->Type())
    throw NotImplementedException(WHERE) << "Currently only boundaries in "
                                         << "matching spaces are supported.";
  if(_b1->GetDimension() != _b2->GetDimension())
    throw NotImplementedException(WHERE) << "Currently only boundaries with "
                                         << "matching dimension are supported.";

  // Try sampling from one boundary and checking to see if the result is within
  // the other. Switch boundaries on each attempt to mitigate the effects of
  // ordering.
  const Boundary* first = _b1,
                * second = _b2;

  std::vector<double> sample;

  size_t attempts = 0;
  while(true) {
    ++attempts;
    sample = first->GetRandomPoint();
    if(second->InBoundary(sample))
      break;
    if(attempts >= _maxAttempts) {
      sample.clear();
      break;
    }
    std::swap(first, second);
  }

  if(_attempts)
    *_attempts = attempts;

  return sample;
}
