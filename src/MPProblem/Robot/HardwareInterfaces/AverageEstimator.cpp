#include "AverageEstimator.h"

#include "MPProblem/Robot/HardwareInterfaces/SensorInterface.h"
#include "MPProblem/Robot/Robot.h"
#include "Simulator/BulletModel.h"


/*------------------------------- Construction -------------------------------*/

AverageEstimator::
AverageEstimator(Robot* const _robot) : StateEstimator(_robot) { }


AverageEstimator::
~AverageEstimator() = default;

/*------------------------- StateEstimator Interface -------------------------*/

void
AverageEstimator::
ApplyObservations(SensorInterface* const _sensor) {
  auto transformations = _sensor->GetLastTransformations();

  if(m_debug) {
    std::cout << "***\nSaw " << transformations.size() << " markers:";
    for(auto& t : transformations) {
      std::cout << "\n\t" << t.translation()
                << "\n\t" << t.rotation()
                << std::endl;
    }
  }

  // If no markers are seen, assume the current simulated state is correct.
  if(transformations.size() == 0) {
    m_estimatedState = m_robot->GetSimulationModel()->GetState();
    return;
  }

  // Average the estimated state and update the robot's simulated position.
  double averageX = 0, averageY = 0;
  for(auto& t : transformations) {
    averageX += t.translation()[0];
    averageY += t.translation()[1];
  }
  averageX /= transformations.size();
  averageY /= transformations.size();
  const double averageT = ComputeRotation(_sensor);

  Cfg updatedPos(m_robot);
  updatedPos.SetLinearPosition(Vector3d(averageX, averageY, 0));
  updatedPos.SetAngularPosition(Vector3d(0, 0, averageT));

  m_estimatedState = updatedPos;
}

/*---------------------------- Estimation Helpers ----------------------------*/

double
AverageEstimator::
ComputeRotation(SensorInterface* const _sensor) {
  auto transformations = _sensor->GetLastTransformations();
  double estimateX = 0, estimateY = 0;

  // Add each marker angle into the vector.
  for(auto& t : transformations) {
    EulerAngle e;
    convertFromMatrix(e, t.rotation().matrix());
    double theta = e.alpha();
    estimateX += std::cos(theta);
    estimateY += std::sin(theta);
  }
  estimateX /= transformations.size();
  estimateY /= transformations.size();

  // Get the estimated angle from the unit vector.
  return std::atan2(estimateY, estimateX);
}

/*----------------------------------------------------------------------------*/
