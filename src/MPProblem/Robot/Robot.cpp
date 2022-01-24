#include "Robot.h"

#include "Actuator.h"

#include "Behaviors/Agents/Agent.h"
#include "Behaviors/Controllers/ControllerMethod.h"
#include "ConfigurationSpace/Cfg.h"
#include "Geometry/Bodies/MultiBody.h"
#include "Geometry/Boundaries/Boundary.h"
#include "Geometry/Boundaries/CSpaceBoundingBox.h"
#include "HardwareInterfaces/Battery.h"
#include "HardwareInterfaces/RobotCommandQueue.h"
#include "HardwareInterfaces/StateEstimator.h"
#include "MPProblem/MPProblem.h"
#include "MPProblem/Environment/Environment.h"
#include "Simulator/Conversions.h"
#include "Simulator/BulletModel.h"
#include "Simulator/MatlabMicroSimulator.h"
#include "Simulator/MicroSimulator.h"
#include "Utilities/XMLNode.h"

#include "Behaviors/Controllers/ControlSetGenerators.h"
#include "Behaviors/Controllers/SimpleController.h"
#include "nonstd/io.h"

#include <algorithm>
#include <sstream>


/*------------------------------ Construction --------------------------------*/

Robot::
Robot(MPProblem* const _p, XMLNode& _node) : m_problem(_p) {
  // Get the unique robot label.
  m_label = _node.Read("label", true, "", "Unique robot label");

  // Get the (optional) holonomicness, assuming holononmic.
  m_nonholonomic = _node.Read("nonholonomic", false, false, "Is the robot "
      "nonholonomic?");

  // Get the (optional) car-likeness, assuming not car-like.
  m_carlike = _node.Read("carlike", false, false, "Is the robot car-like?");

  // Check if this robot is flagged as virtual.
  m_virtual = _node.Read("virtual", false, false, "Virtual robots are imaginary "
      "and will not be included in the simulation or CD checks.");

  // Check if the robot is a manipulator or not
  m_manipulator = _node.Read("manipulator", false, false, "Is the robot a manipulator?");

  // Check if the robot uses a default MP Strategy, it is used for setting different
  // MPStrategies in group decoupled planners. 
  m_defaultStrategyLabel = _node.Read("defaultStrategyLabel", false, "", "The robot individual strategy");

  // Get the (optional) capability type for the robot.
  std::string capability = _node.Read("capability", false, "", "The Robot capability type");
  std::transform(capability.begin(), capability.end(), capability.begin(), ::tolower);
  SetCapability(capability);

  // Get the multibody file name and make sure it exists.
  const std::string file = _node.Read("filename", false, "", "Robot file name"),
                    filename = _node.GetPath() + file;

  if(!file.empty() and !FileExists(filename))
    throw ParseException(_node.Where()) << "File '" << filename
                                        << "' does not exist.";

  if(file.empty()) {
    // If we don't get a filename, assume the robot is defined fully in this
    // Robot node, in the same way it would be in the external file.
    ReadXMLNode(_node);
  }
  else if(filename.find(".xml") != std::string::npos) {
    // If we got an XML file, use that parsing mechanism.
    ReadXMLFile(filename);
  }
  else {
    // Otherwise we got a multibody file, which cannot specify dynamics options
    // like actuators and controls. Assume some defaults for these.
    ReadMultibodyFile(filename);
    ReadXMLNode(_node);

    if(m_actuators.empty()) {
      // Set up a single, velocity-based actuator for all DOF. As this robot is
      // holonomic, we will only use the actuator in simulation.
      std::vector<double> reverse(m_multibody->DOF(), -1),
                          forward(m_multibody->DOF(),  1);
      m_actuators["default"] = std::unique_ptr<Actuator>(
          new Actuator(this, "default",
            IsNonholonomic() ? Actuator::DynamicsType::Force
                             : Actuator::DynamicsType::Velocity));
      m_actuators["default"]->SetLimits(reverse, forward);
      m_actuators["default"]->SetMaxForce(1);
    }

    // Use the simplest controller.
    if(m_controller.get() == nullptr) {
      m_controller = std::unique_ptr<SimpleController>(
          new SimpleController(this, 1)
      );
    }
  }

  // Parse hardware and agent child nodes.
  for(auto& child : _node) {
    if(child.Name() == "Hardware") {
      m_hardware = std::unique_ptr<RobotCommandQueue>(
          new RobotCommandQueue(child));
    }
    else if(child.Name() == "StateEstimator") {
      SetStateEstimator(StateEstimator::Factory(this, child));
    }
    else if(child.Name() == "Agent") {
      SetAgent(Agent::Factory(this, child));
    }
  }

  // Initialize the emulated battery.
  m_battery = std::unique_ptr<Battery>(new Battery());

  //Set color of the robot
  std::string color = _node.Read("color", false, "", "The color of the robot multibody");
  glutils::color c;
  //Try to use RGB notation
  if(color != ""){
    try{
      std::stringstream ss(color);
      ss >> c;
    }
    //Otherwise use color name
    catch(const nonstd::exception&) {
      c = StringToColor(color);
    }
    for(size_t i = 0; i < m_multibody->GetNumBodies(); i++){
      m_multibody->GetBody(i)->SetColor(c);
    }
  }
}


Robot::
Robot(MPProblem* const _p, std::unique_ptr<MultiBody>&& _mb,
    const std::string& _label)
  : m_problem(_p),
    m_multibody(std::move(_mb)),
    m_label(_label)
{
  InitializePlanningSpaces();
  m_multibody->Configure(std::vector<double>(m_multibody->DOF(), 0));

  m_battery = std::unique_ptr<Battery>(new Battery());
}


Robot::
Robot(MPProblem* const _p, const Robot& _r)
  : m_problem(_p),
    m_virtual(_r.m_virtual),
    m_nonholonomic(_r.m_nonholonomic),
    m_carlike(_r.m_carlike),
    m_maxLinearVelocity(_r.m_maxLinearVelocity),
    m_maxAngularVelocity(_r.m_maxAngularVelocity),
    m_capability(_r.m_capability),
    m_manipulator(_r.m_manipulator)
{
  // Copy the robot label. If the robot was copied to the same problem, append
  // _copy to the end of the label to make sure it is unique.
  const bool sameProblem = _p == _r.GetMPProblem();
  m_label = _r.m_label + (sameProblem ? "_copy" : "");

  // Copy multibody.
  m_multibody = std::unique_ptr<MultiBody>(new MultiBody(*_r.m_multibody));

  // Copy actuators.
  for(const auto& labelPointer : _r.m_actuators) {
    auto label    = labelPointer.first;
    auto actuator = labelPointer.second.get();
    m_actuators[label] = std::unique_ptr<Actuator>(new Actuator(this, *actuator));
  }

  // Copy the planning spaces.
  m_cspace = std::unique_ptr<CSpaceBoundingBox>(
      new CSpaceBoundingBox(*_r.m_cspace)
  );
  if(_r.m_vspace)
    m_vspace = std::unique_ptr<CSpaceBoundingBox>(
        new CSpaceBoundingBox(*_r.m_vspace)
    );

  // Copy controller.
  if(_r.m_controller)
    SetController(_r.m_controller->Clone(this));

  // We can't copy the bullet model directly - it must be recreated by adding
  // this robot to a simulation because some of the related data is stored in
  // the Simulation object.

  // We will not copy the agent because each one must be created for a specific
  // robot object. We will only copy the agent label and require the Simulation
  // to handle the rest.

  // We will not copy the hardware interfaces because there should only be one
  // such object for a given piece of hardware.

  // The battery is emulated and may be copied safely.
  if(_r.m_battery.get())
    m_battery = std::unique_ptr<Battery>(new Battery(*_r.m_battery));
}


Robot::
~Robot() = default;


Robot::EndEffector::
EndEffector(XMLNode& _node, MultiBody* const _mb) {
  const size_t bodyIndex = _node.Read("bodyIndex", true,
      (size_t)0, (size_t)0, std::numeric_limits<size_t>::max(),
      "The body's index in the robot manipulator's multibody.");
  effectorBody = _mb->GetBody(bodyIndex);

  const std::string contactPointString = _node.Read("contactPoint", true, "",
      "The point (model coordinates) which establishes a grip with an object.");
  std::istringstream ss(contactPointString);
  ss >> contactPoint;

  // We will align the effector so the vector from the BBox centroid to the
  // contact point is aligned (but backwards) with the part to move's face
  // normal.
  centerToContactDir =
         contactPoint - _mb->GetBody(bodyIndex)->GetBoundingBox().GetCentroid();
}

/*---------------------------------- I/O -------------------------------------*/

void
Robot::
ReadXMLFile(const std::string& _filename) {
  XMLNode node(_filename, "Robot");
  ReadXMLNode(node);
}


void
Robot::
ReadXMLNode(XMLNode& _node) {
  // Read attributes of the robot node.
  m_maxLinearVelocity = _node.Read("maxLinearVelocity", false, 10., 0.,
      std::numeric_limits<double>::max(),
      "The robot's maximum linear velocity in units/sec.");
  m_maxAngularVelocity = _node.Read("maxAngularVelocity", false, 1., 0.,
      std::numeric_limits<double>::max(),
      "The robot's maximum angular velocity in radians/sec.");

  for(auto& child : _node) {
    std::string name = child.Name();
    std::transform(name.begin(), name.end(), name.begin(), ::tolower);

    if(name == "multibody") {
      if(m_multibody.get())
        throw ParseException(_node.Where()) << "Redefinition of multibody.";

      // Read the multibody file. Eventually we'll go full XML and pass the
      // child node directly to the multibody instead.
      const std::string mbFile = child.Read("filename", false, "", "Name of the "
                                            "robot's multibody file");

      // If there is no filename then the multibody information is in the XML
      // child node.
      if(mbFile == "")
        ReadMultiBodyXML(child);
      else
        ReadMultibodyFile(_node.GetPath() + mbFile);
    }
    else if(name == "actuator") {
      // We need a multibody to parse the actuators.
      if(!m_multibody)
        throw ParseException(child.Where()) << "A multibody must be specified "
                                            << "before any actuators.";

      // Parse the actuator.
      std::unique_ptr<Actuator> actuator(new Actuator(this, child));
      m_actuators[actuator->GetLabel()] = std::move(actuator);
    }
    else if(name == "controller") {
      auto controller = ControllerMethod::Factory(this, child);
      SetController(std::move(controller));
    }
    else if(name == "effector") {
      // We need a multibody to parse the end-effector.
      if(!m_multibody)
        throw ParseException(child.Where()) << "A multibody must be specified "
                                            << "before an end-effector.";

      std::cerr << "Warning: only one end-effector node is supported in the "
                << "robot XML."
                << std::endl;

      m_endEffector = EndEffector(child, this->GetMultiBody());
    }
  }
}


void
Robot::
ReadMultiBodyXML(XMLNode& _node) {
  m_multibody = std::unique_ptr<MultiBody>(new MultiBody(_node));

  // Initialize the DOF limits and set the robot to a zero starting configuration.
  InitializePlanningSpaces();
  m_multibody->Configure(std::vector<double>(m_multibody->DOF(), 0));
}


void
Robot::
ReadMultibodyFile(const std::string& _filename) {
  // Open the file.
  CountingStreamBuffer cbs(_filename);
  std::istream ifs(&cbs);

  // Parse the file to get the robot's geometry.
  m_multibody = std::unique_ptr<MultiBody>(new MultiBody(MultiBody::Type::Active));
  m_multibody->Read(ifs, cbs);

  // Initialize the DOF limits and set the robot to a zero starting configuration.
  InitializePlanningSpaces();
  m_multibody->Configure(std::vector<double>(m_multibody->DOF(), 0));
}

/*--------------------------- Planning Interface -----------------------------*/

void
Robot::
InitializePlanningSpaces() {
  m_multibody->InitializeDOFs(m_problem->GetEnvironment()->GetBoundary());

  const size_t dof = m_multibody->DOF();
  const auto& dofInfo = m_multibody->GetDofInfo();

  // Create the configuration space boundary.
  m_cspace = std::unique_ptr<CSpaceBoundingBox>(new CSpaceBoundingBox(dof));

  for(size_t i = 0; i < dof; ++i)
    m_cspace->SetRange(i, dofInfo[i].range);

  // If this is a holonomic robot, we are done.
  if(!m_nonholonomic)
    return;

  // Create the velocity space boundary.
  m_vspace = std::unique_ptr<CSpaceBoundingBox>(new CSpaceBoundingBox(dof));

  const size_t pos = m_multibody->PosDOF(),
               ori = m_multibody->OrientationDOF();

  for(size_t i = 0; i < pos; ++i)
    m_vspace->SetRange(i, -m_maxLinearVelocity, m_maxLinearVelocity);

  for(size_t i = pos; i < pos + ori; ++i)
    m_vspace->SetRange(i, -m_maxAngularVelocity, m_maxAngularVelocity);

  /// @todo Set up a way to specify velocity limits for each joint.
  for(size_t i = pos + ori; i < dof; ++i)
    m_vspace->SetRange(i, -1, 1);
}


const CSpaceBoundingBox*
Robot::
GetCSpace() const noexcept {
  return m_cspace.get();
}


const CSpaceBoundingBox*
Robot::
GetVSpace() const noexcept {
  return m_vspace.get();
}


MPProblem*
Robot::
GetMPProblem() const noexcept {
  return m_problem;
}

/*------------------------- Simulation Interface -----------------------------*/

void
Robot::
Step(const double _dt) {
  // Run the agent's decision-making routine. The agent will apply controls as
  // required to execute its decision.
  if(m_agent and !m_agent->IsChild())
    m_agent->Step(_dt);
}


void
Robot::
SynchronizeModels() noexcept {
  if(m_bulletModel)
    GetMultiBody()->Configure(m_bulletModel->GetState());
}


BulletModel*
Robot::
GetSimulationModel() noexcept {
  return m_bulletModel;
}


void
Robot::
SetSimulationModel(BulletModel* const _m) {
  m_bulletModel = _m;
}

/*--------------------------- Geometry Accessors -----------------------------*/

MultiBody*
Robot::
GetMultiBody() noexcept {
  return m_multibody.get();
}


const MultiBody*
Robot::
GetMultiBody() const noexcept {
  return m_multibody.get();
}



const Robot::EndEffector&
Robot::
GetEndEffector() const noexcept {
  return m_endEffector;
}


/*------------------------------ Agent Accessors -----------------------------*/

Agent*
Robot::
GetAgent() noexcept {
  return m_agent.get();
}


void
Robot::
SetAgent(std::unique_ptr<Agent>&& _a) noexcept {
  m_agent = std::move(_a);
}

/*---------------------------- Control Accessors -----------------------------*/

ControllerMethod*
Robot::
GetController() noexcept {
  return m_controller.get();
}


void
Robot::
SetController(std::unique_ptr<ControllerMethod>&& _c) noexcept {
  m_controller = std::move(_c);
}

/*---------------------------- Actuator Accessors ----------------------------*/

Actuator*
Robot::
GetActuator(const std::string& _label) noexcept {
  return m_actuators[_label].get();
}


const std::unordered_map<std::string, std::unique_ptr<Actuator>>&
Robot::
GetActuators() const noexcept {
  return m_actuators;
}

/*---------------------------- Dynamics Accessors ----------------------------*/

#ifdef PMPL_USE_MATLAB
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wignored-qualifiers"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#include "MatlabEngine.hpp"
#include "MatlabDataArray.hpp"
#pragma GCC diagnostic pop

MatlabMicroSimulator*
Robot::
GetMatlabMicroSimulator() noexcept {
  if(!m_matlabSimulator)
    m_matlabSimulator.reset(new MatlabMicroSimulator(this));

  return m_matlabSimulator.get();
}
#endif


MicroSimulator*
Robot::
GetMicroSimulator() noexcept {
  if(!m_simulator)
    m_simulator.reset(new MicroSimulator(this));

  return m_simulator.get();
}

/*---------------------------- Hardware Interface ----------------------------*/

RobotCommandQueue*
Robot::
GetHardwareQueue() const noexcept {
  return m_hardware.get();
}


Battery*
Robot::
GetBattery() const noexcept {
  return m_battery.get();
}


StateEstimator*
Robot::
GetStateEstimator() const noexcept {
  return m_stateEstimator.get();
}


void
Robot::
SetStateEstimator(std::unique_ptr<StateEstimator>&& _stateEstimator) noexcept {
  m_stateEstimator = std::move(_stateEstimator);
}

/*------------------------------- Other --------------------------------------*/

bool
Robot::
IsVirtual() const noexcept {
  return m_virtual;
}


void
Robot::
SetVirtual(bool _v) noexcept {
  m_virtual = _v;
}


bool
Robot::
IsNonholonomic() const noexcept {
  return m_nonholonomic;
}


bool
Robot::
IsCarlike() const noexcept {
  return m_carlike;
}


double
Robot::
GetMaxLinearVelocity() const noexcept {
  return m_maxLinearVelocity;
}


double
Robot::
GetMaxAngularVelocity() const noexcept {
  return m_maxAngularVelocity;
}


const std::string&
Robot::
GetLabel() const noexcept {
  return m_label;
}

const std::string&
Robot::
GetDefaultStrategyLabel() const noexcept {
  return m_defaultStrategyLabel;
}


const std::string&
Robot::
GetCapability() const noexcept {
  return m_capability;
}


void
Robot::
SetCapability(const std::string& _capability) {
  m_capability = _capability;
}

bool
Robot::
IsManipulator() const noexcept {
  return m_manipulator;
}


bool
Robot::
IsFixed() const noexcept {
  return m_fixed;
}
		
/*---------------------------------- Debug -----------------------------------*/

std::ostream&
operator<<(std::ostream& _os, const Robot& _r) {
  _os << "Robot '" << _r.GetLabel() << "'"
      << "\n\tDOF:      " << _r.GetMultiBody()->DOF()
      << "\n\tPosDOF:   " << _r.GetMultiBody()->PosDOF()
      << "\n\tOriDOF:   " << _r.GetMultiBody()->OrientationDOF()
      << "\n\tJointDOF: " << _r.GetMultiBody()->JointDOF()
      << "\nActuators:\n";
  for(const auto& a : _r.GetActuators())
    _os << *a.second.get() << "\n";
  return _os << std::endl;
}

/*----------------------------------------------------------------------------*/
