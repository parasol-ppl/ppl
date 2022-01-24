#include "TMPBaseObject.h"

#include "MPProblem/MPProblem.h"
#include "TMPLibrary/TMPLibrary.h"

/*-------------------------------- Construction ------------------------------*/

TMPBaseObject::
TMPBaseObject(const std::string& _label, const std::string& _name, bool _debug) :
    m_debug(_debug), m_name(_name), m_label(_label) { }

TMPBaseObject::
TMPBaseObject(XMLNode& _node) {
  m_label = _node.Read("label", true, "", "Label Identifier");
  m_debug = _node.Read("debug", false, false, "Show run-time debug info?");
}

/*------------------------------------ I/O -----------------------------------*/

void
TMPBaseObject::
Print(std::ostream& _os) const {
  _os << this->GetNameAndLabel() << endl;
}

/*-------------------------- Name and Label Accessors ------------------------*/

const std::string&
TMPBaseObject::
GetName() const {
  return m_name;
}

const std::string&
TMPBaseObject::
GetLabel() const {
  return m_label;
}

std::string
TMPBaseObject::
GetNameAndLabel() const {
  return m_name + "::" + m_label;
}

void
TMPBaseObject::
SetLabel(const std::string& _s) {
  m_label = _s;
}

/*----------------------------- TMPLibrary Accessors --------------------------*/

void
TMPBaseObject::
SetTMPLibrary(TMPLibrary* _l) noexcept {
  m_tmpLibrary = _l;
}

TMPLibrary*
TMPBaseObject::
GetTMPLibrary() noexcept {
  return m_tmpLibrary;
}

TMPBaseObject::TMPStrategyMethodPointer
TMPBaseObject::
GetTMPStrategy(const std::string& _label) const noexcept {
  return m_tmpLibrary->GetTMPStrategy(_label);
}

TMPBaseObject::PoIPlacementMethodPointer
TMPBaseObject::
GetPoIPlacementMethod(const std::string& _label) const noexcept {
  return m_tmpLibrary->GetPoIPlacementMethod(_label);
}

TMPBaseObject::TaskEvaluatorMethodPointer
TMPBaseObject::
GetTaskEvaluator(const std::string& _label) const noexcept {
  return m_tmpLibrary->GetTaskEvaluator(_label);
}

TMPBaseObject::TaskDecomposerMethodPointer
TMPBaseObject::
GetTaskDecomposer(const std::string& _label) const noexcept {
  return m_tmpLibrary->GetTaskDecomposer(_label);
}

TMPBaseObject::TaskAllocatorMethodPointer
TMPBaseObject::
GetTaskAllocator(const std::string& _label) const noexcept {
  return m_tmpLibrary->GetTaskAllocator(_label);
}

TMPTools*
TMPBaseObject::
GetTMPTools() const noexcept {
  return m_tmpLibrary->GetTMPTools();
}

/*------------------------------ Problem Accessors ---------------------------*/

MPLibrary*
TMPBaseObject::
GetMPLibrary() const noexcept {
  return m_tmpLibrary->GetMPLibrary();
}

MPProblem*
TMPBaseObject::
GetMPProblem() const noexcept {
  return m_tmpLibrary->GetMPProblem();
}

/*--------------------------- Solution Accessors -----------------------------*/

Plan*
TMPBaseObject::
GetPlan() const noexcept {
  return m_tmpLibrary->GetPlan();
}

TMPBaseObject::StateGraphPointer
TMPBaseObject::
GetStateGraph(const std::string& _label) const noexcept {
  return m_tmpLibrary->GetStateGraph(_label);
}
