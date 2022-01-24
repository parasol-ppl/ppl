#ifndef PPL_TMP_LIBRARY_TESTS_H_
#define PPL_TMP_LIBRARY_TESTS_H_

#include "Testing/TestBaseObject.h"
#include "TMPLibrary/TMPLibrary.h"

#include "Testing/TMPLibrary/TaskAllocators/TaskAllocatorMethodTest.h"
#include "Testing/TMPLibrary/TaskDecomposers/TaskDecomposerMethodTest.h"
#include "Testing/TMPLibrary/TaskEvaluators/TaskEvaluatorMethodTest.h"

class TMPLibraryTests : public TMPLibrary, public TestBaseObject {

  public:

    ///@name LocalTypes
    ///@{

    typedef TestBaseObject::TestResult TestResult;

    ///@}
    ///@name Method Set Types
    ///@{

    typedef TMPMethodSet<TaskAllocatorMethodTest>      TaskAllocatorMethodTestSet;
    typedef TMPMethodSet<TaskDecomposerMethodTest>     TaskDecomposerMethodTestSet;
    typedef TMPMethodSet<TaskEvaluatorMethodTest>      TaskEvaluatorMethodTestSet;
    //typedef TMPMethodSet<TMPStrategyMethod>        TMPStrategyMethodTestSet;
    //typedef TMPMethodSet<PoIPlacementMethod>       PoIPlacementMethodTestSet;
    //typedef TMPMethodSet<StateGraph>               StateGraphSet;

    ///@}
    ///@name Construction
    ///@{

    TMPLibraryTests();

    TMPLibraryTests(const std::string& _xmlFile);

    virtual ~TMPLibraryTests();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() override;

    ///@}

  private:

    ///@name Helper Functions
    ///@{

    void InitializeMethodSets();

    template <typename MethodTypeList>
    void RunMethodSetTests(const MethodTypeList& _mtl,size_t& _passed, 
                           size_t& failed, size_t& _total);

    ///@name XML Helpers
    ///@{

    void ReadXMLFile(const std::string& _filename);

    bool ParseChild(XMLNode& _node);

    ///@} 
    ///@name Internal State
    ///@{

    bool verbose{true};

    ///@}
    ///@name TMPMethod Sets
    ///@{
    /// Method sets hold and offer access to the tmp planning objects of the
    /// corresponding type.

    TaskAllocatorMethodTestSet*   m_taskAllocatorTests;
    TaskDecomposerMethodTestSet*  m_taskDecomposerTests;
    TaskEvaluatorMethodTestSet*   m_taskEvaluatorTests;
    //TMPStrategyMethodTestSet*     m_tmpStrategieTests;
    //PoIPlacementMethodTestSet*    m_poiPlacementMethodTests;
    //StateGraphTestSet*            m_stateGraphTests;

    ///@}
};

template <typename MethodTypeList>
void
TMPLibraryTests::
RunMethodSetTests(const MethodTypeList& _mtl, size_t& _passed, 
                  size_t& _failed, size_t& _total) {

  for(auto iter = _mtl.begin(); iter != _mtl.end(); iter++) {

    std::cout << "Running test for " << iter->first << "..." << std::endl;

    auto test = dynamic_cast<TestBaseObject*>(iter->second.get());
    auto result = test->RunTest();

    _total++;

    if(result.first) {
      std::cout << "PASSED!" << std::endl;
      _passed++;
    }
    else {
      std::cout << "FAILED :(" << std::endl;
      _failed++;
    }

    if(verbose) {
      std::cout << result.second << std::endl;
    }    
  }
}

#endif
