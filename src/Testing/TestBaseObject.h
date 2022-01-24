#ifndef PPL_TEST_BASE_OBJECT
#define PPL_TEST_BASE_OBJECT

/*-----------------------------------------------------------*/
///
/// This is a base class to standardize the testing interface.
/// All child classes should build their tests from the 
/// RunTests function. 
///
/*-----------------------------------------------------------*/

#include <string>
#include <utility>

class TestBaseObject {
  public: 
    ///@name LocalTypes
    ///@{

    typedef std::pair<bool,std::string> TestResult;

    ///@}
    ///@name Construction
    ///@{

    TestBaseObject();

    virtual ~TestBaseObject();

    ///@}
    ///@name Interface
    ///@{

    virtual TestResult RunTest() = 0;

    ///@}

};

#endif
