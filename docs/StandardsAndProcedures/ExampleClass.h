#ifndef EXAMPLE_CLASS_H_
#define EXAMPLE_CLASS_H_

#include "SomeFile.h"

using namespace std;

template<class T>
class ExampleClass : public BaseClass {
  public:
    ExampleClass(int _v);

    void MyFunction(int _v);
    int GetValue() {return m_value;}

  private:
    int m_value;
    T* m_myFriend;
}

template<class T>
ExampleClass<T>::
ExampleClass(int _v) : m_value(_v) {
  if(m_debug) //assume m_debug from BaseClass
    for(int i = 0; i < m_value; i++)
      cout << "Counting::" << i << endl;
}

template<class T>
void
ExampleClass<T>::
MyFunction(int _v) {
  //do some fancy stuff
  m_value += _v;
}

#endif
