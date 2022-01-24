#include "Battery.h"

using namespace std;


Battery::
Battery() {
  SetValues(60,60);
}

void
Battery::
SetValues(double _max, double _cur) {
  m_maxLevel = _max;
  m_curLevel = _cur;
}

void
Battery::
SetToMax() {
  m_curLevel = m_maxLevel;
}

void
Battery::
Print() {
  cout << " current battery level: " << m_curLevel << " max level: " << m_maxLevel << endl;
}

double
Battery::
GetCurLevel() {
  return m_curLevel;
}

double
Battery::
GetMaxLevel() {
  return m_maxLevel;
}

void
Battery::
Charge(double _increaseRate) {
  m_curLevel += _increaseRate;
  if( m_curLevel > m_maxLevel )
    m_curLevel = m_maxLevel;
}

void
Battery::
UpdateValue(double _depletionRateBase, double _depletionRateMoving) {
  m_curLevel -= _depletionRateBase + _depletionRateMoving;
  if( m_curLevel < 0 )
    m_curLevel = 0;
}

void
Battery::
UpdateValue(double _depletionRateBase){
  m_curLevel -= _depletionRateBase;
  if( m_curLevel < 0 )
    m_curLevel = 0;
}
