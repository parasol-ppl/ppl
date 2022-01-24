#ifndef CONDITIONAL_EVALUATION_H
#define CONDITIONAL_EVALUATION_H

#include "MapEvaluatorMethod.h"

////////////////////////////////////////////////////////////////////////////////
/// Evaluates whether a given metric meets a specific numeric condition.
/// @ingroup MapEvaluators
////////////////////////////////////////////////////////////////////////////////
template <typename MPTraits>
class ConditionalEvaluator : public MapEvaluatorMethod<MPTraits> {
  public:

    enum Operator { LT , LEQ, GT, GEQ, MOD }; ///< The supported operators

    ConditionalEvaluator(Operator _operator = LT, string _metric = "",
        double _value = 1.0);
    ConditionalEvaluator(XMLNode& _node);
    virtual ~ConditionalEvaluator() = default;

    virtual void Print(ostream& _os) const;

    virtual bool operator()();

  protected:

    Operator m_operator; ///< The operator to use
    string m_metric; ///< The metric to evaluate
    double m_value; ///< The numeric condition
};


template <typename MPTraits>
ConditionalEvaluator<MPTraits>::
ConditionalEvaluator(Operator _operator, string _metric, double _value) :
    MapEvaluatorMethod<MPTraits>(), m_operator(_operator), m_metric(_metric),
    m_value(_value) {
  this->SetName("ConditionalEvaluator");
}


template <typename MPTraits>
ConditionalEvaluator<MPTraits>::
ConditionalEvaluator(XMLNode& _node) : MapEvaluatorMethod<MPTraits>(_node) {
  this->SetName("ConditionalEvaluator");

  m_metric = _node.Read("metric_method", true, "", "Metric Method");
  m_value = _node.Read("value", true, 1.0, 0.0,
      std::numeric_limits<double>::max(), "the value of the metric");

  string op = _node.Read("operator", true, "", "operator");
  if (op == "<")
    m_operator = LT;
  else if (op == "<=")
    m_operator = LEQ;
  else if (op == ">")
    m_operator = GT;
  else if (op == ">=")
    m_operator = GEQ;
  else if (op == "%")
    m_operator = MOD;
  else
    throw ParseException(WHERE, "Unknown relational operator label '" + op + ".");
}


template <typename MPTraits>
void
ConditionalEvaluator<MPTraits>::
Print(ostream& _os) const {
  MapEvaluatorMethod<MPTraits>::Print(_os);
  _os << "\tmetric method: " << m_metric << endl;
  _os << "\tvalue: " << m_value << endl;
  _os << "\toperator: ";
  switch(m_operator){
    case LT: cout << "<"; break;
    case LEQ: cout << "<="; break;
    case GT: cout << ">"; break;
    case GEQ: cout << ">="; break;
    case MOD: cout << "%"; break;
  }
  _os << endl;
}


template <typename MPTraits>
bool
ConditionalEvaluator<MPTraits>::
operator()() {
  double metricValue = this->GetMetric(m_metric)->operator()();

  switch(m_operator){
    case LT: return metricValue < m_value;
    case LEQ: return metricValue <= m_value;
    case GT: return metricValue > m_value;
    case GEQ: return metricValue >= m_value;
    case MOD:
      static double prevVal=0.0;
      if(prevVal == 0.0 ||
          (floor(metricValue/m_value) != floor(prevVal/m_value) && m_value > 0)) {
        prevVal = metricValue;
        return true;
      }
      prevVal = metricValue;
      return false;
    default:
      cout << "Unknown label is read" << endl;
      return false;
  }
}

#endif
