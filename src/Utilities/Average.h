#ifndef PMPL_AVERAGE_H_
#define PMPL_AVERAGE_H_

#include <cstddef>


////////////////////////////////////////////////////////////////////////////////
/// Keep a running total and count of any type which supports addition and
/// division.
////////////////////////////////////////////////////////////////////////////////
template <typename T>
class Average final
{

  ///@name Internal State
  ///@{

  size_t m_count{0}; ///< The number of elements summed.
  T m_total{T()};    ///< The running total.

  ///@}

  public:

    ///@name Modifiers
    ///@{

    /// Add an element to the average.
    /// @param _value The element to add.
    /// @return A reference to self.
    Average& operator+=(const T& _value) noexcept;

    /// Add one or more elements (pre-summed) to the average.
    /// @param _sum The summed elements to add.
    /// @param _count The number of elements.
    /// @return A reference to self.
    Average& AddSummedValues(const T& _sum, const size_t _count) noexcept;

    ///@}
    ///@name Queries
    ///@{

    /// Get the average.
    T Get() const noexcept;

    /// Get the total sum.
    const T& Sum() const noexcept;

    /// Get the number of elements included.
    size_t Count() const noexcept;

    ///@}

};

/*-------------------------------- Modifiers ---------------------------------*/

template <typename T>
inline
Average<T>&
Average<T>::
operator+=(const T& _value) noexcept {
  m_total += _value;
  ++m_count;
  return *this;
}


template <typename T>
inline
Average<T>&
Average<T>::
AddSummedValues(const T& _sum, const size_t _count) noexcept {
  m_total += _sum;
  m_count += _count;
  return *this;
}

/*--------------------------------- Queries ----------------------------------*/

template <typename T>
inline
T
Average<T>::
Get() const noexcept {
  return m_total / m_count;
}


template <typename T>
inline
const T&
Average<T>::
Sum() const noexcept {
  return m_total;
}


template <typename T>
inline
size_t
Average<T>::
Count() const noexcept {
  return m_count;
}

/*----------------------------------------------------------------------------*/

#endif
