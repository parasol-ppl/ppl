#ifndef EDIT_WIDGETS_H_
#define EDIT_WIDGETS_H_

#include "Geometry/Boundaries/Range.h"

#include <array>
#include <QtGui>

class Connection;
class DHParameters;
class SliderTextWidget;
namespace mathtool {
  class Transformation;
}


////////////////////////////////////////////////////////////////////////////////
/// A widget for editing compound numerical properties.
///
/// @note Rather than hard-linking to an existing object, this object indicates
///       when the GUI value has changed and offers a function to retrieve the
///       updated value. This is intended to decouple the process of adjusting
///       the object with a gui and subsequently updating the original model.
////////////////////////////////////////////////////////////////////////////////
class EditTransformationWidget : public QGroupBox {

  Q_OBJECT

  public:

    ///@name Construction
    ///@{

    /// Create a transform edit widget.
    /// @param _parent The owning widget.
    /// @param _label The transformation label.
    /// @param _t The initial value of the transformation object.
    EditTransformationWidget(QWidget* const _parent, const std::string& _label,
        const mathtool::Transformation& _t);

    ///@}
    ///@name Helpers
    ///@{

    /// Get the current value from the widget values.
    mathtool::Transformation GetValue();

    /// Set the current value displayed in the widget.
    void SetValue(const mathtool::Transformation& _t);

    ///@}

  signals:

    ///@name Signals
    ///@{

    /// Indicate that a value was updated.
    void ValueChanged();

    ///@}

  private:

    ///@name Internal State
    ///@{

    /// A set of combined slider/text widgets for changing the values.
    std::array<SliderTextWidget*, 6> m_sliders;

    ///@}

};


////////////////////////////////////////////////////////////////////////////////
/// A widget for editing DH params.
///
/// @note Rather than hard-linking to an existing object, this object indicates
///       when the GUI value has changed and offers a function to retrieve the
///       updated value. This is intended to decouple the process of adjusting
///       the object with a gui and subsequently updating the original model.
////////////////////////////////////////////////////////////////////////////////
class EditDHParametersWidget : public QGroupBox {

  Q_OBJECT

  public:

    ///@name Construction
    ///@{

    /// Create a dhparam edit widget.
    /// @param _parent The owning widget.
    /// @param _label The DH param label.
    /// @param _dh The initial value of the dhparam object.
    EditDHParametersWidget(QWidget* const _parent, const std::string& _label,
        const DHParameters& _dh);

    ///@}
    ///@name Helpers
    ///@{

    /// Get the current value from the widget values.
    DHParameters GetValue();

    /// Set the current value displayed in the widget.
    void SetValue(const DHParameters& _dh);

    ///@}

  signals:

    ///@name Signals
    ///@{

    /// Indicate that a value was updated.
    void ValueChanged();

    ///@}

  private:

    ///@name Internal State
    ///@{

    /// A set of combined slider/text widgets for changing the values.
    std::array<SliderTextWidget*, 4> m_sliders;

    ///@}


};


////////////////////////////////////////////////////////////////////////////////
/// A widget for editing joint limits.
///
/// @note Rather than hard-linking to an existing object, this object indicates
///       when the GUI value has changed and offers a function to retrieve the
///       updated value. This is intended to decouple the process of adjusting
///       the object with a gui and subsequently updating the original model.
////////////////////////////////////////////////////////////////////////////////
class EditJointLimitsWidget : public QGroupBox {

  Q_OBJECT

  public:

    ///@name Construction
    ///@{

    /// Create a dhparam edit widget.
    /// @param _parent The owning widget.
    /// @param _label The connection label.
    /// @param _dh The initial value of the connection object.
    EditJointLimitsWidget(QWidget* const _parent, const std::string& _label,
        const Connection& _connection);

    ///@}
    ///@name Helpers
    ///@{

    /// Get the current value from the widget values.
    std::vector<Range<double>> GetValue();

    /// Set the current value displayed in the widget.
    void SetValue(const Range<double>& _limit1,
        const Range<double>& _limit2 = {});

    ///@}

  signals:

    ///@name Signals
    ///@{

    /// Indicate that a value was updated.
    void ValueChanged();

    ///@}

  private:

    ///@name Internal State
    ///@{

    /// A set of combined slider/text widgets for changing the values.
    std::array<SliderTextWidget*, 4> m_sliders;

    ///@}

};


#endif
