#ifndef SLIDER_TEXT_WIDGET_H_
#define SLIDER_TEXT_WIDGET_H_

#include "Geometry/Boundaries/Range.h"

#include <string>
#include <QtGui>


////////////////////////////////////////////////////////////////////////////////
/// This is a widget for adjusting numerical values using both a slider bar and
/// text field. The text input is validated against a fixed range.
////////////////////////////////////////////////////////////////////////////////
class SliderTextWidget : public QGroupBox {

  Q_OBJECT

  public:

    ///@name Construction
    ///@{

    /// Create a slider text widget.
    /// @param _parent The parent widget.
    /// @param _label The label.
    /// @param _min The minimum allowed value.
    /// @param _max The maximum allowed value.
    SliderTextWidget(QWidget* const _parent, const std::string& _label,
        const double _min, const double _max);

    /// Create a slider text widget.
    /// @param _parent The parent widget.
    /// @param _label The label.
    /// @param _range The allowed range.
    SliderTextWidget(QWidget* const _parent, const std::string& _label,
        const Range<double>& _range);

    ///@}
    ///@name Helpers
    ///@{

    /// Get the current value.
    double GetValue();

  public slots:

    /// Set the value for both subwidgets.
    void SetValue(const double);

    /// Synchronize the slider to match the text field.
    void SyncToText();

    /// Synchronize the text field to match the slider.
    void SyncToSlider();

    ///@}

  signals:

    ///@name Signals
    ///@{

    /// Indicate a valid value change.
    void ValueChanged(double);

    ///@}

  private:

    ///@name Internal State
    ///@{

    static constexpr int s_numTicks{50000}; ///< Desired number of slider ticks.

    const Range<double> m_range; ///< The valid range.

    QLineEdit* m_text{nullptr};  ///< The text edit element.
    QSlider* m_slider{nullptr};  ///< The slider element.

    ///@}

};


#endif
