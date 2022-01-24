#include "SliderTextWidget.h"


/*------------------------------- Construction -------------------------------*/

SliderTextWidget::
SliderTextWidget(QWidget* const _parent, const std::string& _label,
    const double _min, const double _max)
  : SliderTextWidget(_parent, _label, Range<double>(_min, _max))
{ }


SliderTextWidget::
SliderTextWidget(QWidget* const _parent, const std::string& _label,
    const Range<double>& _range)
  : QGroupBox(_label.c_str(), _parent), m_range(_range)
{
  // Assert the range makes sense.
  if(m_range.min >= m_range.max)
    throw RunTimeException(WHERE, "Nonsense range requested from " +
        std::to_string(m_range.min) + " to " + std::to_string(m_range.max) + ".");

  // Initialize the line edit widget.
  m_text = new QLineEdit(this);
  m_text->setMaxLength(9);
  m_text->setValidator(new QDoubleValidator(m_range.min, m_range.max, 8, m_text));

  // Initialize the slider widget.
  m_slider = new QSlider(this);
  m_slider->setOrientation(Qt::Horizontal);
  m_slider->setTickPosition(QSlider::NoTicks);
  m_slider->setSingleStep(1);
  m_slider->setPageStep(s_numTicks / 30);
  m_slider->setRange(0, s_numTicks);

  // Set the initial value as the midpoint.
  SetValue(m_range.Center());

  // Create a layout for the subcomponents.
  QHBoxLayout* layout = new QHBoxLayout(this);
  layout->addWidget(m_text);
  layout->addWidget(m_slider);
  setLayout(layout);

  // Connect the signals together so that the slider and text change together.
  connect(m_text,   SIGNAL(textEdited(const QString&)),
          this,       SLOT(SyncToText()));
  connect(m_slider, SIGNAL(valueChanged(int)),
          this,       SLOT(SyncToSlider()));
}

/*--------------------------------- Helpers ----------------------------------*/

double
SliderTextWidget::
GetValue() {
  return m_text->text().toDouble();
}


void
SliderTextWidget::
SetValue(const double _value) {
  // Ignore out-of-bounds requests.
  if(!m_range.Contains(_value))
    return;

  // Determine what fraction of the range this represents.
  const double fraction = (_value - m_range.min) / m_range.Length();

  m_slider->setSliderPosition(s_numTicks * fraction);
  m_text->setText(QString::number(_value));
}


void
SliderTextWidget::
SyncToText() {
  // Extract the new value from the text field.
  const double newValue = GetValue();

  // Determine what fraction of the range this represents.
  const double fraction = (newValue - m_range.min) / m_range.Length();

  // Set the slider position as near as possible.
  m_slider->setSliderPosition(s_numTicks * fraction);

  emit ValueChanged(newValue);
}


void
SliderTextWidget::
SyncToSlider() {
  // Extract the new fraction of the range from the slider.
  const double fraction = m_slider->sliderPosition() / double(s_numTicks);

  // Determine what value this represents.
  const double newValue = m_range.min + fraction * m_range.Length();

  // Set the text field from the new value.
  m_text->setText(QString::number(newValue));

  emit ValueChanged(newValue);
}

/*----------------------------------------------------------------------------*/
