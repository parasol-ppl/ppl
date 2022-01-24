#ifndef TEST_WIDGET_H_
#define TEST_WIDGET_H_

#include <QtGui>

class EditTransformationWidget;
class main_window;


////////////////////////////////////////////////////////////////////////////////
/// A test widget for developing graphic features.
////////////////////////////////////////////////////////////////////////////////
class TestWidget : public QWidget {

  Q_OBJECT

  public:

    ///@name Construction
    ///@{

    TestWidget(main_window* const _parent);

    ///@}

  public slots:

    ///@name Slots
    ///@{

    /// Show the output of the transformation tool.
    void ShowTransformationOutput();

    ///@}

  private:

    ///@name Internal State
    ///@{

    main_window* const m_main; ///< The owning main window.

    EditTransformationWidget* m_transformEditor; ///< A transformation editor.

    ///@}

};

#endif
