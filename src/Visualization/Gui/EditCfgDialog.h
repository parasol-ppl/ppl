#ifndef EDIT_CFG_DIALOG_H_
#define EDIT_CFG_DIALOG_H_

#include <QtGui>

class main_window;
class DrawableMultiBody;
class SliderTextWidget;


////////////////////////////////////////////////////////////////////////////////
/// A dialog for editing multibody configurations.
////////////////////////////////////////////////////////////////////////////////
class EditCfgDialog : public QDialog {

  Q_OBJECT

  public:

    ///@name Construction
    ///@{

    /// Construct an edit dialog.
    /// @param _parent The owning main window.
    /// @param _mb The multibody to edit.
    EditCfgDialog(main_window* const _parent, DrawableMultiBody* const _mb);

    virtual ~EditCfgDialog();

    ///@}

  public slots:

    ///@name Helpers
    ///@{

    void UpdateCfg();

    ///@}
    ///@name QDialog Overrides
    ///@{

    virtual void reject() override;

    ///@}

  private:

    ///@name Internal State
    ///@{

    main_window* const m_main;           ///< The owning main window.
    DrawableMultiBody* const m_drawable; ///< The edit multibody.

    std::vector<SliderTextWidget*> m_sliders; ///< The DOF value editors.
    std::vector<double> m_originalCfg;        ///< The original Cfg value.

    ///@}
};

#endif
