#ifndef EDIT_MULTIBODY_DIALOG_H_
#define EDIT_MULTIBODY_DIALOG_H_

#include <QtGui>

class DrawableMultiBody;
class main_window;
class MultiBody;


////////////////////////////////////////////////////////////////////////////////
/// A dialog for editing multibodies.
////////////////////////////////////////////////////////////////////////////////
class EditMultiBodyDialog : public QDialog {

  Q_OBJECT

  public:

    ///@name Construction
    ///@{

    /// Construct an edit dialog.
    /// @param _parent The owning main window.
    /// @param _mb The multibody to edit.
    EditMultiBodyDialog(main_window* const _parent, DrawableMultiBody* const _mb);

    ///@}

  private slots:

    ///@name Helpers
    ///@{

    /// Launch a dialog to edit the currently selected body.
    void EditBody();

    /// Launch a dialog to edit the currently selected connection.
    void EditConnection();

    /// Export the multibody data to the terminal.
    void Export();

    ///@}

  private:

    ///@name Internal State
    ///@{

    main_window* const m_main;            ///< The owning main window.
    DrawableMultiBody* const m_drawable;  ///< The edit drawable.
    MultiBody* const m_multibody;         ///< The edit multibody.

    QListWidget* m_bodyList{nullptr};       ///< The list of editable bodies.
    QListWidget* m_connectionList{nullptr}; ///< The list of editable connections.

    ///@}
};

#endif
