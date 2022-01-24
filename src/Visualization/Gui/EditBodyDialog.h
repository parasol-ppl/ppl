#ifndef EDIT_BODY_DIALOG_H_
#define EDIT_BODY_DIALOG_H_

#include <QtGui>

class Body;
class DrawableBody;
class main_window;


////////////////////////////////////////////////////////////////////////////////
/// A dialog for editing bodies.
////////////////////////////////////////////////////////////////////////////////
class EditBodyDialog : public QDialog {

  Q_OBJECT

  public:

    ///@name Construction
    ///@{

    /// Construct an edit dialog.
    /// @param _parent The owning main window.
    /// @param _b The body to edit.
    EditBodyDialog(main_window* const _parent, DrawableBody* const _b);

    ///@}

  private slots:

    ////@name Helpers
    ///@{
    /// @TODO Not yet implemented.

    /// Edit the body's polyhedral mesh.
    void EditMesh();

    /// Edit the body's type and movement.
    void EditType();

    /// Edit the body's color.
    void EditColor();

    /// Edit the body's physical properties.
    void EditPhysics();

    ///@}

  private:

    ///@name Internal State
    ///@{

    main_window* const m_main;       ///< The owning main window.
    DrawableBody* const m_drawable;  ///< The edit drawable.
    Body* const m_body;              ///< The edit multibody.

    ///@}
};

#endif
