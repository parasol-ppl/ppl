#ifndef EDIT_CONNECTION_DIALOG_H_
#define EDIT_CONNECTION_DIALOG_H_

#include <QtGui>

class Connection;
class DrawableMultiBody;
class EditDHParametersWidget;
class EditJointLimitsWidget;
class EditTransformationWidget;
class main_window;
class MultiBody;
namespace mathtool {
  class Transformation;
}


////////////////////////////////////////////////////////////////////////////////
/// A dialog for editing multibody connections.
////////////////////////////////////////////////////////////////////////////////
class EditConnectionDialog : public QDialog {

  Q_OBJECT

  public:

    ///@name Construction
    ///@{

    /// Construct an edit dialog.
    /// @param _parent The owning main window.
    /// @param _mb The owning multibody drawable.
    /// @param _index The connection index.
    EditConnectionDialog(main_window* const _parent,
        DrawableMultiBody* const _mb, const size_t _index);

    ///@}
    ///@name Helpers
    ///@{

  public slots:

    /// Update the transformation to the DH frame.
    void UpdateTransformationToDHFrame();

    /// Update the transformation to the second body.
    void UpdateTransformationToBody2();

    /// Update the DH parameters.
    void UpdateDHParameters();

    /// Update the joint limits.
    void UpdateJointLimits();

  private:

    /// Update a transformation from the edit tool values.
    void UpdateTransformation(EditTransformationWidget* const _w,
        mathtool::Transformation& _t);

    ///@}
    ///@name Internal State
    ///@{

    main_window* const m_main;            ///< The owning main window.
    DrawableMultiBody* const m_drawable;  ///< The edit drawable.
    Connection* const m_connection;       ///< The edit connection.

    // Editor tools for each of the connection properties.
    EditTransformationWidget* m_transform1Editor{nullptr};
    EditTransformationWidget* m_transform2Editor{nullptr};
    EditDHParametersWidget*   m_dhParamsEditor{nullptr};
    EditJointLimitsWidget*    m_limitsEditor{nullptr};

    ///@}
};

#endif
