/*
 * viewer/lib/qt/action_button.cpp
 */

// based on https://wiki.qt.io/PushButton_Based_On_Action

#include "action_button.hpp"

ActionButton::ActionButton(QAction* action, QWidget* parent) :
        QPushButton(parent)
{
    // store the action
    actionOwner = action;

    // configure the button
    updateButtonStatusFromAction();

    // connect the action and the button
    // so that when the action is changed the
    // button is changed too!
    connect(action, SIGNAL (changed()), this,
            SLOT (updateButtonStatusFromAction()));

    // connect the button to the slot that forwards the
    // signal to the action
    connect(this, SIGNAL (clicked()), actionOwner, SLOT (trigger()));
}

void ActionButton::updateButtonStatusFromAction()
{
    setText(actionOwner->text());
    setStatusTip(actionOwner->statusTip() + tr(" (") +
                 actionOwner->shortcut().toString() + tr(")."));
    setToolTip(actionOwner->toolTip());
    setIcon(actionOwner->icon());
    setEnabled(actionOwner->isEnabled());
    setCheckable(actionOwner->isCheckable());
    setChecked(actionOwner->isChecked());
    setShortcut(actionOwner->shortcut());
}
