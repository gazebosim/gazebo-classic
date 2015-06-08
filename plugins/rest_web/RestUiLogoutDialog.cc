/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#ifndef _GAZEBO_REST_UI_LOGIN_DIALOG_HH_
#define _GAZEBO_REST_UI_LOGIN_DIALOG_HH_

#include "RestUiLogoutDialog.hh"
#include "RestUiWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
RestUiLogoutDialog::RestUiLogoutDialog(QWidget *_parent,
                                       const std::string &_defaultUrl)
  : QDialog(_parent)
{
  this->setWindowTitle(tr("Logout"));
  this->setModal(true);

  /// A label for the url component that appears on the
  /// logout dialog
  QLabel *labelUrl;
  /// The standard dialog buttons
  QDialogButtonBox *buttons;
  labelUrl = new QLabel(this);
  labelUrl->setText(tr(_defaultUrl.c_str()));

  buttons = new QDialogButtonBox(this);
  buttons->addButton(QDialogButtonBox::Ok);
  buttons->button(QDialogButtonBox::Ok)->setText("Logout");
  buttons->button(QDialogButtonBox::Ok)->setDefault(true);
  buttons->addButton(QDialogButtonBox::Cancel);

  // place components
  QGridLayout *formGridLayout = new QGridLayout(this);

  formGridLayout->addWidget(labelUrl, 0, 0, 1, 2);
  formGridLayout->addWidget(buttons, 5, 0, 1, 2);

  this->setLayout(formGridLayout);

  this->connect(buttons->button(QDialogButtonBox::Cancel),
          SIGNAL(clicked()),
          this,
          SLOT(close()));

  this->connect(buttons->button(QDialogButtonBox::Ok),
          SIGNAL(clicked()),
          this,
          SLOT(SlotAcceptLogout()));
}

/////////////////////////////////////////////////
void RestUiLogoutDialog::SlotAcceptLogout()
{
  // close the dialog with success exit code
  accept();
}

#endif
