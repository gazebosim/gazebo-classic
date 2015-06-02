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

#ifndef _REST_UI_LOGIN_DIALOG_HH_
#define _REST_UI_LOGIN_DIALOG_HH_

#include <iostream>
#include <curl/curl.h>

#include "RestUiLogoutDialog.hh"
#include "RestUiWidget.hh"

using namespace gazebo;
using namespace gui;


/////////////////////////////////////////////////
RestUiLogoutDialog::RestUiLogoutDialog(QWidget *_parent,
                                     const std::string &_defaultUrl)
    :QDialog(_parent), url(_defaultUrl.c_str())
{
  this->setWindowTitle(tr("Logout"));
  this->setModal(true);

  this->labelUrl = new QLabel(this);
  this->labelUrl->setText(tr(_defaultUrl.c_str()));

  this->buttons = new QDialogButtonBox(this);
  this->buttons->addButton(QDialogButtonBox::Ok);
  this->buttons->button(QDialogButtonBox::Ok)->setText("Logout");
  this->buttons->button(QDialogButtonBox::Ok)->setDefault(true);
  buttons->addButton(QDialogButtonBox::Cancel);

  // place components
  QGridLayout *formGridLayout = new QGridLayout(this);

  formGridLayout->addWidget(labelUrl, 0, 0, 1, 2);
  formGridLayout->addWidget(buttons, 5, 0, 1, 2);

  setLayout(formGridLayout);

  connect(buttons->button(QDialogButtonBox::Cancel),
          SIGNAL(clicked()),
          this,
          SLOT(close()));

  connect(buttons->button(QDialogButtonBox::Ok),
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
