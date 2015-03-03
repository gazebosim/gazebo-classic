/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include "RestUiLoginDialog.hh"
#include "RestUiWidget.hh"

using namespace gazebo;
using namespace gui;
using namespace std;


/////////////////////////////////////////////////
RestUiLoginDialog::RestUiLoginDialog(QWidget *_parent,
                                     const char* _title,
                                     const char* _urlLabel,
                                     const char* _defaultUrl)
    : QDialog(_parent), url(_defaultUrl)
{
  setWindowTitle(tr(_title));
  setModal(true);

  labelUrl = new QLabel(this);
  labelUrl->setText(tr(_urlLabel));
  editUrl = new QLineEdit(this);
  editUrl->setText(tr(url.c_str()));
  editUrl->setFixedWidth(400);
  labelUrl->setBuddy(editUrl);

  labelUsername = new QLabel(this);
  labelUsername->setText(tr("Username"));
  editUsername = new QLineEdit(this);
  editUsername->setFocus();
  labelUsername->setBuddy(editUsername);

  labelPassword = new QLabel(this);
  labelPassword->setText(tr("Password"));
  editPassword = new QLineEdit(this);
  editPassword->setEchoMode(QLineEdit::Password);
  labelPassword->setBuddy(editPassword);

  labelInfo = new QLabel(this);

  buttons = new QDialogButtonBox(this);
  buttons->addButton(QDialogButtonBox::Ok);
  buttons->button(QDialogButtonBox::Ok)->setText("Login");
  buttons->button(QDialogButtonBox::Ok)->setDefault(true);
  buttons->addButton(QDialogButtonBox::Cancel);

  // place components
  QGridLayout *formGridLayout = new QGridLayout(this);

  formGridLayout->addWidget(labelUrl, 0, 0, 1, 2);
  formGridLayout->addWidget(editUrl,  1, 0, 1, 2);
  formGridLayout->addWidget(labelUsername, 2, 0);
  formGridLayout->addWidget(editUsername,  2, 1);
  formGridLayout->addWidget(labelPassword, 3, 0);
  formGridLayout->addWidget(editPassword,  3, 1);
  formGridLayout->addWidget(labelInfo, 4, 0, 1, 2);
  formGridLayout->addWidget(buttons,   5, 0, 1, 2);

  setLayout(formGridLayout);

  connect(buttons->button(QDialogButtonBox::Cancel),
          SIGNAL(clicked()),
          this,
          SLOT(close()));

  connect(buttons->button(QDialogButtonBox::Ok),
          SIGNAL(clicked()),
          this,
          SLOT(slotAcceptLogin()));
}

/////////////////////////////////////////////////
void RestUiLoginDialog::slotAcceptLogin()
{
  QString user = editUsername->text();
  QString pass = editPassword->text();
  QString u    = editUrl->text();
  username = user.toStdString();
  password = pass.toStdString();
  url = u.toStdString();
  qApp->processEvents();
  emit acceptLogin(u, user, pass);
  // close the dialog with success exit code
  accept();
}

#endif
