/*
 * Copyright 2015 Open Source Robotics Foundation
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

#include <QMessageBox>
#include "RestUiWidget.hh"

using namespace gazebo;

/////////////////////////////////////////////////
RestUiWidget::RestUiWidget(QWidget *_parent,
                           QAction &_login,
                           QAction &_logout,
                           const std::string &_menuTitle,
                           const std::string &_loginTitle,
                           const std::string &_urlLabel,
                           const std::string &_defautlUrl)
  : QWidget(_parent),
    loginMenuAction(_login),
    logoutMenuAction(_logout),
    title(_menuTitle),
    node(new gazebo::transport::Node()),
    logoutDialog(this, _defautlUrl),
    loginDialog(this, _loginTitle, _urlLabel, _defautlUrl)
{
  this->node->Init();
  this->loginPub = node->Advertise<gazebo::msgs::RestLogin>(
      "/gazebo/rest/rest_login");
  this->logoutPub = node->Advertise<gazebo::msgs::RestLogout>(
      "/gazebo/rest/rest_logout");
  this->errorSub = node->Subscribe("/gazebo/rest/rest_error",
                                   &RestUiWidget::OnResponse,
                                   this);
}

/////////////////////////////////////////////////
void RestUiWidget::Logout()
{
  if (this->logoutDialog.exec() != QDialog::Rejected)
  {
    gazebo::msgs::RestLogout msg;
    std::string url = this->loginDialog.GetUrl();
    msg.set_url(url);
    gzmsg << "Logging out from: " << url << std::endl;
    this->logoutPub->Publish(msg);
    this->loginMenuAction.setEnabled(true);
    this->logoutMenuAction.setEnabled(false);
  }
}

/////////////////////////////////////////////////
void RestUiWidget::Login()
{
  if (this->loginDialog.exec() != QDialog::Rejected)
  {
    gazebo::msgs::RestLogin msg;
    msg.set_url(this->loginDialog.GetUrl());
    msg.set_username(this->loginDialog.GetUsername());
    msg.set_password(this->loginDialog.GetPassword());
    this->loginPub->Publish(msg);
    this->loginMenuAction.setEnabled(false);
    this->logoutMenuAction.setEnabled(true);
  }
}

/////////////////////////////////////////////////
void RestUiWidget::OnResponse(ConstRestErrorPtr &_msg)
{
  gzerr << "Error received:" << std::endl;
  gzerr << " type: " << _msg->type() << std::endl;
  gzerr << " msg:  " << _msg->msg() << std::endl;
  // add msg to queue for later processing from the GUI thread
  this->msgRespQ.push_back(_msg);
}

/////////////////////////////////////////////////
void RestUiWidget::Update()
{
  // Login problem?
  while (!this->msgRespQ.empty())
  {
    ConstRestErrorPtr msg = this->msgRespQ.front();
    std::string msgStr = msg->msg();
    this->msgRespQ.pop_front();

    // look for login error, and reenable the login menu if necessary
    if (msgStr.find("There was a problem trying to login to the server") == 0)
    {
      this->loginMenuAction.setEnabled(true);
      this->logoutMenuAction.setEnabled(false);
    }

    if (msg->type() == "Error")
    {
      msgStr += "\n\nIf the server is not available, ";
      msgStr += "logout to hide theses messages.";
      QMessageBox::critical(this,
                            tr(this->title.c_str()),
                            tr(msgStr.c_str()));
    }
    else
    {
      QMessageBox::information(this,
                               tr(this->title.c_str()),
                               tr(msgStr.c_str()));
    }
  }
}
