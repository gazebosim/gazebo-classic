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

#include <gazebo/gui/qt.h>
#include <gazebo/gui/GuiEvents.hh>
#include <gazebo/gui/MainWindow.hh>
#include <gazebo/gui/RenderWidget.hh>
#include <gazebo/gui/TopToolbar.hh>

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
    loginDialog(this, _loginTitle, _urlLabel, _defautlUrl)
{
  this->node->Init();
  this->loginPub = node->Advertise<gazebo::msgs::RestLogin>(
      "/gazebo/rest/rest_login");
  this->logoutPub = node->Advertise<gazebo::msgs::RestLogout>(
      "/gazebo/rest/rest_logout");
  this->responseSub = node->Subscribe("/gazebo/rest/rest_response",
                                   &RestUiWidget::OnResponse,
                                   this);

  this->restID = static_cast<unsigned int>(common::Time::GetWallTime().nsec);

  this->toolbar = NULL;
  this->loginLabelAct = NULL;
  this->spacerAct = NULL;
  this->loginLabel = new QLabel();
  gui::MainWindow *mainWindow = qobject_cast<gui::MainWindow *>(_parent);
  if (mainWindow)
  {
    gui::RenderWidget *renderWidget = mainWindow->GetRenderWidget();
    if (renderWidget)
    {
      this->toolbar = renderWidget->GetToolbar();

      // push icons to the left for the login label
      QAction *tbSpacerAction =
          this->toolbar->findChild<QAction *>("toolbarSpacerAction");
      QToolBar *tb =
          this->toolbar->findChild<QToolBar *>("topToolbarToolbar");
      if (tbSpacerAction && tb)
      {
        QWidget *w = tb->widgetForAction(tbSpacerAction);
        w->setSizePolicy(QSizePolicy::Maximum, QSizePolicy::Maximum);
      }

      // Empty space to push the label to the right
      QWidget *spacer = new QWidget();
      spacer->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
      this->spacerAct = this->toolbar->AddWidget(spacer);
      this->spacerAct->setObjectName("toolbarLoginSpacerAction");

      this->loginLabelAct = this->toolbar->AddWidget(this->loginLabel);

      // Connections
      this->connections.push_back(
          gui::Events::ConnectWindowMode(
          boost::bind(&RestUiWidget::OnWindowMode, this, _1)));
    }
  }
  if (!this->toolbar)
  {
    gzerr << "Unable to find Gazebo toolbar. Log-in status will not be shown"
        << std::endl;
  }
}

/////////////////////////////////////////////////
void RestUiWidget::Logout()
{
  QMessageBox msgBox(QMessageBox::NoIcon, QString("Logout"),
      QString("Are you ready to log out?\n\n"));

  QPushButton *cancelButton = msgBox.addButton("Cancel",
      QMessageBox::RejectRole);
  QPushButton *logoutButton =
      msgBox.addButton("Logout", QMessageBox::AcceptRole);
  msgBox.setDefaultButton(logoutButton);
  msgBox.setEscapeButton(cancelButton);

  msgBox.exec();
  if (msgBox.clickedButton() == cancelButton)
    return;

  gazebo::msgs::RestLogout msg;
  msg.set_id(this->restID);
  std::string url = this->loginDialog.GetUrl();
  msg.set_url(url);
  gzmsg << "Logging out from: " << url << std::endl;
  this->logoutPub->Publish(msg);
  this->loginMenuAction.setEnabled(true);
  this->logoutMenuAction.setEnabled(false);
  this->loginLabel->setText(tr("Logging out..."));
}

/////////////////////////////////////////////////
void RestUiWidget::Login()
{
  if (this->loginDialog.exec() == QDialog::Rejected)
    return;

  gazebo::msgs::RestLogin msg;
  msg.set_id(this->restID);
  msg.set_url(this->loginDialog.GetUrl());
  msg.set_username(this->loginDialog.GetUsername());
  msg.set_password(this->loginDialog.GetPassword());
  this->loginPub->Publish(msg);
  this->loginMenuAction.setEnabled(false);
  this->logoutMenuAction.setEnabled(true);
  this->loginLabel->setText(tr("Logging in..."));
}

/////////////////////////////////////////////////
void RestUiWidget::OnResponse(ConstRestResponsePtr &_msg)
{
  gzmsg << "Response received:" << std::endl;
  gzmsg << " type: " << _msg->type() << std::endl;
  gzmsg << " msg:  " << _msg->msg() << std::endl;
  // add msg to queue for later processing from the GUI thread
  this->msgRespQ.push_back(_msg);
}

/////////////////////////////////////////////////
void RestUiWidget::Update()
{
  // Login problem?
  while (!this->msgRespQ.empty())
  {
    ConstRestResponsePtr msg = this->msgRespQ.front();
    std::string msgStr = msg->msg();
    this->msgRespQ.pop_front();

    // look for login error, and reenable the login menu if necessary
    if (msg->type() == msgs::RestResponse::ERROR)
    {
      this->loginMenuAction.setEnabled(true);
      this->logoutMenuAction.setEnabled(false);
      if (!this->loginLabel->text().isEmpty())
      {
        msgStr += "\n\nUnable to connect to the server.";
        QMessageBox::critical(this,
                              tr(this->title.c_str()),
                              tr(msgStr.c_str()));
      }
      this->loginLabel->setText(tr(""));
    }
    else if (msg->type() == msgs::RestResponse::LOGIN)
    {
      this->loginLabel->setText(
          QString::fromStdString(this->loginDialog.GetUsername()));
    }
    else if (msg->type() == msgs::RestResponse::LOGOUT)
    {
      this->loginLabel->setText(tr(""));
    }
  }
}

/////////////////////////////////////////////////
void RestUiWidget::OnWindowMode(const std::string &/*_mode*/)
{
  if (this->loginLabelAct)
    this->loginLabelAct->setVisible(true);
  if (this->spacerAct)
    this->spacerAct->setVisible(true);
}
