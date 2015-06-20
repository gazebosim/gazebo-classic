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

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/SplashScreen.hh"
#include "gazebo/gui/SplashScreenPrivate.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
SplashScreen::SplashScreen()
  : dataPtr(new SplashScreenPrivate)
{
  this->setObjectName("SplashScreen");
  QPixmap pixmap(":/images/splash.svg");
  QTextEdit *edit = new QTextEdit(
      "<br><br>Welcome to Gazebo!<br><br>"
      "Please prepare for some 3D robot awesomeness.<br><br>"
      "Gazebo is now downloading models to prepare to start the simulation."
      "Depending on your internet connection, this can take an arbitrarily "
      "long amount of time. If you have read this far, we apologize. "
      "Have a nice day.");
  edit->setAcceptRichText(true);
  edit->setContentsMargins(0, 0, 0, 0);
  edit->setFrameStyle(QFrame::NoFrame);
  this->dataPtr->splashScreen = new QSplashScreen(pixmap);
  QHBoxLayout *box = new QHBoxLayout();
  box->setContentsMargins(0, 0, 0, 0);
  this->dataPtr->splashScreen->setLayout(box);
  box->addItem(new QSpacerItem(350, 1));
  box->addWidget(edit);
  this->dataPtr->splashScreen->show();
  ShowMessage("");
  QTimer::singleShot(10, this, SLOT(Update()));
}

/////////////////////////////////////////////////
SplashScreen::~SplashScreen()
{
  delete this->dataPtr->splashScreen;
  this->dataPtr->splashScreen = NULL;

  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void SplashScreen::ShowMessage(const std::string &_message)
{
  this->dataPtr->splashScreen->showMessage(tr(_message.c_str()),
                                           Qt::AlignBottom,
                                           Qt::white);
}

/////////////////////////////////////////////////
void SplashScreen::Update()
{
  rendering::UserCameraPtr cam = get_active_camera();
  if (cam && cam->GetScene() && cam->GetScene()->GetInitialized())
  {
    this->dataPtr->splashScreen->hide();
    return;
  }

  QTimer::singleShot(100, this, SLOT(Update()));
}
