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

  std::string versionText;
  versionText = "Version " + std::string(GAZEBO_VERSION);

  QTextEdit *versionEdit = new QTextEdit(tr(versionText.c_str()));
  versionEdit->setAcceptRichText(true);
  versionEdit->setContentsMargins(0, 0, 0, 0);
  versionEdit->setFrameStyle(QFrame::NoFrame);
  versionEdit->setAlignment(Qt::AlignBottom | Qt::AlignRight);
  versionEdit->setObjectName("splashVersionTextEdit");
  versionEdit->setFixedHeight(20);

  QHBoxLayout *versionLayout = new QHBoxLayout();
  versionLayout->addWidget(versionEdit);

  QVBoxLayout *textLayout = new QVBoxLayout();
  textLayout->addSpacerItem(new QSpacerItem(1, pixmap.size().height(),
        QSizePolicy::Expanding, QSizePolicy::MinimumExpanding));
  textLayout->addLayout(versionLayout);

  this->dataPtr->splashScreen = new QSplashScreen(pixmap);
  this->dataPtr->splashScreen->setObjectName("splashScreenWidget");
  this->dataPtr->splashScreen->setMask(pixmap.mask());
  this->dataPtr->splashScreen->setWindowFlags(
      this->dataPtr->splashScreen->windowFlags() | Qt::WindowStaysOnTopHint);
  this->dataPtr->splashScreen->setLayout(textLayout);
  this->dataPtr->splashScreen->setFixedSize(pixmap.size());

  this->dataPtr->splashScreen->show();
  this->dataPtr->splashScreen->repaint();

  QTimer::singleShot(2000, this, SLOT(Update()));
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

/////////////////////////////////////////////////
bool SplashScreen::Visible() const
{
  return this->dataPtr->splashScreen->isVisible();
}
