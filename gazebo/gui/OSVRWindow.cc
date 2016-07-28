/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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
#include <boost/bind.hpp>

#include <ignition/math/Pose3.hh>

#include "gazebo/gui/OSVRWindow.hh"
#include "gazebo/rendering/OSVRCamera.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/WindowManager.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
OSVRWindow::OSVRWindow(int _x, int _y, const std::string &_visual,
    QWidget *_parent)
  : QWidget(_parent), windowId(-1), isFullScreen(false),
    xPos(_x), yPos(_y), visualName(_visual)
{
  setAttribute(Qt::WA_NativeWindow, true);
  setAttribute(Qt::WA_OpaquePaintEvent, true);
  setAttribute(Qt::WA_PaintOnScreen, true);

  this->setObjectName("osvrWindow");

  this->setWindowIcon(QIcon(":/images/gazebo.svg"));
  this->setWindowTitle(tr("Gazebo: OSVR"));

  this->renderFrame = new QFrame;
  this->renderFrame->setFrameShape(QFrame::NoFrame);
  this->renderFrame->setSizePolicy(QSizePolicy::Expanding,
                                   QSizePolicy::Expanding);
  this->renderFrame->setContentsMargins(0, 0, 0, 0);
  this->renderFrame->show();

  QVBoxLayout *renderLayout = new QVBoxLayout;
  renderLayout->addWidget(this->renderFrame);
  renderLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(renderLayout);
  this->attachCameraThread = NULL;
}

/////////////////////////////////////////////////
OSVRWindow::~OSVRWindow()
{
  this->scene.reset();
  if (this->attachCameraThread)
    this->attachCameraThread->join();
  delete this->attachCameraThread;
  this->osvrCamera.reset();
}

/////////////////////////////////////////////////
void OSVRWindow::keyPressEvent(QKeyEvent *_event)
{
  // Toggle full screen
  if (_event->key() == Qt::Key_F11)
  {
    if (this->isFullScreen)
      this->showFullScreen();
    else
      this->showNormal();

    this->isFullScreen = !this->isFullScreen;
  }
}

/////////////////////////////////////////////////
void OSVRWindow::resizeEvent(QResizeEvent *_e)
{
  if (!this->scene)
    return;

  if (this->windowId >= 0)
  {
    rendering::RenderEngine::Instance()->GetWindowManager()->Resize(
        this->windowId, _e->size().width(), _e->size().height());
    this->osvrCamera->Resize(_e->size().width(), _e->size().height());
  }
}

/////////////////////////////////////////////////
void OSVRWindow::AttachCameraToVisual()
{
  if (!this->scene)
  {
    gzerr << "OSVRWindow::AttachCameraToVisual(): Scene is NULL" << std::endl;
    return;
  }
  int tries = 0;
  while (!this->scene->GetVisual(this->visualName) && tries < 50)
  {
    common::Time::MSleep(100);
    tries++;
  }

  if (tries >= 50)
  {
    gzerr << "OSVR: visual [" << this->visualName << "] not found. "
          << "OSVR is not attached." << std::endl;
    return;
  }

  this->osvrCamera->AttachToVisual(this->visualName, true, 0, 0);

  ignition::math::Vector3d camPos(0.1, 0, 0);
  ignition::math::Vector3d lookAt(0, 0, 0);
  auto delta = lookAt - camPos;

  double yaw = atan2(delta.Y(), delta.X());

  double pitch = atan2(-delta.Z(),
      sqrt(delta.X()*delta.X() + delta.Y()*delta.Y()));

  this->osvrCamera->SetWorldPose(ignition::math::Pose3d(camPos,
      ignition::math::Quaterniond(0.0, pitch, yaw)));
}

/////////////////////////////////////////////////
bool OSVRWindow::CreateCamera()
{
  this->scene = rendering::get_scene();

  if (!this->scene)
    gzerr << "Unable to create an OSVR camera, scene is NULL" << std::endl;

  this->osvrCamera = this->scene->CreateOSVRCamera("gzosvr_camera");
  return this->osvrCamera->Ready();
}

/////////////////////////////////////////////////
void OSVRWindow::showEvent(QShowEvent *_event)
{
  if (this->osvrCamera)
    this->attachCameraThread = new std::thread(
        std::bind(&OSVRWindow::AttachCameraToVisual, this));

  if (this->windowId == -1)
  {
    this->windowId = rendering::RenderEngine::Instance()->GetWindowManager()->
      CreateWindow(this->GetOgreHandle(), this->width(), this->height());
    if (this->osvrCamera)
    {
      rendering::RenderEngine::Instance()->GetWindowManager()->SetCamera(
          this->windowId, this->osvrCamera);
    }
  }

  QWidget::showEvent(_event);

  this->setFocus();

  QSize winSize;
  winSize.setWidth(1920);
  winSize.setHeight(1080);
  this->resize(winSize);

  // Put the window on the osvr screen
  this->setGeometry(this->xPos, this->yPos, 1920, 1080);

  // Make the window full screen
  //this->isFullScreen = true;
  //this->showFullScreen();
}

//////////////////////////////////////////////////
std::string OSVRWindow::GetOgreHandle() const
{
  std::string ogreHandle;

#if defined(WIN32) || defined(__APPLE__)
  ogreHandle = boost::lexical_cast<std::string>(this->winId());
#else
  QX11Info info = x11Info();
  QWidget *q_parent = dynamic_cast<QWidget*>(this->renderFrame);
  ogreHandle = boost::lexical_cast<std::string>(
      reinterpret_cast<uint64_t>(info.display()));
  ogreHandle += ":";
  ogreHandle += boost::lexical_cast<std::string>(
      static_cast<uint32_t>(info.screen()));
  ogreHandle += ":";
  assert(q_parent);
  ogreHandle += boost::lexical_cast<std::string>(
      static_cast<uint64_t>(q_parent->winId()));
#endif

  return ogreHandle;
}
