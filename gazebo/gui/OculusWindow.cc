/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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

#include "gazebo/gui/OculusWindow.hh"
#include "gazebo/rendering/OculusCamera.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/WindowManager.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
OculusWindow::OculusWindow(int _x, int _y, const std::string &_visual,
    QWidget *_parent)
  : QWidget(_parent), windowId(-1), isFullScreen(false),
    xPos(_x), yPos(_y), visualName(_visual)
{
  setAttribute(Qt::WA_NativeWindow, true);
  setAttribute(Qt::WA_OpaquePaintEvent, true);
  setAttribute(Qt::WA_PaintOnScreen, true);

  this->setObjectName("oculusWindow");

  this->setWindowIcon(QIcon(":/images/gazebo.svg"));
  this->setWindowTitle(tr("Gazebo: Oculus"));

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
OculusWindow::~OculusWindow()
{
  this->scene.reset();
  if (this->attachCameraThread)
    this->attachCameraThread->join();
  delete this->attachCameraThread;
  this->oculusCamera.reset();
}

/////////////////////////////////////////////////
void OculusWindow::keyPressEvent(QKeyEvent *_event)
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
void OculusWindow::resizeEvent(QResizeEvent *_e)
{
  if (!this->scene)
    return;

  if (this->windowId >= 0)
  {
    rendering::RenderEngine::Instance()->GetWindowManager()->Resize(
        this->windowId, _e->size().width(), _e->size().height());
    this->oculusCamera->Resize(_e->size().width(), _e->size().height());
  }
}

/////////////////////////////////////////////////
void OculusWindow::AttachCameraToVisual()
{
  if (!this->scene)
  {
    gzerr << "OculusWindow::AttachCameraToVisual(): Scene is NULL" << std::endl;
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
    gzerr << "Oculus: visual [" << this->visualName << "] not found."
          << "Oculus is not attached." << std::endl;
    return;
  }

  this->oculusCamera->AttachToVisual(this->visualName, true);

  math::Vector3 camPos(0.1, 0, 0);
  math::Vector3 lookAt(0, 0, 0);
  math::Vector3 delta = lookAt - camPos;

  double yaw = atan2(delta.y, delta.x);

  double pitch = atan2(-delta.z, sqrt(delta.x*delta.x + delta.y*delta.y));

  this->oculusCamera->SetWorldPose(math::Pose(
        camPos, math::Vector3(0, pitch, yaw)));
}

/////////////////////////////////////////////////
bool OculusWindow::CreateCamera()
{
  this->scene = rendering::get_scene();

  if (!this->scene)
    gzerr << "Unable to create an oculus camera, scene is NULL" << std::endl;

  this->oculusCamera = this->scene->CreateOculusCamera("gzoculus_camera");
  return this->oculusCamera->Ready();
}

/////////////////////////////////////////////////
void OculusWindow::showEvent(QShowEvent *_event)
{
  if (this->oculusCamera)
    this->attachCameraThread = new boost::thread(
        boost::bind(&OculusWindow::AttachCameraToVisual, this));

  if (this->windowId == -1)
  {
    this->windowId = rendering::RenderEngine::Instance()->GetWindowManager()->
      CreateWindow(this->GetOgreHandle(), this->width(), this->height());
    if (this->oculusCamera)
    {
      rendering::RenderEngine::Instance()->GetWindowManager()->SetCamera(
          this->windowId, this->oculusCamera);
    }
  }

  QWidget::showEvent(_event);

  this->setFocus();

  QSize winSize;
  winSize.setWidth(1280);
  winSize.setHeight(800);
  this->resize(winSize);

  // Put the window on the oculus screen
  this->setGeometry(this->xPos, this->yPos, 1280, 800);

  // Make the window full screen
  this->isFullScreen = true;
  this->showFullScreen();
}

//////////////////////////////////////////////////
std::string OculusWindow::GetOgreHandle() const
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
