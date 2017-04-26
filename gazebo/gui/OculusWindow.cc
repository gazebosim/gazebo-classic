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

#include <ignition/math/Matrix4.hh>
#include <ignition/math/Pose3.hh>

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

  this->setFocusPolicy(Qt::StrongFocus);
  this->setMouseTracking(true);
  this->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  this->attachCameraThread = nullptr;
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
    gzerr << "OculusWindow::AttachCameraToVisual(): Scene is null" << std::endl;
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

  this->oculusCamera->AttachToVisual(this->visualName, true, 0, 0);

  ignition::math::Vector3d camPos(0.1, 0, 0);
  ignition::math::Vector3d lookAt(0, 0, 0);
  auto mat = ignition::math::Matrix4d::LookAt(camPos, lookAt);

  this->oculusCamera->SetWorldPose(mat.Pose());
}

/////////////////////////////////////////////////
bool OculusWindow::CreateCamera()
{
  this->scene = rendering::get_scene();

  if (!this->scene)
    gzerr << "Unable to create an oculus camera, scene is null" << std::endl;

  this->oculusCamera = this->scene->CreateOculusCamera("gzoculus_camera");
  return this->oculusCamera->Ready();
}

/////////////////////////////////////////////////
void OculusWindow::showEvent(QShowEvent *_event)
{
  if (this->oculusCamera)
    this->attachCameraThread = new std::thread(
        std::bind(&OculusWindow::AttachCameraToVisual, this));

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
  return std::to_string(static_cast<uint64_t>(this->winId()));
}
