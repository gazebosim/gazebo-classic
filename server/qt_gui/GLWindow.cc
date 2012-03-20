/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#include <math.h>
#include <stdint.h>
#include <boost/lexical_cast.hpp>
#include "Global.hh"
#include "UserCamera.hh"
#include "GLWindow.hh"

using namespace gazebo;

/////////////////////////////////////////////////
GLWindow::GLWindow(QWidget *_parent)
  : QWidget(_parent), renderFrame(0)
{
  this->userCamera = NULL;
  setMinimumSize(640, 480);
  setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);

  // This mouse offset is a hack. The glwindow window is not properly sized
  // when first created....
  this->setFocusPolicy(Qt::StrongFocus);

  this->windowId = -1;

  setAttribute(Qt::WA_OpaquePaintEvent, true);
  setAttribute(Qt::WA_PaintOnScreen, true);

  this->renderFrame = new QFrame;
  this->renderFrame->setLineWidth(1);
  this->renderFrame->setFrameShadow(QFrame::Sunken);
  this->renderFrame->setFrameShape(QFrame::Box);
  this->renderFrame->setSizePolicy(QSizePolicy::Expanding,
                                   QSizePolicy::Expanding);
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(this->renderFrame);
  this->setLayout(mainLayout);

  this->renderFrame->setMouseTracking(true);
  this->setMouseTracking(true);

  this->installEventFilter(this);
  this->renderFrame->show();
}

/////////////////////////////////////////////////
GLWindow::~GLWindow()
{
}

/////////////////////////////////////////////////
void GLWindow::Init()
{
  this->userCamera->Init();
  this->FrontView();
}

/////////////////////////////////////////////////
void GLWindow::CreateCameras()
{
  this->renderFrame->show();
  // Create the default camera.
  this->userCamera = new UserCamera( this );
  this->userCamera->Load(NULL);
  this->userCamera->SetFOV(1.04719755);

}

/////////////////////////////////////////////////
bool GLWindow::eventFilter(QObject * /*_obj*/, QEvent *_event)
{
  if (_event->type() == QEvent::Enter)
  {
    this->setFocus(Qt::OtherFocusReason);
    return true;
  }

  return false;
}

/////////////////////////////////////////////////
void GLWindow::showEvent(QShowEvent *_event)
{
  QApplication::flush();
  
 // this->windowId = rendering::WindowManager::Instance()->CreateWindow(
 //     this->GetOgreHandle(), this->width(), this->height());

  QWidget::showEvent(_event);

  //if (this->userCamera)
  //  rendering::WindowManager::Instance()->SetCamera(this->windowId,
  //                                                  this->userCamera);
  this->setFocus();
}


/////////////////////////////////////////////////
void GLWindow::paintEvent(QPaintEvent *_e)
{
  _e->accept();
}

/////////////////////////////////////////////////
void GLWindow::resizeEvent(QResizeEvent *_e)
{
  if (this->userCamera)
    this->userCamera->Resize(_e->size().width(), _e->size().height());
}

//////////////////////////////////////////////////
std::string GLWindow::GetOgreHandle() const
{
  std::string ogreHandle;

#ifdef WIN32
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
  ogreHandle += boost::lexical_cast<std::string>(
      static_cast<uint64_t>(q_parent->winId()));
#endif

  return ogreHandle;
}

void GLWindow::keyReleaseEvent(QKeyEvent *_event)
{
}

void GLWindow::FrontView()
{
  Pose3d pose;
  pose.pos.x = -5;
  pose.pos.y = 0;
  pose.pos.z = 2;
  pose.rot.SetFromEuler(Vector3(0, 0, 0));

  this->userCamera->SetWorldPose(pose);
}

void GLWindow::SideView()
{
  Pose3d pose;
  pose.pos.x = 0;
  pose.pos.y = -5;
  pose.pos.z = 2;
  pose.rot.SetFromEuler(Vector3(0, 0, DTOR(90)));

  this->userCamera->SetWorldPose(pose);
}

void GLWindow::AngledView()
{
  Pose3d pose;
  pose.pos.x = -5;
  pose.pos.y = -5;
  pose.pos.z = 5;
  pose.rot.SetFromEuler(Vector3(0, DTOR(45), DTOR(45)));

  this->userCamera->SetWorldPose(pose);
}
