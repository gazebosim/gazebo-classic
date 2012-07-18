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
#include <iomanip>

#include "rendering/UserCamera.hh"
#include "rendering/Rendering.hh"
#include "rendering/Scene.hh"

#include "gui/Gui.hh"
#include "gui/GLWidget.hh"
#include "gui/GuiEvents.hh"
#include "gui/RenderWidget.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
RenderWidget::RenderWidget(QWidget *_parent)
  : QWidget(_parent)
{
  this->setObjectName("renderWidget");
  this->show();

  this->clear = false;
  this->create = false;

  QVBoxLayout *mainLayout = new QVBoxLayout;
  this->mainFrame = new QFrame;
  this->mainFrame->setFrameShape(QFrame::NoFrame);
  this->mainFrame->show();

  QVBoxLayout *frameLayout = new QVBoxLayout;
  frameLayout->setContentsMargins(0, 0, 0, 0);

  this->glWidget = new GLWidget(this->mainFrame);
  rendering::ScenePtr scene = rendering::create_scene(gui::get_world(), true);

  /*this->xPosEdit = new QLineEdit;
  this->xPosEdit->setReadOnly(true);
  this->xPosEdit->setValidator(new QDoubleValidator(this->xPosEdit));
  this->xPosEdit->setInputMethodHints(Qt::ImhDigitsOnly);

  this->yPosEdit = new QLineEdit;
  this->yPosEdit->setReadOnly(true);
  this->yPosEdit->setValidator(new QDoubleValidator(this->yPosEdit));
  this->yPosEdit->setInputMethodHints(Qt::ImhDigitsOnly);

  this->zPosEdit = new QLineEdit;
  this->zPosEdit->setReadOnly(true);
  this->zPosEdit->setValidator(new QDoubleValidator(this->zPosEdit));
  this->zPosEdit->setInputMethodHints(Qt::ImhDigitsOnly);

  this->rollEdit = new QLineEdit;
  this->rollEdit->setReadOnly(true);
  this->rollEdit->setValidator(new QDoubleValidator(this->rollEdit));
  this->rollEdit->setInputMethodHints(Qt::ImhDigitsOnly);

  this->pitchEdit = new QLineEdit;
  this->pitchEdit->setReadOnly(true);
  this->pitchEdit->setValidator(new QDoubleValidator(this->pitchEdit));
  this->pitchEdit->setInputMethodHints(Qt::ImhDigitsOnly);

  this->yawEdit = new QLineEdit;
  this->yawEdit->setReadOnly(true);
  this->yawEdit->setValidator(new QDoubleValidator(this->yawEdit));
  this->yawEdit->setInputMethodHints(Qt::ImhDigitsOnly);
  */

  /*this->fpsEdit = new QLineEdit;
  this->fpsEdit->setReadOnly(true);
  this->fpsEdit->setFixedWidth(50);

  this->trianglesEdit = new QLineEdit;
  this->trianglesEdit->setReadOnly(true);
  this->trianglesEdit->setFixedWidth(80);
  */

  // this->xyzLabel = new QLabel(tr("XYZ:"));
  // this->rpyLabel = new QLabel(tr("RPY:"));

  /*
  bottomBarLayout = new QHBoxLayout;
  bottomBarLayout->addWidget(this->xyzLabel);
  bottomBarLayout->addWidget(this->xPosEdit);
  bottomBarLayout->addWidget(this->yPosEdit);
  bottomBarLayout->addWidget(this->zPosEdit);

  bottomBarLayout->addItem(new QSpacerItem(10, 20));
  bottomBarLayout->addWidget(this->rpyLabel);
  bottomBarLayout->addWidget(this->rollEdit);
  bottomBarLayout->addWidget(this->pitchEdit);
  bottomBarLayout->addWidget(this->yawEdit);
  */

  //bottomBarLayout->addItem(
  //    new QSpacerItem(40, 20, QSizePolicy::Expanding, QSizePolicy::Minimum));
  //bottomBarLayout->addSpacing(10);

  frameLayout->addWidget(this->glWidget);
  //frameLayout->addLayout(bottomBarLayout);

  this->mainFrame->setLayout(frameLayout);
  this->mainFrame->layout()->setContentsMargins(0, 0, 0, 0);

  mainLayout->addWidget(this->mainFrame);

  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  this->timer = new QTimer(this);
  connect(this->timer, SIGNAL(timeout()), this, SLOT(update()));
  this->timer->start(44);

  this->connections.push_back(
      gui::Events::ConnectFullScreen(
        boost::bind(&RenderWidget::OnFullScreen, this, _1)));
}

/////////////////////////////////////////////////
RenderWidget::~RenderWidget()
{
  delete this->glWidget;
}

/////////////////////////////////////////////////
void RenderWidget::OnFullScreen(bool &_value)
{
  if (_value)
  {
    this->mainFrame->layout()->removeItem(this->bottomBarLayout);
    this->mainFrame->setLineWidth(0);
    this->mainFrame->layout()->setContentsMargins(0, 0, 0, 0);
    this->glWidget->layout()->setContentsMargins(0, 0, 0, 0);
    this->layout()->setContentsMargins(0, 0, 0, 0);
    // this->xyzLabel->hide();
    // this->rpyLabel->hide();

    // this->xPosEdit->hide();
    // this->yPosEdit->hide();
    // this->zPosEdit->hide();

    // this->rollEdit->hide();
    // this->pitchEdit->hide();
    // this->yawEdit->hide();
  }
  else
  {
    this->mainFrame->layout()->addItem(this->bottomBarLayout);
    this->mainFrame->setLineWidth(1);
    this->mainFrame->layout()->setContentsMargins(4, 4, 4, 4);
    // this->xyzLabel->show();
    // this->rpyLabel->show();

    // this->xPosEdit->show();
    // this->yPosEdit->show();
    // this->zPosEdit->show();

    // this->rollEdit->show();
    // this->pitchEdit->show();
    // this->yawEdit->show();
  }
}

void RenderWidget::update()
{
  if (this->clear)
  {
    rendering::remove_scene(this->clearName);
    this->clear = false;
    return;
  }
  else if (this->create)
  {
    rendering::create_scene(this->createName, true);
    this->create = false;
    return;
  }

  rendering::UserCameraPtr cam = this->glWidget->GetCamera();

  if (!cam || !cam->IsInitialized())
  {
    event::Events::preRender();
    return;
  }

  // float fps = cam->GetAvgFPS();
  // int triangleCount = cam->GetTriangleCount();
  // math::Pose pose = cam->GetWorldPose();

  // std::ostringstream stream;

  // stream << std::fixed << std::setprecision(2) << pose.pos.x;
  // this->xPosEdit->setText(tr(stream.str().c_str()));
  // stream.str("");

  // stream << std::fixed << std::setprecision(2) << pose.pos.y;
  // this->yPosEdit->setText(tr(stream.str().c_str()));
  // stream.str("");

  // stream << std::fixed << std::setprecision(2) << pose.pos.z;
  // this->zPosEdit->setText(tr(stream.str().c_str()));
  // stream.str("");

  // stream << std::fixed << std::setprecision(2)
  //        << GZ_RTOD(pose.rot.GetAsEuler().x);
  // this->rollEdit->setText(tr(stream.str().c_str()));
  // stream.str("");

  // stream << std::fixed << std::setprecision(2)
  //        << GZ_RTOD(pose.rot.GetAsEuler().y);
  // this->pitchEdit->setText(tr(stream.str().c_str()));
  // stream.str("");

  // stream << std::fixed << std::setprecision(2)
  //        << GZ_RTOD(pose.rot.GetAsEuler().z);
  // this->yawEdit->setText(tr(stream.str().c_str()));
  // stream.str("");

  /*stream << std::fixed << std::setprecision(1) << fps;
  this->fpsEdit->setText(tr(stream.str().c_str()));
  stream.str("");

  stream << std::fixed << std::setprecision(2) << triangleCount;
  this->trianglesEdit->setText(tr(stream.str().c_str()));
  */

  this->glWidget->update();
}

void RenderWidget::RemoveScene(const std::string &_name)
{
  this->clear = true;
  this->clearName = _name;
}

void RenderWidget::CreateScene(const std::string &_name)
{
  this->create = true;
  this->createName = _name;
}


