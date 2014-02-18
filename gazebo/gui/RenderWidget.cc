/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GLWidget.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/TimePanel.hh"
#include "gazebo/gui/RenderWidget.hh"
#include "gazebo/gui/building/BuildingEditorWidget.hh"

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

  QFrame *toolFrame = new QFrame;
  toolFrame->setObjectName("toolFrame");
  toolFrame->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Minimum);

  QToolBar *toolbar = new QToolBar;
  QHBoxLayout *toolLayout = new QHBoxLayout;
  toolLayout->setContentsMargins(0, 0, 0, 0);

  QActionGroup *actionGroup = new QActionGroup(toolFrame);
  actionGroup->addAction(g_arrowAct);
  actionGroup->addAction(g_translateAct);
  actionGroup->addAction(g_rotateAct);
  actionGroup->addAction(g_scaleAct);

  toolbar->addAction(g_arrowAct);
  toolbar->addAction(g_translateAct);
  toolbar->addAction(g_rotateAct);
  toolbar->addAction(g_scaleAct);

  toolbar->addSeparator();
  toolbar->addAction(g_boxCreateAct);
  toolbar->addAction(g_sphereCreateAct);
  toolbar->addAction(g_cylinderCreateAct);
  toolbar->addSeparator();
  toolbar->addAction(g_pointLghtCreateAct);
  toolbar->addAction(g_spotLghtCreateAct);
  toolbar->addAction(g_dirLghtCreateAct);
  toolbar->addSeparator();
  toolbar->addAction(g_screenshotAct);

  toolLayout->addSpacing(10);
  toolLayout->addWidget(toolbar);
  toolFrame->setLayout(toolLayout);

  this->glWidget = new GLWidget(this->mainFrame);
  rendering::ScenePtr scene = rendering::create_scene(gui::get_world(), true);

  this->buildingEditorWidget = new BuildingEditorWidget(this);
  this->buildingEditorWidget->setSizePolicy(QSizePolicy::Expanding,
      QSizePolicy::Expanding);
  this->buildingEditorWidget->hide();

  this->msgOverlayLabel = new QLabel(this->glWidget);
  this->msgOverlayLabel->setStyleSheet(
      "QLabel { background-color : white; color : gray; }");
  this->msgOverlayLabel->setVisible(false);

  QHBoxLayout *bottomPanelLayout = new QHBoxLayout;

  TimePanel *timePanel = new TimePanel(this);

  this->bottomFrame = new QFrame;
  this->bottomFrame->setObjectName("renderBottomFrame");
  this->bottomFrame->setSizePolicy(QSizePolicy::Expanding,
      QSizePolicy::Minimum);

  bottomPanelLayout->addWidget(timePanel, 0);
  bottomPanelLayout->setSpacing(0);
  bottomPanelLayout->setContentsMargins(0, 0, 0, 0);
  this->bottomFrame->setLayout(bottomPanelLayout);

  QFrame *render3DFrame = new QFrame;
  render3DFrame->setObjectName("render3DFrame");
  QVBoxLayout *render3DLayout = new QVBoxLayout;
  render3DLayout->addWidget(toolFrame);
  render3DLayout->addWidget(this->glWidget);
  render3DLayout->setContentsMargins(0, 0, 0, 0);
  render3DLayout->setSpacing(0);
  render3DFrame->setLayout(render3DLayout);

  QSplitter *splitter = new QSplitter(this);
  splitter->addWidget(this->buildingEditorWidget);
  splitter->addWidget(render3DFrame);
  QList<int> sizes;
  sizes.push_back(300);
  sizes.push_back(300);
  splitter->setSizes(sizes);
  splitter->setStretchFactor(0, 1);
  splitter->setStretchFactor(1, 1);
  splitter->setOrientation(Qt::Vertical);

  frameLayout->addWidget(splitter);
  frameLayout->addWidget(this->bottomFrame);
  frameLayout->setContentsMargins(0, 0, 0, 0);
  frameLayout->setSpacing(0);

  this->mainFrame->setLayout(frameLayout);
  this->mainFrame->layout()->setContentsMargins(0, 0, 0, 0);

  mainLayout->addWidget(this->mainFrame);

  this->setLayout(mainLayout);
  this->layout()->setContentsMargins(0, 0, 0, 0);

  this->timer = new QTimer(this);
  connect(this->timer, SIGNAL(timeout()), this, SLOT(update()));
  this->timer->start(44);

  this->connections.push_back(
      gui::Events::ConnectFollow(
        boost::bind(&RenderWidget::OnFollow, this, _1)));
}

/////////////////////////////////////////////////
RenderWidget::~RenderWidget()
{
  delete this->glWidget;
}

/////////////////////////////////////////////////
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

  if (!cam || !cam->GetInitialized())
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

/////////////////////////////////////////////////
void RenderWidget::ShowEditor(bool _show)
{
  if (_show)
  {
    this->buildingEditorWidget->show();
    this->baseOverlayMsg = "Building is View Only";
    this->OnClearOverlayMsg();
    this->bottomFrame->hide();
  }
  else
  {
    this->buildingEditorWidget->hide();
    this->baseOverlayMsg = "";
    this->OnClearOverlayMsg();
    this->bottomFrame->show();
  }
}

/////////////////////////////////////////////////
void RenderWidget::RemoveScene(const std::string &_name)
{
  this->clear = true;
  this->clearName = _name;
}

/////////////////////////////////////////////////
void RenderWidget::CreateScene(const std::string &_name)
{
  this->create = true;
  this->createName = _name;
}

/////////////////////////////////////////////////
void RenderWidget::DisplayOverlayMsg(const std::string &_msg, int _duration)
{
  std::string msg = this->baseOverlayMsg.empty() ? _msg
      : this->baseOverlayMsg + "\n" + _msg;
  this->msgOverlayLabel->setText(tr(msg.c_str()));
  if (msg.empty())
  {
    this->msgOverlayLabel->setVisible(false);
    return;
  }
  this->msgOverlayLabel->resize(
      this->msgOverlayLabel->fontMetrics().width(tr(msg.c_str())),
      this->msgOverlayLabel->fontMetrics().height());
  this->msgOverlayLabel->setVisible(true);

  if (_duration > 0)
    QTimer::singleShot(_duration, this, SLOT(OnClearOverlayMsg()));
}

/////////////////////////////////////////////////
std::string RenderWidget::GetOverlayMsg() const
{
  return this->msgOverlayLabel->text().toStdString();
}

/////////////////////////////////////////////////
void RenderWidget::OnClearOverlayMsg()
{
  this->DisplayOverlayMsg("");
}

/////////////////////////////////////////////////
void RenderWidget::OnFollow(const std::string &_modelName)
{
  if (_modelName.empty())
  {
    g_translateAct->setEnabled(true);
    g_rotateAct->setEnabled(true);
  }
  else
  {
    g_translateAct->setEnabled(false);
    g_rotateAct->setEnabled(false);
  }
}
