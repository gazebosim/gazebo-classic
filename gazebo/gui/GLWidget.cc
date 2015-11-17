/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <math.h>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/math/gzmath.hh"

#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Heightmap.hh"
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/WindowManager.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/OrbitViewController.hh"
#include "gazebo/rendering/FPSViewController.hh"

#include "gazebo/gui/ModelAlign.hh"
#include "gazebo/gui/ModelSnap.hh"
#include "gazebo/gui/ModelManipulator.hh"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/KeyEventHandler.hh"
#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/ModelRightMenu.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GLWidget.hh"

using namespace gazebo;
using namespace gui;

extern bool g_fullscreen;
extern ModelRightMenu *g_modelRightMenu;

/////////////////////////////////////////////////
GLWidget::GLWidget(QWidget *_parent)
  : QWidget(_parent)
{
  this->setObjectName("GLWidget");
  this->state = "select";
  this->copyEntityName = "";
  this->modelEditorEnabled = false;

  this->setFocusPolicy(Qt::StrongFocus);

  this->windowId = -1;

  this->setAttribute(Qt::WA_OpaquePaintEvent, true);
  this->setAttribute(Qt::WA_PaintOnScreen, true);

  this->renderFrame = new QFrame;
  this->renderFrame->setFrameShape(QFrame::NoFrame);
  this->renderFrame->setSizePolicy(QSizePolicy::Expanding,
                                   QSizePolicy::Expanding);
  this->renderFrame->setContentsMargins(0, 0, 0, 0);
  this->renderFrame->show();

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(this->renderFrame);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  this->setLayout(mainLayout);

  this->connections.push_back(
      rendering::Events::ConnectRemoveScene(
        boost::bind(&GLWidget::OnRemoveScene, this, _1)));

  this->connections.push_back(
      gui::Events::ConnectMoveMode(
        boost::bind(&GLWidget::OnMoveMode, this, _1)));

  this->connections.push_back(
      gui::Events::ConnectCreateEntity(
        boost::bind(&GLWidget::OnCreateEntity, this, _1, _2)));

  this->connections.push_back(
      gui::Events::ConnectFPS(
        boost::bind(&GLWidget::OnFPS, this)));

  this->connections.push_back(
      gui::Events::ConnectOrbit(
        boost::bind(&GLWidget::OnOrbit, this)));

  this->connections.push_back(
      gui::Events::ConnectManipMode(
        boost::bind(&GLWidget::OnManipMode, this, _1)));

  this->connections.push_back(
     event::Events::ConnectSetSelectedEntity(
       boost::bind(&GLWidget::OnSetSelectedEntity, this, _1, _2)));

  this->connections.push_back(
      gui::Events::ConnectAlignMode(
        boost::bind(&GLWidget::OnAlignMode, this, _1, _2, _3, _4)));

  this->renderFrame->setMouseTracking(true);
  this->setMouseTracking(true);

  this->entityMaker = NULL;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->modelPub = this->node->Advertise<msgs::Model>("~/model/modify");

  this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");

  // Publishes information about user selections.
  this->selectionPub =
    this->node->Advertise<msgs::Selection>("~/selection");

  this->requestSub = this->node->Subscribe("~/request",
      &GLWidget::OnRequest, this);

  this->installEventFilter(this);
  this->keyModifiers = 0;

  MouseEventHandler::Instance()->AddPressFilter("glwidget",
      boost::bind(&GLWidget::OnMousePress, this, _1));

  MouseEventHandler::Instance()->AddReleaseFilter("glwidget",
      boost::bind(&GLWidget::OnMouseRelease, this, _1));

  MouseEventHandler::Instance()->AddMoveFilter("glwidget",
      boost::bind(&GLWidget::OnMouseMove, this, _1));

  MouseEventHandler::Instance()->AddDoubleClickFilter("glwidget",
      boost::bind(&GLWidget::OnMouseDoubleClick, this, _1));

  connect(g_copyAct, SIGNAL(triggered()), this, SLOT(OnCopy()));
  connect(g_pasteAct, SIGNAL(triggered()), this, SLOT(OnPaste()));

  connect(g_editModelAct, SIGNAL(toggled(bool)), this,
      SLOT(OnModelEditor(bool)));

  // Connect the ortho action
  connect(g_cameraOrthoAct, SIGNAL(triggered()), this,
          SLOT(OnOrtho()));

  // Connect the perspective action
  connect(g_cameraPerspectiveAct, SIGNAL(triggered()), this,
          SLOT(OnPerspective()));

  // Create the scene. This must be done in the constructor so that
  // we can then create a user camera.
  this->scene = rendering::create_scene(gui::get_world(), true);

  if (!this->scene)
  {
    gzerr << "GLWidget could not create a scene. This will likely result "
      << "in a blank screen.\n";
  }
  else
  {
    // This will ultimately create a user camera. We need to create a user
    // camera in the constructor so that communications (such as via the
    // ~/gui topic) can work properly (see MainWindow::OnGUI).
    //
    // All of this means that we must have a GL Context by this point. So,
    // we have to create a dummy 1x1 window in RenderEngine::Load.
    this->OnCreateScene(this->scene->GetName());
  }
}

/////////////////////////////////////////////////
GLWidget::~GLWidget()
{
  MouseEventHandler::Instance()->RemovePressFilter("glwidget");
  MouseEventHandler::Instance()->RemoveReleaseFilter("glwidget");
  MouseEventHandler::Instance()->RemoveMoveFilter("glwidget");
  MouseEventHandler::Instance()->RemoveDoubleClickFilter("glwidget");

  this->connections.clear();
  this->node.reset();
  this->modelPub.reset();
  this->selectionPub.reset();

  ModelManipulator::Instance()->Clear();
  ModelSnap::Instance()->Clear();
  ModelAlign::Instance()->Clear();

  if (this->userCamera)
    this->userCamera->Fini();

  this->userCamera.reset();
  this->scene.reset();
}

/////////////////////////////////////////////////
bool GLWidget::eventFilter(QObject * /*_obj*/, QEvent *_event)
{
  if (_event->type() == QEvent::Enter)
  {
    this->setFocus(Qt::OtherFocusReason);
    return true;
  }

  return false;
}

/////////////////////////////////////////////////
void GLWidget::showEvent(QShowEvent *_event)
{
  // These two functions are most applicable for Linux.
  QApplication::flush();
  QApplication::syncX();

  // Get the window handle in a form that OGRE can use.
  std::string winHandle = this->GetOgreHandle();

  // Create the OGRE render window
  this->windowId = rendering::RenderEngine::Instance()->GetWindowManager()->
    CreateWindow(winHandle, this->width(), this->height());

  // Attach the user camera to the window
  rendering::RenderEngine::Instance()->GetWindowManager()->SetCamera(
      this->windowId, this->userCamera);

  // Let QT continue processing the show event.
  QWidget::showEvent(_event);

  // Grab focus.
  this->setFocus();
}

/////////////////////////////////////////////////
void GLWidget::enterEvent(QEvent * /*_event*/)
{
}

/////////////////////////////////////////////////
void GLWidget::moveEvent(QMoveEvent *_e)
{
  QWidget::moveEvent(_e);

  if (_e->isAccepted() && this->windowId >= 0)
  {
    rendering::RenderEngine::Instance()->GetWindowManager()->Moved(
        this->windowId);
  }
}

/////////////////////////////////////////////////
void GLWidget::paintEvent(QPaintEvent *_e)
{
  rendering::UserCameraPtr cam = gui::get_active_camera();
  if (cam && cam->GetInitialized())
  {
    event::Events::preRender();

    // Tell all the cameras to render
    event::Events::render();

    event::Events::postRender();
  }
  else
  {
    event::Events::preRender();
  }

  this->update();

  _e->accept();
}

/////////////////////////////////////////////////
void GLWidget::resizeEvent(QResizeEvent *_e)
{
  if (this->windowId >= 0)
  {
    rendering::RenderEngine::Instance()->GetWindowManager()->Resize(
        this->windowId, _e->size().width(), _e->size().height());

    if (this->userCamera)
      this->userCamera->Resize(_e->size().width(), _e->size().height());
  }
}

/////////////////////////////////////////////////
void GLWidget::keyPressEvent(QKeyEvent *_event)
{
  if (!this->scene)
    return;

  if (_event->isAutoRepeat() && !KeyEventHandler::Instance()->GetAutoRepeat())
    return;

  this->keyText = _event->text().toStdString();
  this->keyModifiers = _event->modifiers();

  this->keyEvent.key = _event->key();
  this->keyEvent.text = this->keyText;

  // Toggle full screen
  if (_event->key() == Qt::Key_F11)
  {
    g_fullscreen = !g_fullscreen;
    gui::Events::fullScreen(g_fullscreen);
  }

  // Trigger a model delete if the Delete key was pressed, and a model
  // is currently selected.
  if (_event->key() == Qt::Key_Delete &&
      this->selectionLevel == SelectionLevels::MODEL)
  {
    boost::mutex::scoped_lock lock(this->selectedVisMutex);
    while (!this->selectedVisuals.empty())
    {
      std::string name = this->selectedVisuals.back()->GetName();
      int id = this->selectedVisuals.back()->GetId();
      this->selectedVisuals.pop_back();

      // Publish message about visual deselection
      msgs::Selection msg;
      msg.set_id(id);
      msg.set_name(name);
      msg.set_selected(false);
      this->selectionPub->Publish(msg);

      g_deleteAct->Signal(name);
    }
  }

  if (_event->key() == Qt::Key_Escape)
  {
    event::Events::setSelectedEntity("", "normal");
    if (this->state == "make_entity")
    {
      if (this->entityMaker)
        this->entityMaker->Stop();
    }
  }

  this->keyEvent.control =
    this->keyModifiers & Qt::ControlModifier ? true : false;
  this->keyEvent.shift =
    this->keyModifiers & Qt::ShiftModifier ? true : false;
  this->keyEvent.alt =
    this->keyModifiers & Qt::AltModifier ? true : false;

  this->mouseEvent.SetControl(this->keyEvent.control);
  this->mouseEvent.SetShift(this->keyEvent.shift);
  this->mouseEvent.SetAlt(this->keyEvent.alt);

  if (this->mouseEvent.Control())
  {
    if (_event->key() == Qt::Key_C && !this->selectedVisuals.empty()
       && !this->modelEditorEnabled && g_copyAct->isEnabled())
    {
      g_copyAct->trigger();
    }
    else if (_event->key() == Qt::Key_V && !this->copyEntityName.empty()
       && !this->modelEditorEnabled && g_pasteAct->isEnabled())
    {
      g_pasteAct->trigger();
    }
  }

  // Process Key Events
  if (!KeyEventHandler::Instance()->HandlePress(this->keyEvent))
  {
    // model editor exit pop-up message is modal so can block event propagation.
    // So using hotkeys to exit will leave the control variable in a bad state.
    // Manually override and reset the control value.
    if (this->modelEditorEnabled && this->mouseEvent.Control())
      this->mouseEvent.SetControl(false);

    ModelManipulator::Instance()->OnKeyPressEvent(this->keyEvent);
    this->userCamera->HandleKeyPressEvent(this->keyText);
  }
}

/////////////////////////////////////////////////
void GLWidget::keyReleaseEvent(QKeyEvent *_event)
{
  if (!this->scene)
    return;

  // this shouldn't happen, but in case it does...
  if (_event->isAutoRepeat() && !KeyEventHandler::Instance()->GetAutoRepeat())
    return;

  this->keyModifiers = _event->modifiers();

  if (this->keyModifiers & Qt::ControlModifier &&
      _event->key() == Qt::Key_Z)
  {
    this->PopHistory();
  }

  /// Switch between RTS modes
  if (this->keyModifiers == Qt::NoModifier && this->state != "make_entity")
  {
    if (_event->key() == Qt::Key_R && g_rotateAct->isEnabled())
      g_rotateAct->trigger();
    else if (_event->key() == Qt::Key_T && g_translateAct->isEnabled())
      g_translateAct->trigger();
    else if (_event->key() == Qt::Key_S && g_scaleAct->isEnabled())
      g_scaleAct->trigger();
    else if (_event->key() == Qt::Key_N && g_snapAct->isEnabled())
      g_snapAct->trigger();
    else if (_event->key() == Qt::Key_Escape && g_arrowAct->isEnabled())
      g_arrowAct->trigger();
  }

  this->keyEvent.control =
    this->keyModifiers & Qt::ControlModifier ? true : false;
  this->keyEvent.shift =
    this->keyModifiers & Qt::ShiftModifier ? true : false;
  this->keyEvent.alt =
    this->keyModifiers & Qt::AltModifier ? true : false;

  this->mouseEvent.SetControl(this->keyEvent.control);
  this->mouseEvent.SetShift(this->keyEvent.shift);
  this->mouseEvent.SetAlt(this->keyEvent.alt);

  ModelManipulator::Instance()->OnKeyReleaseEvent(this->keyEvent);
  this->keyText = "";

  this->userCamera->HandleKeyReleaseEvent(_event->text().toStdString());

  // Process Key Events
  KeyEventHandler::Instance()->HandleRelease(this->keyEvent);
}

/////////////////////////////////////////////////
void GLWidget::mouseDoubleClickEvent(QMouseEvent *_event)
{
  if (!this->scene)
    return;

  this->mouseEvent.SetPressPos(_event->pos().x(), _event->pos().y());
  this->mouseEvent.SetPrevPos(this->mouseEvent.PressPos());

  /// Set the button which cause the press event
  this->SetMouseEventButton(_event->button());

  this->mouseEvent.SetButtons(common::MouseEvent::NO_BUTTON);
  this->mouseEvent.SetType(common::MouseEvent::PRESS);

  this->SetMouseEventButtons(_event->buttons());

  this->mouseEvent.SetDragging(false);

  // Process Mouse Events
  MouseEventHandler::Instance()->HandleDoubleClick(this->mouseEvent);
}

/////////////////////////////////////////////////
void GLWidget::mousePressEvent(QMouseEvent *_event)
{
  if (!this->scene)
    return;

  this->mouseEvent.SetPressPos(_event->pos().x(), _event->pos().y());
  this->mouseEvent.SetPrevPos(this->mouseEvent.PressPos());

  /// Set the button which cause the press event
  this->SetMouseEventButton(_event->button());

  this->mouseEvent.SetButtons(common::MouseEvent::NO_BUTTON);
  this->mouseEvent.SetType(common::MouseEvent::PRESS);

  this->SetMouseEventButtons(_event->buttons());

  this->mouseEvent.SetDragging(false);

  // Process Mouse Events
  MouseEventHandler::Instance()->HandlePress(this->mouseEvent);
}

/////////////////////////////////////////////////
bool GLWidget::OnMousePress(const common::MouseEvent & /*_event*/)
{
  if (this->state == "make_entity")
    this->OnMousePressMakeEntity();
  else if (this->state == "select")
    this->OnMousePressNormal();
  else if (this->state == "translate" || this->state == "rotate"
      || this->state == "scale")
    ModelManipulator::Instance()->OnMousePressEvent(this->mouseEvent);
  else if (this->state == "snap")
    ModelSnap::Instance()->OnMousePressEvent(this->mouseEvent);

  return true;
}

/////////////////////////////////////////////////
bool GLWidget::OnMouseRelease(const common::MouseEvent & /*_event*/)
{
  if (this->state == "make_entity")
    this->OnMouseReleaseMakeEntity();
  else if (this->state == "select")
    this->OnMouseReleaseNormal();
  else if (this->state == "translate" || this->state == "rotate"
      || this->state == "scale")
    ModelManipulator::Instance()->OnMouseReleaseEvent(this->mouseEvent);
  else if (this->state == "snap")
    ModelSnap::Instance()->OnMouseReleaseEvent(this->mouseEvent);

  return true;
}

/////////////////////////////////////////////////
bool GLWidget::OnMouseMove(const common::MouseEvent & /*_event*/)
{
  // Update the view depending on the current GUI state
  if (this->state == "make_entity")
    this->OnMouseMoveMakeEntity();
  else if (this->state == "select")
    this->OnMouseMoveNormal();
  else if (this->state == "translate" || this->state == "rotate"
      || this->state == "scale")
    ModelManipulator::Instance()->OnMouseMoveEvent(this->mouseEvent);
  else if (this->state == "snap")
    ModelSnap::Instance()->OnMouseMoveEvent(this->mouseEvent);

  return true;
}

/////////////////////////////////////////////////
bool GLWidget::OnMouseDoubleClick(const common::MouseEvent & /*_event*/)
{
  rendering::VisualPtr vis =
    this->userCamera->GetVisual(this->mouseEvent.Pos());

  if (vis && gui::get_entity_id(vis->GetRootVisual()->GetName()))
  {
    if (vis->IsPlane())
    {
      math::Pose pose;
      ignition::math::Pose3d camPose;
      camPose = this->userCamera->WorldPose();
      if (this->scene->GetFirstContact(this->userCamera,
            this->mouseEvent.Pos(), pose.pos))
      {
        this->userCamera->SetFocalPoint(pose.pos);
        ignition::math::Vector3d dir = pose.pos.Ign() - camPose.Pos();
        pose.pos = camPose.Pos() + (dir * 0.8);
        pose.rot = this->userCamera->WorldRotation();
        this->userCamera->MoveToPosition(pose.Ign(), 0.5);
      }
    }
    else
    {
      this->userCamera->MoveToVisual(vis);
    }
  }
  else
    return false;

  return true;
}

/////////////////////////////////////////////////
void GLWidget::OnMousePressNormal()
{
  if (!this->userCamera)
    return;

  rendering::VisualPtr vis = this->userCamera->GetVisual(
      this->mouseEvent.Pos());

  this->userCamera->HandleMouseEvent(this->mouseEvent);
}

/////////////////////////////////////////////////
void GLWidget::OnMousePressMakeEntity()
{
  if (!this->userCamera)
    return;

  // Allow camera orbiting while making an entity
  this->userCamera->HandleMouseEvent(this->mouseEvent);
}

/////////////////////////////////////////////////
void GLWidget::wheelEvent(QWheelEvent *_event)
{
  if (!this->scene)
    return;

  if (_event->delta() > 0)
    this->mouseEvent.SetScroll(this->mouseEvent.Scroll().X(), -1);
  else
    this->mouseEvent.SetScroll(this->mouseEvent.Scroll().X(), 1);

  this->mouseEvent.SetType(common::MouseEvent::SCROLL);

  this->SetMouseEventButtons(_event->buttons());

  this->userCamera->HandleMouseEvent(this->mouseEvent);
}

/////////////////////////////////////////////////
void GLWidget::mouseMoveEvent(QMouseEvent *_event)
{
  if (!this->scene)
    return;

  this->setFocus(Qt::MouseFocusReason);

  this->mouseEvent.SetPos(_event->pos().x(), _event->pos().y());
  this->mouseEvent.SetType(common::MouseEvent::MOVE);

  this->SetMouseEventButtons(_event->buttons());

  if (_event->buttons())
    this->mouseEvent.SetDragging(true);
  else
    this->mouseEvent.SetDragging(false);

  // Process Mouse Events
  MouseEventHandler::Instance()->HandleMove(this->mouseEvent);

  this->mouseEvent.SetPrevPos(this->mouseEvent.Pos());
}

/////////////////////////////////////////////////
void GLWidget::OnMouseMoveMakeEntity()
{
  if (!this->userCamera)
    return;

  if (this->entityMaker)
  {
    // Allow camera orbiting while inserting a new model
    if (this->mouseEvent.Dragging())
      this->userCamera->HandleMouseEvent(this->mouseEvent);
    else
      this->entityMaker->OnMouseMove(this->mouseEvent);
  }
}

/////////////////////////////////////////////////
void GLWidget::OnMouseMoveNormal()
{
  if (!this->userCamera)
    return;

  rendering::VisualPtr vis = this->userCamera->GetVisual(
      this->mouseEvent.Pos());

  if (vis && !vis->IsPlane())
    QApplication::setOverrideCursor(Qt::PointingHandCursor);
  else
    QApplication::setOverrideCursor(Qt::ArrowCursor);

  this->userCamera->HandleMouseEvent(this->mouseEvent);
}

/////////////////////////////////////////////////
void GLWidget::mouseReleaseEvent(QMouseEvent *_event)
{
  if (!this->scene)
    return;

  this->mouseEvent.SetPos(_event->pos().x(), _event->pos().y());
  this->mouseEvent.SetPrevPos(this->mouseEvent.Pos());

  this->SetMouseEventButton(_event->button());

  this->mouseEvent.SetButtons(common::MouseEvent::NO_BUTTON);
  this->mouseEvent.SetType(common::MouseEvent::RELEASE);

  this->SetMouseEventButtons(_event->buttons());

  // Process Mouse Events
  MouseEventHandler::Instance()->HandleRelease(this->mouseEvent);

  emit clicked();
}

//////////////////////////////////////////////////
void GLWidget::OnMouseReleaseMakeEntity()
{
  if (this->entityMaker)
    this->entityMaker->OnMouseRelease(this->mouseEvent);
}

//////////////////////////////////////////////////
void GLWidget::OnMouseReleaseNormal()
{
  if (!this->userCamera)
    return;

  if (!this->mouseEvent.Dragging())
  {
    rendering::VisualPtr vis =
      this->userCamera->GetVisual(this->mouseEvent.Pos());

    if (vis)
    {
      rendering::VisualPtr selectVis;
      rendering::VisualPtr linkVis = vis->GetParent();
      if (!linkVis)
      {
        gzerr << "Link visual not found, this should not happen." << std::endl;
        return;
      }
      rendering::VisualPtr modelVis = vis->GetRootVisual();
      if (!modelVis)
      {
        gzerr << "Model visual not found, this should not happen." << std::endl;
        return;
      }

      // Flags to check if we should select a link or a model
      bool rightButton = (this->mouseEvent.Button() ==
          common::MouseEvent::RIGHT);
      bool modelHighlighted = modelVis->GetHighlighted();
      int linkCount = 0;
      bool linkHighlighted = false;
      for (unsigned int i = 0; i < modelVis->GetChildCount(); ++i)
      {
        // Find out if there's only one link in the model
        uint32_t flags = modelVis->GetChild(i)->GetVisibilityFlags();
        if ((flags != GZ_VISIBILITY_ALL) && (flags & GZ_VISIBILITY_GUI))
        {
          continue;
        }
        linkCount++;

        // A link from the same model is currently selected
        if (modelVis->GetChild(i)->GetHighlighted())
        {
          linkHighlighted = true;
        }
      }

      // Select link
      if (linkCount > 1 && !this->mouseEvent.Control() &&
          ((modelHighlighted && !rightButton) || linkHighlighted))
      {
        selectVis = linkVis;
        this->selectionLevel = SelectionLevels::LINK;
      }
      // Select model
      else
      {
        // Can't select a link and a model at the same time
        if (this->selectionLevel == SelectionLevels::LINK)
          this->DeselectAllVisuals();

        selectVis = modelVis;
        this->selectionLevel = SelectionLevels::MODEL;
      }
      this->SetSelectedVisual(selectVis);
      event::Events::setSelectedEntity(selectVis->GetName(), "normal");

      // Open context menu
      if (rightButton)
      {
        if (selectVis == modelVis)
        {
          g_modelRightMenu->Run(selectVis->GetName(), QCursor::pos(),
              ModelRightMenu::EntityTypes::MODEL);
        }
        else if (selectVis == linkVis)
        {
          g_modelRightMenu->Run(selectVis->GetName(), QCursor::pos(),
              ModelRightMenu::EntityTypes::LINK);
        }
      }
    }
    else
      this->SetSelectedVisual(rendering::VisualPtr());
  }

  this->userCamera->HandleMouseEvent(this->mouseEvent);
}

//////////////////////////////////////////////////
void GLWidget::ViewScene(rendering::ScenePtr _scene)
{
  // The user camera name.
  std::string cameraBaseName = "gzclient_camera";
  std::string cameraName = cameraBaseName;

  transport::ConnectionPtr connection = transport::connectToMaster();
  if (connection)
  {
    std::string topicData;
    msgs::Packet packet;
    msgs::Request request;
    msgs::GzString_V topics;

    request.set_id(0);
    request.set_request("get_topics");
    connection->EnqueueMsg(msgs::Package("request", request), true);
    connection->Read(topicData);

    packet.ParseFromString(topicData);
    topics.ParseFromString(packet.serialized_data());

    std::string searchable;
    for (int i = 0; i < topics.data_size(); ++i)
      searchable += topics.data(i);

    int i = 0;
    while (searchable.find(cameraName) != std::string::npos)
    {
      cameraName = cameraBaseName + boost::lexical_cast<std::string>(++i);
    }
  }
  else
    gzerr << "Unable to connect to a running Gazebo master.\n";

  if (_scene->GetUserCameraCount() == 0)
  {
    this->userCamera = _scene->CreateUserCamera(cameraName,
        gazebo::gui::getINIProperty<int>("rendering.stereo", 0));
  }
  else
  {
    this->userCamera = _scene->GetUserCamera(0);
  }

  gui::set_active_camera(this->userCamera);
  this->scene = _scene;

  math::Vector3 camPos(5, -5, 2);
  math::Vector3 lookAt(0, 0, 0);
  math::Vector3 delta = lookAt - camPos;

  double yaw = atan2(delta.y, delta.x);

  double pitch = atan2(-delta.z, sqrt(delta.x*delta.x + delta.y*delta.y));
  this->userCamera->SetDefaultPose(math::Pose(camPos,
        math::Vector3(0, pitch, yaw)));
}

/////////////////////////////////////////////////
rendering::ScenePtr GLWidget::GetScene() const
{
  return this->scene;
}

/////////////////////////////////////////////////
void GLWidget::Clear()
{
  gui::clear_active_camera();
  this->userCamera.reset();
  this->scene.reset();
  this->SetSelectedVisual(rendering::VisualPtr());
  this->keyModifiers = 0;
}

//////////////////////////////////////////////////
rendering::UserCameraPtr GLWidget::GetCamera() const
{
  return this->userCamera;
}

//////////////////////////////////////////////////
std::string GLWidget::GetOgreHandle() const
{
  std::string ogreHandle;

#if defined(__APPLE__)
  ogreHandle = std::to_string(this->winId());
#elif defined(WIN32)
  ogreHandle = std::to_string(
      reinterpret_cast<uint32_t>(this->renderFrame->winId()));
#else
  QX11Info info = x11Info();
  QWidget *q_parent = dynamic_cast<QWidget*>(this->renderFrame);
  GZ_ASSERT(q_parent, "q_parent is null");

  ogreHandle =
    std::to_string(reinterpret_cast<uint64_t>(info.display())) + ":" +
    std::to_string(static_cast<uint32_t>(info.screen())) + ":" +
    std::to_string(static_cast<uint64_t>(q_parent->winId()));
#endif

  return ogreHandle;
}

/////////////////////////////////////////////////
void GLWidget::OnRemoveScene(const std::string &_name)
{
  if (this->scene && this->scene->GetName() == _name)
  {
    this->Clear();
  }
}

/////////////////////////////////////////////////
void GLWidget::OnCreateScene(const std::string &_name)
{
  this->SetSelectedVisual(rendering::VisualPtr());

  this->ViewScene(rendering::get_scene(_name));

  ModelManipulator::Instance()->Init();
  ModelSnap::Instance()->Init();
  ModelAlign::Instance()->Init();
}

/////////////////////////////////////////////////
void GLWidget::OnMoveMode(bool _mode)
{
  if (_mode)
  {
    this->entityMaker = NULL;
    this->state = "select";
  }
}

/////////////////////////////////////////////////
void GLWidget::OnCreateEntity(const std::string &_type,
                              const std::string &_data)
{
  if (this->modelEditorEnabled)
    return;

  this->ClearSelection();

  if (this->entityMaker)
    this->entityMaker->Stop();

  this->entityMaker = NULL;

  if (_type == "box")
  {
    if (this->modelMaker.InitSimpleShape(ModelMaker::SimpleShapes::BOX))
      this->entityMaker = &this->modelMaker;
  }
  else if (_type == "sphere")
  {
    if (this->modelMaker.InitSimpleShape(ModelMaker::SimpleShapes::SPHERE))
      this->entityMaker = &this->modelMaker;
  }
  else if (_type == "cylinder")
  {
    if (this->modelMaker.InitSimpleShape(ModelMaker::SimpleShapes::CYLINDER))
      this->entityMaker = &this->modelMaker;
  }
  else if (_type == "model" && !_data.empty())
  {
    if (this->modelMaker.InitFromFile(_data))
      this->entityMaker = &this->modelMaker;
  }
  else if (_type == "pointlight")
    this->entityMaker =  &this->pointLightMaker;
  else if (_type == "spotlight")
    this->entityMaker =  &this->spotLightMaker;
  else if (_type == "directionallight")
    this->entityMaker =  &this->directionalLightMaker;

  if (this->entityMaker)
  {
    gui::Events::manipMode("make_entity");
    // TODO: change the cursor to a cross
    this->entityMaker->Start();
  }
  else
  {
    this->state = "select";
    // TODO: make sure cursor state stays at the default
  }
}

/////////////////////////////////////////////////
void GLWidget::OnFPS()
{
  this->userCamera->SetViewController(
      rendering::FPSViewController::GetTypeString());
}
/////////////////////////////////////////////////
void GLWidget::OnOrbit()
{
  this->userCamera->SetViewController(
      rendering::OrbitViewController::GetTypeString());
}

/////////////////////////////////////////////////
std::vector<rendering::VisualPtr> GLWidget::SelectedVisuals() const
{
  return this->selectedVisuals;
}

/////////////////////////////////////////////////
void GLWidget::SetSelectedVisual(rendering::VisualPtr _vis)
{
  // deselect all if not in multi-selection mode.
  if (!this->mouseEvent.Control())
  {
    this->DeselectAllVisuals();
  }

  boost::mutex::scoped_lock lock(this->selectedVisMutex);

  msgs::Selection msg;

  if (_vis && !_vis->IsPlane())
  {
    if (_vis == _vis->GetRootVisual())
      this->selectionLevel = SelectionLevels::MODEL;
    else
      this->selectionLevel = SelectionLevels::LINK;

    _vis->SetHighlighted(true);

    // enable multi-selection if control is pressed
    if (this->selectedVisuals.empty() || this->mouseEvent.Control())
    {
      std::vector<rendering::VisualPtr>::iterator it =
        std::find(this->selectedVisuals.begin(),
            this->selectedVisuals.end(), _vis);
      if (it == this->selectedVisuals.end())
        this->selectedVisuals.push_back(_vis);
      else
      {
        // if element already exists, move to the back of vector
        rendering::VisualPtr vis = (*it);
        this->selectedVisuals.erase(it);
        this->selectedVisuals.push_back(vis);
      }
    }
    g_copyAct->setEnabled(true);

    msg.set_id(_vis->GetId());
    msg.set_name(_vis->GetName());
    msg.set_selected(true);
    this->selectionPub->Publish(msg);
  }
  else if (g_copyAct)
  {
    g_copyAct->setEnabled(false);
  }

  if (g_alignAct)
    g_alignAct->setEnabled(this->selectedVisuals.size() > 1);
}

/////////////////////////////////////////////////
void GLWidget::DeselectAllVisuals()
{
  boost::mutex::scoped_lock lock(this->selectedVisMutex);

  msgs::Selection msg;
  for (unsigned int i = 0; i < this->selectedVisuals.size(); ++i)
  {
    this->selectedVisuals[i]->SetHighlighted(false);
    msg.set_id(this->selectedVisuals[i]->GetId());
    msg.set_name(this->selectedVisuals[i]->GetName());
    msg.set_selected(false);
    this->selectionPub->Publish(msg);
  }
  this->selectedVisuals.clear();
}

/////////////////////////////////////////////////
void GLWidget::OnManipMode(const std::string &_mode)
{
  this->state = _mode;

  if (!this->selectedVisuals.empty())
  {
    boost::mutex::scoped_lock lock(this->selectedVisMutex);
    ModelManipulator::Instance()->SetAttachedVisual(
        this->selectedVisuals.back());
  }

  ModelManipulator::Instance()->SetManipulationMode(_mode);
  ModelSnap::Instance()->Reset();

  if (this->state != "select")
  {
    boost::mutex::scoped_lock lock(this->selectedVisMutex);
    // only support multi-model selection in select mode for now.
    // deselect 0 to n-1 models.
    if (this->selectedVisuals.size() > 1)
    {
      for (std::vector<rendering::VisualPtr>::iterator it
          = this->selectedVisuals.begin(); it != --this->selectedVisuals.end();)
      {
         (*it)->SetHighlighted(false);
         it = this->selectedVisuals.erase(it);
      }
    }
  }
}

/////////////////////////////////////////////////
void GLWidget::OnCopy()
{
  boost::mutex::scoped_lock lock(this->selectedVisMutex);
  if (!this->selectedVisuals.empty() && !this->modelEditorEnabled)
  {
    this->Copy(this->selectedVisuals.back()->GetName());
  }
}

/////////////////////////////////////////////////
void GLWidget::OnPaste()
{
  if (!this->modelEditorEnabled)
    this->Paste(this->copyEntityName);
}

/////////////////////////////////////////////////
void GLWidget::Copy(const std::string &_name)
{
  this->copyEntityName = _name;
  g_pasteAct->setEnabled(true);
}

/////////////////////////////////////////////////
void GLWidget::Paste(const std::string &_name)
{
  if (!_name.empty())
  {
    bool isModel = false;
    bool isLight = false;
    if (scene->GetLight(_name))
      isLight = true;
    else if (scene->GetVisual(_name))
      isModel = true;

    if (isLight || isModel)
    {
      this->ClearSelection();
      if (this->entityMaker)
        this->entityMaker->Stop();

      if (isLight && this->lightMaker.InitFromLight(_name))
      {
        this->entityMaker = &this->lightMaker;
        this->entityMaker->Start();
        // this makes the entity appear at the mouse cursor
        this->entityMaker->OnMouseMove(this->mouseEvent);
        gui::Events::manipMode("make_entity");
      }
      else if (isModel && this->modelMaker.InitFromModel(_name))
      {
        this->entityMaker = &this->modelMaker;
        this->entityMaker->Start();
        // this makes the entity appear at the mouse cursor
        this->entityMaker->OnMouseMove(this->mouseEvent);
        gui::Events::manipMode("make_entity");
      }
    }
  }
}

/////////////////////////////////////////////////
void GLWidget::ClearSelection()
{
  this->SetSelectedVisual(rendering::VisualPtr());

  this->scene->SelectVisual("", "normal");
}

/////////////////////////////////////////////////
void GLWidget::OnSetSelectedEntity(const std::string &_name,
                                   const std::string &_mode)
{
  if (!_name.empty())
  {
    std::string name = _name;
    boost::replace_first(name, gui::get_world()+"::", "");

    rendering::VisualPtr selection = this->scene->GetVisual(name);

    std::vector<rendering::VisualPtr>::iterator it =
      std::find(this->selectedVisuals.begin(),
          this->selectedVisuals.end(), selection);

    // Shortcircuit the case when GLWidget already selected the visual.
    if (it == this->selectedVisuals.end() || _name != (*it)->GetName())
    {
      this->SetSelectedVisual(selection);
      this->scene->SelectVisual(name, _mode);
    }
  }
  else if (!this->selectedVisuals.empty())
  {
    this->SetSelectedVisual(rendering::VisualPtr());
    this->scene->SelectVisual("", _mode);
  }
}

/////////////////////////////////////////////////
void GLWidget::PushHistory(const std::string &_visName, const math::Pose &_pose)
{
  if (this->moveHistory.empty() ||
      this->moveHistory.back().first != _visName ||
      this->moveHistory.back().second != _pose)
  {
    this->moveHistory.push_back(std::make_pair(_visName, _pose));
  }
}

/////////////////////////////////////////////////
void GLWidget::PopHistory()
{
  if (!this->moveHistory.empty())
  {
    msgs::Model msg;
    msg.set_id(gui::get_entity_id(this->moveHistory.back().first));
    msg.set_name(this->moveHistory.back().first);

    msgs::Set(msg.mutable_pose(), this->moveHistory.back().second.Ign());
    this->scene->GetVisual(this->moveHistory.back().first)->SetWorldPose(
        this->moveHistory.back().second.Ign());

    this->modelPub->Publish(msg);

    this->moveHistory.pop_back();
  }
}

/////////////////////////////////////////////////
void GLWidget::OnRequest(ConstRequestPtr &_msg)
{
  if (_msg->request() == "entity_delete")
  {
    boost::mutex::scoped_lock lock(this->selectedVisMutex);
    if (!this->selectedVisuals.empty())
    {
      for (std::vector<rendering::VisualPtr>::iterator it =
          this->selectedVisuals.begin(); it != this->selectedVisuals.end();
          ++it)
      {
        if ((*it)->GetName() == _msg->data())
        {
          ModelManipulator::Instance()->Detach();
          this->selectedVisuals.erase(it);
          break;
        }
      }
    }

    if (this->copyEntityName == _msg->data())
    {
      this->copyEntityName = "";
      g_pasteAct->setEnabled(false);
    }
  }
}

/////////////////////////////////////////////////
void GLWidget::OnAlignMode(const std::string &_axis, const std::string &_config,
    const std::string &_target, bool _preview)
{
  ModelAlign::Instance()->AlignVisuals(this->selectedVisuals, _axis, _config,
      _target, !_preview);
}

/////////////////////////////////////////////////
void GLWidget::OnModelEditor(bool _checked)
{
  this->modelEditorEnabled = _checked;
  g_arrowAct->trigger();
  event::Events::setSelectedEntity("", "normal");

  // Manually deselect, in case the editor was opened with Ctrl
  this->DeselectAllVisuals();
}

/////////////////////////////////////////////////
void GLWidget::OnOrtho()
{
  // Disable view control options when in ortho projection
  g_fpsAct->setEnabled(false);
  g_orbitAct->setEnabled(false);
  this->userCamera->SetProjectionType("orthographic");
}

/////////////////////////////////////////////////
void GLWidget::OnPerspective()
{
  // Enable view control options when in perspective projection
  g_fpsAct->setEnabled(true);
  g_orbitAct->setEnabled(true);
  this->userCamera->SetProjectionType("perspective");
}

/////////////////////////////////////////////////
QPaintEngine *GLWidget::paintEngine() const
{
  return NULL;
}

/////////////////////////////////////////////////
void GLWidget::SetMouseEventButtons(const Qt::MouseButtons &_buttons)
{
  if (_buttons & Qt::LeftButton)
  {
    this->mouseEvent.SetButtons(
        this->mouseEvent.Buttons() | common::MouseEvent::LEFT);
  }
  else
  {
    this->mouseEvent.SetButtons(this->mouseEvent.Buttons() | 0x0);
  }

  if (_buttons & Qt::RightButton)
  {
    this->mouseEvent.SetButtons(
        this->mouseEvent.Buttons() | common::MouseEvent::RIGHT);
  }
  else
  {
    this->mouseEvent.SetButtons(this->mouseEvent.Buttons() | 0x0);
  }

  if (_buttons & Qt::MidButton)
  {
    this->mouseEvent.SetButtons(
        this->mouseEvent.Buttons() | common::MouseEvent::MIDDLE);
  }
  else
  {
    this->mouseEvent.SetButtons(this->mouseEvent.Buttons() | 0x0);
  }
}

/////////////////////////////////////////////////
void GLWidget::SetMouseEventButton(const Qt::MouseButton &_button)
{
  if (_button == Qt::LeftButton)
    this->mouseEvent.SetButton(common::MouseEvent::LEFT);
  else if (_button == Qt::RightButton)
    this->mouseEvent.SetButton(common::MouseEvent::RIGHT);
  else if (_button == Qt::MidButton)
    this->mouseEvent.SetButton(common::MouseEvent::MIDDLE);
}
