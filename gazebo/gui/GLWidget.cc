/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#include <functional>
#include <boost/algorithm/string.hpp>
#include <math.h>

#include <ignition/math/Pose3.hh>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/FPSViewController.hh"
#include "gazebo/rendering/Heightmap.hh"
#include "gazebo/rendering/OrbitViewController.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/WindowManager.hh"
#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GLWidget.hh"
#include "gazebo/gui/GLWidgetPrivate.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/KeyEventHandler.hh"
#include "gazebo/gui/ModelAlign.hh"
#include "gazebo/gui/ModelManipulator.hh"
#include "gazebo/gui/ModelRightMenu.hh"
#include "gazebo/gui/ModelSnap.hh"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/transport/transport.hh"

using namespace gazebo;
using namespace gui;

extern bool g_fullscreen;
extern ModelRightMenu *g_modelRightMenu;

/////////////////////////////////////////////////
GLWidget::GLWidget(QWidget *_parent)
  : QWidget(_parent),
    dataPtr(new GLWidgetPrivate())
{
  this->setObjectName("GLWidget");
  this->dataPtr->state = "select";
  this->dataPtr->copyEntityName = "";
  this->dataPtr->modelEditorEnabled = false;

  this->dataPtr->updateTimer = new QTimer(this);
  connect(this->dataPtr->updateTimer, SIGNAL(timeout()),
  this, SLOT(update()));

  this->setFocusPolicy(Qt::StrongFocus);

  this->dataPtr->windowId = -1;

  this->setAttribute(Qt::WA_OpaquePaintEvent, true);
  this->setAttribute(Qt::WA_PaintOnScreen, true);

  this->dataPtr->renderFrame = new QFrame;
  this->dataPtr->renderFrame->setFrameShape(QFrame::NoFrame);
  this->dataPtr->renderFrame->setSizePolicy(QSizePolicy::Expanding,
                                   QSizePolicy::Expanding);
  this->dataPtr->renderFrame->setContentsMargins(0, 0, 0, 0);
  this->dataPtr->renderFrame->show();

  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(this->dataPtr->renderFrame);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  this->setLayout(mainLayout);

  this->dataPtr->connections.push_back(
      rendering::Events::ConnectRemoveScene(
        std::bind(&GLWidget::OnRemoveScene, this, std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      gui::Events::ConnectMoveMode(
        std::bind(&GLWidget::OnMoveMode, this, std::placeholders::_1)));

  this->dataPtr->connections.push_back(
      gui::Events::ConnectCreateEntity(
        std::bind(&GLWidget::OnCreateEntity, this, std::placeholders::_1,
          std::placeholders::_2)));

  this->dataPtr->connections.push_back(
      gui::Events::ConnectFPS(
        std::bind(&GLWidget::OnFPS, this)));

  this->dataPtr->connections.push_back(
      gui::Events::ConnectOrbit(
        std::bind(&GLWidget::OnOrbit, this)));

  this->dataPtr->connections.push_back(
      gui::Events::ConnectManipMode(
        std::bind(&GLWidget::OnManipMode, this, std::placeholders::_1)));

  this->dataPtr->connections.push_back(
    event::Events::ConnectSetSelectedEntity(
      std::bind(&GLWidget::OnSetSelectedEntity, this, std::placeholders::_1,
        std::placeholders::_2)));

  this->dataPtr->connections.push_back(
      gui::Events::ConnectAlignMode(
        std::bind(&GLWidget::OnAlignMode, this, std::placeholders::_1,
          std::placeholders::_2, std::placeholders::_3,
          std::placeholders::_4, std::placeholders::_5)));

  this->dataPtr->renderFrame->setMouseTracking(true);
  this->setMouseTracking(true);

  this->dataPtr->entityMaker = NULL;

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();

  // Publishes information about user selections.
  this->dataPtr->selectionPub =
    this->dataPtr->node->Advertise<msgs::Selection>("~/selection");

  this->dataPtr->requestSub = this->dataPtr->node->Subscribe("~/request",
      &GLWidget::OnRequest, this);

  this->installEventFilter(this);
  this->dataPtr->keyModifiers = 0;

  MouseEventHandler::Instance()->AddPressFilter("glwidget",
      std::bind(&GLWidget::OnMousePress, this, std::placeholders::_1));

  MouseEventHandler::Instance()->AddReleaseFilter("glwidget",
      std::bind(&GLWidget::OnMouseRelease, this, std::placeholders::_1));

  MouseEventHandler::Instance()->AddMoveFilter("glwidget",
      std::bind(&GLWidget::OnMouseMove, this, std::placeholders::_1));

  MouseEventHandler::Instance()->AddDoubleClickFilter("glwidget",
      std::bind(&GLWidget::OnMouseDoubleClick, this, std::placeholders::_1));

  if (g_copyAct)
    connect(g_copyAct, SIGNAL(triggered()), this, SLOT(OnCopy()));
  if (g_pasteAct)
    connect(g_pasteAct, SIGNAL(triggered()), this, SLOT(OnPaste()));
  if (g_editModelAct)
  {
    connect(g_editModelAct, SIGNAL(toggled(bool)), this,
        SLOT(OnModelEditor(bool)));
  }
  // Connect the ortho action
  if (g_cameraOrthoAct)
  {
    connect(g_cameraOrthoAct, SIGNAL(triggered()), this,
            SLOT(OnOrtho()));
  }
  if (g_cameraPerspectiveAct)
  {
    // Connect the perspective action
    connect(g_cameraPerspectiveAct, SIGNAL(triggered()), this,
            SLOT(OnPerspective()));
  }

  // Create the scene. This must be done in the constructor so that
  // we can then create a user camera.
  this->dataPtr->scene = rendering::create_scene(gui::get_world(), true);

  if (!this->dataPtr->scene)
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
    this->OnCreateScene(this->dataPtr->scene->Name());
  }
}

/////////////////////////////////////////////////
GLWidget::~GLWidget()
{
  MouseEventHandler::Instance()->RemovePressFilter("glwidget");
  MouseEventHandler::Instance()->RemoveReleaseFilter("glwidget");
  MouseEventHandler::Instance()->RemoveMoveFilter("glwidget");
  MouseEventHandler::Instance()->RemoveDoubleClickFilter("glwidget");

  this->dataPtr->requestSub.reset();
  this->dataPtr->selectionPub.reset();
  this->dataPtr->node->Fini();
  this->dataPtr->node.reset();

  this->dataPtr->connections.clear();

  ModelManipulator::Instance()->Clear();
  ModelSnap::Instance()->Clear();
  ModelAlign::Instance()->Clear();

  if (this->dataPtr->userCamera)
    this->dataPtr->userCamera->Fini();

  this->dataPtr->userCamera.reset();
  this->dataPtr->scene.reset();
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

  if (this->dataPtr->windowId <=0)
  {
    // Get the window handle in a form that OGRE can use.
    std::string winHandle = this->OgreHandle();

    // Create the OGRE render window
    this->dataPtr->windowId =
      rendering::RenderEngine::Instance()->GetWindowManager()->
        CreateWindow(winHandle, this->width(), this->height());

    // Attach the user camera to the window
    rendering::RenderEngine::Instance()->GetWindowManager()->SetCamera(
        this->dataPtr->windowId, this->dataPtr->userCamera);
  }

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

  if (_e->isAccepted() && this->dataPtr->windowId >= 0)
  {
    rendering::RenderEngine::Instance()->GetWindowManager()->Moved(
        this->dataPtr->windowId);
  }
}

/////////////////////////////////////////////////
void GLWidget::paintEvent(QPaintEvent *_e)
{
  rendering::UserCameraPtr cam = gui::get_active_camera();
  if (cam && cam->Initialized())
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

  _e->accept();
}

/////////////////////////////////////////////////
void GLWidget::resizeEvent(QResizeEvent *_e)
{
  if (this->dataPtr->windowId >= 0)
  {
    rendering::RenderEngine::Instance()->GetWindowManager()->Resize(
        this->dataPtr->windowId, _e->size().width(), _e->size().height());

    if (this->dataPtr->userCamera)
    {
      this->dataPtr->userCamera->Resize(
          _e->size().width(), _e->size().height());
    }
  }
}

/////////////////////////////////////////////////
void GLWidget::keyPressEvent(QKeyEvent *_event)
{
  if (!this->dataPtr->scene)
    return;

  if (_event->isAutoRepeat() && !KeyEventHandler::Instance()->AutoRepeat())
    return;

  this->dataPtr->keyText = _event->text().toStdString();
  this->dataPtr->keyModifiers = _event->modifiers();

  this->dataPtr->keyEvent.key = _event->key();
  this->dataPtr->keyEvent.text = this->dataPtr->keyText;

  // Toggle full screen
  if (_event->key() == Qt::Key_F11)
  {
    g_fullscreen = !g_fullscreen;
    gui::Events::fullScreen(g_fullscreen);
  }

  // Trigger a model delete if the Delete key was pressed, and a model
  // is currently selected.
  if (_event->key() == Qt::Key_Delete &&
      this->dataPtr->selectionLevel == SelectionLevels::MODEL)
  {
    ModelManipulator::Instance()->Detach();
    std::lock_guard<std::mutex> lock(this->dataPtr->selectedVisMutex);
    while (!this->dataPtr->selectedVisuals.empty())
    {
      std::string name = this->dataPtr->selectedVisuals.back()->GetName();
      int id = this->dataPtr->selectedVisuals.back()->GetId();
      this->dataPtr->selectedVisuals.pop_back();

      // Publish message about visual deselection
      msgs::Selection msg;
      msg.set_id(id);
      msg.set_name(name);
      msg.set_selected(false);
      this->dataPtr->selectionPub->Publish(msg);

      g_deleteAct->Signal(name);
    }
  }

  if (_event->key() == Qt::Key_Escape)
  {
    event::Events::setSelectedEntity("", "normal");
    if (this->dataPtr->state == "make_entity")
    {
      if (this->dataPtr->entityMaker)
        this->dataPtr->entityMaker->Stop();
    }
  }

  /// Switch between RTS modes
  if (this->dataPtr->keyModifiers == Qt::NoModifier &&
      this->dataPtr->state != "make_entity")
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

  this->dataPtr->keyEvent.control =
    (this->dataPtr->keyModifiers & Qt::ControlModifier) ? true : false;
  this->dataPtr->keyEvent.shift =
    (this->dataPtr->keyModifiers & Qt::ShiftModifier) ? true : false;
  this->dataPtr->keyEvent.alt =
    (this->dataPtr->keyModifiers & Qt::AltModifier) ? true : false;

  this->dataPtr->mouseEvent.SetControl(this->dataPtr->keyEvent.control);
  this->dataPtr->mouseEvent.SetShift(this->dataPtr->keyEvent.shift);
  this->dataPtr->mouseEvent.SetAlt(this->dataPtr->keyEvent.alt);

  if (this->dataPtr->mouseEvent.Control())
  {
    if (_event->key() == Qt::Key_C && !this->dataPtr->selectedVisuals.empty()
       && !this->dataPtr->modelEditorEnabled && g_copyAct->isEnabled())
    {
      g_copyAct->trigger();
    }
    else if (_event->key() == Qt::Key_V &&
             !this->dataPtr->copyEntityName.empty() &&
             !this->dataPtr->modelEditorEnabled && g_pasteAct->isEnabled())
    {
      g_pasteAct->trigger();
    }
  }

  // Process Key Events
  if (!KeyEventHandler::Instance()->HandlePress(this->dataPtr->keyEvent))
  {
    // model editor exit pop-up message is modal so can block event propagation.
    // So using hotkeys to exit will leave the control variable in a bad state.
    // Manually override and reset the control value.
    if (this->dataPtr->modelEditorEnabled &&
        this->dataPtr->mouseEvent.Control())
    {
      this->dataPtr->mouseEvent.SetControl(false);
    }

    ModelManipulator::Instance()->OnKeyPressEvent(this->dataPtr->keyEvent);
    this->dataPtr->userCamera->HandleKeyPressEvent(this->dataPtr->keyText);
  }
}

/////////////////////////////////////////////////
void GLWidget::keyReleaseEvent(QKeyEvent *_event)
{
  if (!this->dataPtr->scene)
    return;

  // this shouldn't happen, but in case it does...
  if (_event->isAutoRepeat() && !KeyEventHandler::Instance()->AutoRepeat())
    return;

  this->dataPtr->keyModifiers = _event->modifiers();

  this->dataPtr->keyEvent.control =
    (this->dataPtr->keyModifiers & Qt::ControlModifier) ? true : false;
  this->dataPtr->keyEvent.shift =
    (this->dataPtr->keyModifiers & Qt::ShiftModifier) ? true : false;
  this->dataPtr->keyEvent.alt =
    (this->dataPtr->keyModifiers & Qt::AltModifier) ? true : false;

  this->dataPtr->mouseEvent.SetControl(this->dataPtr->keyEvent.control);
  this->dataPtr->mouseEvent.SetShift(this->dataPtr->keyEvent.shift);
  this->dataPtr->mouseEvent.SetAlt(this->dataPtr->keyEvent.alt);

  ModelManipulator::Instance()->OnKeyReleaseEvent(this->dataPtr->keyEvent);
  this->dataPtr->keyText = "";

  this->dataPtr->userCamera->HandleKeyReleaseEvent(
      _event->text().toStdString());

  // Process Key Events
  KeyEventHandler::Instance()->HandleRelease(this->dataPtr->keyEvent);
}

/////////////////////////////////////////////////
void GLWidget::mouseDoubleClickEvent(QMouseEvent *_event)
{
  if (!this->dataPtr->scene)
    return;

  this->dataPtr->mouseEvent.SetPressPos(_event->pos().x(), _event->pos().y());
  this->dataPtr->mouseEvent.SetPrevPos(this->dataPtr->mouseEvent.PressPos());

  /// Set the button which cause the press event
  this->SetMouseEventButton(_event->button());

  this->dataPtr->mouseEvent.SetButtons(common::MouseEvent::NO_BUTTON);
  this->dataPtr->mouseEvent.SetType(common::MouseEvent::PRESS);

  this->SetMouseEventButtons(_event->buttons());

  this->dataPtr->mouseEvent.SetDragging(false);

  // Process Mouse Events
  MouseEventHandler::Instance()->HandleDoubleClick(this->dataPtr->mouseEvent);
}

/////////////////////////////////////////////////
void GLWidget::mousePressEvent(QMouseEvent *_event)
{
  if (!this->dataPtr->scene)
    return;

  this->dataPtr->mouseEvent.SetPressPos(_event->pos().x(), _event->pos().y());
  this->dataPtr->mouseEvent.SetPrevPos(this->dataPtr->mouseEvent.PressPos());

  /// Set the button which cause the press event
  this->SetMouseEventButton(_event->button());

  this->dataPtr->mouseEvent.SetButtons(common::MouseEvent::NO_BUTTON);
  this->dataPtr->mouseEvent.SetType(common::MouseEvent::PRESS);

  this->SetMouseEventButtons(_event->buttons());

  this->dataPtr->mouseEvent.SetDragging(false);

  // Process Mouse Events
  MouseEventHandler::Instance()->HandlePress(this->dataPtr->mouseEvent);
}

/////////////////////////////////////////////////
bool GLWidget::OnMousePress(const common::MouseEvent & /*_event*/)
{
  if (this->dataPtr->state == "make_entity")
    this->OnMousePressMakeEntity();
  else if (this->dataPtr->state == "select")
    this->OnMousePressNormal();
  else if (this->dataPtr->state == "translate" ||
           this->dataPtr->state == "rotate"    ||
           this->dataPtr->state == "scale")
  {
    ModelManipulator::Instance()->OnMousePressEvent(this->dataPtr->mouseEvent);
  }
  else if (this->dataPtr->state == "snap")
    ModelSnap::Instance()->OnMousePressEvent(this->dataPtr->mouseEvent);

  return true;
}

/////////////////////////////////////////////////
bool GLWidget::OnMouseRelease(const common::MouseEvent & /*_event*/)
{
  if (this->dataPtr->state == "make_entity")
    this->OnMouseReleaseMakeEntity();
  else if (this->dataPtr->state == "select")
    this->OnMouseReleaseNormal();
  else if (this->dataPtr->state == "translate" ||
           this->dataPtr->state == "rotate"    ||
           this->dataPtr->state == "scale")
  {
    ModelManipulator::Instance()->OnMouseReleaseEvent(
        this->dataPtr->mouseEvent);
  }
  else if (this->dataPtr->state == "snap")
    ModelSnap::Instance()->OnMouseReleaseEvent(this->dataPtr->mouseEvent);

  return true;
}

/////////////////////////////////////////////////
bool GLWidget::OnMouseMove(const common::MouseEvent & /*_event*/)
{
  // Update the view depending on the current GUI state
  if (this->dataPtr->state == "select")
  {
    this->OnMouseMoveNormal();
  }
  else if (this->dataPtr->state == "translate" ||
           this->dataPtr->state == "rotate"    ||
           this->dataPtr->state == "scale")
  {
    ModelManipulator::Instance()->OnMouseMoveEvent(this->dataPtr->mouseEvent);
  }
  else if (this->dataPtr->state == "make_entity")
  {
    this->OnMouseMoveMakeEntity();
  }
  else if (this->dataPtr->state == "snap")
  {
    ModelSnap::Instance()->OnMouseMoveEvent(this->dataPtr->mouseEvent);
  }

  return true;
}

/////////////////////////////////////////////////
bool GLWidget::OnMouseDoubleClick(const common::MouseEvent & /*_event*/)
{
  rendering::VisualPtr vis =
    this->dataPtr->userCamera->GetVisual(this->dataPtr->mouseEvent.Pos());

  if (vis && gui::get_entity_id(vis->GetRootVisual()->GetName()))
  {
    if (vis->IsPlane())
    {
      ignition::math::Pose3d pose;
      ignition::math::Pose3d camPose;
      camPose = this->dataPtr->userCamera->WorldPose();
      if (this->dataPtr->scene->FirstContact(this->dataPtr->userCamera,
            this->dataPtr->mouseEvent.Pos(), pose.Pos()))
      {
        this->dataPtr->userCamera->SetFocalPoint(pose.Pos());
        ignition::math::Vector3d dir = pose.Pos() - camPose.Pos();
        pose.Pos() = camPose.Pos() + (dir * 0.8);
        pose.Rot() = this->dataPtr->userCamera->WorldRotation();
        this->dataPtr->userCamera->MoveToPosition(pose, 0.5);
      }
    }
    else
    {
      this->dataPtr->userCamera->MoveToVisual(vis);
    }
  }
  else
    return false;

  return true;
}

/////////////////////////////////////////////////
void GLWidget::OnMousePressNormal()
{
  if (!this->dataPtr->userCamera)
    return;

  rendering::VisualPtr vis = this->dataPtr->userCamera->GetVisual(
      this->dataPtr->mouseEvent.Pos());

  this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
}

/////////////////////////////////////////////////
void GLWidget::OnMousePressMakeEntity()
{
  if (!this->dataPtr->userCamera)
    return;

  // Allow camera orbiting while making an entity
  this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
}

/////////////////////////////////////////////////
void GLWidget::wheelEvent(QWheelEvent *_event)
{
  if (!this->dataPtr->scene)
    return;

  if (_event->delta() > 0)
  {
    this->dataPtr->mouseEvent.SetScroll(
        this->dataPtr->mouseEvent.Scroll().X(), -1);
  }
  else
  {
    this->dataPtr->mouseEvent.SetScroll(
        this->dataPtr->mouseEvent.Scroll().X(), 1);
  }

  this->dataPtr->mouseEvent.SetType(common::MouseEvent::SCROLL);

  this->SetMouseEventButtons(_event->buttons());

  this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
}

/////////////////////////////////////////////////
void GLWidget::mouseMoveEvent(QMouseEvent *_event)
{
  if (!this->dataPtr->scene)
    return;

  this->setFocus(Qt::MouseFocusReason);

  this->dataPtr->mouseEvent.SetPos(_event->pos().x(), _event->pos().y());
  this->dataPtr->mouseEvent.SetType(common::MouseEvent::MOVE);

  this->SetMouseEventButtons(_event->buttons());

  if (_event->buttons())
    this->dataPtr->mouseEvent.SetDragging(true);
  else
    this->dataPtr->mouseEvent.SetDragging(false);

  // Process Mouse Events
  MouseEventHandler::Instance()->HandleMove(this->dataPtr->mouseEvent);

  this->dataPtr->mouseEvent.SetPrevPos(this->dataPtr->mouseEvent.Pos());
}

/////////////////////////////////////////////////
void GLWidget::OnMouseMoveMakeEntity()
{
  if (!this->dataPtr->userCamera)
    return;

  if (this->dataPtr->entityMaker)
  {
    // Allow camera orbiting while inserting a new model
    if (this->dataPtr->mouseEvent.Dragging())
      this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
    else
      this->dataPtr->entityMaker->OnMouseMove(this->dataPtr->mouseEvent);
  }
}

/////////////////////////////////////////////////
void GLWidget::OnMouseMoveNormal()
{
  if (!this->dataPtr->userCamera)
    return;

  rendering::VisualPtr vis = this->dataPtr->userCamera->GetVisual(
      this->dataPtr->mouseEvent.Pos());

  if (vis && !vis->IsPlane())
    QApplication::setOverrideCursor(Qt::PointingHandCursor);
  else
    QApplication::setOverrideCursor(Qt::ArrowCursor);

  this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
}

/////////////////////////////////////////////////
void GLWidget::mouseReleaseEvent(QMouseEvent *_event)
{
  if (!this->dataPtr->scene)
    return;

  this->dataPtr->mouseEvent.SetPos(_event->pos().x(), _event->pos().y());
  this->dataPtr->mouseEvent.SetPrevPos(this->dataPtr->mouseEvent.Pos());

  this->SetMouseEventButton(_event->button());

  this->dataPtr->mouseEvent.SetButtons(common::MouseEvent::NO_BUTTON);
  this->dataPtr->mouseEvent.SetType(common::MouseEvent::RELEASE);

  this->SetMouseEventButtons(_event->buttons());

  // Process Mouse Events
  MouseEventHandler::Instance()->HandleRelease(this->dataPtr->mouseEvent);

  emit clicked();
}

//////////////////////////////////////////////////
void GLWidget::OnMouseReleaseMakeEntity()
{
  if (this->dataPtr->entityMaker)
    this->dataPtr->entityMaker->OnMouseRelease(this->dataPtr->mouseEvent);
}

//////////////////////////////////////////////////
void GLWidget::OnMouseReleaseNormal()
{
  if (!this->dataPtr->userCamera)
    return;

  if (!this->dataPtr->mouseEvent.Dragging())
  {
    rendering::VisualPtr vis =
      this->dataPtr->userCamera->GetVisual(this->dataPtr->mouseEvent.Pos());

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
      bool rightButton = (this->dataPtr->mouseEvent.Button() ==
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
      if (linkCount > 1 && !this->dataPtr->mouseEvent.Control() &&
          ((modelHighlighted && !rightButton) || linkHighlighted))
      {
        selectVis = linkVis;
        this->dataPtr->selectionLevel = SelectionLevels::LINK;
      }
      // Select model
      else
      {
        // Can't select a link and a model at the same time
        if (this->dataPtr->selectionLevel == SelectionLevels::LINK)
          this->DeselectAllVisuals();

        selectVis = modelVis;
        this->dataPtr->selectionLevel = SelectionLevels::MODEL;
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

  this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
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

  if (_scene->UserCameraCount() == 0)
  {
    this->dataPtr->userCamera = _scene->CreateUserCamera(cameraName,
        gazebo::gui::getINIProperty<int>("rendering.stereo", 0));
  }
  else
  {
    this->dataPtr->userCamera = _scene->GetUserCamera(0);
  }

  gui::set_active_camera(this->dataPtr->userCamera);
  this->dataPtr->scene = _scene;

  ignition::math::Vector3d camPos(5, -5, 2);
  ignition::math::Vector3d lookAt(0, 0, 0);
  auto delta = lookAt - camPos;

  double yaw = atan2(delta.Y(), delta.X());

  double pitch = atan2(-delta.Z(),
      sqrt(delta.X()*delta.X() + delta.Y()*delta.Y()));
  this->dataPtr->userCamera->SetDefaultPose(ignition::math::Pose3d(camPos,
        ignition::math::Quaterniond(0, pitch, yaw)));

  // Update at the camera's update rate
  this->dataPtr->updateTimer->start(
      static_cast<int>(
        std::round(1000.0 / this->dataPtr->userCamera->RenderRate())));
}

/////////////////////////////////////////////////
rendering::ScenePtr GLWidget::Scene() const
{
  return this->dataPtr->scene;
}

/////////////////////////////////////////////////
void GLWidget::Clear()
{
  gui::clear_active_camera();
  this->dataPtr->userCamera.reset();
  this->dataPtr->scene.reset();
  this->SetSelectedVisual(rendering::VisualPtr());
  this->dataPtr->keyModifiers = 0;
}

//////////////////////////////////////////////////
rendering::UserCameraPtr GLWidget::Camera() const
{
  return this->dataPtr->userCamera;
}

//////////////////////////////////////////////////
std::string GLWidget::OgreHandle() const
{
  std::string ogreHandle;

#if defined(__APPLE__)
  ogreHandle = std::to_string(this->winId());
#elif defined(WIN32)
  ogreHandle = std::to_string(
      reinterpret_cast<uint32_t>(this->dataPtr->renderFrame->winId()));
#else
  QX11Info info = x11Info();
  QWidget *q_parent = dynamic_cast<QWidget*>(this->dataPtr->renderFrame);
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
  if (this->dataPtr->scene && this->dataPtr->scene->Name() == _name)
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
    this->dataPtr->entityMaker = NULL;
    this->dataPtr->state = "select";
  }
}

/////////////////////////////////////////////////
void GLWidget::OnCreateEntity(const std::string &_type,
                              const std::string &_data)
{
  if (this->dataPtr->modelEditorEnabled)
    return;

  this->ClearSelection();

  if (this->dataPtr->entityMaker)
    this->dataPtr->entityMaker->Stop();

  this->dataPtr->entityMaker = nullptr;

  if (_type == "box")
  {
    if (this->dataPtr->modelMaker.InitSimpleShape(
          ModelMaker::SimpleShapes::BOX))
    {
      this->dataPtr->entityMaker = &this->dataPtr->modelMaker;
    }
  }
  else if (_type == "sphere")
  {
    if (this->dataPtr->modelMaker.InitSimpleShape(
          ModelMaker::SimpleShapes::SPHERE))
    {
      this->dataPtr->entityMaker = &this->dataPtr->modelMaker;
    }
  }
  else if (_type == "cylinder")
  {
    if (this->dataPtr->modelMaker.InitSimpleShape(
          ModelMaker::SimpleShapes::CYLINDER))
    {
      this->dataPtr->entityMaker = &this->dataPtr->modelMaker;
    }
  }
  else if (_type == "model" && !_data.empty())
  {
    if (this->dataPtr->modelMaker.InitFromFile(_data))
      this->dataPtr->entityMaker = &this->dataPtr->modelMaker;
  }
  else if (_type == "pointlight")
    this->dataPtr->entityMaker =  &this->dataPtr->pointLightMaker;
  else if (_type == "spotlight")
    this->dataPtr->entityMaker =  &this->dataPtr->spotLightMaker;
  else if (_type == "directionallight")
    this->dataPtr->entityMaker =  &this->dataPtr->directionalLightMaker;

  if (this->dataPtr->entityMaker)
  {
    gui::Events::manipMode("make_entity");
    // TODO: change the cursor to a cross
    this->dataPtr->entityMaker->Start();
  }
  else
  {
    this->dataPtr->state = "select";
    // TODO: make sure cursor state stays at the default
  }
}

/////////////////////////////////////////////////
void GLWidget::OnFPS()
{
  this->dataPtr->userCamera->SetViewController(
      rendering::FPSViewController::GetTypeString());
}
/////////////////////////////////////////////////
void GLWidget::OnOrbit()
{
  this->dataPtr->userCamera->SetViewController(
      rendering::OrbitViewController::GetTypeString());
}

/////////////////////////////////////////////////
std::vector<rendering::VisualPtr> GLWidget::SelectedVisuals() const
{
  return this->dataPtr->selectedVisuals;
}

/////////////////////////////////////////////////
void GLWidget::SetSelectedVisual(rendering::VisualPtr _vis)
{
  // deselect all if not in multi-selection mode.
  if (!this->dataPtr->mouseEvent.Control())
  {
    this->DeselectAllVisuals();
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->selectedVisMutex);

  msgs::Selection msg;

  if (_vis && !_vis->IsPlane())
  {
    if (_vis == _vis->GetRootVisual())
      this->dataPtr->selectionLevel = SelectionLevels::MODEL;
    else
      this->dataPtr->selectionLevel = SelectionLevels::LINK;

    _vis->SetHighlighted(true);

    // enable multi-selection if control is pressed
    if (this->dataPtr->selectedVisuals.empty() ||
        this->dataPtr->mouseEvent.Control())
    {
      std::vector<rendering::VisualPtr>::iterator it =
        std::find(this->dataPtr->selectedVisuals.begin(),
            this->dataPtr->selectedVisuals.end(), _vis);
      if (it == this->dataPtr->selectedVisuals.end())
        this->dataPtr->selectedVisuals.push_back(_vis);
      else
      {
        // if element already exists, move to the back of vector
        rendering::VisualPtr vis = (*it);
        this->dataPtr->selectedVisuals.erase(it);
        this->dataPtr->selectedVisuals.push_back(vis);
      }
    }
    g_copyAct->setEnabled(true);

    msg.set_id(_vis->GetId());
    msg.set_name(_vis->GetName());
    msg.set_selected(true);
    this->dataPtr->selectionPub->Publish(msg);
  }
  else if (g_copyAct)
  {
    g_copyAct->setEnabled(false);
  }

  if (g_alignAct)
    g_alignAct->setEnabled(this->dataPtr->selectedVisuals.size() > 1);
}

/////////////////////////////////////////////////
void GLWidget::DeselectAllVisuals()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->selectedVisMutex);

  msgs::Selection msg;
  for (unsigned int i = 0; i < this->dataPtr->selectedVisuals.size(); ++i)
  {
    this->dataPtr->selectedVisuals[i]->SetHighlighted(false);
    msg.set_id(this->dataPtr->selectedVisuals[i]->GetId());
    msg.set_name(this->dataPtr->selectedVisuals[i]->GetName());
    msg.set_selected(false);
    this->dataPtr->selectionPub->Publish(msg);
  }
  this->dataPtr->selectedVisuals.clear();
}

/////////////////////////////////////////////////
void GLWidget::OnManipMode(const std::string &_mode)
{
  this->dataPtr->state = _mode;

  if (!this->dataPtr->selectedVisuals.empty())
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->selectedVisMutex);
    ModelManipulator::Instance()->SetAttachedVisual(
        this->dataPtr->selectedVisuals.back());

    if (_mode == "select")
    {
      this->dataPtr->scene->SelectVisual("", "select");
    }
    else
    {
      // Make sure model is not updated by server during manipulation
      this->dataPtr->scene->SelectVisual(
          this->dataPtr->selectedVisuals.back()->GetName(), "move");
    }
  }

  ModelManipulator::Instance()->SetManipulationMode(_mode);
  ModelSnap::Instance()->Reset();

  if (this->dataPtr->state != "select")
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->selectedVisMutex);
    // only support multi-model selection in select mode for now.
    // deselect 0 to n-1 models.
    if (this->dataPtr->selectedVisuals.size() > 1)
    {
      for (std::vector<rendering::VisualPtr>::iterator it
              = this->dataPtr->selectedVisuals.begin();
              it != --this->dataPtr->selectedVisuals.end();)
      {
        (*it)->SetHighlighted(false);
        it = this->dataPtr->selectedVisuals.erase(it);
      }
    }
  }
}

/////////////////////////////////////////////////
void GLWidget::OnCopy()
{
  std::lock_guard<std::mutex> lock(this->dataPtr->selectedVisMutex);
  if (!this->dataPtr->selectedVisuals.empty() &&
      !this->dataPtr->modelEditorEnabled)
  {
    this->Copy(this->dataPtr->selectedVisuals.back()->GetName());
  }
}

/////////////////////////////////////////////////
void GLWidget::OnPaste()
{
  if (!this->dataPtr->modelEditorEnabled)
    this->Paste(this->dataPtr->copyEntityName);
}

/////////////////////////////////////////////////
void GLWidget::Copy(const std::string &_name)
{
  this->dataPtr->copyEntityName = _name;
  g_pasteAct->setEnabled(true);
}

/////////////////////////////////////////////////
void GLWidget::Paste(const std::string &_name)
{
  if (!_name.empty())
  {
    bool isModel = false;
    bool isLight = false;
    if (this->dataPtr->scene->GetLight(_name))
      isLight = true;
    else if (this->dataPtr->scene->GetVisual(_name))
      isModel = true;

    if (isLight || isModel)
    {
      this->ClearSelection();
      if (this->dataPtr->entityMaker)
        this->dataPtr->entityMaker->Stop();

      if (isLight && this->dataPtr->lightMaker.InitFromLight(_name))
      {
        this->dataPtr->entityMaker = &this->dataPtr->lightMaker;
        this->dataPtr->entityMaker->Start();
        // this makes the entity appear at the mouse cursor
        this->dataPtr->entityMaker->OnMouseMove(this->dataPtr->mouseEvent);
        gui::Events::manipMode("make_entity");
      }
      else if (isModel && this->dataPtr->modelMaker.InitFromModel(_name))
      {
        this->dataPtr->entityMaker = &this->dataPtr->modelMaker;
        this->dataPtr->entityMaker->Start();
        // this makes the entity appear at the mouse cursor
        this->dataPtr->entityMaker->OnMouseMove(this->dataPtr->mouseEvent);
        gui::Events::manipMode("make_entity");
      }
    }
  }
}

/////////////////////////////////////////////////
void GLWidget::ClearSelection()
{
  this->SetSelectedVisual(rendering::VisualPtr());

  this->dataPtr->scene->SelectVisual("", "normal");
}

/////////////////////////////////////////////////
void GLWidget::OnSetSelectedEntity(const std::string &_name,
                                   const std::string &_mode)
{
  if (!_name.empty())
  {
    std::string name = _name;
    boost::replace_first(name, gui::get_world()+"::", "");

    rendering::VisualPtr selection = this->dataPtr->scene->GetVisual(name);

    std::vector<rendering::VisualPtr>::iterator it =
      std::find(this->dataPtr->selectedVisuals.begin(),
          this->dataPtr->selectedVisuals.end(), selection);

    // Shortcircuit the case when GLWidget already selected the visual.
    if (it == this->dataPtr->selectedVisuals.end() || _name != (*it)->GetName())
    {
      this->SetSelectedVisual(selection);
      this->dataPtr->scene->SelectVisual(name, _mode);
    }
  }
  else if (!this->dataPtr->selectedVisuals.empty())
  {
    this->SetSelectedVisual(rendering::VisualPtr());
    this->dataPtr->scene->SelectVisual("", _mode);
  }
}

/////////////////////////////////////////////////
void GLWidget::OnRequest(ConstRequestPtr &_msg)
{
  if (_msg->request() == "entity_delete")
  {
    std::lock_guard<std::mutex> lock(this->dataPtr->selectedVisMutex);
    if (!this->dataPtr->selectedVisuals.empty())
    {
      for (std::vector<rendering::VisualPtr>::iterator it =
          this->dataPtr->selectedVisuals.begin();
          it != this->dataPtr->selectedVisuals.end();
          ++it)
      {
        if ((*it)->GetName() == _msg->data())
        {
          ModelManipulator::Instance()->Detach();
          this->dataPtr->selectedVisuals.erase(it);
          break;
        }
      }
    }

    if (this->dataPtr->copyEntityName == _msg->data())
    {
      this->dataPtr->copyEntityName = "";
      g_pasteAct->setEnabled(false);
    }
  }
}

/////////////////////////////////////////////////
void GLWidget::OnAlignMode(const std::string &_axis, const std::string &_config,
    const std::string &_target, const bool _preview, const bool _inverted)
{
  ModelAlign::Instance()->AlignVisuals(this->dataPtr->selectedVisuals, _axis,
      _config, _target, !_preview, _inverted);
}

/////////////////////////////////////////////////
void GLWidget::OnModelEditor(bool _checked)
{
  this->dataPtr->modelEditorEnabled = _checked;
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
  this->dataPtr->userCamera->SetProjectionType("orthographic");
}

/////////////////////////////////////////////////
void GLWidget::OnPerspective()
{
  // Enable view control options when in perspective projection
  g_fpsAct->setEnabled(true);
  g_orbitAct->setEnabled(true);
  this->dataPtr->userCamera->SetProjectionType("perspective");
}

/////////////////////////////////////////////////
QPaintEngine *GLWidget::paintEngine() const
{
  return nullptr;
}

/////////////////////////////////////////////////
void GLWidget::SetMouseEventButtons(const Qt::MouseButtons &_buttons)
{
  if (_buttons & Qt::LeftButton)
  {
    this->dataPtr->mouseEvent.SetButtons(
        this->dataPtr->mouseEvent.Buttons() | common::MouseEvent::LEFT);
  }
  else
  {
    this->dataPtr->mouseEvent.SetButtons(
        this->dataPtr->mouseEvent.Buttons() | 0x0);
  }

  if (_buttons & Qt::RightButton)
  {
    this->dataPtr->mouseEvent.SetButtons(
        this->dataPtr->mouseEvent.Buttons() | common::MouseEvent::RIGHT);
  }
  else
  {
    this->dataPtr->mouseEvent.SetButtons(
        this->dataPtr->mouseEvent.Buttons() | 0x0);
  }

  if (_buttons & Qt::MidButton)
  {
    this->dataPtr->mouseEvent.SetButtons(
        this->dataPtr->mouseEvent.Buttons() | common::MouseEvent::MIDDLE);
  }
  else
  {
    this->dataPtr->mouseEvent.SetButtons(
        this->dataPtr->mouseEvent.Buttons() | 0x0);
  }
}

/////////////////////////////////////////////////
void GLWidget::SetMouseEventButton(const Qt::MouseButton &_button)
{
  if (_button == Qt::LeftButton)
    this->dataPtr->mouseEvent.SetButton(common::MouseEvent::LEFT);
  else if (_button == Qt::RightButton)
    this->dataPtr->mouseEvent.SetButton(common::MouseEvent::RIGHT);
  else if (_button == Qt::MidButton)
    this->dataPtr->mouseEvent.SetButton(common::MouseEvent::MIDDLE);
}
