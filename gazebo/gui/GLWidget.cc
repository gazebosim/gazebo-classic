/*
 * Copyright 2012 Open Source Robotics Foundation
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

#include "gazebo/common/Exception.hh"
#include "gazebo/math/gzmath.hh"

#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/Heightmap.hh"
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/Rendering.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/WindowManager.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/OrbitViewController.hh"
#include "gazebo/rendering/FPSViewController.hh"
#include "gazebo/rendering/SelectionObj.hh"

#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/KeyEventHandler.hh"
#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/Gui.hh"
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
  this->sceneCreated = false;

  this->setFocusPolicy(Qt::StrongFocus);

  this->windowId = -1;

  setAttribute(Qt::WA_OpaquePaintEvent, true);
  setAttribute(Qt::WA_PaintOnScreen, true);
//  setMinimumSize(320, 240);

  this->renderFrame = new QFrame;
  this->renderFrame->setFrameShape(QFrame::NoFrame);
  this->renderFrame->setSizePolicy(QSizePolicy::Expanding,
                                   QSizePolicy::Expanding);
  this->renderFrame->show();
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(this->renderFrame);
  mainLayout->setContentsMargins(0, 0, 0, 0);
  this->setLayout(mainLayout);

  this->connections.push_back(
      rendering::Events::ConnectCreateScene(
        boost::bind(&GLWidget::OnCreateScene, this, _1)));

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

  this->renderFrame->setMouseTracking(true);
  this->setMouseTracking(true);

  this->entityMaker = NULL;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->modelPub = this->node->Advertise<msgs::Model>("~/model/modify");
  this->lightPub = this->node->Advertise<msgs::Light>("~/light");

  this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");
  this->selectionSub = this->node->Subscribe("~/selection",
      &GLWidget::OnSelectionMsg, this);
  this->requestSub = this->node->Subscribe("~/request",
      &GLWidget::OnRequest, this);

  this->installEventFilter(this);
  this->keyModifiers = 0;

  this->selectedVis.reset();
  this->mouseMoveVis.reset();

  MouseEventHandler::Instance()->AddPressFilter("glwidget",
      boost::bind(&GLWidget::OnMousePress, this, _1));

  MouseEventHandler::Instance()->AddReleaseFilter("glwidget",
      boost::bind(&GLWidget::OnMouseRelease, this, _1));

  MouseEventHandler::Instance()->AddMoveFilter("glwidget",
      boost::bind(&GLWidget::OnMouseMove, this, _1));
}

/////////////////////////////////////////////////
GLWidget::~GLWidget()
{
  this->connections.clear();
  this->node.reset();
  this->modelPub.reset();
  this->lightPub.reset();
  this->selectionSub.reset();
  this->selectionObj.reset();

  this->userCamera.reset();
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
  QApplication::flush();
  this->windowId = rendering::RenderEngine::Instance()->GetWindowManager()->
    CreateWindow(this->GetOgreHandle(), this->width(), this->height());

  QWidget::showEvent(_event);

  if (this->userCamera)
    rendering::RenderEngine::Instance()->GetWindowManager()->SetCamera(
        this->windowId, this->userCamera);
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
  // Timing may cause GLWidget to miss the OnCreateScene event. So, we check
  // here to make sure it's handled.
  if (!this->sceneCreated && rendering::get_scene())
    this->OnCreateScene(rendering::get_scene()->GetName());

  rendering::UserCameraPtr cam = gui::get_active_camera();
  if (cam && cam->GetInitialized())
  {
    event::Events::preRender();

    // Tell all the cameras to render
    event::Events::render();

    event::Events::postRender();
  }

  _e->accept();
}

/////////////////////////////////////////////////
void GLWidget::resizeEvent(QResizeEvent *_e)
{
  if (!this->scene)
    return;

  if (this->windowId >= 0)
  {
    rendering::RenderEngine::Instance()->GetWindowManager()->Resize(
        this->windowId, _e->size().width(), _e->size().height());
    this->userCamera->Resize(_e->size().width(), _e->size().height());
  }
}

/////////////////////////////////////////////////
void GLWidget::keyPressEvent(QKeyEvent *_event)
{
  if (!this->scene)
    return;

  if (_event->isAutoRepeat())
    return;

  this->keyText = _event->text().toStdString();
  this->keyModifiers = _event->modifiers();

  this->keyEvent.key = _event->key();

  // Toggle full screen
  if (_event->key() == Qt::Key_F11)
  {
    g_fullscreen = !g_fullscreen;
    gui::Events::fullScreen(g_fullscreen);
  }

  // Trigger a model delete if the Delete key was pressed, and a model
  // is currently selected.
  if (_event->key() == Qt::Key_Delete && this->selectedVis)
    g_deleteAct->Signal(this->selectedVis->GetName());

  if (_event->key() == Qt::Key_Escape)
    event::Events::setSelectedEntity("", "normal");

  this->mouseEvent.control =
    this->keyModifiers & Qt::ControlModifier ? true : false;
  this->mouseEvent.shift =
    this->keyModifiers & Qt::ShiftModifier ? true : false;
  this->mouseEvent.alt =
    this->keyModifiers & Qt::AltModifier ? true : false;

  // reset mouseMoveVisStartPose if in manipulation mode.
  if (this->state == "translate" || this->state == "rotate"
      || this->state == "scale")
  {
    if (this->keyText == "x" || this->keyText == "y" || this->keyText == "z")
    {
      this->mouseEvent.pressPos = this->mouseEvent.pos;
      if (this->mouseMoveVis)
      {
        this->mouseMoveVisStartPose = this->mouseMoveVis->GetWorldPose();
      }
    }
  }

  this->userCamera->HandleKeyPressEvent(this->keyText);

  // Process Key Events
  KeyEventHandler::Instance()->HandlePress(this->keyEvent);
}

/////////////////////////////////////////////////
void GLWidget::keyReleaseEvent(QKeyEvent *_event)
{
  if (!this->scene)
    return;

  if (_event->isAutoRepeat())
    return;

  this->keyModifiers = _event->modifiers();

  if (this->keyModifiers & Qt::ControlModifier &&
      _event->key() == Qt::Key_Z)
  {
    this->PopHistory();
  }
  else if (_event->key() == Qt::Key_R)
    g_rotateAct->trigger();
  else if (_event->key() == Qt::Key_T)
    g_translateAct->trigger();
  else if (_event->key() == Qt::Key_S)
    g_scaleAct->trigger();

  this->mouseEvent.control =
    this->keyModifiers & Qt::ControlModifier ? true : false;
  this->mouseEvent.shift =
    this->keyModifiers & Qt::ShiftModifier ? true : false;
  this->mouseEvent.alt =
    this->keyModifiers & Qt::AltModifier ? true : false;

  // Reset the mouse move info when the user hits keys.
  if (this->state == "translate" || this->state == "rotate"
      || this->state == "scale")
  {
    if (this->keyText == "x" || this->keyText == "y" || this->keyText == "z")
    {
      this->mouseEvent.pressPos = this->mouseEvent.pos;
      if (this->mouseMoveVis)
      {
        this->mouseMoveVisStartPose = this->mouseMoveVis->GetWorldPose();
      }
    }
  }

  this->keyText = "";

  this->userCamera->HandleKeyReleaseEvent(_event->text().toStdString());

  // Process Key Events
  KeyEventHandler::Instance()->HandleRelease(this->keyEvent);
}

/////////////////////////////////////////////////
void GLWidget::mouseDoubleClickEvent(QMouseEvent * /*_event*/)
{
  rendering::VisualPtr vis = this->userCamera->GetVisual(this->mouseEvent.pos);
  if (vis)
  {
    if (vis->IsPlane())
    {
      math::Pose pose, camPose;
      camPose = this->userCamera->GetWorldPose();
      if (this->scene->GetFirstContact(this->userCamera,
                                   this->mouseEvent.pos, pose.pos))
      {
        this->userCamera->SetFocalPoint(pose.pos);

        math::Vector3 dir = pose.pos - camPose.pos;
        pose.pos = camPose.pos + (dir * 0.8);

        pose.rot = this->userCamera->GetWorldRotation();
        this->userCamera->MoveToPosition(pose, 0.5);
      }
    }
    else
    {
      this->userCamera->MoveToVisual(vis);
    }
  }
}

/////////////////////////////////////////////////
void GLWidget::mousePressEvent(QMouseEvent *_event)
{
  if (!this->scene)
    return;

  this->mouseEvent.pressPos.Set(_event->pos().x(), _event->pos().y());
  this->mouseEvent.prevPos = this->mouseEvent.pressPos;

  /// Set the button which cause the press event
  if (_event->button() == Qt::LeftButton)
    this->mouseEvent.button = common::MouseEvent::LEFT;
  else if (_event->button() == Qt::RightButton)
    this->mouseEvent.button = common::MouseEvent::RIGHT;
  else if (_event->button() == Qt::MidButton)
    this->mouseEvent.button = common::MouseEvent::MIDDLE;

  this->mouseEvent.buttons = common::MouseEvent::NO_BUTTON;
  this->mouseEvent.type = common::MouseEvent::PRESS;

  this->mouseEvent.buttons |= _event->buttons() & Qt::LeftButton ?
    common::MouseEvent::LEFT : 0x0;
  this->mouseEvent.buttons |= _event->buttons() & Qt::RightButton ?
    common::MouseEvent::RIGHT : 0x0;
  this->mouseEvent.buttons |= _event->buttons() & Qt::MidButton ?
    common::MouseEvent::MIDDLE : 0x0;

  this->mouseEvent.dragging = false;

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
    this->OnMousePressTranslate();

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
    this->OnMouseReleaseTranslate();

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
    this->OnMouseMoveTranslate();

  return true;
}

/////////////////////////////////////////////////
void GLWidget::OnMousePressTranslate()
{
  this->SetMouseMoveVisual(rendering::VisualPtr());

  rendering::VisualPtr vis;
  rendering::VisualPtr mouseVis
      = this->userCamera->GetVisual(this->mouseEvent.pos);

  if (this->selectionObj->GetMode() == rendering::SelectionObj::SELECTION_NONE
      || (mouseVis && mouseVis != this->selectionObj->GetParent()))
  {
    vis = mouseVis;
  }
  else
  {
    vis = this->selectionObj->GetParent();
  }
//  rendering::VisualPtr vis = this->userCamera->GetVisual(this->mouseEvent.pos);

  if (vis && !vis->IsPlane() &&
      this->mouseEvent.button == common::MouseEvent::LEFT)
  {
    vis = vis->GetRootVisual();
    this->mouseMoveVisStartPose = vis->GetWorldPose();

    this->SetMouseMoveVisual(vis);

    event::Events::setSelectedEntity(this->mouseMoveVis->GetName(), "move");
    QApplication::setOverrideCursor(Qt::ClosedHandCursor);

    if (this->mouseMoveVis && !this->mouseMoveVis->IsPlane())
    {
      this->selectionObj->Attach(this->mouseMoveVis);
      this->selectionObj->SetMode(this->state);
    }
    else
    {
      this->selectionObj->SetMode(rendering::SelectionObj::SELECTION_NONE);
      this->selectionObj->Detach();
    }

  }
  else
    this->userCamera->HandleMouseEvent(this->mouseEvent);
}

/////////////////////////////////////////////////
void GLWidget::OnMousePressNormal()
{
  if (!this->userCamera)
    return;

  rendering::VisualPtr vis = this->userCamera->GetVisual(this->mouseEvent.pos);

  this->SetMouseMoveVisual(rendering::VisualPtr());

  this->userCamera->HandleMouseEvent(this->mouseEvent);
}

/////////////////////////////////////////////////
void GLWidget::OnMousePressMakeEntity()
{
  if (this->entityMaker)
    this->entityMaker->OnMousePush(this->mouseEvent);
}

/////////////////////////////////////////////////
void GLWidget::wheelEvent(QWheelEvent *_event)
{
  if (!this->scene)
    return;

  this->mouseEvent.scroll.y = _event->delta() > 0 ? -1 : 1;
  this->mouseEvent.type = common::MouseEvent::SCROLL;
  this->mouseEvent.buttons |= _event->buttons() & Qt::LeftButton ?
    common::MouseEvent::LEFT : 0x0;
  this->mouseEvent.buttons |= _event->buttons() & Qt::RightButton ?
    common::MouseEvent::RIGHT : 0x0;
  this->mouseEvent.buttons |= _event->buttons() & Qt::MidButton ?
    common::MouseEvent::MIDDLE : 0x0;

  this->userCamera->HandleMouseEvent(this->mouseEvent);
}

/////////////////////////////////////////////////
void GLWidget::mouseMoveEvent(QMouseEvent *_event)
{
  if (!this->scene)
    return;

  this->setFocus(Qt::MouseFocusReason);

  this->mouseEvent.pos.Set(_event->pos().x(), _event->pos().y());
  this->mouseEvent.type = common::MouseEvent::MOVE;
  this->mouseEvent.buttons |= _event->buttons() & Qt::LeftButton ?
    common::MouseEvent::LEFT : 0x0;
  this->mouseEvent.buttons |= _event->buttons() & Qt::RightButton ?
    common::MouseEvent::RIGHT : 0x0;
  this->mouseEvent.buttons |= _event->buttons() & Qt::MidButton ?
    common::MouseEvent::MIDDLE : 0x0;

  if (_event->buttons())
    this->mouseEvent.dragging = true;
  else
    this->mouseEvent.dragging = false;

  // Process Mouse Events
  MouseEventHandler::Instance()->HandleMove(this->mouseEvent);

  this->mouseEvent.prevPos = this->mouseEvent.pos;
}

/////////////////////////////////////////////////
void GLWidget::OnMouseMoveMakeEntity()
{
  if (this->entityMaker)
  {
    if (this->mouseEvent.dragging)
      this->entityMaker->OnMouseDrag(this->mouseEvent);
    else
      this->entityMaker->OnMouseMove(this->mouseEvent);
  }
}

/////////////////////////////////////////////////
void GLWidget::SmartMoveVisual(rendering::VisualPtr _vis)
{
  if (!this->mouseEvent.dragging)
    return;

  // Get the point on the plane which correspoinds to the mouse
  math::Vector3 pp;

  // Rotate the visual using the middle mouse button
  if (this->mouseEvent.buttons == common::MouseEvent::MIDDLE)
  {
    math::Vector3 rpy = this->mouseMoveVisStartPose.rot.GetAsEuler();
    math::Vector2i delta = this->mouseEvent.pos - this->mouseEvent.pressPos;
    double yaw = (delta.x * 0.01) + rpy.z;
    if (!this->mouseEvent.shift)
    {
      double snap = rint(yaw / (M_PI * .25)) * (M_PI * 0.25);

      if (fabs(yaw - snap) < GZ_DTOR(10))
        yaw = snap;
    }

    _vis->SetWorldRotation(math::Quaternion(rpy.x, rpy.y, yaw));
  }
  else if (this->mouseEvent.buttons == common::MouseEvent::RIGHT)
  {
    math::Vector3 rpy = this->mouseMoveVisStartPose.rot.GetAsEuler();
    math::Vector2i delta = this->mouseEvent.pos - this->mouseEvent.pressPos;
    double pitch = (delta.y * 0.01) + rpy.y;
    if (!this->mouseEvent.shift)
    {
      double snap = rint(pitch / (M_PI * .25)) * (M_PI * 0.25);

      if (fabs(pitch - snap) < GZ_DTOR(10))
        pitch = snap;
    }

    _vis->SetWorldRotation(math::Quaternion(rpy.x, pitch, rpy.z));
  }
  else if (this->mouseEvent.buttons & common::MouseEvent::LEFT &&
           this->mouseEvent.buttons & common::MouseEvent::RIGHT)
  {
    math::Vector3 rpy = this->mouseMoveVisStartPose.rot.GetAsEuler();
    math::Vector2i delta = this->mouseEvent.pos - this->mouseEvent.pressPos;
    double roll = (delta.x * 0.01) + rpy.x;
    if (!this->mouseEvent.shift)
    {
      double snap = rint(roll / (M_PI * .25)) * (M_PI * 0.25);

      if (fabs(roll - snap) < GZ_DTOR(10))
        roll = snap;
    }

    _vis->SetWorldRotation(math::Quaternion(roll, rpy.y, rpy.z));
  }
  else
  {
    this->TranslateEntity(_vis);
  }
}

/////////////////////////////////////////////////
void GLWidget::OnMouseMoveTranslate()
{
  if (this->mouseEvent.dragging)
  {
    if (this->mouseMoveVis &&
        this->mouseEvent.button == common::MouseEvent::LEFT)
    {
      if (this->selectionObj->GetState()
          == rendering::SelectionObj::TRANS_X)
      {
        this->keyText = "x";
        this->TranslateEntity(this->mouseMoveVis, true);
        this->keyText = "";
      }
      else if (this->selectionObj->GetState()
          == rendering::SelectionObj::TRANS_Y)
      {
        this->keyText = "y";
        this->TranslateEntity(this->mouseMoveVis, true);
        this->keyText = "";
      }
      else if (this->selectionObj->GetState()
          == rendering::SelectionObj::TRANS_Z)
      {
        this->keyText = "z";
        this->TranslateEntity(this->mouseMoveVis, true);
        this->keyText = "";
      }
      else if (this->selectionObj->GetState()
          == rendering::SelectionObj::ROT_X)
      {
        this->keyText = "x";
        this->RotateEntity(this->mouseMoveVis);
        this->keyText = "";
      }
      else if (this->selectionObj->GetState()
          == rendering::SelectionObj::ROT_Y)
      {
        this->keyText = "y";
        this->RotateEntity(this->mouseMoveVis);
        this->keyText = "";
      }
      else if (this->selectionObj->GetState()
          == rendering::SelectionObj::ROT_Z)
      {
        this->keyText = "z";
        this->RotateEntity(this->mouseMoveVis);
        this->keyText = "";
      }
      else if (this->selectionObj->GetState()
          == rendering::SelectionObj::SCALE_X)
      {
        this->keyText = "x";
        this->ScaleEntity(this->mouseMoveVis, true);
        this->keyText = "";
      }
      else if (this->selectionObj->GetState()
          == rendering::SelectionObj::SCALE_Y)
      {
        this->keyText = "y";
        this->ScaleEntity(this->mouseMoveVis, true);
        this->keyText = "";
      }
      else if (this->selectionObj->GetState()
          == rendering::SelectionObj::SCALE_Z)
      {
        this->keyText = "z";
        this->ScaleEntity(this->mouseMoveVis, true);
        this->keyText = "";
      }
      else
        this->TranslateEntity(this->mouseMoveVis);
//        rendering::VisualPtr attachedVis = this->selectionObj->GetParent();

//        this->TranslateEntity(this->mouseMoveVis);
//      this->keyText = "";

      /*if (this->state == "translate")
        this->TranslateEntity(this->mouseMoveVis);
      else if (this->state == "rotate")
        this->RotateEntity(this->mouseMoveVis);
      else if (this->state == "scale")
      {
        this->ScaleEntity(this->mouseMoveVis);
      }*/
    }
    else
      this->userCamera->HandleMouseEvent(this->mouseEvent);
  }
  else
  {
    std::string manipState;
    this->userCamera->GetVisual(this->mouseEvent.pos, manipState);
    this->selectionObj->SetState(manipState);

    if (!manipState.empty())
      QApplication::setOverrideCursor(Qt::OpenHandCursor);
    else
    {
      rendering::VisualPtr vis = this->userCamera->GetVisual(
          this->mouseEvent.pos);

      if (vis && !vis->IsPlane())
        QApplication::setOverrideCursor(Qt::OpenHandCursor);
      else
        QApplication::setOverrideCursor(Qt::ArrowCursor);
      this->userCamera->HandleMouseEvent(this->mouseEvent);
    }
  }
}

/////////////////////////////////////////////////
void GLWidget::OnMouseMoveNormal()
{
  if (!this->userCamera)
    return;

  rendering::VisualPtr vis = this->userCamera->GetVisual(this->mouseEvent.pos);

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

  this->mouseEvent.pos.Set(_event->pos().x(), _event->pos().y());
  this->mouseEvent.prevPos = this->mouseEvent.pos;

  if (_event->button() == Qt::LeftButton)
    this->mouseEvent.button = common::MouseEvent::LEFT;
  else if (_event->button() == Qt::RightButton)
    this->mouseEvent.button = common::MouseEvent::RIGHT;
  else if (_event->button() == Qt::MidButton)
    this->mouseEvent.button = common::MouseEvent::MIDDLE;

  this->mouseEvent.buttons = common::MouseEvent::NO_BUTTON;
  this->mouseEvent.type = common::MouseEvent::RELEASE;

  this->mouseEvent.buttons |= _event->buttons() & Qt::LeftButton ?
    common::MouseEvent::LEFT : 0x0;

  this->mouseEvent.buttons |= _event->buttons() & Qt::RightButton ?
    common::MouseEvent::RIGHT : 0x0;

  this->mouseEvent.buttons |= _event->buttons() & Qt::MidButton ?
    common::MouseEvent::MIDDLE : 0x0;

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
void GLWidget::OnMouseReleaseTranslate()
{
  if (this->mouseEvent.dragging)
  {
    // If we were dragging a visual around, then publish its new pose to the
    // server
    if (this->mouseMoveVis)
    {
      if (this->state == "scale")
      {
        this->PublishVisualScale(this->mouseMoveVis);
        this->mouseMoveVis->SetScale(this->mouseVisualScale);
      }
      this->PublishVisualPose(this->mouseMoveVis);
      this->SetMouseMoveVisual(rendering::VisualPtr());
      QApplication::setOverrideCursor(Qt::OpenHandCursor);
      this->selectionObj->SetMode(rendering::SelectionObj::SELECTION_NONE);
      this->selectionObj->Detach();
    }
    this->SetSelectedVisual(rendering::VisualPtr());
    event::Events::setSelectedEntity("", "normal");
  }
  else
  {
    if (this->mouseEvent.button == common::MouseEvent::LEFT)
    {
      rendering::VisualPtr vis =
        this->userCamera->GetVisual(this->mouseEvent.pos);
      if (vis && vis->IsPlane())
      {
        this->selectionObj->SetMode(rendering::SelectionObj::SELECTION_NONE);
        this->selectionObj->Detach();
      }
    }
  }

  this->userCamera->HandleMouseEvent(this->mouseEvent);
}

//////////////////////////////////////////////////
void GLWidget::OnMouseReleaseNormal()
{
  if (!this->userCamera)
    return;

  if (!this->mouseEvent.dragging)
  {
    rendering::VisualPtr vis =
      this->userCamera->GetVisual(this->mouseEvent.pos);
    if (vis)
    {
      if (this->mouseEvent.button == common::MouseEvent::RIGHT)
      {
        g_modelRightMenu->Run(vis->GetName(), QCursor::pos());
      }
      else if (this->mouseEvent.button == common::MouseEvent::LEFT)
      {
        vis = vis->GetRootVisual();
        this->SetSelectedVisual(vis);
        event::Events::setSelectedEntity(vis->GetName(), "normal");
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
  if (_scene->GetUserCameraCount() == 0)
    this->userCamera = _scene->CreateUserCamera("rc_camera");
  else
    this->userCamera = _scene->GetUserCamera(0);

  gui::set_active_camera(this->userCamera);
  this->scene = _scene;

  math::Vector3 camPos(5, -5, 2);
  math::Vector3 lookAt(0, 0, 0);
  math::Vector3 delta = lookAt - camPos;

  double yaw = atan2(delta.y, delta.x);

  double pitch = atan2(-delta.z, sqrt(delta.x*delta.x + delta.y*delta.y));
  this->userCamera->SetWorldPose(math::Pose(camPos,
        math::Vector3(0, pitch, yaw)));

  if (this->windowId >= 0)
  {
    rendering::RenderEngine::Instance()->GetWindowManager()->SetCamera(
        this->windowId, this->userCamera);
  }
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
  this->SetMouseMoveVisual(rendering::VisualPtr());
  this->hoverVis.reset();
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
  assert(q_parent);
  ogreHandle += boost::lexical_cast<std::string>(
      static_cast<uint64_t>(q_parent->winId()));
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
  this->hoverVis.reset();
  this->SetSelectedVisual(rendering::VisualPtr());
  this->SetMouseMoveVisual(rendering::VisualPtr());

  this->ViewScene(rendering::get_scene(_name));
  this->sceneCreated = true;

  this->selectionObj.reset(new rendering::SelectionObj("__GL_MANIP__",
      rendering::get_scene(_name)->GetWorldVisual()));
  this->selectionObj->Load();
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
  this->ClearSelection();

  if (this->entityMaker)
    this->entityMaker->Stop();

  this->entityMaker = NULL;

  if (_type == "box")
  {
    this->boxMaker.Start(this->userCamera);
    if (this->modelMaker.InitFromSDFString(this->boxMaker.GetSDFString()))
      this->entityMaker = &this->modelMaker;
  }
  else if (_type == "sphere")
  {
    this->sphereMaker.Start(this->userCamera);
    if (this->modelMaker.InitFromSDFString(this->sphereMaker.GetSDFString()))
      this->entityMaker = &this->modelMaker;
  }
  else if (_type == "cylinder")
  {
    this->cylinderMaker.Start(this->userCamera);
    if (this->modelMaker.InitFromSDFString(this->cylinderMaker.GetSDFString()))
      this->entityMaker = &this->modelMaker;
  }
  else if (_type == "mesh" && !_data.empty())
  {
    this->meshMaker.Init(_data);
    this->entityMaker = &this->meshMaker;
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
    this->entityMaker->Start(this->userCamera);
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

/*/////////////////////////////////////////////////
void GLWidget::RotateEntity(rendering::VisualPtr &_vis)
{
  math::Vector3 planeNorm, planeNorm2;
  math::Vector3 p1, p2;
  math::Vector3 a, b;
  math::Vector3 ray(0, 0, 0);

  math::Pose pose = _vis->GetPose();

  math::Vector2i diff = this->mouseEvent.pos - this->mouseEvent.pressPos;
  math::Vector3 rpy = this->mouseMoveVisStartPose.rot.GetAsEuler();

  math::Vector3 rpyAmt;

  if (this->keyText == "x" || this->keyText == "X")
    rpyAmt.x = 1.0;
  else if (this->keyText == "y" || this->keyText == "Y")
    rpyAmt.y = 1.0;
  else
    rpyAmt.z = 1.0;

  double amt = diff.y * 0.04;

  if (this->mouseEvent.shift)
    amt = rint(amt / (M_PI * 0.25)) * (M_PI * 0.25);

  rpy += rpyAmt * amt;

  _vis->SetRotation(math::Quaternion(rpy));
}*/

/////////////////////////////////////////////////
void GLWidget::RotateEntity(rendering::VisualPtr &_vis, bool /*_local*/)
{
  math::Vector3 axis;
  math::Vector3 normal;

  if (this->keyText == "x" || this->keyText == "X")
  {
    normal = mouseMoveVisStartPose.rot.GetXAxis();
    axis.x = 1.0;
  }
  else if (this->keyText == "y" || this->keyText == "Y")
  {
    normal = mouseMoveVisStartPose.rot.GetYAxis();
    axis.y = 1.0;
  }
  else
  {
    normal = mouseMoveVisStartPose.rot.GetZAxis();
    axis.z = 1.0;
  }
  double offset = this->mouseMoveVisStartPose.pos.Dot(normal);

  math::Vector3 pressPoint;
  this->userCamera->GetWorldPointOnPlane(this->mouseEvent.pressPos.x,
      this->mouseEvent.pressPos.y, math::Plane(normal, offset), pressPoint);

  math::Vector3 newPoint;
  this->userCamera->GetWorldPointOnPlane(this->mouseEvent.pos.x,
      this->mouseEvent.pos.y, math::Plane(normal, offset), newPoint);

  math::Vector3 v1 = pressPoint - this->mouseMoveVisStartPose.pos;
  math::Vector3 v2 = newPoint - this->mouseMoveVisStartPose.pos;
  v1 = v1.Normalize();
  v2 = v2.Normalize();
  double signTest = v1.Cross(v2).Dot(normal);
  double angle = atan2((v1.Cross(v2)).GetLength(), v1.Dot(v2));

  if (signTest < 0 )
    angle *= -1;

  if (this->mouseEvent.shift)
    angle = rint(angle / (M_PI * 0.25)) * (M_PI * 0.25);

//  gzerr << " rptAmt " << rpyAmt << " rpy " << rpy << std::endl;
  math::Quaternion rot(axis, angle);

  _vis->SetRotation(this->mouseMoveVisStartPose.rot * rot);
}

/////////////////////////////////////////////////
void GLWidget::ScaleEntity(rendering::VisualPtr &_vis, bool _local)
{
  math::Box bbox = _vis->GetBoundingBox();
  math::Pose pose = _vis->GetPose();

  math::Vector3 origin1, dir1, p1;
  math::Vector3 origin2, dir2, p2;

  // Cast two rays from the camera into the world
  this->userCamera->GetCameraToViewportRay(this->mouseEvent.pos.x,
      this->mouseEvent.pos.y, origin1, dir1);
  this->userCamera->GetCameraToViewportRay(this->mouseEvent.pressPos.x,
      this->mouseEvent.pressPos.y, origin2, dir2);

  math::Vector3 moveVector(0, 0, 0);
  math::Vector3 planeNorm(0, 0, 0);
  math::Vector3 projNorm(0, 0, 0);

  math::Vector3 planeNormOther(0, 0, 0);

  if (this->keyText == "z")
  {
    moveVector.z = 1;
    planeNorm.y = 1;
    projNorm.x = 1;
    planeNormOther.x = 1;
  }
  else if (this->keyText == "x")
  {
    moveVector.x = 1;
    planeNorm.z = 1;
    projNorm.y = 1;
    planeNormOther.y = 1;

  }
  else if (this->keyText == "y")
  {
    moveVector.y = 1;
    planeNorm.z = 1;
    projNorm.x = 1;
    planeNormOther.x = 1;
  }
  else
  {
    moveVector.Set(1, 1, 0);
    planeNorm.z = 1;
    projNorm.z = 1;
  }

  if (_local)
  {
    planeNorm = pose.rot.RotateVector(planeNorm);
    projNorm = pose.rot.RotateVector(projNorm);
  }

  // Fine tune translation
  // Make sure we don't cast a ray parallel to planeNorm
  double angle = dir1.Dot(planeNorm);
  if (_local)
    planeNormOther = pose.rot.RotateVector(planeNormOther);
  double angleOther = dir1.Dot(planeNormOther);
  if (fabs(angleOther) > fabs(angle))
  {
    projNorm = planeNorm;
    planeNorm = planeNormOther;
  }

  // Compute the distance from the camera to plane of translation
  double d = pose.pos.Dot(planeNorm);
  math::Plane plane(planeNorm, d);
  double dist1 = plane.Distance(origin1, dir1);
  double dist2 = plane.Distance(origin2, dir2);

  // Compute two points on the plane. The first point is the current
  // mouse position, the second is the previous mouse position
  p1 = origin1 + dir1 * dist1;
  p2 = origin2 + dir2 * dist2;

  if (_local)
    p1 = p1 - (p1-p2).Dot(projNorm) * projNorm;

  math::Vector3 distance = p1 - p2;

  if (!_local)
    distance *= moveVector;

  // gzerr << " bbox " << bbox.GetXLength() << " " << bbox.GetYLength() << " "
  //   << bbox.GetZLength() << std::endl;

  // gzerr << " distance " << distance << std::endl;

  math::Vector3 bboxSize = bbox.GetSize() * this->mouseVisualScale;
  math::Vector3 scale = (bboxSize + distance/2.0)/bboxSize;

  // a bit hacky to check for unit sphere and cylinder simple shapes in order
  // to constrain the scaling dimensions.
  if (_vis->GetName().find("unit_sphere") != std::string::npos)
  {
    if (moveVector.x > 0)
    {
      scale.y = scale.x;
      scale.z = scale.x;
    }
    else if (moveVector.y > 0)
    {
      scale.x = scale.y;
      scale.z = scale.y;
    }
    else if (moveVector.z > 0)
    {
      scale.x = scale.z;
      scale.y = scale.z;
    }
  }
  else if (_vis->GetName().find("unit_cylinder") != std::string::npos)
  {
    if (moveVector.x > 0)
    {
      scale.y = scale.x;
    }
    else if (moveVector.y > 0)
    {
      scale.x = scale.y;
    }
  }

  scale = pose.rot.RotateVectorReverse(scale);

  _vis->SetScale(this->mouseVisualScale * scale);
}

/////////////////////////////////////////////////
void GLWidget::TranslateEntity(rendering::VisualPtr &_vis, bool _local)
{
  math::Pose pose = _vis->GetPose();

  math::Vector3 origin1, dir1, p1;
  math::Vector3 origin2, dir2, p2;

  // Cast two rays from the camera into the world
  this->userCamera->GetCameraToViewportRay(this->mouseEvent.pos.x,
      this->mouseEvent.pos.y, origin1, dir1);
  this->userCamera->GetCameraToViewportRay(this->mouseEvent.pressPos.x,
      this->mouseEvent.pressPos.y, origin2, dir2);

  math::Vector3 moveVector(0, 0, 0);
  math::Vector3 planeNorm(0, 0, 0);
  math::Vector3 projNorm(0, 0, 0);

  math::Vector3 planeNormOther(0, 0, 0);

  if (this->keyText == "z")
  {
    moveVector.z = 1;
    planeNorm.y = 1;
    projNorm.x = 1;
    planeNormOther.x = 1;
  }
  else if (this->keyText == "x")
  {
    moveVector.x = 1;
    planeNorm.z = 1;
    projNorm.y = 1;
    planeNormOther.y = 1;

  }
  else if (this->keyText == "y")
  {
    moveVector.y = 1;
    planeNorm.z = 1;
    projNorm.x = 1;
    planeNormOther.x = 1;
  }
  else
  {
    moveVector.Set(1, 1, 0);
    planeNorm.z = 1;
    projNorm.z = 1;
  }

  if (_local)
  {
    planeNorm = pose.rot.RotateVector(planeNorm);
    projNorm = pose.rot.RotateVector(projNorm);
  }

  // Fine tune translation
  // Make sure we don't cast a ray parallel to planeNorm
  double angle = dir1.Dot(planeNorm);
  if (_local)
    planeNormOther = pose.rot.RotateVector(planeNormOther);
  double angleOther = dir1.Dot(planeNormOther);
  if (fabs(angleOther) > fabs(angle))
  {
    projNorm = planeNorm;
    planeNorm = planeNormOther;
  }

  // Compute the distance from the camera to plane of translation
  double d = pose.pos.Dot(planeNorm);
  math::Plane plane(planeNorm, d);
  double dist1 = plane.Distance(origin1, dir1);
  double dist2 = plane.Distance(origin2, dir2);

  // Compute two points on the plane. The first point is the current
  // mouse position, the second is the previous mouse position
  p1 = origin1 + dir1 * dist1;
  p2 = origin2 + dir2 * dist2;

  if (_local)
    p1 = p1 - (p1-p2).Dot(projNorm) * projNorm;

  math::Vector3 distance = p1 - p2;

  if (!_local)
    distance *= moveVector;

  pose.pos = this->mouseMoveVisStartPose.pos + distance;

  if (this->mouseEvent.shift)
  {
    if (ceil(pose.pos.x) - pose.pos.x <= .4)
        pose.pos.x = ceil(pose.pos.x);
    else if (pose.pos.x - floor(pose.pos.x) <= .4)
      pose.pos.x = floor(pose.pos.x);

    if (ceil(pose.pos.y) - pose.pos.y <= .4)
        pose.pos.y = ceil(pose.pos.y);
    else if (pose.pos.y - floor(pose.pos.y) <= .4)
      pose.pos.y = floor(pose.pos.y);

    if (moveVector.z > 0.0)
    {
      if (ceil(pose.pos.z) - pose.pos.z <= .4)
        pose.pos.z = ceil(pose.pos.z);
      else if (pose.pos.z - floor(pose.pos.z) <= .4)
        pose.pos.z = floor(pose.pos.z);
    }
  }

  if (this->keyText != "z" && !_local)
    pose.pos.z = _vis->GetPose().pos.z;

  _vis->SetPose(pose);
}

/////////////////////////////////////////////////
void GLWidget::OnSelectionMsg(ConstSelectionPtr &_msg)
{
  if (_msg->has_selected())
  {
    if (_msg->selected())
    {
      this->SetSelectedVisual(this->scene->GetVisual(_msg->name()));
    }
    else
    {
      this->SetSelectedVisual(rendering::VisualPtr());
      this->SetMouseMoveVisual(rendering::VisualPtr());
    }
  }
}

/////////////////////////////////////////////////
void GLWidget::SetSelectedVisual(rendering::VisualPtr _vis)
{
  if (this->selectedVis)
  {
    this->selectedVis->SetHighlighted(false);
  }

  this->selectedVis = _vis;

  if (this->selectedVis && !this->selectedVis->IsPlane())
  {
    this->selectedVis->SetHighlighted(true);
  }
}

/////////////////////////////////////////////////
void GLWidget::SetMouseMoveVisual(rendering::VisualPtr _vis)
{
  this->mouseMoveVis = _vis;
  if (_vis)
    this->mouseVisualScale = _vis->GetScale();
  else this->mouseVisualScale = math::Vector3::One;
}

/////////////////////////////////////////////////
void GLWidget::OnManipMode(const std::string &_mode)
{
  this->state = _mode;

  if (this->selectionObj->GetMode() != rendering::SelectionObj::SELECTION_NONE)
  {
    this->selectionObj->SetMode(this->state);
  }
  if (this->selectedVis && !this->selectedVis->IsPlane())
  {
    this->selectionObj->Attach(this->selectedVis);
    this->selectionObj->SetMode(this->state);
  }
}

/////////////////////////////////////////////////
void GLWidget::Paste(const std::string &_object)
{
  if (!_object.empty())
  {
    this->ClearSelection();
    if (this->entityMaker)
      this->entityMaker->Stop();

    this->modelMaker.InitFromModel(_object);
    this->entityMaker = &this->modelMaker;
    this->entityMaker->Start(this->userCamera);
    gui::Events::manipMode("make_entity");
  }
}

/////////////////////////////////////////////////
void GLWidget::PublishVisualPose(rendering::VisualPtr _vis)
{
  if (_vis)
  {
    // Check to see if the visual is a model.
    if (gui::get_entity_id(_vis->GetName()))
    {
      msgs::Model msg;
      msg.set_id(gui::get_entity_id(_vis->GetName()));
      msg.set_name(_vis->GetName());

      msgs::Set(msg.mutable_pose(), _vis->GetWorldPose());
      this->modelPub->Publish(msg);
    }
    // Otherwise, check to see if the visual is a light
    else if (this->scene->GetLight(_vis->GetName()))
    {
      msgs::Light msg;
      msg.set_name(_vis->GetName());
      msgs::Set(msg.mutable_pose(), _vis->GetWorldPose());
      this->lightPub->Publish(msg);
    }
  }
}

/////////////////////////////////////////////////
void GLWidget::PublishVisualScale(rendering::VisualPtr _vis)
{
  if (_vis)
  {
    // Check to see if the visual is a model.
    if (gui::get_entity_id(_vis->GetName()))
    {
      msgs::Model msg;
      msg.set_id(gui::get_entity_id(_vis->GetName()));
      msg.set_name(_vis->GetName());

      msgs::Set(msg.mutable_scale(), _vis->GetScale());
      this->modelPub->Publish(msg);
    }
  }
}

/////////////////////////////////////////////////
void GLWidget::ClearSelection()
{
  if (this->hoverVis)
  {
    this->hoverVis->SetEmissive(common::Color(0, 0, 0));
    this->hoverVis.reset();
  }

  this->SetSelectedVisual(rendering::VisualPtr());

  this->scene->SelectVisual("", "normal");
}

/////////////////////////////////////////////////
void GLWidget::OnSetSelectedEntity(const std::string &_name,
                                   const std::string &_mode)

{
  std::map<std::string, unsigned int>::iterator iter;
  if (!_name.empty())
  {
    std::string name = _name;
    boost::replace_first(name, gui::get_world()+"::", "");

    this->SetSelectedVisual(this->scene->GetVisual(name));
    this->scene->SelectVisual(name, _mode);
  }
  else
  {
    this->SetSelectedVisual(rendering::VisualPtr());
    this->scene->SelectVisual("", _mode);
  }

  this->hoverVis.reset();
}

/////////////////////////////////////////////////
void GLWidget::PushHistory(const std::string &_visName, const math::Pose &_pose)
{
  if (this->moveHistory.size() == 0 ||
      this->moveHistory.back().first != _visName ||
      this->moveHistory.back().second != _pose)
  {
    this->moveHistory.push_back(std::make_pair(_visName, _pose));
  }
}

/////////////////////////////////////////////////
void GLWidget::PopHistory()
{
  if (this->moveHistory.size() > 0)
  {
    msgs::Model msg;
    msg.set_id(gui::get_entity_id(this->moveHistory.back().first));
    msg.set_name(this->moveHistory.back().first);

    msgs::Set(msg.mutable_pose(), this->moveHistory.back().second);
    this->scene->GetVisual(this->moveHistory.back().first)->SetWorldPose(
        this->moveHistory.back().second);

    this->modelPub->Publish(msg);

    this->moveHistory.pop_back();
  }
}

/////////////////////////////////////////////////
void GLWidget::OnRequest(ConstRequestPtr &_msg)
{
  if (_msg->request() == "entity_delete")
  {
    if (this->selectedVis && this->selectedVis->GetName() == _msg->data())
    {
      this->SetSelectedVisual(rendering::VisualPtr());
    }
    if (this->mouseMoveVis && this->mouseMoveVis->GetName() == _msg->data())
      this->SetMouseMoveVisual(rendering::VisualPtr());
  }
}
