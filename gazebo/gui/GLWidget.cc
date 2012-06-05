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

#include "common/Exception.hh"
#include "math/gzmath.hh"

#include "transport/transport.hh"

#include "rendering/RenderEvents.hh"
#include "rendering/Rendering.hh"
#include "rendering/Visual.hh"
#include "rendering/WindowManager.hh"
#include "rendering/Scene.hh"
#include "rendering/UserCamera.hh"
#include "rendering/SelectionObj.hh"
#include "rendering/OrbitViewController.hh"
#include "rendering/FPSViewController.hh"

#include "gui/Gui.hh"
#include "gui/ModelRightMenu.hh"
#include "gui/GuiEvents.hh"
#include "gui/GLWidget.hh"

using namespace gazebo;
using namespace gui;

extern bool g_fullscreen;
extern ModelRightMenu *g_modelRightMenu;

/////////////////////////////////////////////////
GLWidget::GLWidget(QWidget *_parent)
  : QWidget(_parent)
{
  this->state = "normal";

  // This mouse offset is a hack. The glwindow window is not properly sized
  // when first created....
  this->mouseOffset = -10;
  this->setFocusPolicy(Qt::StrongFocus);

  this->windowId = -1;

  setAttribute(Qt::WA_OpaquePaintEvent, true);
  setAttribute(Qt::WA_PaintOnScreen, true);
//  setMinimumSize(320, 240);

  this->renderFrame = new QFrame;
  this->renderFrame->setLineWidth(1);
  this->renderFrame->setFrameShadow(QFrame::Sunken);
  this->renderFrame->setFrameShape(QFrame::Box);
  this->renderFrame->show();
  QVBoxLayout *mainLayout = new QVBoxLayout;
  mainLayout->addWidget(this->renderFrame);
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
       boost::bind(&GLWidget::OnSetSelectedEntity, this, _1)));

  this->renderFrame->setMouseTracking(true);
  this->setMouseTracking(true);

  this->entityMaker = NULL;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->modelPub = this->node->Advertise<msgs::Model>("~/model/modify");
  this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");
  this->selectionSub = this->node->Subscribe("~/selection",
      &GLWidget::OnSelectionMsg, this);
  this->requestSub = this->node->Subscribe("~/request",
      &GLWidget::OnRequest, this);

  this->installEventFilter(this);
  this->keyModifiers = 0;

  this->selectionObj = NULL;
  this->mouseMoveVis.reset();
}

/////////////////////////////////////////////////
GLWidget::~GLWidget()
{
  this->connections.clear();
  this->node.reset();
  this->modelPub.reset();
  this->selectionSub.reset();

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
  this->windowId = rendering::WindowManager::Instance()->CreateWindow(
      this->GetOgreHandle(), this->width(), this->height());

  QWidget::showEvent(_event);

  if (this->userCamera)
    rendering::WindowManager::Instance()->SetCamera(this->windowId,
                                                    this->userCamera);
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
    rendering::WindowManager::Instance()->Moved(this->windowId);
  }
}

/////////////////////////////////////////////////
void GLWidget::paintEvent(QPaintEvent *_e)
{
  if (this->userCamera && this->userCamera->IsInitialized())
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
    rendering::WindowManager::Instance()->Resize(this->windowId,
        _e->size().width(), _e->size().height());
    this->userCamera->Resize(_e->size().width(), _e->size().height());
  }
}

/////////////////////////////////////////////////
void GLWidget::keyPressEvent(QKeyEvent *_event)
{
  if (!this->scene)
    return;

  std::string keyText = _event->text().toStdString();
  this->keyModifiers = _event->modifiers();

  // Toggle full screen
  if (_event->key() == Qt::Key_F11)
  {
    this->mouseOffset = 0;
    g_fullscreen = !g_fullscreen;
    gui::Events::fullScreen(g_fullscreen);
  }

  // Return the mouse interaction state to normal
  if (this->state == "ring")
  {
    if (_event->key() == Qt::Key_Return)
    {
      event::Events::setSelectedEntity("");
    }
    else if (_event->key() == Qt::Key_Escape)
    {
      this->PopHistory();
      event::Events::setSelectedEntity("");
    }
  }

  this->mouseEvent.control =
    this->keyModifiers & Qt::ControlModifier ? true : false;
  this->mouseEvent.shift =
    this->keyModifiers & Qt::ShiftModifier ? true : false;

  this->userCamera->HandleKeyPressEvent(keyText);
}

/////////////////////////////////////////////////
void GLWidget::keyReleaseEvent(QKeyEvent *_event)
{
  if (!this->scene)
    return;

  this->keyModifiers = _event->modifiers();

  if (!(this->keyModifiers & Qt::ControlModifier))
    this->setCursor(Qt::ArrowCursor);

  if (this->state == "ring")
    this->OnKeyReleaseRing(_event);

  if (this->keyModifiers & Qt::ControlModifier &&
      _event->key() == Qt::Key_Z)
  {
    this->PopHistory();
  }

  this->mouseEvent.control =
    this->keyModifiers & Qt::ControlModifier ? true : false;
  this->mouseEvent.shift =
    this->keyModifiers & Qt::ShiftModifier ? true : false;
  this->userCamera->HandleKeyReleaseEvent(_event->text().toStdString());
}

/////////////////////////////////////////////////
void GLWidget::OnKeyReleaseRing(QKeyEvent *_event)
{
  if (this->keyModifiers & Qt::ControlModifier)
  {
    if (_event->key() == Qt::Key_C)
    {
      if (this->mouseMoveVis)
        this->copiedObject = this->mouseMoveVis->GetName();
      else if (this->hoverVis)
        this->copiedObject = this->hoverVis->GetName();
      else
        this->copiedObject.clear();
    }
    else if (_event->key() == Qt::Key_V)
    {
      this->Paste(this->copiedObject);
    }
  }
}

/////////////////////////////////////////////////
void GLWidget::mousePressEvent(QMouseEvent *_event)
{
  if (!this->scene)
    return;

  this->mouseEvent.pressPos.Set(_event->pos().x() + this->mouseOffset,
                                 _event->pos().y() + this->mouseOffset);
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
  gui::Events::mousePress(this->mouseEvent);

  if (this->state == "ring")
    this->OnMousePressRing();
  else if (this->state == "make_entity")
    this->OnMousePressMakeEntity();
  else
    this->OnMousePressNormal();
}

/////////////////////////////////////////////////
void GLWidget::OnMousePressNormal()
{
  this->setCursor(Qt::ArrowCursor);
  this->userCamera->HandleMouseEvent(this->mouseEvent);
}

/////////////////////////////////////////////////
void GLWidget::OnMousePressMakeEntity()
{
  this->setCursor(Qt::ArrowCursor);
  if (this->entityMaker)
    this->entityMaker->OnMousePush(this->mouseEvent);
}

/////////////////////////////////////////////////
void GLWidget::OnMousePressRing()
{
  if (this->selectionObj)
  {
    this->scene->GetVisualAt(this->userCamera, this->mouseEvent.pressPos,
                             this->selectionMod);
    if (!this->selectionMod.empty())
    {
      this->mouseMoveVisStartPose = this->mouseMoveVis->GetWorldPose();
    }
    else
      this->userCamera->HandleMouseEvent(this->mouseEvent);
  }

  if (this->mouseMoveVis)
  {
    this->setCursor(Qt::PointingHandCursor);
    this->onShiftMousePos = QCursor::pos();
    this->mouseMoveVisStartPose = this->mouseMoveVis->GetWorldPose();
  }
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

  this->mouseEvent.pos.Set(_event->pos().x()+this->mouseOffset,
                            _event->pos().y()+this->mouseOffset);
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

  // Update the view depending on the current GUI state
  if (this->state == "ring")
    this->OnMouseMoveRing();
  else if (this->state == "make_entity")
    this->OnMouseMoveMakeEntity();
  else
    this->OnMouseMoveNormal();

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
void GLWidget::OnMouseMoveRing()
{
  // Handle moving a visual around using the selection object
  if (this->selectionObj && !this->selectionMod.empty())
  {
    this->scene->GetSelectionObj()->SetActive(true);
    if (this->selectionMod.substr(0, 3) == "rot")
    {
      this->RotateEntity(this->mouseMoveVis);
      return;
    }
    else
    {
      this->TranslateEntity(this->mouseMoveVis);
      return;
    }
  }

  // Higlight other objects when mouse passes over them
  if (this->selectionMod.empty() && !this->mouseEvent.dragging)
  {
    rendering::VisualPtr newHoverVis;
    std::string mod;
    newHoverVis = this->scene->GetVisualAt(this->userCamera,
                                           this->mouseEvent.pos, mod);
    if (!mod.empty())
    {
      this->setCursor(Qt::PointingHandCursor);
      this->selectionObj->SetHighlight(mod);
    }
    else
    {
      this->setCursor(Qt::ArrowCursor);
      this->selectionObj->SetHighlight("");
    }

    if (newHoverVis && !newHoverVis->IsPlane())
    {
      if (this->hoverVis)
        this->hoverVis->SetEmissive(common::Color(0, 0, 0));

      this->hoverVis = this->scene->GetVisual(newHoverVis->GetName().substr(0,
            newHoverVis->GetName().find("::")));

      this->setCursor(Qt::PointingHandCursor);
      this->hoverVis->SetEmissive(common::Color(0.8, 0.8, 0.8));
    }
    else if (this->hoverVis)
    {
      this->hoverVis->SetEmissive(common::Color(0, 0, 0));
      this->hoverVis.reset();
    }
  }
  // Use the smart move feature when dragging the selected object
  else if (this->mouseEvent.dragging && this->hoverVis && this->mouseMoveVis &&
           this->hoverVis == this->mouseMoveVis)
  {
    this->SmartMoveVisual(this->mouseMoveVis);
  }
  // Move the camera when not moving the object
  else if (this->selectionMod.empty() && this->mouseEvent.dragging)
  {
    this->userCamera->HandleMouseEvent(this->mouseEvent);
  }
  else
  {
    if (this->hoverVis)
      this->hoverVis->SetEmissive(common::Color(0, 0, 0));
    this->hoverVis.reset();
  }
}

/////////////////////////////////////////////////
void GLWidget::OnMouseMoveNormal()
{
  if (this->mouseEvent.dragging)
  {
    this->scene->GetSelectionObj()->SetActive(false);
    this->userCamera->HandleMouseEvent(this->mouseEvent);
  }
}

/////////////////////////////////////////////////
void GLWidget::mouseReleaseEvent(QMouseEvent *_event)
{
  if (!this->scene)
    return;

  this->mouseEvent.pos.Set(_event->pos().x()+this->mouseOffset,
                            _event->pos().y()+this->mouseOffset);
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

  gui::Events::mouseRelease(this->mouseEvent);
  emit clicked();

  this->scene->GetSelectionObj()->SetActive(false);

  if (this->state == "ring")
    this->OnMouseReleaseRing();
  else if (this->state == "make_entity")
    this->OnMouseReleaseMakeEntity();
  else
    this->OnMouseReleaseNormal();
}

//////////////////////////////////////////////////
void GLWidget::OnMouseReleaseMakeEntity()
{
  if (this->entityMaker)
    this->entityMaker->OnMouseRelease(this->mouseEvent);
}

//////////////////////////////////////////////////
void GLWidget::OnMouseReleaseRing()
{
  if (!this->mouseEvent.dragging)
  {
    if (this->mouseEvent.button == common::MouseEvent::LEFT)
    {
      this->hoverVis = this->scene->GetVisualAt(this->userCamera,
                                                this->mouseEvent.pos);

      // Select the current hovervis for positioning
      if (this->hoverVis && !this->hoverVis->IsPlane())
      {
        this->hoverVis = this->scene->GetVisual(
            this->hoverVis->GetName().substr(0,
              this->hoverVis->GetName().find("::")));

        if (this->hoverVis != this->mouseMoveVis)
        {
          if (this->mouseMoveVis)
            this->PublishVisualPose(this->mouseMoveVis);
          this->PushHistory(this->hoverVis->GetName(),
                            this->hoverVis->GetWorldPose());
        }

        this->mouseMoveVis = this->hoverVis;
        this->selectionObj = this->scene->GetSelectionObj();
        event::Events::setSelectedEntity(this->mouseMoveVis->GetName());

        this->scene->SelectVisual(this->mouseMoveVis->GetName());

        this->hoverVis.reset();
      }
      else if (this->mouseMoveVis)
      {
        event::Events::setSelectedEntity("");
      }
    }
    else if (this->mouseEvent.button == common::MouseEvent::RIGHT)
    {
      if (!this->mouseEvent.dragging)
        if (this->hoverVis)
        {
          g_modelRightMenu->Run(this->hoverVis->GetName(), QCursor::pos());
        }
    }
  }

  this->selectionMod.clear();
  this->userCamera->HandleMouseEvent(this->mouseEvent);
}

//////////////////////////////////////////////////
void GLWidget::OnMouseReleaseNormal()
{
  this->userCamera->HandleMouseEvent(this->mouseEvent);
  if (!this->mouseEvent.dragging)
  {
    if (this->mouseEvent.button == common::MouseEvent::RIGHT)
    {
      this->hoverVis = this->scene->GetModelVisualAt(this->userCamera,
                                                     this->mouseEvent.pos);
      if (this->hoverVis)
      {
        g_modelRightMenu->Run(this->hoverVis->GetName(), QCursor::pos());
      }
    }
  }
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

  this->userCamera->SetWorldPose(math::Pose(-5, 0, 1, 0, GZ_DTOR(11.31), 0));

  if (this->windowId >= 0)
  {
    rendering::WindowManager::Instance()->SetCamera(this->windowId,
                                                    this->userCamera);
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
  this->mouseMoveVis.reset();
  this->hoverVis.reset();
  this->selectionMod.clear();
  this->selectionId = 0;
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
  this->mouseMoveVis.reset();

  this->ViewScene(rendering::get_scene(_name));
}

/////////////////////////////////////////////////
void GLWidget::OnMoveMode(bool _mode)
{
  if (_mode)
  {
    this->entityMaker = NULL;
    this->state = "normal";
  }
}

/////////////////////////////////////////////////
void GLWidget::OnCreateEntity(const std::string &_type,
                              const std::string &_data)
{
  this->ClearSelection();

  if (this->entityMaker)
    this->entityMaker->Stop();

  if (_type == "box")
    this->entityMaker = &this->boxMaker;
  else if (_type == "sphere")
    this->entityMaker = &this->sphereMaker;
  else if (_type == "cylinder")
    this->entityMaker = &this->cylinderMaker;
  else if (_type == "mesh" && !_data.empty())
  {
    this->meshMaker.Init(_data);
    this->entityMaker = &this->meshMaker;
  }
  else if (_type == "model" && !_data.empty())
  {
    this->modelMaker.InitFromFile(_data);
    this->entityMaker = &this->modelMaker;
  }
  else if (_type == "pointlight")
    this->entityMaker =  &this->pointLightMaker;
  else if (_type == "spotlight")
    this->entityMaker =  &this->spotLightMaker;
  else if (_type == "directionallight")
    this->entityMaker =  &this->directionalLightMaker;
  else
    this->entityMaker = NULL;

  if (this->entityMaker)
  {
    gui::Events::manipMode("make_entity");
    // TODO: change the cursor to a cross
    this->entityMaker->Start(this->userCamera);
  }
  else
  {
    this->state = "normal";
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
void GLWidget::RotateEntity(rendering::VisualPtr &_vis)
{
  math::Vector3 planeNorm, planeNorm2;
  math::Vector3 p1, p2;
  math::Vector3 a, b;
  math::Vector3 ray(0, 0, 0);

  math::Pose pose = _vis->GetPose();

  // Figure out which axis to rotate around
  if (this->selectionMod == "rotx")
    ray.x = 1.0;
  else if (this->selectionMod == "roty")
    ray.y = 1.0;
  else if (this->selectionMod == "rotz")
    ray.z = 1.0;

  // Compute the normal to the plane on which to rotate
  planeNorm = pose.rot.RotateVector(ray);
  double d = -pose.pos.GetDotProd(planeNorm);

  if (!this->userCamera->GetWorldPointOnPlane(this->mouseEvent.pos.x,
       this->mouseEvent.pos.y, math::Plane(planeNorm, d), p1))
  {
    gzerr << "Invalid mouse point\n";
  }

  if (!this->userCamera->GetWorldPointOnPlane(this->mouseEvent.prevPos.x,
       this->mouseEvent.prevPos.y, math::Plane(planeNorm, d), p2))
  {
    gzerr << "Invalid mouse point\n";
  }

  // Get point vectors relative to the entity's pose
  a = p1 - _vis->GetWorldPose().pos;
  b = p2 - _vis->GetWorldPose().pos;

  a.Normalize();
  b.Normalize();

  // Get the angle between the two vectors. This is the amount to
  // rotate the entity
  float angle = acos(a.GetDotProd(b));
  if (math::isnan(angle))
    angle = 0;

  // Compute the normal to the plane which is defined by the
  // direction of rotation
  planeNorm2 = a.GetCrossProd(b);
  planeNorm2.Normalize();

  // Switch rotation direction if the two normals don't line up
  if (planeNorm.GetDotProd(planeNorm2) > 0)
    angle *= -1;

    math::Quaternion delta;
    delta.SetFromAxis(ray.x, ray.y, ray.z, angle);

    _vis->SetRotation(pose.rot * delta);

    // TODO: send message

/*  if (entity->GetType() == Entity::MODEL)
  {
    Quatern delta;
    delta.SetFromAxis(ray.x, ray.y, ray.z, angle);

    pose.rot = pose.rot * delta;
    entity->SetWorldPose(pose);
  }
  else
  {
    ((Body*)entity)->SetTorque(planeNorm * angle * Gui::forceMultiplier);
  }
*/
}

/////////////////////////////////////////////////
void GLWidget::TranslateEntity(rendering::VisualPtr &_vis)
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
  math::Vector3 planeNorm(0, 0, 1);
  if (!this->selectionMod.empty())
  {
    if (this->selectionMod == "transx")
      moveVector.x = 1;
    else if (this->selectionMod == "transy")
      moveVector.y = 1;
    else if (this->selectionMod == "transz")
    {
      moveVector.z = 1;
      planeNorm.Set(1, 0, 0);
    }
  }
  else
    moveVector.Set(1, 1, 0);

  // Compute the distance from the camera to plane of translation
  double d = -pose.pos.GetDotProd(planeNorm);
  math::Plane plane(planeNorm, d);
  double dist1 = plane.Distance(origin1, dir1);
  double dist2 = plane.Distance(origin2, dir2);

  // Compute two points on the plane. The first point is the current
  // mouse position, the second is the previous mouse position
  p1 = origin1 + dir1 * dist1;
  p2 = origin2 + dir2 * dist2;

  moveVector *= p1 - p2;
  pose.pos = this->mouseMoveVisStartPose.pos + moveVector;

  if (!this->mouseEvent.shift)
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

  _vis->SetPose(pose);
}

/////////////////////////////////////////////////
void GLWidget::OnSelectionMsg(ConstSelectionPtr &_msg)
{
  if (_msg->has_selected())
  {
    if (_msg->selected())
      this->mouseMoveVis = this->scene->GetVisual(_msg->name());
    else
      this->mouseMoveVis.reset();
  }
}

/////////////////////////////////////////////////
void GLWidget::OnManipMode(const std::string &_mode)
{
  if (this->state == "ring" && _mode != "ring")
  {
    this->ClearSelection();
  }

  this->state = _mode;
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
  msgs::Model msg;
  msg.set_id(gui::get_entity_id(_vis->GetName()));
  msg.set_name(_vis->GetName());

  msgs::Set(msg.mutable_pose(), _vis->GetWorldPose());
  this->modelPub->Publish(msg);
}

/////////////////////////////////////////////////
void GLWidget::ClearSelection()
{
  if (this->hoverVis)
  {
    this->hoverVis->SetEmissive(common::Color(0, 0, 0));
    this->hoverVis.reset();
  }

  if (this->mouseMoveVis)
  {
    this->PublishVisualPose(this->mouseMoveVis);
    this->mouseMoveVis->SetEmissive(common::Color(0, 0, 0));
    this->mouseMoveVis.reset();
  }

  this->scene->SelectVisual("");
  this->selectionObj = NULL;
}

/////////////////////////////////////////////////
void GLWidget::OnSetSelectedEntity(const std::string &_name)
{
  std::map<std::string, unsigned int>::iterator iter;
  if (!_name.empty())
  {
    std::string name = _name;
    boost::replace_first(name, gui::get_world()+"::", "");

    this->mouseMoveVis = this->scene->GetVisual(name);
    this->selectionObj = this->scene->GetSelectionObj();
    this->scene->SelectVisual(name);

    gui::Events::manipMode("ring");
  }
  else
  {
    this->scene->SelectVisual("");
    gui::Events::manipMode("normal");
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
    if (this->hoverVis && this->hoverVis->GetName() == _msg->data())
      this->hoverVis.reset();
    if (this->mouseMoveVis && this->mouseMoveVis->GetName() == _msg->data())
      this->mouseMoveVis.reset();
  }
}
