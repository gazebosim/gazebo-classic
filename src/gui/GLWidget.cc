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
#include "math/gzmath.h"

#include "transport/transport.h"

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
#include "gui/GuiEvents.hh"
#include "gui/GLWidget.hh"

using namespace gazebo;
using namespace gui;

extern bool g_fullscreen;

/////////////////////////////////////////////////
GLWidget::GLWidget(QWidget *_parent)
  : QWidget(_parent)
{
  this->copy = false;

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
        boost::bind(&GLWidget::OnCreateEntity, this, _1)));

  this->connections.push_back(
      gui::Events::ConnectFPS(
        boost::bind(&GLWidget::OnFPS, this)));

  this->connections.push_back(
      gui::Events::ConnectOrbit(
        boost::bind(&GLWidget::OnOrbit, this)));

  this->connections.push_back(
      gui::Events::ConnectInsertModel(
        boost::bind(&GLWidget::OnMouseMoveVisual, this, _1)));


  this->renderFrame->setMouseTracking(true);
  this->setMouseTracking(true);

  this->entityMaker = NULL;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->modelPub = this->node->Advertise<msgs::Model>("~/model/modify");
  this->factoryPub = this->node->Advertise<msgs::Factory>("~/factory");
  this->selectionSub = this->node->Subscribe("~/selection",
      &GLWidget::OnSelectionMsg, this);

  this->installEventFilter(this);
  this->keyModifiers = 0;
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
  printf("Enter Event\n");
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
  if (this->userCamera && this->userCamera->initialized)
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


  if (this->keyModifiers & Qt::ControlModifier)
  {
    if (_event->key() == Qt::Key_C)
    {
      if (this->hoverVis)
        this->copiedObject =
          this->hoverVis->GetParent()->GetParent()->GetName();
      else if (this->mouseMoveVis)
        this->copiedObject = this->mouseMoveVis->GetName();
    }
    else if (_event->key() == Qt::Key_V)
    {
      this->Paste(this->copiedObject);
    }
  }

  this->mouseEvent.control =
    this->keyModifiers & Qt::ControlModifier ? true : false;
  this->mouseEvent.shift =
    this->keyModifiers & Qt::ShiftModifier ? true : false;
  this->userCamera->HandleKeyReleaseEvent(_event->text().toStdString());
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

  if (this->entityMaker)
    this->entityMaker->OnMousePush(this->mouseEvent);
  /*else if (this->selectionVis)
  {
    this->scene->GetVisualAt(this->userCamera, this->mouseEvent.pressPos,
                             this->selectionMod);
  }*/
  else if (this->mouseMoveVis && _event->button() == Qt::MidButton)
  {
    this->onShiftMousePos = QCursor::pos();
    this->mouseMoveVisStartPose = this->mouseMoveVis->GetWorldPose();
    QCursor::setPos(this->onShiftMousePos.x() + 50, this->onShiftMousePos.y());
  }

  this->userCamera->HandleMouseEvent(this->mouseEvent);
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

  // Handle moving a visual around with the mouse
  if (this->mouseMoveVis)
  {
    // Get the point on the plane which correspoinds to the mouse
    math::Vector3 pp;
    if (!this->userCamera->GetWorldPointOnPlane(
          this->mouseEvent.pos.x, this->mouseEvent.pos.y,
          math::Plane(math::Vector3(0, 0, 1)),pp))
    {
      gzerr << "Unable to get Point on plane....\n";
    }

    // Rotate the visual using the middle mouse button
    if (this->mouseEvent.buttons & common::MouseEvent::MIDDLE)
    {
      math::Vector3 rpy = this->mouseMoveVisStartPose.rot.GetAsEuler();
      math::Vector3 mp = this->mouseMoveVisStartPose.pos;
      double yaw = atan2((pp.y - mp.y) , (pp.x - mp.x)) + M_PI * 0.5;
      if (!this->mouseEvent.shift)
      {
        yaw = rint(yaw / (M_PI * .25)) * (M_PI * 0.25);
      }
      this->mouseMoveVis->SetWorldRotation(
          math::Quaternion(rpy.x, rpy.y, rpy.z + yaw));
    }
    else
    {
      // Translate the visual by just moving the mouse around
      if (!this->mouseEvent.shift)
      {
        pp.x = rint(pp.x);
        pp.y = rint(pp.y);
      }

      math::Vector3 pt(pp.x, pp.y, 100.0);

      std::vector<rendering::VisualPtr> below;
      this->scene->GetVisualsBelowPoint(pt, below);
      double maxZ = 0;

      for (unsigned int i = 0; i < below.size(); ++i)
      {
        if (below[i]->GetName().find(this->mouseMoveVis->GetName()) != 0
            && below[i]->GetBoundingBox().max.z > maxZ)
          maxZ = below[i]->GetBoundingBox().max.z;
      }
      pp.z = maxZ;

      this->mouseMoveVis->SetWorldPosition(pp);
    }
  }

  if (_event->buttons())
  {
    this->mouseEvent.dragging = true;
  }
  else
  {
    std::string mod;
    this->mouseEvent.dragging = false;

    rendering::VisualPtr newHoverVis;

    if (this->keyModifiers & Qt::ControlModifier && !this->entityMaker)
    {
      if (this->mouseMoveVis)
      {
        newHoverVis = this->scene->GetVisualAt(this->userCamera,
                                               this->mouseEvent.pos, mod);
      }
      else
      {
        newHoverVis = this->scene->GetVisualAt(this->userCamera,
                                               this->mouseEvent.pos);
      }

      if (!mod.empty())
      {
        this->setCursor(Qt::SizeAllCursor);
        this->scene->GetSelectionObj()->SetHighlight(mod);
      }
      else if (newHoverVis && !this->mouseMoveVis)
      {
        this->scene->GetSelectionObj()->SetHighlight("");

        if (!newHoverVis->IsPlane())
        {
          if (this->hoverVis && this->hoverVis->GetName() !=
              newHoverVis->GetName())
          {
            this->hoverVis->SetEmissive(common::Color(0, 0, 0));
          }
          newHoverVis->SetEmissive(common::Color(.5, .5, .5));
          this->setCursor(Qt::PointingHandCursor);
        }
        else
        {
          if (this->hoverVis)
          {
            this->hoverVis->SetEmissive(common::Color(0, 0, 0));
          }

          this->setCursor(Qt::ArrowCursor);
          newHoverVis.reset();
        }
      }
      else
      {
        this->scene->GetSelectionObj()->SetHighlight("");
        if (this->hoverVis)
        {
          this->hoverVis->SetEmissive(common::Color(0, 0, 0));
        }

        this->setCursor(Qt::ArrowCursor);
      }
    }
    else
    {
      if (this->hoverVis)
      {
        this->hoverVis->SetEmissive(common::Color(0, 0, 0));
      }
      this->setCursor(Qt::ArrowCursor);
    }

    this->hoverVis = newHoverVis;
  }

  if (this->mouseEvent.dragging)
  {
    this->scene->GetSelectionObj()->SetActive(false);

    if (this->entityMaker)
      this->entityMaker->OnMouseDrag(this->mouseEvent);
    /*else if (this->selectionVis && !this->selectionMod.empty())
    {
      this->scene->GetSelectionObj()->SetActive(true);
      if (this->selectionMod.substr(0, 3) == "rot")
        this->RotateEntity(this->selectionVis);
      else
        this->TranslateEntity(this->selectionVis);
    }*/
    else if (!(this->mouseMoveVis &&
               this->mouseEvent.buttons & common::MouseEvent::MIDDLE))
    {
      this->userCamera->HandleMouseEvent(this->mouseEvent);
    }

    this->mouseEvent.prevPos = this->mouseEvent.pos;
  }
  else
  {
    if (this->entityMaker)
      this->entityMaker->OnMouseMove(this->mouseEvent);
    else
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

  if (this->entityMaker)
    this->entityMaker->OnMouseRelease(this->mouseEvent);
  else if (this->mouseEvent.dragging == false)
  {
    if (_event->button() & Qt::LeftButton)
    {
      if (this->hoverVis)
      {
        this->mouseMoveVis = this->hoverVis;
        if (this->mouseMoveVis)
        {
          // Get the model associated with the visual
          this->mouseMoveVis = this->mouseMoveVis->GetParent()->GetParent();
          this->scene->SelectVisual(this->mouseMoveVis->GetName());
        }
      }
      else if (this->mouseMoveVis)
      {
        this->scene->SelectVisual("");
        if (this->copy)
        {
          msgs::Factory msg;
          msg.set_clone_model_name(this->copiedObject);
          msgs::Set(msg.mutable_pose(), this->mouseMoveVis->GetWorldPose());
          this->factoryPub->Publish(msg);
          this->copy = false;
          this->scene->RemoveVisual(this->mouseMoveVis);
          this->mouseMoveVis.reset();
        }
        else
        {
          msgs::Model msg;
          msg.set_id(gui::get_entity_id(this->mouseMoveVis->GetName()));
          msg.set_name(this->mouseMoveVis->GetName());

          msgs::Set(msg.mutable_pose(), this->mouseMoveVis->GetWorldPose());
          this->modelPub->Publish(msg);
        }

        this->mouseMoveVis.reset();
      }
    }
    else
      this->scene->SelectVisual("");
  }
  else if (this->mouseMoveVis && _event->button() == Qt::MidButton)
  {
    QCursor::setPos(this->onShiftMousePos);
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

  this->userCamera->SetWorldPosition(math::Vector3(-5, 0, 5));
  this->userCamera->SetWorldRotation(
      math::Quaternion::EulerToQuaternion(0, 0, 0));

  if (this->windowId >= 0)
    rendering::WindowManager::Instance()->SetCamera(this->windowId,
        this->userCamera);
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
void GLWidget::CreateEntity(const std::string &_name)
{
  if (this->entityMaker)
    this->entityMaker->Stop();

  if (_name == "box")
    this->entityMaker = &this->boxMaker;
  else if (_name == "sphere")
    this->entityMaker = &this->sphereMaker;
  else if (_name == "cylinder")
    this->entityMaker = &this->cylinderMaker;
  else if (_name == "pointlight")
    this->entityMaker =  &this->pointLightMaker;
  else if (_name == "spotlight")
    this->entityMaker =  &this->spotLightMaker;
  else if (_name == "directionallight")
    this->entityMaker =  &this->directionalLightMaker;
  else
    this->entityMaker = NULL;

  if (this->entityMaker)
  {
    // TODO: change the cursor to a cross
    this->entityMaker->Start(this->userCamera);
  }
  else
  {
    // TODO: make sure cursor state stays at the default
  }
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
    // TODO: set cursor to default state
    this->entityMaker = NULL;
  }
}

/////////////////////////////////////////////////
void GLWidget::OnCreateEntity(const std::string &_type)
{
  this->CreateEntity(_type);
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
  if (isnan(angle))
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
  this->userCamera->GetCameraToViewportRay(this->mouseEvent.prevPos.x,
      this->mouseEvent.prevPos.y, origin2, dir2);

  math::Vector3 moveVector(0, 0, 0);
  math::Vector3 planeNorm(0, 0, 1);
  if (this->selectionMod == "transx")
    moveVector.x = 1;
  else if (this->selectionMod == "transy")
    moveVector.y = 1;
  else if (this->selectionMod == "transz")
  {
    moveVector.z = 1;
    planeNorm.Set(1, 0, 0);
  }

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
  pose.pos += moveVector;

  _vis->SetPose(pose);

  /*if (entity->GetType() == Entity::MODEL)
  {
    pose.pos += moveVector;
    entity->SetRelativePose(pose);
  }
  else if (entity->GetType() == Entity::LINK)
  {
    Body *body = (Body*)(entity);
    moveVector *= Gui::forceMultiplier;
    body->SetForce(moveVector);
  }
*/
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
void GLWidget::OnMouseMoveVisual(const std::string &_visualName)
{
  if (_visualName.empty())
    this->mouseMoveVis.reset();
  else
    this->mouseMoveVis = this->scene->GetVisual(_visualName);
}

/////////////////////////////////////////////////
void GLWidget::Paste(const std::string &_object)
{
  rendering::VisualPtr newVis =
    this->scene->CloneVisual(_object, _object + "_clone_tmp");
  this->mouseMoveVis = newVis;
  this->copy = true;
}
