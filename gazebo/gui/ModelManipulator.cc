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

#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/SelectionObj.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiIface.hh"

#include "gazebo/gui/ModelManipulatorPrivate.hh"
#include "gazebo/gui/ModelManipulator.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelManipulator::ModelManipulator()
  : dataPtr(new ModelManipulatorPrivate)
{
  this->dataPtr->initialized = false;
  this->dataPtr->selectionObj.reset();
  this->dataPtr->mouseMoveVis.reset();

  this->dataPtr->manipMode = "";
  this->dataPtr->globalManip = false;
}

/////////////////////////////////////////////////
ModelManipulator::~ModelManipulator()
{
  this->dataPtr->modelPub.reset();
  this->dataPtr->selectionObj.reset();
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void ModelManipulator::Init()
{
  if (this->dataPtr->initialized)
    return;

  rendering::UserCameraPtr cam = gui::get_active_camera();
  if (!cam)
    return;

  if (!cam->GetScene())
    return;

  this->dataPtr->userCamera = cam;
  this->dataPtr->scene =  cam->GetScene();

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();
  this->dataPtr->modelPub =
      this->dataPtr->node->Advertise<msgs::Model>("~/model/modify");
  this->dataPtr->lightPub =
      this->dataPtr->node->Advertise<msgs::Light>("~/light");

  this->dataPtr->selectionObj.reset(new rendering::SelectionObj("__GL_MANIP__",
      this->dataPtr->scene->GetWorldVisual()));
  this->dataPtr->selectionObj->Load();

  this->dataPtr->initialized = true;
}

/////////////////////////////////////////////////
void ModelManipulator::RotateEntity(rendering::VisualPtr &_vis,
    const math::Vector3 &_axis, bool _local)
{
  math::Vector3 normal;

  if (_local)
  {
    if (_axis.x > 0)
      normal = this->dataPtr->mouseMoveVisStartPose.rot.GetXAxis();
    else if (_axis.y > 0)
      normal = this->dataPtr->mouseMoveVisStartPose.rot.GetYAxis();
    else if (_axis.z > 0)
      normal = this->dataPtr->mouseMoveVisStartPose.rot.GetZAxis();
  }
  else
    normal = _axis;

  double offset = this->dataPtr->mouseMoveVisStartPose.pos.Dot(normal);

  math::Vector3 pressPoint;
  this->dataPtr->userCamera->GetWorldPointOnPlane(
      this->dataPtr->mouseEvent.pressPos.x,
      this->dataPtr->mouseEvent.pressPos.y,
      math::Plane(normal, offset), pressPoint);

  math::Vector3 newPoint;
  this->dataPtr->userCamera->GetWorldPointOnPlane(
      this->dataPtr->mouseEvent.pos.x,
      this->dataPtr->mouseEvent.pos.y,
      math::Plane(normal, offset), newPoint);

  math::Vector3 v1 = pressPoint - this->dataPtr->mouseMoveVisStartPose.pos;
  math::Vector3 v2 = newPoint - this->dataPtr->mouseMoveVisStartPose.pos;
  v1 = v1.Normalize();
  v2 = v2.Normalize();
  double signTest = v1.Cross(v2).Dot(normal);
  double angle = atan2((v1.Cross(v2)).GetLength(), v1.Dot(v2));

  if (signTest < 0 )
    angle *= -1;

  if (this->dataPtr->mouseEvent.control)
    angle = rint(angle / (M_PI * 0.25)) * (M_PI * 0.25);

  math::Quaternion rot(_axis, angle);

  if (_local)
    rot = this->dataPtr->mouseMoveVisStartPose.rot * rot;
  else
    rot = rot * this->dataPtr->mouseMoveVisStartPose.rot;

  _vis->SetWorldRotation(rot);
}

/////////////////////////////////////////////////
math::Vector3 ModelManipulator::GetMousePositionOnPlane(
    rendering::CameraPtr _camera,
    const common::MouseEvent &_event)
{
  math::Vector3 origin1, dir1, p1;

  // Cast ray from the camera into the world
  _camera->GetCameraToViewportRay(_event.pos.x, _event.pos.y,
      origin1, dir1);

  // Compute the distance from the camera to plane of translation
  math::Plane plane(math::Vector3(0, 0, 1), 0);
  double dist1 = plane.Distance(origin1, dir1);

  p1 = origin1 + dir1 * dist1;

  return p1;
}

/////////////////////////////////////////////////
math::Vector3 ModelManipulator::SnapPoint(const math::Vector3 &_point,
    double _interval, double _sensitivity)
{
  if (_interval < 0)
  {
    gzerr << "Interval distance must be greater than or equal to 0"
        << std::endl;
    return math::Vector3::Zero;
  }

  if (_sensitivity < 0 || _sensitivity > 1.0)
  {
    gzerr << "Sensitivity must be between 0 and 1" << std::endl;
    return math::Vector3::Zero;
  }

  math::Vector3 point = _point;
  double snap = _interval * _sensitivity;

  double remainder = fmod(point.x, _interval);
  int sign = remainder >= 0 ? 1 : -1;
  if (fabs(remainder) < snap)
      point.x -= remainder;
  else if (fabs(remainder) > (_interval - snap))
      point.x = point.x - remainder + _interval * sign;

  remainder = fmod(point.y, _interval);
  sign = remainder >= 0 ? 1 : -1;
  if (fabs(remainder) < snap)
      point.y -= remainder;
  else if (fabs(remainder) > (_interval - snap))
      point.y = point.y - remainder + _interval * sign;

  remainder = fmod(point.z, _interval);
  sign = remainder >= 0 ? 1 : -1;
  if (fabs(remainder) < snap)
      point.z -= remainder;
  else if (fabs(remainder) > (_interval - snap))
      point.z = point.z - remainder + _interval * sign;

  return point;
}

/////////////////////////////////////////////////
math::Vector3 ModelManipulator::GetMouseMoveDistance(
    rendering::CameraPtr _camera,
    const math::Vector2i &_start, const math::Vector2i &_end,
    const math::Pose &_pose, const math::Vector3 &_axis, bool _local)
{
  math::Pose pose = _pose;

  math::Vector3 origin1, dir1, p1;
  math::Vector3 origin2, dir2, p2;

  // Cast two rays from the camera into the world
  _camera->GetCameraToViewportRay(_end.x,
      _end.y, origin1, dir1);
  _camera->GetCameraToViewportRay(_start.x,
      _start.y, origin2, dir2);

  math::Vector3 planeNorm(0, 0, 0);
  math::Vector3 projNorm(0, 0, 0);

  math::Vector3 planeNormOther(0, 0, 0);

  if (_axis.x > 0 && _axis.y > 0)
  {
    planeNorm.z = 1;
    projNorm.z = 1;
  }
  else if (_axis.z > 0)
  {
    planeNorm.y = 1;
    projNorm.x = 1;
    planeNormOther.x = 1;
  }
  else if (_axis.x > 0)
  {
    planeNorm.z = 1;
    projNorm.y = 1;
    planeNormOther.y = 1;
  }
  else if (_axis.y > 0)
  {
    planeNorm.z = 1;
    projNorm.x = 1;
    planeNormOther.x = 1;
  }

  if (_local)
  {
    planeNorm = pose.rot.RotateVector(planeNorm);
    projNorm = pose.rot.RotateVector(projNorm);
  }

  // Fine tune ray casting: cast a second ray and compare the two rays' angle
  // to plane. Use the one that is less parallel to plane for better results.
  double angle = dir1.Dot(planeNorm);
  if (_local)
    planeNormOther = pose.rot.RotateVector(planeNormOther);
  double angleOther = dir1.Dot(planeNormOther);
  if (fabs(angleOther) > fabs(angle))
  {
    projNorm = planeNorm;
    planeNorm = planeNormOther;
  }

  // Compute the distance from the camera to plane
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
    distance *= _axis;

  return distance;
}

/////////////////////////////////////////////////
math::Vector3 ModelManipulator::GetMouseMoveDistance(const math::Pose &_pose,
    const math::Vector3 &_axis, bool _local) const
{
  return GetMouseMoveDistance(this->dataPtr->userCamera,
      this->dataPtr->mouseStart, math::Vector2i(this->dataPtr->mouseEvent.pos.x,
      this->dataPtr->mouseEvent.pos.y), _pose, _axis, _local);
}

/////////////////////////////////////////////////
void ModelManipulator::ScaleEntity(rendering::VisualPtr &_vis,
    const math::Vector3 &_axis, bool _local)
{
  math::Box bbox = this->dataPtr->mouseVisualBbox;
  math::Pose pose = _vis->GetWorldPose();
  math::Vector3 distance =  this->GetMouseMoveDistance(pose, _axis, _local);

  math::Vector3 bboxSize = bbox.GetSize();
  math::Vector3 scale = (bboxSize + pose.rot.RotateVectorReverse(distance))
      / bboxSize;

  // a bit hacky to check for unit sphere and cylinder simple shapes in order
  // to restrict the scaling dimensions.
  if (this->dataPtr->keyEvent.key == Qt::Key_Shift ||
      _vis->GetName().find("unit_sphere") != std::string::npos)
  {
    if (_axis.x > 0)
    {
      scale.y = scale.x;
      scale.z = scale.x;
    }
    else if (_axis.y > 0)
    {
      scale.x = scale.y;
      scale.z = scale.y;
    }
    else if (_axis.z > 0)
    {
      scale.x = scale.z;
      scale.y = scale.z;
    }
  }
  else if (_vis->GetName().find("unit_cylinder") != std::string::npos)
  {
    if (_axis.x > 0)
    {
      scale.y = scale.x;
    }
    else if (_axis.y > 0)
    {
      scale.x = scale.y;
    }
  }
  else if (_vis->GetName().find("unit_box") != std::string::npos)
  {
  }
  else
  {
    // TODO scaling for complex models are not yet functional.
    // Limit scaling to simple shapes for now.
    gzwarn << " Scaling is currently limited to simple shapes." << std::endl;
    return;
  }

  math::Vector3 newScale = this->dataPtr->mouseVisualScale * scale.GetAbs();

  if (this->dataPtr->mouseEvent.control)
  {
    newScale = SnapPoint(newScale);
    // prevent setting zero scale
    newScale.x = std::max(1e-4, newScale.x);
    newScale.y = std::max(1e-4, newScale.y);
    newScale.z = std::max(1e-4, newScale.z);
  }

  _vis->SetScale(newScale);
}

/////////////////////////////////////////////////
void ModelManipulator::TranslateEntity(rendering::VisualPtr &_vis,
    const math::Vector3 &_axis, bool _local)
{
  math::Pose pose = _vis->GetWorldPose();
  math::Vector3 distance =  this->GetMouseMoveDistance(pose, _axis, _local);

  pose.pos = this->dataPtr->mouseMoveVisStartPose.pos + distance;

  if (this->dataPtr->mouseEvent.control)
  {
    pose.pos = SnapPoint(pose.pos);
  }

  if (!(_axis.z > 0) && !_local)
    pose.pos.z = _vis->GetWorldPose().pos.z;

  _vis->SetWorldPose(pose);
}

/////////////////////////////////////////////////
void ModelManipulator::PublishVisualPose(rendering::VisualPtr _vis)
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
      this->dataPtr->modelPub->Publish(msg);
    }
    // Otherwise, check to see if the visual is a light
    else if (this->dataPtr->scene->GetLight(_vis->GetName()))
    {
      msgs::Light msg;
      msg.set_name(_vis->GetName());
      msgs::Set(msg.mutable_pose(), _vis->GetWorldPose());
      this->dataPtr->lightPub->Publish(msg);
    }
  }
}

/////////////////////////////////////////////////
void ModelManipulator::PublishVisualScale(rendering::VisualPtr _vis)
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
      this->dataPtr->modelPub->Publish(msg);
      _vis->SetScale(this->dataPtr->mouseVisualScale);
    }
  }
}

/////////////////////////////////////////////////
void ModelManipulator::OnMousePressEvent(const common::MouseEvent &_event)
{
  this->dataPtr->mouseEvent = _event;
  this->dataPtr->mouseStart = _event.pressPos;
  this->SetMouseMoveVisual(rendering::VisualPtr());

  rendering::VisualPtr vis;
  rendering::VisualPtr mouseVis
      = this->dataPtr->userCamera->GetVisual(this->dataPtr->mouseEvent.pos);
  // set the new mouse vis only if there are no modifier keys pressed and the
  // entity was different from the previously selected one.
  if (!this->dataPtr->keyEvent.key && (this->dataPtr->selectionObj->GetMode() ==
       rendering::SelectionObj::SELECTION_NONE
      || (mouseVis && mouseVis != this->dataPtr->selectionObj->GetParent())))
  {
    vis = mouseVis;
  }
  else
  {
    vis = this->dataPtr->selectionObj->GetParent();
  }

  if (vis && !vis->IsPlane() &&
      this->dataPtr->mouseEvent.button == common::MouseEvent::LEFT)
  {
    if (gui::get_entity_id(vis->GetRootVisual()->GetName()))
    {
      vis = vis->GetRootVisual();
    }

    this->dataPtr->mouseMoveVisStartPose = vis->GetWorldPose();

    this->SetMouseMoveVisual(vis);

    event::Events::setSelectedEntity(
        this->dataPtr->mouseMoveVis->GetName(), "move");
    QApplication::setOverrideCursor(Qt::ClosedHandCursor);

    if (this->dataPtr->mouseMoveVis && !this->dataPtr->mouseMoveVis->IsPlane())
    {
      this->dataPtr->selectionObj->Attach(this->dataPtr->mouseMoveVis);
      this->dataPtr->selectionObj->SetMode(this->dataPtr->manipMode);
    }
    else
    {
      this->dataPtr->selectionObj->SetMode(
          rendering::SelectionObj::SELECTION_NONE);
      this->dataPtr->selectionObj->Detach();
    }
  }
  else
    this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
}

/////////////////////////////////////////////////
void ModelManipulator::OnMouseMoveEvent(const common::MouseEvent &_event)
{
  this->dataPtr->mouseEvent = _event;
  if (this->dataPtr->mouseEvent.dragging)
  {
    if (this->dataPtr->mouseMoveVis &&
        this->dataPtr->mouseEvent.button == common::MouseEvent::LEFT)
    {
      math::Vector3 axis = math::Vector3::Zero;
      if (this->dataPtr->keyEvent.key == Qt::Key_X)
        axis.x = 1;
      else if (this->dataPtr->keyEvent.key == Qt::Key_Y)
        axis.y = 1;
      else if (this->dataPtr->keyEvent.key == Qt::Key_Z)
        axis.z = 1;

      if (this->dataPtr->selectionObj->GetMode() ==
          rendering::SelectionObj::TRANS)
      {
        if (axis != math::Vector3::Zero)
        {
          this->TranslateEntity(this->dataPtr->mouseMoveVis, axis, false);
        }
        else if (this->dataPtr->selectionObj->GetState()
            == rendering::SelectionObj::TRANS_X)
        {
          this->TranslateEntity(this->dataPtr->mouseMoveVis,
              math::Vector3::UnitX, !this->dataPtr->globalManip);
        }
        else if (this->dataPtr->selectionObj->GetState()
            == rendering::SelectionObj::TRANS_Y)
        {
          this->TranslateEntity(this->dataPtr->mouseMoveVis,
              math::Vector3::UnitY, !this->dataPtr->globalManip);
        }
        else if (this->dataPtr->selectionObj->GetState()
            == rendering::SelectionObj::TRANS_Z)
        {
          this->TranslateEntity(this->dataPtr->mouseMoveVis,
            math::Vector3::UnitZ, !this->dataPtr->globalManip);
        }
        else
        {
          this->TranslateEntity(
              this->dataPtr->mouseMoveVis, math::Vector3(1, 1, 0));
        }
      }
      else if (this->dataPtr->selectionObj->GetMode()
          == rendering::SelectionObj::ROT)
      {
        if (axis != math::Vector3::Zero)
        {
          this->RotateEntity(this->dataPtr->mouseMoveVis, axis, false);
        }
        else if (this->dataPtr->selectionObj->GetState()
            == rendering::SelectionObj::ROT_X
            || this->dataPtr->keyEvent.key == Qt::Key_X)
        {
          this->RotateEntity(this->dataPtr->mouseMoveVis, math::Vector3::UnitX,
              !this->dataPtr->globalManip);
        }
        else if (this->dataPtr->selectionObj->GetState()
            == rendering::SelectionObj::ROT_Y
            || this->dataPtr->keyEvent.key == Qt::Key_Y)
        {
          this->RotateEntity(this->dataPtr->mouseMoveVis, math::Vector3::UnitY,
              !this->dataPtr->globalManip);
        }
        else if (this->dataPtr->selectionObj->GetState()
            == rendering::SelectionObj::ROT_Z
            || this->dataPtr->keyEvent.key == Qt::Key_Z)
        {
          this->RotateEntity(this->dataPtr->mouseMoveVis, math::Vector3::UnitZ,
              !this->dataPtr->globalManip);
        }
      }
      else if (this->dataPtr->selectionObj->GetMode()
          == rendering::SelectionObj::SCALE)
      {
        if (axis != math::Vector3::Zero)
        {
          this->ScaleEntity(this->dataPtr->mouseMoveVis, axis, false);
        }
        else if (this->dataPtr->selectionObj->GetState()
            == rendering::SelectionObj::SCALE_X
            || this->dataPtr->keyEvent.key == Qt::Key_X)
        {
          this->ScaleEntity(this->dataPtr->mouseMoveVis,
              math::Vector3::UnitX, true);
        }
        else if (this->dataPtr->selectionObj->GetState()
            == rendering::SelectionObj::SCALE_Y
            || this->dataPtr->keyEvent.key == Qt::Key_Y)
        {
          this->ScaleEntity(this->dataPtr->mouseMoveVis,
              math::Vector3::UnitY, true);
        }
        else if (this->dataPtr->selectionObj->GetState()
            == rendering::SelectionObj::SCALE_Z
            || this->dataPtr->keyEvent.key == Qt::Key_Z)
        {
          this->ScaleEntity(this->dataPtr->mouseMoveVis,
              math::Vector3::UnitZ, true);
        }
      }
    }
    else
      this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
  }
  else
  {
    std::string manipState;
    this->dataPtr->userCamera->GetVisual(this->dataPtr->mouseEvent.pos,
        manipState);
    this->dataPtr->selectionObj->SetState(manipState);

    if (!manipState.empty())
      QApplication::setOverrideCursor(Qt::OpenHandCursor);
    else
    {
      rendering::VisualPtr vis = this->dataPtr->userCamera->GetVisual(
          this->dataPtr->mouseEvent.pos);

      if (vis && !vis->IsPlane())
        QApplication::setOverrideCursor(Qt::OpenHandCursor);
      else
        QApplication::setOverrideCursor(Qt::ArrowCursor);
      this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
    }
  }
}

//////////////////////////////////////////////////
void ModelManipulator::OnMouseReleaseEvent(const common::MouseEvent &_event)
{
  this->dataPtr->mouseEvent = _event;
  if (this->dataPtr->mouseEvent.dragging)
  {
    // If we were dragging a visual around, then publish its new pose to the
    // server
    if (this->dataPtr->mouseMoveVis)
    {
      if (this->dataPtr->manipMode == "scale")
      {
        this->dataPtr->selectionObj->UpdateSize();
        this->PublishVisualScale(this->dataPtr->mouseMoveVis);
      }
      else
        this->PublishVisualPose(this->dataPtr->mouseMoveVis);
      this->SetMouseMoveVisual(rendering::VisualPtr());
      QApplication::setOverrideCursor(Qt::OpenHandCursor);
    }
    event::Events::setSelectedEntity("", "normal");
  }
  else
  {
    if (this->dataPtr->mouseEvent.button == common::MouseEvent::LEFT)
    {
      rendering::VisualPtr vis =
        this->dataPtr->userCamera->GetVisual(this->dataPtr->mouseEvent.pos);
      if (vis && vis->IsPlane())
      {
        this->dataPtr->selectionObj->SetMode(
            rendering::SelectionObj::SELECTION_NONE);
        this->dataPtr->selectionObj->Detach();
      }
    }
  }
  this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
}

//////////////////////////////////////////////////
void ModelManipulator::SetManipulationMode(const std::string &_mode)
{
  this->dataPtr->manipMode = _mode;
  if (this->dataPtr->selectionObj->GetMode() !=
      rendering::SelectionObj::SELECTION_NONE ||  this->dataPtr->mouseMoveVis)
  {
    this->dataPtr->selectionObj->SetMode(this->dataPtr->manipMode);
    if (this->dataPtr->manipMode != "translate"
        && this->dataPtr->manipMode != "rotate"
        && this->dataPtr->manipMode != "scale")
      this->SetMouseMoveVisual(rendering::VisualPtr());
  }
}

/////////////////////////////////////////////////
void ModelManipulator::SetAttachedVisual(rendering::VisualPtr _vis)
{
  rendering::VisualPtr vis = _vis;

  if (gui::get_entity_id(vis->GetRootVisual()->GetName()))
    vis = vis->GetRootVisual();

  this->dataPtr->mouseMoveVisStartPose = vis->GetWorldPose();

  this->SetMouseMoveVisual(vis);

  if (this->dataPtr->mouseMoveVis && !this->dataPtr->mouseMoveVis->IsPlane())
    this->dataPtr->selectionObj->Attach(this->dataPtr->mouseMoveVis);
}

/////////////////////////////////////////////////
void ModelManipulator::SetMouseMoveVisual(rendering::VisualPtr _vis)
{
  this->dataPtr->mouseMoveVis = _vis;
  if (_vis)
  {
    this->dataPtr->mouseVisualScale = _vis->GetScale();
    this->dataPtr->mouseVisualBbox = _vis->GetBoundingBox();
  }
  else
    this->dataPtr->mouseVisualScale = math::Vector3::One;
}

//////////////////////////////////////////////////
void ModelManipulator::OnKeyPressEvent(const common::KeyEvent &_event)
{
  this->dataPtr->keyEvent = _event;
  // reset mouseMoveVisStartPose if in manipulation mode.
  if (this->dataPtr->manipMode == "translate"
      || this->dataPtr->manipMode == "rotate"
      || this->dataPtr->manipMode == "scale")
  {
    if (_event.key == Qt::Key_X || _event.key == Qt::Key_Y
        || _event.key == Qt::Key_Z)
    {
      this->dataPtr->mouseStart = this->dataPtr->mouseEvent.pos;
      if (this->dataPtr->mouseMoveVis)
      {
        this->dataPtr->mouseMoveVisStartPose =
            this->dataPtr->mouseMoveVis->GetWorldPose();
      }
    }
    else  if (this->dataPtr->keyEvent.key == Qt::Key_Shift)
    {
      this->dataPtr->globalManip = true;
      this->dataPtr->selectionObj->SetGlobal(this->dataPtr->globalManip);
    }
  }
}

//////////////////////////////////////////////////
void ModelManipulator::OnKeyReleaseEvent(const common::KeyEvent &_event)
{
  this->dataPtr->keyEvent = _event;
  // reset mouseMoveVisStartPose if in manipulation mode.
  if (this->dataPtr->manipMode == "translate"
      || this->dataPtr->manipMode == "rotate"
      || this->dataPtr->manipMode == "scale")
  {
    if (_event.key == Qt::Key_X || _event.key == Qt::Key_Y
        || _event.key == Qt::Key_Z)
    {
      this->dataPtr->mouseStart = this->dataPtr->mouseEvent.pos;
      if (this->dataPtr->mouseMoveVis)
      {
        this->dataPtr->mouseMoveVisStartPose =
            this->dataPtr->mouseMoveVis->GetWorldPose();
      }
    }
    else  if (this->dataPtr->keyEvent.key == Qt::Key_Shift)
    {
      this->dataPtr->globalManip = false;
      this->dataPtr->selectionObj->SetGlobal(this->dataPtr->globalManip);
    }
  }
  this->dataPtr->keyEvent.key = 0;
}

// Function migrated here from GLWidget.cc and commented out since it doesn't
// seem like it's currently used. Kept here for future references
/////////////////////////////////////////////////
/*void GLWidget::SmartMoveVisual(rendering::VisualPtr _vis)
{
  if (!this->dataPtr->mouseEvent.dragging)
    return;

  // Get the point on the plane which correspoinds to the mouse
  math::Vector3 pp;

  // Rotate the visual using the middle mouse button
  if (this->dataPtr->mouseEvent.buttons == common::MouseEvent::MIDDLE)
  {
    math::Vector3 rpy = this->dataPtr->mouseMoveVisStartPose.rot.GetAsEuler();
    math::Vector2i delta = this->dataPtr->mouseEvent.pos -
        this->dataPtr->mouseEvent.pressPos;
    double yaw = (delta.x * 0.01) + rpy.z;
    if (!this->dataPtr->mouseEvent.shift)
    {
      double snap = rint(yaw / (M_PI * .25)) * (M_PI * 0.25);

      if (fabs(yaw - snap) < GZ_DTOR(10))
        yaw = snap;
    }

    _vis->SetWorldRotation(math::Quaternion(rpy.x, rpy.y, yaw));
  }
  else if (this->dataPtr->mouseEvent.buttons == common::MouseEvent::RIGHT)
  {
    math::Vector3 rpy = this->dataPtr->mouseMoveVisStartPose.rot.GetAsEuler();
    math::Vector2i delta = this->dataPtr->mouseEvent.pos -
        this->dataPtr->mouseEvent.pressPos;
    double pitch = (delta.y * 0.01) + rpy.y;
    if (!this->dataPtr->mouseEvent.shift)
    {
      double snap = rint(pitch / (M_PI * .25)) * (M_PI * 0.25);

      if (fabs(pitch - snap) < GZ_DTOR(10))
        pitch = snap;
    }

    _vis->SetWorldRotation(math::Quaternion(rpy.x, pitch, rpy.z));
  }
  else if (this->dataPtr->mouseEvent.buttons & common::MouseEvent::LEFT &&
           this->dataPtr->mouseEvent.buttons & common::MouseEvent::RIGHT)
  {
    math::Vector3 rpy = this->dataPtr->mouseMoveVisStartPose.rot.GetAsEuler();
    math::Vector2i delta = this->dataPtr->mouseEvent.pos -
        this->dataPtr->mouseEvent.pressPos;
    double roll = (delta.x * 0.01) + rpy.x;
    if (!this->dataPtr->mouseEvent.shift)
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
}*/
