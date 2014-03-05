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

#include "gazebo/gui/ModelManipulator.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelManipulator::ModelManipulator()
{
  this->initialized = false;
  this->selectionObj.reset();
  this->mouseMoveVis.reset();

  this->manipMode = "";
  this->globalManip = false;
}

/////////////////////////////////////////////////
ModelManipulator::~ModelManipulator()
{
  this->modelPub.reset();
  this->selectionObj.reset();
}

/////////////////////////////////////////////////
void ModelManipulator::Init()
{
  if (this->initialized)
    return;

  rendering::UserCameraPtr cam = gui::get_active_camera();
  if (!cam)
    return;

  if (!cam->GetScene())
    return;

  this->userCamera = cam;
  this->scene =  cam->GetScene();

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init();
  this->modelPub = this->node->Advertise<msgs::Model>("~/model/modify");
  this->lightPub = this->node->Advertise<msgs::Light>("~/light");

  this->selectionObj.reset(new rendering::SelectionObj("__GL_MANIP__",
      this->scene->GetWorldVisual()));
  this->selectionObj->Load();

  this->initialized = true;
}

/////////////////////////////////////////////////
void ModelManipulator::RotateEntity(rendering::VisualPtr &_vis,
    const math::Vector3 &_axis, bool _local)
{
  math::Vector3 normal;

  if (_local)
  {
    if (_axis.x > 0)
      normal = mouseMoveVisStartPose.rot.GetXAxis();
    else if (_axis.y > 0)
      normal = mouseMoveVisStartPose.rot.GetYAxis();
    else if (_axis.z > 0)
      normal = mouseMoveVisStartPose.rot.GetZAxis();
  }
  else
    normal = _axis;

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

  if (this->mouseEvent.control)
    angle = rint(angle / (M_PI * 0.25)) * (M_PI * 0.25);

  math::Quaternion rot(_axis, angle);

  if (_local)
    rot = this->mouseMoveVisStartPose.rot * rot;
  else
    rot = rot * this->mouseMoveVisStartPose.rot;

  _vis->SetWorldRotation(rot);
}


/////////////////////////////////////////////////
math::Vector3 ModelManipulator::GetMouseMoveDistance(const math::Pose &_pose,
    const math::Vector3 &_axis, bool _local) const
{
  math::Pose pose = _pose;

  math::Vector3 origin1, dir1, p1;
  math::Vector3 origin2, dir2, p2;

  // Cast two rays from the camera into the world
  this->userCamera->GetCameraToViewportRay(this->mouseEvent.pos.x,
      this->mouseEvent.pos.y, origin1, dir1);
  this->userCamera->GetCameraToViewportRay(this->mouseStart.x,
      this->mouseStart.y, origin2, dir2);

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
void ModelManipulator::ScaleEntity(rendering::VisualPtr &_vis,
    const math::Vector3 &_axis, bool _local)
{
  math::Box bbox = _vis->GetBoundingBox();
  math::Pose pose = _vis->GetWorldPose();
  math::Vector3 distance =  this->GetMouseMoveDistance(pose, _axis, _local);

  math::Vector3 bboxSize = bbox.GetSize() * this->mouseVisualScale;
  math::Vector3 scale = (bboxSize + pose.rot.RotateVectorReverse(distance))
      /bboxSize;

  // a bit hacky to check for unit sphere and cylinder simple shapes in order
  // to restrict the scaling dimensions.

  if (this->keyEvent.key == Qt::Key_Shift ||
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

  math::Vector3 newScale = this->mouseVisualScale * scale.GetAbs();

  if (this->mouseEvent.control)
  {
    if (ceil(newScale.x) - newScale.x <= .4)
        newScale.x = ceil(newScale.x);
    else if (newScale.x - floor(newScale.x) <= .4)
      newScale.x = floor(newScale.x);

    if (ceil(newScale.y) - newScale.y <= .4)
        newScale.y = ceil(newScale.y);
    else if (newScale.y - floor(newScale.y) <= .4)
      newScale.y = floor(newScale.y);

    if (_axis.z > 0.0)
    {
      if (ceil(newScale.z) - newScale.z <= .4)
        newScale.z = ceil(newScale.z);
      else if (newScale.z - floor(newScale.z) <= .4)
        newScale.z = floor(newScale.z);
    }
  }

  _vis->SetScale(newScale);
}

/////////////////////////////////////////////////
void ModelManipulator::TranslateEntity(rendering::VisualPtr &_vis,
    const math::Vector3 &_axis, bool _local)
{
  math::Pose pose = _vis->GetWorldPose();
  math::Vector3 distance =  this->GetMouseMoveDistance(pose, _axis, _local);

  pose.pos = this->mouseMoveVisStartPose.pos + distance;

  if (this->mouseEvent.control)
  {
    if (ceil(pose.pos.x) - pose.pos.x <= .4)
        pose.pos.x = ceil(pose.pos.x);
    else if (pose.pos.x - floor(pose.pos.x) <= .4)
      pose.pos.x = floor(pose.pos.x);

    if (ceil(pose.pos.y) - pose.pos.y <= .4)
        pose.pos.y = ceil(pose.pos.y);
    else if (pose.pos.y - floor(pose.pos.y) <= .4)
      pose.pos.y = floor(pose.pos.y);

    if (_axis.z > 0.0)
    {
      if (ceil(pose.pos.z) - pose.pos.z <= .4)
        pose.pos.z = ceil(pose.pos.z);
      else if (pose.pos.z - floor(pose.pos.z) <= .4)
        pose.pos.z = floor(pose.pos.z);
    }
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
      this->modelPub->Publish(msg);
      _vis->SetScale(this->mouseVisualScale);
    }
  }
}

/////////////////////////////////////////////////
void ModelManipulator::OnMousePressEvent(const common::MouseEvent &_event)
{
  this->mouseEvent = _event;
  this->mouseStart = _event.pressPos;
  this->SetMouseMoveVisual(rendering::VisualPtr());

  rendering::VisualPtr vis;
  rendering::VisualPtr mouseVis
      = this->userCamera->GetVisual(this->mouseEvent.pos);

  // set the new mouse vis only if there are no modifier keys pressed and the
  // entity was different from the previously selected one.
  if (!this->keyEvent.key && (this->selectionObj->GetMode() ==
       rendering::SelectionObj::SELECTION_NONE
      || (mouseVis && mouseVis != this->selectionObj->GetParent())))
  {
    vis = mouseVis;
  }
  else
  {
    vis = this->selectionObj->GetParent();
  }

  if (vis && !vis->IsPlane() &&
      this->mouseEvent.button == common::MouseEvent::LEFT)
  {
    if (gui::get_entity_id(vis->GetRootVisual()->GetName()))
      vis = vis->GetRootVisual();

    this->mouseMoveVisStartPose = vis->GetWorldPose();

    this->SetMouseMoveVisual(vis);

    event::Events::setSelectedEntity(this->mouseMoveVis->GetName(), "move");
    QApplication::setOverrideCursor(Qt::ClosedHandCursor);

    if (this->mouseMoveVis && !this->mouseMoveVis->IsPlane())
    {
      this->selectionObj->Attach(this->mouseMoveVis);
      this->selectionObj->SetMode(this->manipMode);
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
void ModelManipulator::OnMouseMoveEvent(const common::MouseEvent &_event)
{
  this->mouseEvent = _event;
  if (this->mouseEvent.dragging)
  {
    if (this->mouseMoveVis &&
        this->mouseEvent.button == common::MouseEvent::LEFT)
    {
      math::Vector3 axis = math::Vector3::Zero;
      if (this->keyEvent.key == Qt::Key_X)
        axis.x = 1;
      else if (this->keyEvent.key == Qt::Key_Y)
        axis.y = 1;
      else if (this->keyEvent.key == Qt::Key_Z)
        axis.z = 1;

      if (this->selectionObj->GetMode() == rendering::SelectionObj::TRANS)
      {
        if (axis != math::Vector3::Zero)
        {
          this->TranslateEntity(this->mouseMoveVis, axis, false);
        }
        else if (this->selectionObj->GetState()
            == rendering::SelectionObj::TRANS_X)
        {
          this->TranslateEntity(this->mouseMoveVis,
              math::Vector3::UnitX, !this->globalManip);
        }
        else if (this->selectionObj->GetState()
            == rendering::SelectionObj::TRANS_Y)
        {
          this->TranslateEntity(this->mouseMoveVis,
              math::Vector3::UnitY, !this->globalManip);
        }
        else if (this->selectionObj->GetState()
            == rendering::SelectionObj::TRANS_Z)
        {
          this->TranslateEntity(this->mouseMoveVis,
            math::Vector3::UnitZ, !this->globalManip);
        }
        else
          this->TranslateEntity(this->mouseMoveVis, math::Vector3(1, 1, 0));
      }
      else if (this->selectionObj->GetMode() == rendering::SelectionObj::ROT)
      {
        if (axis != math::Vector3::Zero)
        {
          this->RotateEntity(this->mouseMoveVis, axis, false);
        }
        else if (this->selectionObj->GetState()
            == rendering::SelectionObj::ROT_X
            || this->keyEvent.key == Qt::Key_X)
        {
          this->RotateEntity(this->mouseMoveVis, math::Vector3::UnitX,
              !this->globalManip);
        }
        else if (this->selectionObj->GetState()
            == rendering::SelectionObj::ROT_Y
            || this->keyEvent.key == Qt::Key_Y)
        {
          this->RotateEntity(this->mouseMoveVis, math::Vector3::UnitY,
              !this->globalManip);
        }
        else if (this->selectionObj->GetState()
            == rendering::SelectionObj::ROT_Z
            || this->keyEvent.key == Qt::Key_Z)
        {
          this->RotateEntity(this->mouseMoveVis, math::Vector3::UnitZ,
              !this->globalManip);
        }
      }
      else if (this->selectionObj->GetMode() == rendering::SelectionObj::SCALE)
      {
        if (axis != math::Vector3::Zero)
        {
          this->ScaleEntity(this->mouseMoveVis, axis, false);
        }
        else if (this->selectionObj->GetState()
            == rendering::SelectionObj::SCALE_X
            || this->keyEvent.key == Qt::Key_X)
        {
          this->ScaleEntity(this->mouseMoveVis, math::Vector3::UnitX, true);
        }
        else if (this->selectionObj->GetState()
            == rendering::SelectionObj::SCALE_Y
            || this->keyEvent.key == Qt::Key_Y)
        {
          this->ScaleEntity(this->mouseMoveVis, math::Vector3::UnitY, true);
        }
        else if (this->selectionObj->GetState()
            == rendering::SelectionObj::SCALE_Z
            || this->keyEvent.key == Qt::Key_Z)
        {
          this->ScaleEntity(this->mouseMoveVis, math::Vector3::UnitZ, true);
        }
      }
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

//////////////////////////////////////////////////
void ModelManipulator::OnMouseReleaseEvent(const common::MouseEvent &_event)
{
  this->mouseEvent = _event;
  if (this->mouseEvent.dragging)
  {
    // If we were dragging a visual around, then publish its new pose to the
    // server
    if (this->mouseMoveVis)
    {
      if (this->manipMode == "scale")
      {
        this->selectionObj->UpdateSize();
        this->PublishVisualScale(this->mouseMoveVis);
      }
      else
        this->PublishVisualPose(this->mouseMoveVis);
      this->SetMouseMoveVisual(rendering::VisualPtr());
      QApplication::setOverrideCursor(Qt::OpenHandCursor);
    }
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
void ModelManipulator::SetManipulationMode(const std::string &_mode)
{
  this->manipMode = _mode;
  if (this->selectionObj->GetMode() != rendering::SelectionObj::SELECTION_NONE
      ||  this->mouseMoveVis)
  {
    this->selectionObj->SetMode(this->manipMode);
    if (this->manipMode != "translate" && this->manipMode != "rotate"
        && this->manipMode != "scale")
      this->SetMouseMoveVisual(rendering::VisualPtr());
  }
}

/////////////////////////////////////////////////
void ModelManipulator::SetAttachedVisual(rendering::VisualPtr _vis)
{
  rendering::VisualPtr vis = _vis;

  if (gui::get_entity_id(vis->GetRootVisual()->GetName()))
    vis = vis->GetRootVisual();

  this->mouseMoveVisStartPose = vis->GetWorldPose();

  this->SetMouseMoveVisual(vis);

  if (this->mouseMoveVis && !this->mouseMoveVis->IsPlane())
    this->selectionObj->Attach(this->mouseMoveVis);
}

/////////////////////////////////////////////////
void ModelManipulator::SetMouseMoveVisual(rendering::VisualPtr _vis)
{
  this->mouseMoveVis = _vis;
  if (_vis)
    this->mouseVisualScale = _vis->GetScale();
  else
    this->mouseVisualScale = math::Vector3::One;
}

//////////////////////////////////////////////////
void ModelManipulator::OnKeyPressEvent(const common::KeyEvent &_event)
{
  this->keyEvent = _event;
  // reset mouseMoveVisStartPose if in manipulation mode.
  if (this->manipMode == "translate" || this->manipMode == "rotate"
      || this->manipMode == "scale")
  {
    if (_event.key == Qt::Key_X || _event.key == Qt::Key_Y
        || _event.key == Qt::Key_Z)
    {
      this->mouseStart = this->mouseEvent.pos;
      if (this->mouseMoveVis)
      {
        this->mouseMoveVisStartPose = this->mouseMoveVis->GetWorldPose();
      }
    }
    else  if (this->keyEvent.key == Qt::Key_Shift)
    {
      this->globalManip = true;
      this->selectionObj->SetGlobal(this->globalManip);
    }
  }
}

//////////////////////////////////////////////////
void ModelManipulator::OnKeyReleaseEvent(const common::KeyEvent &_event)
{
  this->keyEvent = _event;
  // reset mouseMoveVisStartPose if in manipulation mode.
  if (this->manipMode == "translate" || this->manipMode == "rotate"
      || this->manipMode == "scale")
  {
    if (_event.key == Qt::Key_X || _event.key == Qt::Key_Y
        || _event.key == Qt::Key_Z)
    {
      this->mouseStart = this->mouseEvent.pos;
      if (this->mouseMoveVis)
      {
        this->mouseMoveVisStartPose = this->mouseMoveVis->GetWorldPose();
      }
    }
    else  if (this->keyEvent.key == Qt::Key_Shift)
    {
      this->globalManip = false;
      this->selectionObj->SetGlobal(this->globalManip);
    }
  }
  this->keyEvent.key = 0;
}

// Function migrated here from GLWidget.cc and commented out since it doesn't
// seem like it's currently used. Kept here for future references
/////////////////////////////////////////////////
/*void GLWidget::SmartMoveVisual(rendering::VisualPtr _vis)
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
}*/
