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

#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/RenderEngine.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/SelectionObj.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/GuiEvents.hh"
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
}

/////////////////////////////////////////////////
ModelManipulator::~ModelManipulator()
{
  this->Clear();
}

/////////////////////////////////////////////////
void ModelManipulator::Clear()
{
  this->dataPtr->userCmdPub.reset();
  if (this->dataPtr->node)
    this->dataPtr->node->Fini();
  this->dataPtr->node.reset();

  if (this->dataPtr->selectionObj)
    this->dataPtr->selectionObj->Fini();
  this->dataPtr->selectionObj.reset();
  this->dataPtr->mouseMoveVis.reset();

  this->dataPtr->scene.reset();
  this->dataPtr->userCamera.reset();

  this->dataPtr->mouseChildVisualScale.clear();
  this->dataPtr->manipMode = "";
  this->dataPtr->globalManip = false;
  this->dataPtr->initialized = false;
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
  this->dataPtr->userCmdPub =
      this->dataPtr->node->Advertise<msgs::UserCmd>("~/user_cmd");

  this->dataPtr->selectionObj.reset(new rendering::SelectionObj("__GL_MANIP__",
      this->dataPtr->scene->WorldVisual()));
  this->dataPtr->selectionObj->Load();

  this->dataPtr->transparent = false;

  this->dataPtr->initialized = true;
}

/////////////////////////////////////////////////
void ModelManipulator::Detach()
{
  this->dataPtr->selectionObj->SetMode(
      rendering::SelectionObj::SELECTION_NONE);
  this->dataPtr->selectionObj->Detach();
}

/////////////////////////////////////////////////
void ModelManipulator::RotateEntity(rendering::VisualPtr &_vis,
    const math::Vector3 &_axis, bool _local)
{
  this->RotateEntity(_vis, _axis.Ign(), _local);
}

/////////////////////////////////////////////////
void ModelManipulator::RotateEntity(rendering::VisualPtr &_vis,
    const ignition::math::Vector3d &_axis, const bool _local)
{
  ignition::math::Vector3d normal;

  if (_local)
  {
    if (_axis.X() > 0)
      normal = this->dataPtr->mouseMoveVisStartPose.Rot().XAxis();
    else if (_axis.Y() > 0)
      normal = this->dataPtr->mouseMoveVisStartPose.Rot().YAxis();
    else if (_axis.Z() > 0)
      normal = this->dataPtr->mouseMoveVisStartPose.Rot().ZAxis();
  }
  else
    normal = _axis;

  double offset = this->dataPtr->mouseMoveVisStartPose.Pos().Dot(normal);

  ignition::math::Vector3d pressPoint;
  this->dataPtr->userCamera->WorldPointOnPlane(
      this->dataPtr->mouseEvent.PressPos().X(),
      this->dataPtr->mouseEvent.PressPos().Y(),
      ignition::math::Planed(normal, offset), pressPoint);

  ignition::math::Vector3d newPoint;
  this->dataPtr->userCamera->WorldPointOnPlane(
      this->dataPtr->mouseEvent.Pos().X(),
      this->dataPtr->mouseEvent.Pos().Y(),
      ignition::math::Planed(normal, offset), newPoint);

  ignition::math::Vector3d v1 = pressPoint -
    this->dataPtr->mouseMoveVisStartPose.Pos();
  ignition::math::Vector3d v2 = newPoint -
    this->dataPtr->mouseMoveVisStartPose.Pos();
  v1 = v1.Normalize();
  v2 = v2.Normalize();
  double signTest = v1.Cross(v2).Dot(normal);
  double angle = atan2((v1.Cross(v2)).Length(), v1.Dot(v2));

  if (signTest < 0 )
    angle *= -1;

  // Using Qt control modifier instead of Gazebo's for now.
  // See GLWidget::keyPressEvent
  if (QApplication::keyboardModifiers() & Qt::ControlModifier)
    angle = rint(angle / (M_PI * 0.25)) * (M_PI * 0.25);

  ignition::math::Quaterniond rot(_axis, angle);

  if (_local)
    rot = this->dataPtr->mouseMoveVisStartPose.Rot() * rot;
  else
    rot = rot * this->dataPtr->mouseMoveVisStartPose.Rot();

  _vis->SetWorldRotation(rot);
  Events::moveEntity(_vis->GetName(), _vis->GetWorldPose().Ign(), false);
}

/////////////////////////////////////////////////
math::Vector3 ModelManipulator::GetMousePositionOnPlane(
    rendering::CameraPtr _camera,
    const common::MouseEvent &_event)
{
  return MousePositionOnPlane(_camera, _event);
}

/////////////////////////////////////////////////
ignition::math::Vector3d ModelManipulator::MousePositionOnPlane(
    rendering::CameraPtr _camera, const common::MouseEvent &_event)
{
  ignition::math::Vector3d origin1, dir1, p1;

  // Cast ray from the camera into the world
  _camera->CameraToViewportRay(_event.Pos().X(), _event.Pos().Y(),
      origin1, dir1);

  // Compute the distance from the camera to plane of translation
  ignition::math::Planed plane(ignition::math::Vector3d(0, 0, 1), 0);
  double dist1 = plane.Distance(origin1, dir1);

  p1 = origin1 + dir1 * dist1;

  return p1;
}

/////////////////////////////////////////////////
math::Vector3 ModelManipulator::SnapPoint(const math::Vector3 &_point,
    double _interval, double _sensitivity)
{
  return SnapPoint(_point.Ign(), _interval, _sensitivity);
}

/////////////////////////////////////////////////
ignition::math::Vector3d ModelManipulator::SnapPoint(
    const ignition::math::Vector3d &_point,
    const double _interval, const double _sensitivity)
{
  if (_interval < 0)
  {
    gzerr << "Interval distance must be greater than or equal to 0"
        << std::endl;
    return ignition::math::Vector3d::Zero;
  }

  if (_sensitivity < 0 || _sensitivity > 1.0)
  {
    gzerr << "Sensitivity must be between 0 and 1" << std::endl;
    return ignition::math::Vector3d::Zero;
  }

  ignition::math::Vector3d point = _point;
  double snap = _interval * _sensitivity;

  double remainder = fmod(point.X(), _interval);
  int sign = remainder >= 0 ? 1 : -1;
  if (fabs(remainder) < snap)
      point.X() -= remainder;
  else if (fabs(remainder) > (_interval - snap))
      point.X() = point.X() - remainder + _interval * sign;

  remainder = fmod(point.Y(), _interval);
  sign = remainder >= 0 ? 1 : -1;
  if (fabs(remainder) < snap)
      point.Y() -= remainder;
  else if (fabs(remainder) > (_interval - snap))
      point.Y() = point.Y() - remainder + _interval * sign;

  remainder = fmod(point.Z(), _interval);
  sign = remainder >= 0 ? 1 : -1;
  if (fabs(remainder) < snap)
      point.Z() -= remainder;
  else if (fabs(remainder) > (_interval - snap))
      point.Z() = point.Z() - remainder + _interval * sign;

  return point;
}

/////////////////////////////////////////////////
math::Vector3 ModelManipulator::GetMouseMoveDistance(
    rendering::CameraPtr _camera,
    const math::Vector2i &_start, const math::Vector2i &_end,
    const math::Pose &_pose, const math::Vector3 &_axis, bool _local)
{
  return MouseMoveDistance(_camera, _start.Ign(), _end.Ign(), _pose.Ign(),
      _axis.Ign(), _local);
}

/////////////////////////////////////////////////
ignition::math::Vector3d ModelManipulator::MouseMoveDistance(
    rendering::CameraPtr _camera,
    const ignition::math::Vector2i &_start,
    const ignition::math::Vector2i &_end,
    const ignition::math::Pose3d &_pose,
    const ignition::math::Vector3d &_axis, const bool _local)
{
  ignition::math::Pose3d pose = _pose;

  ignition::math::Vector3d origin1, dir1, p1;
  ignition::math::Vector3d origin2, dir2, p2;

  // Cast two rays from the camera into the world
  _camera->CameraToViewportRay(_end.X(), _end.Y(), origin1, dir1);
  _camera->CameraToViewportRay(_start.X(), _start.Y(), origin2, dir2);

  ignition::math::Vector3d planeNorm(0, 0, 0);
  ignition::math::Vector3d projNorm(0, 0, 0);

  ignition::math::Vector3d planeNormOther(0, 0, 0);

  if (_axis.X() > 0 && _axis.Y() > 0)
  {
    planeNorm.Z(1);
    projNorm.Z(1);
  }
  else if (_axis.Z() > 0)
  {
    planeNorm.Y(1);
    projNorm.X(1);
    planeNormOther.X(1);
  }
  else if (_axis.X() > 0)
  {
    planeNorm.Z(1);
    projNorm.Y(1);
    planeNormOther.Y(1);
  }
  else if (_axis.Y() > 0)
  {
    planeNorm.Z(1);
    projNorm.X(1);
    planeNormOther.X(1);
  }

  if (_local)
  {
    planeNorm = pose.Rot().RotateVector(planeNorm);
    projNorm = pose.Rot().RotateVector(projNorm);
  }

  // Fine tune ray casting: cast a second ray and compare the two rays' angle
  // to plane. Use the one that is less parallel to plane for better results.
  double angle = dir1.Dot(planeNorm);
  if (_local)
    planeNormOther = pose.Rot().RotateVector(planeNormOther);
  double angleOther = dir1.Dot(planeNormOther);
  if (fabs(angleOther) > fabs(angle))
  {
    projNorm = planeNorm;
    planeNorm = planeNormOther;
  }

  // Compute the distance from the camera to plane
  double d = pose.Pos().Dot(planeNorm);
  ignition::math::Planed plane(planeNorm, d);
  double dist1 = plane.Distance(origin1, dir1);
  double dist2 = plane.Distance(origin2, dir2);

  // Compute two points on the plane. The first point is the current
  // mouse position, the second is the previous mouse position
  p1 = origin1 + dir1 * dist1;
  p2 = origin2 + dir2 * dist2;

  if (_local)
    p1 = p1 - (p1-p2).Dot(projNorm) * projNorm;

  ignition::math::Vector3d distance = p1 - p2;

  if (!_local)
    distance *= _axis;

  return distance;
}

/////////////////////////////////////////////////
ignition::math::Vector3d ModelManipulator::MouseMoveDistance(
    const ignition::math::Pose3d &_pose,
    const ignition::math::Vector3d &_axis, const bool _local) const
{
  return MouseMoveDistance(this->dataPtr->userCamera,
      this->dataPtr->mouseStart,
      ignition::math::Vector2i(this->dataPtr->mouseEvent.Pos().X(),
      this->dataPtr->mouseEvent.Pos().Y()), _pose, _axis, _local);
}

/////////////////////////////////////////////////
void ModelManipulator::ScaleEntity(rendering::VisualPtr &_vis,
    const math::Vector3 &_axis, bool _local)
{
  this->ScaleEntity(_vis, _axis.Ign(), _local);
}

/////////////////////////////////////////////////
void ModelManipulator::ScaleEntity(rendering::VisualPtr &_vis,
    const ignition::math::Vector3d &_axis, const bool _local)
{
  auto bbox = this->dataPtr->mouseVisualBbox;
  auto pose = _vis->GetWorldPose().Ign();
  auto distance =  this->MouseMoveDistance(pose, _axis, _local);

  auto bboxSize = bbox.Size();
  auto scale = (bboxSize + pose.Rot().RotateVectorReverse(distance)) / bboxSize;

  // extended scaling to work in model editor mode by checking geometry
  // type of first visual child.
  std::string geomType;
  if (_vis == _vis->GetRootVisual())
  {
    // link-level visuals
    for (unsigned int i = 0; i < _vis->GetChildCount(); ++i)
    {
      rendering::VisualPtr childVis = _vis->GetChild(i);

      if (childVis->GetPose().pos != ignition::math::Vector3d::Zero)
      {
        gzwarn << "Scaling is currently limited to simple shapes with their "
            << "origin in the centroid." << std::endl;
        return;
      }
      // visual/collision level visuals
      for (unsigned int j = 0; j < childVis->GetChildCount(); ++j)
      {
        rendering::VisualPtr grandChildVis = childVis->GetChild(j);
        std::string thisGeomType = grandChildVis->GetGeometryType();

        if (grandChildVis->GetPose().pos != ignition::math::Vector3d::Zero)
        {
          gzwarn << "Scaling is currently limited to simple shapes with their "
              << "origin in the centroid." << std::endl;
          return;
        }

        if (thisGeomType == "")
          continue;

        if (geomType == "")
        {
          geomType = thisGeomType;
        }
        else if (thisGeomType != geomType)
        {
          gzwarn << "Scaling is currently limited to models consisting of a " <<
              "single simple geometry type." << std::endl;
          return;
        }
      }
    }

    if (QApplication::keyboardModifiers() & Qt::ShiftModifier ||
        geomType == "sphere")
    {
      scale = this->UpdateScale(_axis, scale, "sphere");
    }
    else if (geomType == "cylinder")
    {
      scale = this->UpdateScale(_axis, scale, "cylinder");
    }
    else if (geomType == "box")
    {
      // keep new scale as it is
    }
    else
    {
      // TODO scaling for complex models are not yet functional.
      // Limit scaling to simple shapes for now.
      gzwarn << " Scaling is currently limited to simple shapes." << std::endl;
      return;
    }

    auto newScale = this->dataPtr->mouseVisualScale * scale.Abs();

    if (QApplication::keyboardModifiers() & Qt::ControlModifier)
    {
      newScale = this->SnapPoint(newScale);
      // prevent setting zero scale
      newScale.X(std::max(1e-4, newScale.X()));
      newScale.Y(std::max(1e-4, newScale.Y()));
      newScale.Z(std::max(1e-4, newScale.Z()));
    }
    _vis->SetScale(newScale);
    Events::scaleEntity(_vis->GetName(), newScale);
  }
  else
  {
    // model editor mode -> apply scaling to individual visuals
    if (this->dataPtr->mouseChildVisualScale.size() != _vis->GetChildCount())
    {
      gzerr << "Incorrect number of child visuals to be scaled. " <<
          "This should not happen" << std::endl;
      return;
    }

    for (unsigned int i = 0; i < _vis->GetChildCount(); ++i)
    {
      rendering::VisualPtr childVis = _vis->GetChild(i);
      geomType = childVis->GetGeometryType();
      if (childVis != this->dataPtr->selectionObj &&
          geomType != "" && geomType != "mesh")
      {
        ignition::math::Vector3d geomScale;
        if (QApplication::keyboardModifiers() & Qt::ShiftModifier)
        {
          geomScale = this->UpdateScale(_axis, scale, "sphere");
        }
        else
        {
          geomScale = this->UpdateScale(_axis, scale, geomType);
        }

        auto newScale = this->dataPtr->mouseChildVisualScale[i]
            * geomScale.Abs();

        if (QApplication::keyboardModifiers() & Qt::ControlModifier)
        {
          newScale = this->SnapPoint(newScale);
          // prevent setting zero scale
          newScale.X(std::max(1e-4, newScale.X()));
          newScale.Y(std::max(1e-4, newScale.Y()));
          newScale.Z(std::max(1e-4, newScale.Z()));
        }

        childVis->SetScale(newScale);
        Events::scaleEntity(childVis->GetName(), newScale);
      }
    }
  }
}

/////////////////////////////////////////////////
ignition::math::Vector3d ModelManipulator::UpdateScale(
    const ignition::math::Vector3d &_axis,
    const ignition::math::Vector3d &_scale, const std::string &_geom)
{
  ignition::math::Vector3d scale = _scale;
  if (_geom == "sphere")
  {
    if (_axis.X() > 0)
    {
      scale.Y(scale.X());
      scale.Z(scale.X());
    }
    else if (_axis.Y() > 0)
    {
      scale.X(scale.Y());
      scale.Z(scale.Y());
    }
    else if (_axis.Z() > 0)
    {
      scale.X(scale.Z());
      scale.Y(scale.Z());
    }
  }
  else if (_geom == "cylinder")
  {
    if (_axis.X() > 0)
    {
      scale.Y(scale.X());
    }
    else if (_axis.Y() > 0)
    {
      scale.X(scale.Y());
    }
  }

  return scale;
}

/////////////////////////////////////////////////
void ModelManipulator::TranslateEntity(rendering::VisualPtr &_vis,
    const math::Vector3 &_axis, bool _local)
{
  this->TranslateEntity(_vis, _axis.Ign(), _local);
}

/////////////////////////////////////////////////
void ModelManipulator::TranslateEntity(rendering::VisualPtr &_vis,
    const ignition::math::Vector3d &_axis, const bool _local)
{
  auto pose = _vis->GetWorldPose().Ign();
  auto distance =  this->MouseMoveDistance(pose, _axis, _local);

  pose.Pos() = this->dataPtr->mouseMoveVisStartPose.Pos() + distance;

  if (QApplication::keyboardModifiers() & Qt::ControlModifier)
  {
    pose.Pos() = SnapPoint(pose.Pos());
  }

  if (!(_axis.Z() > 0) && !_local)
    pose.Pos().Z() = _vis->GetWorldPose().Ign().Pos().Z();

  _vis->SetWorldPose(pose);
  Events::moveEntity(_vis->GetName(), pose, false);
}

/////////////////////////////////////////////////
void ModelManipulator::PublishVisualPose(rendering::VisualPtr _vis)
{
  if (!_vis)
    return;

  // Register user command on server
  std::string description;
  if (this->dataPtr->manipMode == "translate")
  {
    description = "Translate [";
  }
  else if (this->dataPtr->manipMode == "rotate")
  {
    description = "Rotate [";
  }
  else
  {
    gzerr << "Unknown mode [" << this->dataPtr->manipMode << "]. " <<
        "Not sending user command." << std::endl;
    return;
  }

  msgs::UserCmd userCmdMsg;
  userCmdMsg.set_description(description + _vis->GetName() + "]");
  userCmdMsg.set_type(msgs::UserCmd::MOVING);

  // Only publish for models
  if (_vis->GetType() == gazebo::rendering::Visual::VT_MODEL)
  {
    msgs::Model msg;

    auto id = gui::get_entity_id(_vis->GetName());
    if (id)
      msg.set_id(id);

    msg.set_name(_vis->GetName());
    msgs::Set(msg.mutable_pose(), _vis->GetWorldPose().Ign());

    auto modelMsg = userCmdMsg.add_model();
    modelMsg->CopyFrom(msg);
  }
  // Otherwise, check to see if the visual is a light
  else if (this->dataPtr->scene->GetLight(_vis->GetName()))
  {
    msgs::Light msg;
    msg.set_name(_vis->GetName());
    msgs::Set(msg.mutable_pose(), _vis->GetWorldPose().Ign());

    auto lightMsg = userCmdMsg.add_light();
    lightMsg->CopyFrom(msg);
  }
  else
  {
    gzerr << "Visual [" << _vis->GetName() << "] isn't a model or a light"
        << std::endl;
    return;
  }
  this->dataPtr->userCmdPub->Publish(userCmdMsg);
}

/////////////////////////////////////////////////
void ModelManipulator::PublishVisualScale(rendering::VisualPtr _vis)
{
  if (!_vis || this->dataPtr->manipMode != "scale" ||
      _vis->GetType() != gazebo::rendering::Visual::VT_MODEL)
  {
    return;
  }

  // Register user command on server
  msgs::UserCmd userCmdMsg;
  userCmdMsg.set_description("Scale [" + _vis->GetName() + "]");
  userCmdMsg.set_type(msgs::UserCmd::SCALING);

  msgs::Model msg;

  auto id = gui::get_entity_id(_vis->GetName());
  if (id)
    msg.set_id(id);

  msg.set_name(_vis->GetName());
  msgs::Set(msg.mutable_scale(), _vis->GetScale().Ign());

  auto modelMsg = userCmdMsg.add_model();
  modelMsg->CopyFrom(msg);

  this->dataPtr->userCmdPub->Publish(userCmdMsg);
  _vis->SetScale(this->dataPtr->mouseVisualScale);
}

/////////////////////////////////////////////////
void ModelManipulator::OnMousePressEvent(const common::MouseEvent &_event)
{
  this->dataPtr->mouseEvent = _event;
  this->dataPtr->mouseStart = _event.PressPos();
  this->SetMouseMoveVisual(rendering::VisualPtr());

  rendering::VisualPtr vis;
  std::string manipState;
  rendering::VisualPtr mouseVis
      = this->dataPtr->userCamera->GetVisual(this->dataPtr->mouseStart,
      manipState);

  this->dataPtr->selectionObj->SetState(manipState);

  // See issue #1510
  bool keyIsPressed = (QApplication::keyboardModifiers() != Qt::NoModifier);

  // set the new mouse vis only if there are no modifier keys pressed and the
  // entity was different from the previously selected one.
  if (!keyIsPressed && (this->dataPtr->selectionObj->GetMode() ==
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
      this->dataPtr->mouseEvent.Button() == common::MouseEvent::LEFT)
  {
    // Root visual
    rendering::VisualPtr rootVis = vis->GetRootVisual();

    // Root visual's immediate child
    rendering::VisualPtr topLevelVis = vis->GetNthAncestor(2);

    // If the root visual's ID can be found, it is a model in the main window
    // TODO gui::get_entity_id always return 0 in QTestFixture due to nullptr
    // g_main_win
    if (gui::get_entity_id(rootVis->GetName()))
    {
      // select model
      vis = rootVis;
    }
    // If it is not a model and its parent is either a direct child or
    // grandchild of the world, this is a light, so just keep vis = vis
    else if (vis->GetParent() == rootVis ||
        vis->GetParent() == this->dataPtr->scene->WorldVisual())
    {
      // select light
    }
    // Otherwise, this is a visual in the model editor, so we want to get its
    // top level visual below the root.
    else
    {
      // select link / nested model
      vis = topLevelVis;
    }

    this->dataPtr->mouseMoveVisStartPose = vis->GetWorldPose().Ign();

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
  if (this->dataPtr->mouseEvent.Dragging())
  {
    if (this->dataPtr->mouseMoveVis &&
        this->dataPtr->mouseEvent.Button() == common::MouseEvent::LEFT)
    {
      if (this->dataPtr->transparent)
      {
        this->dataPtr->mouseMoveVis->SetTransparency(
          (1.0 - this->dataPtr->mouseMoveVis->GetTransparency()) * 0.5);
        this->dataPtr->transparent = false;
      }
      ignition::math::Vector3d axis = ignition::math::Vector3d::Zero;
      if (this->dataPtr->keyEvent.key == Qt::Key_X)
        axis.X() = 1;
      else if (this->dataPtr->keyEvent.key == Qt::Key_Y)
        axis.Y() = 1;
      else if (this->dataPtr->keyEvent.key == Qt::Key_Z)
        axis.Z() = 1;

      if (this->dataPtr->selectionObj->GetMode() ==
          rendering::SelectionObj::TRANS)
      {
        if (axis != ignition::math::Vector3d::Zero)
        {
          this->TranslateEntity(this->dataPtr->mouseMoveVis, axis, false);
        }
        else if (this->dataPtr->selectionObj->GetState()
            == rendering::SelectionObj::TRANS_X)
        {
          this->TranslateEntity(this->dataPtr->mouseMoveVis,
              ignition::math::Vector3d::UnitX, !this->dataPtr->globalManip);
        }
        else if (this->dataPtr->selectionObj->GetState()
            == rendering::SelectionObj::TRANS_Y)
        {
          this->TranslateEntity(this->dataPtr->mouseMoveVis,
              ignition::math::Vector3d::UnitY, !this->dataPtr->globalManip);
        }
        else if (this->dataPtr->selectionObj->GetState()
            == rendering::SelectionObj::TRANS_Z)
        {
          this->TranslateEntity(this->dataPtr->mouseMoveVis,
            ignition::math::Vector3d::UnitZ, !this->dataPtr->globalManip);
        }
        else
        {
          this->TranslateEntity(
              this->dataPtr->mouseMoveVis, ignition::math::Vector3d(1, 1, 0));
        }
      }
      else if (this->dataPtr->selectionObj->GetMode()
          == rendering::SelectionObj::ROT)
      {
        if (axis != ignition::math::Vector3d::Zero)
        {
          this->RotateEntity(this->dataPtr->mouseMoveVis, axis, false);
        }
        else if (this->dataPtr->selectionObj->GetState()
            == rendering::SelectionObj::ROT_X
            || this->dataPtr->keyEvent.key == Qt::Key_X)
        {
          this->RotateEntity(this->dataPtr->mouseMoveVis,
              ignition::math::Vector3d::UnitX,
              !this->dataPtr->globalManip);
        }
        else if (this->dataPtr->selectionObj->GetState()
            == rendering::SelectionObj::ROT_Y
            || this->dataPtr->keyEvent.key == Qt::Key_Y)
        {
          this->RotateEntity(this->dataPtr->mouseMoveVis,
              ignition::math::Vector3d::UnitY,
              !this->dataPtr->globalManip);
        }
        else if (this->dataPtr->selectionObj->GetState()
            == rendering::SelectionObj::ROT_Z
            || this->dataPtr->keyEvent.key == Qt::Key_Z)
        {
          this->RotateEntity(this->dataPtr->mouseMoveVis,
              ignition::math::Vector3d::UnitZ,
              !this->dataPtr->globalManip);
        }
      }
      else if (this->dataPtr->selectionObj->GetMode()
          == rendering::SelectionObj::SCALE)
      {
        if (axis != ignition::math::Vector3d::Zero)
        {
          this->ScaleEntity(this->dataPtr->mouseMoveVis, axis, false);
        }
        else if (this->dataPtr->selectionObj->GetState()
            == rendering::SelectionObj::SCALE_X
            || this->dataPtr->keyEvent.key == Qt::Key_X)
        {
          this->ScaleEntity(this->dataPtr->mouseMoveVis,
              ignition::math::Vector3d::UnitX, true);
        }
        else if (this->dataPtr->selectionObj->GetState()
            == rendering::SelectionObj::SCALE_Y
            || this->dataPtr->keyEvent.key == Qt::Key_Y)
        {
          this->ScaleEntity(this->dataPtr->mouseMoveVis,
              ignition::math::Vector3d::UnitY, true);
        }
        else if (this->dataPtr->selectionObj->GetState()
            == rendering::SelectionObj::SCALE_Z
            || this->dataPtr->keyEvent.key == Qt::Key_Z)
        {
          this->ScaleEntity(this->dataPtr->mouseMoveVis,
              ignition::math::Vector3d::UnitZ, true);
        }
      }
    }
    else
      this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
  }
  else
  {
    std::string manipState;
    this->dataPtr->userCamera->GetVisual(this->dataPtr->mouseEvent.Pos(),
        manipState);
    this->dataPtr->selectionObj->SetState(manipState);

    if (!manipState.empty())
      QApplication::setOverrideCursor(Qt::OpenHandCursor);
    else
    {
      rendering::VisualPtr vis = this->dataPtr->userCamera->GetVisual(
          this->dataPtr->mouseEvent.Pos());

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
  if (this->dataPtr->mouseEvent.Dragging())
  {
    // If we were dragging a visual around, then publish its new pose to the
    // server
    if (this->dataPtr->mouseMoveVis)
    {
      this->dataPtr->mouseMoveVis->SetTransparency(
        std::abs(this->dataPtr->mouseMoveVis->GetTransparency()*2.0-1.0));
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
    if (this->dataPtr->mouseEvent.Button() == common::MouseEvent::LEFT)
    {
      rendering::VisualPtr vis =
        this->dataPtr->userCamera->GetVisual(this->dataPtr->mouseEvent.Pos());
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

  this->dataPtr->mouseMoveVisStartPose = vis->GetWorldPose().Ign();

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
    this->dataPtr->transparent = true;
    this->dataPtr->mouseVisualScale = _vis->GetScale().Ign();
    this->dataPtr->mouseChildVisualScale.clear();
    // keep track of all child visual scale for scaling to work in
    // model editor mode.
    for (unsigned int i = 0; i < _vis->GetChildCount(); ++i)
    {
      rendering::VisualPtr childVis = _vis->GetChild(i);
      this->dataPtr->mouseChildVisualScale.push_back(
          childVis->GetScale().Ign());
    }
    this->dataPtr->mouseVisualBbox = _vis->GetBoundingBox().Ign();
  }
  else
    this->dataPtr->mouseVisualScale = ignition::math::Vector3d::One;
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
      this->dataPtr->mouseStart = this->dataPtr->mouseEvent.Pos();
      if (this->dataPtr->mouseMoveVis)
      {
        this->dataPtr->mouseMoveVisStartPose =
            this->dataPtr->mouseMoveVis->GetWorldPose().Ign();
      }
    }
    else if (QApplication::keyboardModifiers() & Qt::ShiftModifier)
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
      this->dataPtr->mouseStart = this->dataPtr->mouseEvent.Pos();
      if (this->dataPtr->mouseMoveVis)
      {
        this->dataPtr->mouseMoveVisStartPose =
            this->dataPtr->mouseMoveVis->GetWorldPose().Ign();
      }
    }
    else if (this->dataPtr->keyEvent.key == Qt::Key_Shift)
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
  ignition::math::Vector3d pp;

  // Rotate the visual using the middle mouse button
  if (this->dataPtr->mouseEvent.buttons == common::MouseEvent::MIDDLE)
  {
    auto rpy = this->dataPtr->mouseMoveVisStartPose.Rot().Euler();
    auto delta = this->dataPtr->mouseEvent.Pos() -
        this->dataPtr->mouseEvent.pressPos;
    double yaw = (delta.X() * 0.01) + rpy.Z();
    if (!this->dataPtr->mouseEvent.shift)
    {
      double snap = rint(yaw / (M_PI * .25)) * (M_PI * 0.25);

      if (fabs(yaw - snap) < GZ_DTOR(10))
        yaw = snap;
    }

    _vis->SetWorldRotation(ignition::math::Quaterniond(rpy.X(), rpy.Y(), yaw));
  }
  else if (this->dataPtr->mouseEvent.buttons == common::MouseEvent::RIGHT)
  {
    auto rpy = this->dataPtr->mouseMoveVisStartPose.Rot().Euler();
    auto delta = this->dataPtr->mouseEvent.Pos() -
        this->dataPtr->mouseEvent.pressPos;
    double pitch = (delta.Y() * 0.01) + rpy.Y();
    if (!this->dataPtr->mouseEvent.shift)
    {
      double snap = rint(pitch / (M_PI * .25)) * (M_PI * 0.25);

      if (fabs(pitch - snap) < GZ_DTOR(10))
        pitch = snap;
    }

    _vis->SetWorldRotation(
        ignition::math::Quaterniond(rpy.X(), pitch, rpy.Z()));
  }
  else if (this->dataPtr->mouseEvent.buttons & common::MouseEvent::LEFT &&
           this->dataPtr->mouseEvent.buttons & common::MouseEvent::RIGHT)
  {
    auto rpy = this->dataPtr->mouseMoveVisStartPose.Rot().Euler();
    auto delta = this->dataPtr->mouseEvent.Pos() -
        this->dataPtr->mouseEvent.pressPos;
    double roll = (delta.X() * 0.01) + rpy.X();
    if (!this->dataPtr->mouseEvent.shift)
    {
      double snap = rint(roll / (M_PI * .25)) * (M_PI * 0.25);

      if (fabs(roll - snap) < GZ_DTOR(10))
        roll = snap;
    }

    _vis->SetWorldRotation(ignition::math::Quaterniond(roll, rpy.Y(), rpy.Z()));
  }
  else
  {
    this->TranslateEntity(_vis);
  }
}*/
