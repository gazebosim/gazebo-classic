/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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
#include <functional>
#include <mutex>
#include <ignition/math/Triangle.hh>
#include <ignition/math/Vector3.hh>

#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/RayQuery.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GuiEvents.hh"

#include "gazebo/gui/ModelSnapPrivate.hh"
#include "gazebo/gui/ModelSnap.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelSnap::ModelSnap()
  : dataPtr(new ModelSnapPrivate)
{
  this->dataPtr->initialized = false;
  this->dataPtr->selectedTriangleDirty = false;
  this->dataPtr->hoverTriangleDirty = false;
  this->dataPtr->snapLines = nullptr;
  this->dataPtr->snapHighlight = nullptr;
}

/////////////////////////////////////////////////
ModelSnap::~ModelSnap()
{
  this->Fini();
}

/////////////////////////////////////////////////
void ModelSnap::Fini()
{
  this->Clear();
}

/////////////////////////////////////////////////
void ModelSnap::Clear()
{
  this->dataPtr->selectedTriangleDirty = false;
  this->dataPtr->hoverTriangleDirty = false;

  this->dataPtr->selectedTriangle.Set(
      ignition::math::Vector3d::Zero,
      ignition::math::Vector3d::Zero,
      ignition::math::Vector3d::Zero);
  this->dataPtr->hoverTriangle.Set(
      ignition::math::Vector3d::Zero,
      ignition::math::Vector3d::Zero,
      ignition::math::Vector3d::Zero);

  this->dataPtr->selectedVis.reset();
  this->dataPtr->hoverVis.reset();

  this->dataPtr->userCmdPub.reset();
  if (this->dataPtr->node)
    this->dataPtr->node->Fini();
  this->dataPtr->node.reset();

  this->dataPtr->renderConnection.reset();

  if (this->dataPtr->snapVisual)
  {
    this->dataPtr->snapVisual->DeleteDynamicLine(this->dataPtr->snapLines);
    this->dataPtr->snapLines = nullptr;
    this->dataPtr->snapVisual->Fini();
    this->dataPtr->snapVisual.reset();
  }
  if (this->dataPtr->highlightVisual)
  {
    this->dataPtr->highlightVisual->DeleteDynamicLine(
        this->dataPtr->snapHighlight);
    this->dataPtr->snapHighlight = nullptr;
    this->dataPtr->highlightVisual->Fini();
    this->dataPtr->highlightVisual.reset();
  }

  this->dataPtr->scene.reset();
  this->dataPtr->userCamera.reset();
  this->dataPtr->rayQuery.reset();

  this->dataPtr->initialized = false;
}

/////////////////////////////////////////////////
void ModelSnap::Init()
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
  this->dataPtr->node->TryInit(common::Time::Maximum());
  this->dataPtr->userCmdPub =
      this->dataPtr->node->Advertise<msgs::UserCmd>("~/user_cmd");

  this->dataPtr->rayQuery.reset(
      new rendering::RayQuery(this->dataPtr->userCamera));

  this->dataPtr->initialized = true;
}

/////////////////////////////////////////////////
void ModelSnap::Reset()
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->updateMutex);
  this->dataPtr->selectedVis.reset();
  this->dataPtr->selectedTriangle.Set(
      ignition::math::Vector3d::Zero,
      ignition::math::Vector3d::Zero,
      ignition::math::Vector3d::Zero);

  this->dataPtr->hoverVis.reset();
  this->dataPtr->hoverTriangle.Set(
    ignition::math::Vector3d::Zero,
    ignition::math::Vector3d::Zero,
    ignition::math::Vector3d::Zero);

  this->dataPtr->hoverTriangleDirty = false;
  this->dataPtr->selectedTriangleDirty = false;

  if (this->dataPtr->snapVisual)
  {
    if (this->dataPtr->snapVisual->GetVisible())
      this->dataPtr->snapVisual->SetVisible(false);
  }

  if (this->dataPtr->highlightVisual)
  {
    if (this->dataPtr->highlightVisual->GetVisible())
      this->dataPtr->highlightVisual->SetVisible(false);
  }

  this->dataPtr->renderConnection.reset();
}

/////////////////////////////////////////////////
void ModelSnap::OnMousePressEvent(const common::MouseEvent &_event)
{
  this->dataPtr->mouseEvent = _event;
  this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
}

/////////////////////////////////////////////////
void ModelSnap::OnMouseMoveEvent(const common::MouseEvent &_event)
{
  this->dataPtr->mouseEvent = _event;

  rendering::VisualPtr vis = this->dataPtr->userCamera->Visual(
      this->dataPtr->mouseEvent.Pos());

  if (vis && !vis->IsPlane())
  {
    // get the triangle being hovered so that it can be highlighted
    ignition::math::Vector3d intersect;
    ignition::math::Triangle3d hoverTriangle;
    this->dataPtr->rayQuery->SelectMeshTriangle(_event.Pos().X(),
        _event.Pos().Y(), vis->GetRootVisual(), intersect, hoverTriangle);

    if (hoverTriangle.Valid())
    {
      std::lock_guard<std::recursive_mutex> lock(this->dataPtr->updateMutex);
      this->dataPtr->hoverVis = vis;
      this->dataPtr->hoverTriangle = hoverTriangle;
      this->dataPtr->hoverTriangleDirty = true;

      if (!this->dataPtr->renderConnection)
      {
        this->dataPtr->renderConnection = event::Events::ConnectRender(
            std::bind(&ModelSnap::Update, this));
      }
    }
  }
  else
  {
    std::lock_guard<std::recursive_mutex> lock(this->dataPtr->updateMutex);
    this->dataPtr->hoverVis.reset();
    this->dataPtr->hoverTriangle.Set(
        ignition::math::Vector3d::Zero,
        ignition::math::Vector3d::Zero,
        ignition::math::Vector3d::Zero);
    this->dataPtr->hoverTriangleDirty = true;
  }

  this->dataPtr->mouseEvent = _event;
  this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
}

//////////////////////////////////////////////////
void ModelSnap::OnMouseReleaseEvent(const common::MouseEvent &_event)
{
  this->dataPtr->mouseEvent = _event;

  rendering::VisualPtr vis = this->dataPtr->userCamera->Visual(
      this->dataPtr->mouseEvent.Pos());

  if (vis && !vis->IsPlane() &&
      this->dataPtr->mouseEvent.Button() == common::MouseEvent::LEFT)
  {
    // Parent model or parent link
    rendering::VisualPtr currentParent = vis->GetRootVisual();
    rendering::VisualPtr previousParent;
    rendering::VisualPtr topLevelVis = vis->GetNthAncestor(2);

    if (gui::get_entity_id(currentParent->Name()))
    {
      if (this->dataPtr->selectedVis)
        previousParent = this->dataPtr->selectedVis->GetRootVisual();
    }
    else
    {
      currentParent = topLevelVis;
      if (this->dataPtr->selectedVis)
      {
        previousParent = this->dataPtr->selectedVis->GetNthAncestor(2);
      }
    }

    // Select first triangle on any mesh
    // Update triangle if the new triangle is on the same model/link
    if (!this->dataPtr->selectedVis || (currentParent  == previousParent))
    {
      ignition::math::Vector3d intersect;
      this->dataPtr->rayQuery->SelectMeshTriangle(_event.Pos().X(),
          _event.Pos().Y(), currentParent, intersect,
          this->dataPtr->selectedTriangle);

      if (this->dataPtr->selectedTriangle.Valid())
      {
        this->dataPtr->selectedVis = vis;
        this->dataPtr->selectedTriangleDirty = true;
      }
      if (!this->dataPtr->renderConnection)
      {
        this->dataPtr->renderConnection = event::Events::ConnectRender(
            std::bind(&ModelSnap::Update, this));
      }
    }
    else
    {
      // select triangle on the target
      ignition::math::Vector3d intersect;
      ignition::math::Triangle3d triangle;
      this->dataPtr->rayQuery->SelectMeshTriangle(_event.Pos().X(),
          _event.Pos().Y(), currentParent, intersect, triangle);

      if (triangle.Valid())
      {
        this->Snap(this->dataPtr->selectedTriangle, triangle, previousParent);

        this->Reset();
        gui::Events::manipMode("select");
      }
    }
  }
  else
    this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
}

//////////////////////////////////////////////////
void ModelSnap::Snap(const ignition::math::Triangle3d &_triangleSrc,
    const ignition::math::Triangle3d &_triangleDest,
    rendering::VisualPtr _visualSrc)
{
  ignition::math::Vector3d translation;
  ignition::math::Quaterniond rotation;

  this->SnapTransform(_triangleSrc, _triangleDest,
      _visualSrc->WorldPose(), translation, rotation);

  _visualSrc->SetWorldPose(
      ignition::math::Pose3d(_visualSrc->WorldPose().Pos() +
      translation, rotation * _visualSrc->WorldPose().Rot()));

  Events::moveEntity(_visualSrc->Name(), _visualSrc->WorldPose(),
      true);

  this->PublishVisualPose(_visualSrc);
}

//////////////////////////////////////////////////
void ModelSnap::SnapTransform(
    const ignition::math::Triangle3d &_triangleSrc,
    const ignition::math::Triangle3d &_triangleDest,
    const ignition::math::Pose3d &_poseSrc, ignition::math::Vector3d &_trans,
    ignition::math::Quaterniond &_rot)
{
  // snap the centroid of one triangle to another
  auto centroidSrc =
      (_triangleSrc[0] + _triangleSrc[1] + _triangleSrc[2]) / 3.0;

  auto centroidDest =
      (_triangleDest[0] + _triangleDest[1] + _triangleDest[2]) / 3.0;

  auto normalSrc = _triangleSrc.Normal();

  auto normalDest = _triangleDest.Normal();

  auto u = normalDest.Normalize() * -1;
  auto v = normalSrc.Normalize();
  double cosTheta = v.Dot(u);
  double angle = acos(cosTheta);
  // check the parallel case
  if (ignition::math::equal(angle, M_PI))
    _rot.Axis(u.Perpendicular(), angle);
  else
    _rot.Axis((v.Cross(u)).Normalize(), angle);

  // Get translation needed for alignment
  // taking into account the rotated position of the mesh
  _trans = centroidDest - (_rot * (centroidSrc - _poseSrc.Pos()) +
      _poseSrc.Pos());
}

/////////////////////////////////////////////////
void ModelSnap::PublishVisualPose(rendering::VisualPtr _vis)
{
  if (!_vis)
    return;

  // Only publish for models
  if (_vis->GetType() == gazebo::rendering::Visual::VT_MODEL)
  {
    // Register user command on server
    msgs::UserCmd userCmdMsg;
    userCmdMsg.set_description("Snap [" + _vis->Name() + "]");
    userCmdMsg.set_type(msgs::UserCmd::MOVING);

    msgs::Model msg;

    auto id = gui::get_entity_id(_vis->Name());
    if (id)
      msg.set_id(id);

    msg.set_name(_vis->Name());
    msgs::Set(msg.mutable_pose(), _vis->WorldPose());

    auto modelMsg = userCmdMsg.add_model();
    modelMsg->CopyFrom(msg);

    this->dataPtr->userCmdPub->Publish(userCmdMsg);
  }
}

/////////////////////////////////////////////////
void ModelSnap::Update()
{
  std::lock_guard<std::recursive_mutex> lock(this->dataPtr->updateMutex);
  if (this->dataPtr->hoverTriangleDirty)
  {
    if (this->dataPtr->hoverTriangle.Valid())
    {
      // convert triangle to local coordinates relative to parent visual
      ignition::math::Triangle3d hoverTriangle;
      for (unsigned int i = 0; i < 3; ++i)
      {
        hoverTriangle.Set(i,
            this->dataPtr->hoverVis->WorldPose().Rot().Inverse() *
            (this->dataPtr->hoverTriangle[i] -
            this->dataPtr->hoverVis->WorldPose().Pos()));
      }

      if (!this->dataPtr->highlightVisual)
      {
        // create the highlight
        std::string highlightVisName = "_SNAP_HIGHLIGHT_";
        this->dataPtr->highlightVisual.reset(new rendering::Visual(
            highlightVisName, this->dataPtr->hoverVis, false));
        this->dataPtr->highlightVisual->Load();
        this->dataPtr->snapHighlight =
            this->dataPtr->highlightVisual->CreateDynamicLine(
            rendering::RENDERING_TRIANGLE_FAN);
        GZ_OGRE_SET_MATERIAL_BY_NAME(this->dataPtr->snapHighlight,
            "Gazebo/RedTransparent");
        this->dataPtr->snapHighlight->AddPoint(hoverTriangle[0]);
        this->dataPtr->snapHighlight->AddPoint(hoverTriangle[1]);
        this->dataPtr->snapHighlight->AddPoint(hoverTriangle[2]);
        this->dataPtr->snapHighlight->AddPoint(hoverTriangle[0]);
        this->dataPtr->highlightVisual->SetVisible(true);
        this->dataPtr->highlightVisual->GetSceneNode()->setInheritScale(false);
        this->dataPtr->highlightVisual->SetVisibilityFlags(
            GZ_VISIBILITY_GUI & ~GZ_VISIBILITY_SELECTABLE);
      }
      else
      {
        // set new highlight position
        if (!this->dataPtr->highlightVisual->GetVisible())
          this->dataPtr->highlightVisual->SetVisible(true);
        if (this->dataPtr->hoverVis !=
            this->dataPtr->highlightVisual->GetParent())
        {
          if (this->dataPtr->highlightVisual->GetParent())
          {
            this->dataPtr->highlightVisual->GetParent()->DetachVisual(
                this->dataPtr->highlightVisual);
          }
          this->dataPtr->hoverVis->AttachVisual(this->dataPtr->highlightVisual);
        }
        this->dataPtr->snapHighlight->SetPoint(0, hoverTriangle[0]);
        this->dataPtr->snapHighlight->SetPoint(1, hoverTriangle[1]);
        this->dataPtr->snapHighlight->SetPoint(2, hoverTriangle[2]);
        this->dataPtr->snapHighlight->SetPoint(3, hoverTriangle[0]);
      }
    }
    else
    {
      // turn of visualization if no mesh triangles are hovered
      this->dataPtr->highlightVisual->SetVisible(false);
    }
    this->dataPtr->hoverTriangleDirty = false;
  }

  if (this->dataPtr->selectedTriangleDirty &&
      this->dataPtr->selectedTriangle.Valid())
  {
    // convert triangle to local coordinates relative to parent visual
    ignition::math::Triangle3d triangle;
    for (unsigned int i = 0; i < 3; ++i)
    {
      triangle.Set(i,
          this->dataPtr->selectedVis->WorldPose().Rot().Inverse() *
          (this->dataPtr->selectedTriangle[i] -
          this->dataPtr->selectedVis->WorldPose().Pos()));
    }

    if (!this->dataPtr->snapVisual)
    {
      // draw a border around selected triangle
      rendering::UserCameraPtr camera = gui::get_active_camera();

      std::string snapVisName = "_SNAP_";
      this->dataPtr->snapVisual.reset(new rendering::Visual(
          snapVisName, this->dataPtr->selectedVis, false));
      this->dataPtr->snapVisual->Load();
      this->dataPtr->snapLines =
          this->dataPtr->snapVisual->CreateDynamicLine(
          rendering::RENDERING_LINE_STRIP);
      GZ_OGRE_SET_MATERIAL_BY_NAME(this->dataPtr->snapLines, "Gazebo/RedGlow");
      this->dataPtr->snapLines->AddPoint(triangle[0]);
      this->dataPtr->snapLines->AddPoint(triangle[1]);
      this->dataPtr->snapLines->AddPoint(triangle[2]);
      this->dataPtr->snapLines->AddPoint(triangle[0]);
      this->dataPtr->snapVisual->SetVisible(true);
      this->dataPtr->snapVisual->GetSceneNode()->setInheritScale(false);
      this->dataPtr->snapVisual->SetVisibilityFlags(
          GZ_VISIBILITY_GUI & ~GZ_VISIBILITY_SELECTABLE);
    }
    else
    {
      // update border if the selected triangle changes
      if (!this->dataPtr->snapVisual->GetVisible())
        this->dataPtr->snapVisual->SetVisible(true);
      if (this->dataPtr->selectedVis != this->dataPtr->snapVisual->GetParent())
      {
        if (this->dataPtr->snapVisual->GetParent())
        {
          this->dataPtr->snapVisual->GetParent()->DetachVisual(
              this->dataPtr->snapVisual);
        }
        this->dataPtr->selectedVis->AttachVisual(this->dataPtr->snapVisual);
      }
      this->dataPtr->snapLines->SetPoint(0, triangle[0]);
      this->dataPtr->snapLines->SetPoint(1, triangle[1]);
      this->dataPtr->snapLines->SetPoint(2, triangle[2]);
      this->dataPtr->snapLines->SetPoint(3, triangle[0]);
    }
    this->dataPtr->selectedTriangleDirty = false;
  }
}

//////////////////////////////////////////////////
ModelSnap* ModelSnap::Instance()
{
#ifndef _WIN32
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
  return SingletonT<ModelSnap>::Instance();
#ifndef _WIN32
  #pragma GCC diagnostic pop
#endif
}
