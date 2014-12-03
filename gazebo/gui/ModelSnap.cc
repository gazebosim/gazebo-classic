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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

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
  this->dataPtr->snapLines = NULL;

  this->dataPtr->updateMutex = new boost::recursive_mutex();
}

/////////////////////////////////////////////////
ModelSnap::~ModelSnap()
{
  this->dataPtr->modelPub.reset();

  {
    boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
    if (this->dataPtr->snapLines)
      delete this->dataPtr->snapLines;
    this->dataPtr->snapVisual.reset();
  }

  event::Events::DisconnectRender(this->dataPtr->renderConnection);
  this->dataPtr->renderConnection.reset();

  delete this->dataPtr->updateMutex;

  delete this->dataPtr;
  this->dataPtr = NULL;
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
  this->dataPtr->node->Init();
  this->dataPtr->modelPub =
      this->dataPtr->node->Advertise<msgs::Model>("~/model/modify");

  this->dataPtr->rayQuery.reset(
      new rendering::RayQuery(this->dataPtr->userCamera));

  this->dataPtr->initialized = true;
}

/////////////////////////////////////////////////
void ModelSnap::Reset()
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
  this->dataPtr->selectedVis.reset();
  this->dataPtr->selectedTriangle.clear();

  this->dataPtr->hoverVis.reset();
  this->dataPtr->hoverTriangle.clear();

  this->dataPtr->hoverTriangleDirty = false;
  this->dataPtr->selectedTriangleDirty = false;

  if (this->dataPtr->snapVisual)
  {
    this->dataPtr->snapVisual->SetVisible(false);
    if (this->dataPtr->snapVisual->GetParent())
    {
      this->dataPtr->snapVisual->GetParent()->DetachVisual(
          this->dataPtr->snapVisual);
    }
  }

  if (this->dataPtr->highlightVisual)
  {
    this->dataPtr->highlightVisual->SetVisible(false);
    if (this->dataPtr->highlightVisual->GetParent())
    {
      this->dataPtr->highlightVisual->GetParent()->DetachVisual(
          this->dataPtr->highlightVisual);
    }
  }

  event::Events::DisconnectRender(this->dataPtr->renderConnection);
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

  rendering::VisualPtr vis = this->dataPtr->userCamera->GetVisual(
      this->dataPtr->mouseEvent.pos);
  if (vis && !vis->IsPlane())
  {
    // get the triangle being hovered so that it can be highlighted
    math::Vector3 intersect;
    std::vector<math::Vector3> hoverTriangle;
    this->dataPtr->rayQuery->SelectMeshTriangle(_event.pos.x, _event.pos.y,
        vis->GetRootVisual(), intersect, hoverTriangle);

    if (!hoverTriangle.empty())
    {
      boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
      this->dataPtr->hoverVis = vis;
      this->dataPtr->hoverTriangle = hoverTriangle;
      this->dataPtr->hoverTriangleDirty = true;

      if (!this->dataPtr->renderConnection)
      {
        this->dataPtr->renderConnection = event::Events::ConnectRender(
            boost::bind(&ModelSnap::Update, this));
      }
    }
  }
  else
  {
    boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
    this->dataPtr->hoverVis.reset();
    this->dataPtr->hoverTriangle.clear();
    this->dataPtr->hoverTriangleDirty = true;
  }

  this->dataPtr->mouseEvent = _event;
  this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
}

//////////////////////////////////////////////////
void ModelSnap::OnMouseReleaseEvent(const common::MouseEvent &_event)
{
  this->dataPtr->mouseEvent = _event;

  rendering::VisualPtr vis = this->dataPtr->userCamera->GetVisual(
      this->dataPtr->mouseEvent.pos);

  if (vis && !vis->IsPlane() &&
      this->dataPtr->mouseEvent.button == common::MouseEvent::LEFT)
  {
    // select first triangle on any mesh, if it's on the same mesh, update the
    // triangle vertices.
    if (!this->dataPtr->selectedVis ||
       (vis->GetRootVisual()  == this->dataPtr->selectedVis->GetRootVisual()))
    {
      math::Vector3 intersect;
      this->dataPtr->rayQuery->SelectMeshTriangle(_event.pos.x, _event.pos.y,
          vis->GetRootVisual(), intersect, this->dataPtr->selectedTriangle);

      if (!this->dataPtr->selectedTriangle.empty())
      {
        this->dataPtr->selectedVis = vis;
        this->dataPtr->selectedTriangleDirty = true;
      }
      if (!this->dataPtr->renderConnection)
      {
        this->dataPtr->renderConnection = event::Events::ConnectRender(
              boost::bind(&ModelSnap::Update, this));
      }
    }
    else
    {
      // select triangle on the target
      math::Vector3 intersect;
      std::vector<math::Vector3> vertices;
      this->dataPtr->rayQuery->SelectMeshTriangle(_event.pos.x, _event.pos.y,
          vis->GetRootVisual(), intersect, vertices);

      if (!vertices.empty())
      {
        this->Snap(this->dataPtr->selectedTriangle, vertices,
            this->dataPtr->selectedVis->GetRootVisual());

        this->Reset();
        gui::Events::manipMode("select");
      }
    }
  }
  else
    this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
}

//////////////////////////////////////////////////
void ModelSnap::Snap(const std::vector<math::Vector3> &_triangleSrc,
    const std::vector<math::Vector3> &_triangleDest,
    rendering::VisualPtr _visualSrc)
{
  math::Vector3 translation;
  math::Quaternion rotation;

  this->GetSnapTransform(_triangleSrc, _triangleDest,
      _visualSrc->GetWorldPose(), translation, rotation);

  _visualSrc->SetWorldPose(
      math::Pose(_visualSrc->GetWorldPose().pos + translation,
      rotation * _visualSrc->GetWorldPose().rot));

  this->PublishVisualPose(_visualSrc);
}

//////////////////////////////////////////////////
void ModelSnap::GetSnapTransform(const std::vector<math::Vector3> &_triangleSrc,
    const std::vector<math::Vector3> &_triangleDest,
    const math::Pose &_poseSrc, math::Vector3 &_trans,
    math::Quaternion &_rot)
{
  // snap the centroid of one triangle to another
  math::Vector3 centroidSrc = (_triangleSrc[0] + _triangleSrc[1] +
      _triangleSrc[2]) / 3.0;

  math::Vector3 centroidDest =
      (_triangleDest[0] + _triangleDest[1] + _triangleDest[2]) / 3.0;

  math::Vector3 normalSrc = math::Vector3::GetNormal(
      _triangleSrc[0], _triangleSrc[1], _triangleSrc[2]);

  math::Vector3 normalDest = math::Vector3::GetNormal(
      _triangleDest[0], _triangleDest[1], _triangleDest[2]);

  math::Vector3 u = normalDest.Normalize() * -1;
  math::Vector3 v = normalSrc.Normalize();
  double cosTheta = v.Dot(u);
  double angle = acos(cosTheta);
  // check the parallel case
  if (math::equal(angle, M_PI))
    _rot.SetFromAxis(u.GetPerpendicular(), angle);
  else
    _rot.SetFromAxis((v.Cross(u)).Normalize(), angle);

  // Get translation needed for alignment
  // taking into account the rotated position of the mesh
  _trans = centroidDest - (_rot * (centroidSrc - _poseSrc.pos) + _poseSrc.pos);
}

/////////////////////////////////////////////////
void ModelSnap::PublishVisualPose(rendering::VisualPtr _vis)
{
  if (!_vis)
    return;

  // Check to see if the visual is a model.
  if (gui::get_entity_id(_vis->GetName()))
  {
    msgs::Model msg;
    msg.set_id(gui::get_entity_id(_vis->GetName()));
    msg.set_name(_vis->GetName());

    msgs::Set(msg.mutable_pose(), _vis->GetWorldPose());
    this->dataPtr->modelPub->Publish(msg);
  }
}

/////////////////////////////////////////////////
void ModelSnap::Update()
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
  if (this->dataPtr->hoverTriangleDirty)
  {
    if (!this->dataPtr->hoverTriangle.empty())
    {
      // convert triangle to local coordinates relative to parent visual
      std::vector<math::Vector3> hoverTriangle;
      for (unsigned int i = 0; i < this->dataPtr->hoverTriangle.size(); ++i)
      {
        hoverTriangle.push_back(
            this->dataPtr->hoverVis->GetWorldPose().rot.GetInverse() *
            (this->dataPtr->hoverTriangle[i] -
            this->dataPtr->hoverVis->GetWorldPose().pos));
      }

      if (!this->dataPtr->highlightVisual)
      {
        // create the highlight
        std::string highlightVisName = "_SNAP_HIGHLIGHT_";
        this->dataPtr->highlightVisual.reset(new rendering::Visual(
            highlightVisName, this->dataPtr->hoverVis, false));
        this->dataPtr->snapHighlight =
            this->dataPtr->highlightVisual->CreateDynamicLine(
            rendering::RENDERING_TRIANGLE_FAN);
        this->dataPtr->snapHighlight->setMaterial("Gazebo/RedTransparent");
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
      !this->dataPtr->selectedTriangle.empty())
  {
    // convert triangle to local coordinates relative to parent visual
    std::vector<math::Vector3> triangle;
    for (unsigned int i = 0; i < this->dataPtr->selectedTriangle.size(); ++i)
    {
      triangle.push_back(
          this->dataPtr->selectedVis->GetWorldPose().rot.GetInverse() *
          (this->dataPtr->selectedTriangle[i] -
          this->dataPtr->selectedVis->GetWorldPose().pos));
    }

    if (!this->dataPtr->snapVisual)
    {
      // draw a border around selected triangle
      rendering::UserCameraPtr camera = gui::get_active_camera();

      std::string snapVisName = "_SNAP_";
      this->dataPtr->snapVisual.reset(new rendering::Visual(
          snapVisName, this->dataPtr->selectedVis, false));

      this->dataPtr->snapLines =
          this->dataPtr->snapVisual->CreateDynamicLine(
          rendering::RENDERING_LINE_STRIP);
      this->dataPtr->snapLines->setMaterial("Gazebo/RedGlow");
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
