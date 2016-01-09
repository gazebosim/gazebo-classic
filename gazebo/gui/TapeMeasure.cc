/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <boost/bind.hpp>

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

#include "gazebo/gui/TapeMeasurePrivate.hh"
#include "gazebo/gui/TapeMeasure.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
TapeMeasure::TapeMeasure() : dataPtr(new TapeMeasurePrivate)
{
  this->dataPtr->initialized = false;
  this->dataPtr->selectedVertexDirty = false;
  this->dataPtr->hoverVertexDirty = false;
  this->dataPtr->dynamicLines = NULL;
  this->dataPtr->updateMutex = NULL;
}

/////////////////////////////////////////////////
TapeMeasure::~TapeMeasure()
{
  this->Clear();
}

/////////////////////////////////////////////////
void TapeMeasure::Clear()
{
gzdbg << "Clear" << std::endl;
  this->dataPtr->selectedVertexDirty = false;
  this->dataPtr->hoverVertexDirty = false;
  this->dataPtr->selectedVis.reset();
  this->dataPtr->hoverVis.reset();
/*
  if (this->dataPtr->updateMutex)
  {
    boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
    delete this->dataPtr->snapLines;
    this->dataPtr->snapLines = NULL;
    this->dataPtr->snapVisual.reset();

    if (this->dataPtr->snapHighlight != NULL &&
        this->dataPtr->highlightVisual != NULL)
    {
      this->dataPtr->highlightVisual->
          DeleteDynamicLine(this->dataPtr->snapHighlight);
    }
    this->dataPtr->highlightVisual.reset();
  }
*/

  if (this->dataPtr->renderConnection)
    event::Events::DisconnectRender(this->dataPtr->renderConnection);
  this->dataPtr->renderConnection.reset();

  delete this->dataPtr->updateMutex;
  this->dataPtr->updateMutex = NULL;

  this->dataPtr->scene.reset();
  this->dataPtr->userCamera.reset();
  this->dataPtr->rayQuery.reset();

  this->dataPtr->initialized = false;
}

/////////////////////////////////////////////////
void TapeMeasure::Init()
{
  if (this->dataPtr->initialized)
    return;

  auto cam = gui::get_active_camera();
  if (!cam)
    return;

  if (!cam->GetScene())
    return;

  this->dataPtr->userCamera = cam;
  this->dataPtr->scene = cam->GetScene();

  this->dataPtr->updateMutex = new boost::recursive_mutex();

  this->dataPtr->rayQuery.reset(
      new rendering::RayQuery(this->dataPtr->userCamera));

  // Highlight visual
  this->dataPtr->highlightVis.reset(new rendering::Visual(
      "_TAPEMEASURE_POINT_0_", this->dataPtr->scene, false));

  this->dataPtr->highlightVis->Load();
  this->dataPtr->highlightVis->AttachMesh("unit_sphere");
  this->dataPtr->highlightVis->SetScale(
      ignition::math::Vector3d(0.05, 0.05, 0.05));
  this->dataPtr->highlightVis->SetCastShadows(false);
  this->dataPtr->highlightVis->SetMaterial("Gazebo/RedTransparent");
  this->dataPtr->highlightVis->SetVisible(false);
  this->dataPtr->highlightVis->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  this->dataPtr->highlightVis->GetSceneNode()->setInheritScale(false);

  // Point visuals
  for (int i = 0; i < 2; ++i)
  {
    std::string name = "_TAPEMEASURE_POINT_" + std::to_string(i) + "_";
    rendering::VisualPtr pointVis;
    pointVis.reset(new rendering::Visual(name, this->dataPtr->scene, false));

    pointVis->Load();
    pointVis->AttachMesh("unit_sphere");
    pointVis->SetScale(ignition::math::Vector3d(0.05, 0.05, 0.05));
    pointVis->SetCastShadows(false);
    pointVis->SetMaterial("Gazebo/BlueTransparent");
    pointVis->SetVisible(false);
    pointVis->SetVisibilityFlags(GZ_VISIBILITY_GUI);
    pointVis->GetSceneNode()->setInheritScale(false);
    this->dataPtr->pointVisuals.push_back(pointVis);
  }

  // Line
  std::string name = "_TAPEMEASURE_LINE_";
  this->dataPtr->lineVisual.reset(new rendering::Visual(
      name, this->dataPtr->scene, false));
  this->dataPtr->lineVisual->SetVisible(false);
  this->dataPtr->lineVisual->GetSceneNode()->setInheritScale(false);
  this->dataPtr->lineVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);

  this->dataPtr->dynamicLines =
      this->dataPtr->lineVisual->CreateDynamicLine(
      rendering::RENDERING_LINE_LIST);
  this->dataPtr->dynamicLines->AddPoint(ignition::math::Vector3d::Zero);
  this->dataPtr->dynamicLines->AddPoint(ignition::math::Vector3d::Zero);
  this->dataPtr->dynamicLines->setMaterial("Gazebo/Blue");

  this->dataPtr->initialized = true;
}

/////////////////////////////////////////////////
void TapeMeasure::Reset()
{
gzdbg << "Reset" << std::endl;
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
  this->dataPtr->selectedVis.reset();
  this->dataPtr->selectedVertex = ignition::math::Vector3d::Zero;

  this->dataPtr->hoverVis.reset();
  this->dataPtr->hoverVertex = ignition::math::Vector3d::Zero;

  this->dataPtr->hoverVertexDirty = false;
  this->dataPtr->selectedVertexDirty = false;
/*
  if (this->dataPtr->snapVisual)
  {
    if (this->dataPtr->snapVisual->GetVisible())
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
*/
}

/////////////////////////////////////////////////
void TapeMeasure::OnMousePressEvent(const common::MouseEvent &_event)
{
  this->dataPtr->mouseEvent = _event;
  this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
}

/////////////////////////////////////////////////
void TapeMeasure::OnMouseMoveEvent(const common::MouseEvent &_event)
{
  this->dataPtr->mouseEvent = _event;

  // \todo If holding shift, snap to a vertex on a mesh
  auto vis = this->dataPtr->userCamera->GetVisual(
      this->dataPtr->mouseEvent.Pos());
  // Get the first contact point
  ignition::math::Pose3d pose;
  if (this->dataPtr->scene->FirstContact(this->dataPtr->userCamera,
        this->dataPtr->mouseEvent.Pos(), pose.Pos()) && vis)
  {
    this->dataPtr->hoverVertex = pose.Pos();
    this->dataPtr->hoverVertexDirty = true;
    this->dataPtr->hoverVis = vis;

    if (!this->dataPtr->renderConnection)
    {
      this->dataPtr->renderConnection = event::Events::ConnectRender(
          boost::bind(&TapeMeasure::Update, this));
    }
  }
  else
  {
    boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
    this->dataPtr->hoverVis.reset();
    this->dataPtr->hoverVertex = ignition::math::Vector3d::Zero;
    this->dataPtr->hoverVertexDirty = true;
  }

  // Line
  if (this->dataPtr->pointVisuals[0]->GetDepth() != 0 &&
      this->dataPtr->pointVisuals[1]->GetDepth() == 0)
  {
    this->dataPtr->dynamicLines->SetPoint(1, this->dataPtr->hoverVertex);
  }


/*



  auto vis = this->dataPtr->userCamera->GetVisual(
      this->dataPtr->mouseEvent.Pos());
  if (vis)
  {
    // get the triangle being hovered so that it can be highlighted
    math::Vector3 intersect;
    std::vector<math::Vector3> hoverVertex;
    this->dataPtr->rayQuery->SelectMeshTriangle(_event.Pos().X(),
        _event.Pos().Y(), vis->GetRootVisual(), intersect, hoverVertex);

    if (!hoverVertex.empty())
    {
      boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
      this->dataPtr->hoverVis = vis;
      this->dataPtr->hoverVertex = hoverVertex;
      this->dataPtr->hoverVertexDirty = true;
    }
  }
  else
  {
    boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);
    this->dataPtr->hoverVis.reset();
    this->dataPtr->hoverVertex.clear();
    this->dataPtr->hoverVertexDirty = true;
  }


  this->dataPtr->mouseEvent = _event;
  this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
*/
}

//////////////////////////////////////////////////
void TapeMeasure::OnMouseReleaseEvent(const common::MouseEvent &_event)
{
  this->dataPtr->mouseEvent = _event;

  if (this->dataPtr->mouseEvent.Button() == common::MouseEvent::LEFT)
  {
    this->dataPtr->selectedVertex = this->dataPtr->hoverVertex;
    this->dataPtr->selectedVis = this->dataPtr->hoverVis;
    this->dataPtr->selectedVertexDirty = true;
/*
    // Select first triangle on any mesh
    // Update triangle if the new triangle is on the same model/link
    if (!this->dataPtr->selectedVis || (currentParent  == previousParent))
    {
      math::Vector3 intersect;
      this->dataPtr->rayQuery->SelectMeshTriangle(_event.Pos().X(),
          _event.Pos().Y(), currentParent, intersect,
          this->dataPtr->selectedVertex);

      if (!this->dataPtr->selectedVertex.empty())
      {
        this->dataPtr->selectedVertexDirty = true;
      }
      if (!this->dataPtr->renderConnection)
      {
        this->dataPtr->renderConnection = event::Events::ConnectRender(
            boost::bind(&TapeMeasure::Update, this));
      }
    }
    else
    {
      // select triangle on the target
      math::Vector3 intersect;
      std::vector<math::Vector3> vertices;
      this->dataPtr->rayQuery->SelectMeshTriangle(_event.Pos().X(),
          _event.Pos().Y(), currentParent, intersect, vertices);

      if (!vertices.empty())
      {
        this->Snap(this->dataPtr->selectedVertex, vertices, previousParent);

        this->Reset();
        gui::Events::manipMode("select");
      }
    }
*/
  }
/*
  else
    this->dataPtr->userCamera->HandleMouseEvent(this->dataPtr->mouseEvent);
*/
}

/////////////////////////////////////////////////
void TapeMeasure::Update()
{
  boost::recursive_mutex::scoped_lock lock(*this->dataPtr->updateMutex);

  // Highlight visual
  if (this->dataPtr->hoverVertexDirty)
  {
    if (this->dataPtr->hoverVertex != ignition::math::Vector3d::Zero)
    {
      // Set new point position
      if (!this->dataPtr->highlightVis->GetVisible())
        this->dataPtr->highlightVis->SetVisible(true);

      if (this->dataPtr->hoverVis !=
          this->dataPtr->highlightVis->GetParent())
      {
        if (this->dataPtr->highlightVis->GetParent())
        {
          this->dataPtr->highlightVis->GetParent()->DetachVisual(
              this->dataPtr->highlightVis);
        }
        this->dataPtr->hoverVis->AttachVisual(this->dataPtr->highlightVis);
      }

      // convert triangle to local coordinates relative to parent visual
      auto hoverVertex =
            this->dataPtr->hoverVis->GetWorldPose().Ign().Rot().Inverse() *
            (this->dataPtr->hoverVertex -
            this->dataPtr->hoverVis->GetWorldPose().Ign().Pos());

      this->dataPtr->highlightVis->SetPosition(hoverVertex);
    }
    else
    {
      // turn off visualization if no visuals are hovered
      this->dataPtr->highlightVis->SetVisible(false);
    }
    this->dataPtr->hoverVertexDirty = false;
  }

  // Point visuals
  if (this->dataPtr->selectedVertexDirty &&
      this->dataPtr->selectedVertex != ignition::math::Vector3d::Zero)
  {
    // convert triangle to local coordinates relative to parent visual
    auto selectedVertex =
          this->dataPtr->selectedVis->GetWorldPose().Ign().Rot().Inverse() *
          (this->dataPtr->selectedVertex -
          this->dataPtr->selectedVis->GetWorldPose().Ign().Pos());

    if (this->dataPtr->pointVisuals[0]->GetDepth() == 0)
    {
      if (this->dataPtr->selectedVis !=
          this->dataPtr->pointVisuals[0]->GetParent())
      {
        if (this->dataPtr->pointVisuals[0]->GetParent())
        {
          this->dataPtr->pointVisuals[0]->GetParent()->DetachVisual(
              this->dataPtr->pointVisuals[0]);
        }
        this->dataPtr->selectedVis->AttachVisual(this->dataPtr->pointVisuals[0]);
      }

      this->dataPtr->pointVisuals[0]->SetVisible(true);
      this->dataPtr->pointVisuals[0]->SetPosition(selectedVertex);

      this->dataPtr->dynamicLines->SetPoint(0, this->dataPtr->selectedVertex);
      this->dataPtr->lineVisual->SetVisible(true);
    }
    else
    {
      if (this->dataPtr->selectedVis !=
          this->dataPtr->pointVisuals[1]->GetParent())
      {
        if (this->dataPtr->pointVisuals[1]->GetParent())
        {
          this->dataPtr->pointVisuals[1]->GetParent()->DetachVisual(
              this->dataPtr->pointVisuals[1]);
        }
        this->dataPtr->selectedVis->AttachVisual(this->dataPtr->pointVisuals[1]);
      }

      this->dataPtr->pointVisuals[1]->SetVisible(true);
      this->dataPtr->pointVisuals[1]->SetPosition(selectedVertex);

      this->dataPtr->dynamicLines->SetPoint(1, this->dataPtr->selectedVertex);
    }
/*
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
      this->dataPtr->snapLines->AddPoint(triangle[0].Ign());
      this->dataPtr->snapLines->AddPoint(triangle[1].Ign());
      this->dataPtr->snapLines->AddPoint(triangle[2].Ign());
      this->dataPtr->snapLines->AddPoint(triangle[0].Ign());
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
      this->dataPtr->snapLines->SetPoint(0, triangle[0].Ign());
      this->dataPtr->snapLines->SetPoint(1, triangle[1].Ign());
      this->dataPtr->snapLines->SetPoint(2, triangle[2].Ign());
      this->dataPtr->snapLines->SetPoint(3, triangle[0].Ign());
    }
*/
    this->dataPtr->selectedVertexDirty = false;
  }
}
