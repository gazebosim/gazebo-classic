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

#include "gazebo/transport/transport.hh"

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/RenderEvents.hh"
#include "gazebo/rendering/RenderingIface.hh"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/PointVisual.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/Conversions.hh"
#include "gazebo/rendering/RayQuery.hh"

#include "gazebo/gui/Actions.hh"
#include "gazebo/gui/GuiEvents.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/qt.h"

#include "gazebo/gui/TapeMeasurePrivate.hh"
#include "gazebo/gui/TapeMeasure.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
TapeMeasure::TapeMeasure() : dataPtr(new TapeMeasurePrivate)
{
  this->dataPtr->initialized = false;
  this->dataPtr->selectedPtDirty = false;
  this->dataPtr->hoverPtDirty = false;
  this->dataPtr->lines = NULL;
}

/////////////////////////////////////////////////
TapeMeasure::~TapeMeasure()
{
  this->Clear();
}

/////////////////////////////////////////////////
void TapeMeasure::Enable()
{
  MouseEventHandler::Instance()->AddReleaseFilter("tape_measure",
      std::bind(&TapeMeasure::OnMouseRelease, this, std::placeholders::_1));

  MouseEventHandler::Instance()->AddPressFilter("tape_measure",
      std::bind(&TapeMeasure::OnMousePress, this, std::placeholders::_1));

  MouseEventHandler::Instance()->AddMoveFilter("tape_measure",
      std::bind(&TapeMeasure::OnMouseMove, this, std::placeholders::_1));
}

/////////////////////////////////////////////////
void TapeMeasure::Disable()
{
  MouseEventHandler::Instance()->RemoveReleaseFilter("tape_measure");
  MouseEventHandler::Instance()->RemovePressFilter("tape_measure");
  MouseEventHandler::Instance()->RemoveMoveFilter("tape_measure");
}

/////////////////////////////////////////////////
void TapeMeasure::Clear()
{
  this->dataPtr->selectedPtDirty = false;
  this->dataPtr->hoverPtDirty = false;
  this->dataPtr->selectedVis.reset();
  this->dataPtr->hoverVis.reset();
/*
  {
    std::unique_lock<std::recursive_mutex> lock(this->dataPtr->updateMutex);

    delete this->dataPtr->lines;
    this->dataPtr->lines = NULL;
    this->dataPtr->lineVisual.reset();

    if (this->dataPtr->highlightVis != NULL)
    {
      this->dataPtr->highlightVis->
          DeleteDynamicLine(this->dataPtr->snapHighlight);
    }
    this->dataPtr->highlightVis.reset();
  }
*/
  if (this->dataPtr->renderConnection)
    event::Events::DisconnectRender(this->dataPtr->renderConnection);
  this->dataPtr->renderConnection.reset();

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

  // Get user camera and scene
  auto cam = gui::get_active_camera();
  if (!cam)
    return;

  if (!cam->GetScene())
    return;

  this->dataPtr->userCamera = cam;
  this->dataPtr->scene = cam->GetScene();
  auto worldVis = this->dataPtr->scene->WorldVisual();;

  // Ray query to find mesh triangles
  this->dataPtr->rayQuery.reset(
      new rendering::RayQuery(this->dataPtr->userCamera));

  ignition::math::Vector3d scale(0.5, 0.5, 0.5);

  // Highlight visual
  this->dataPtr->highlightVis.reset(new rendering::PointVisual(
      "_TAPEMEASURE_HIGHLIGHT_", worldVis));

  this->dataPtr->highlightVis->Load();
  this->dataPtr->highlightVis->SetScale(scale);
  this->dataPtr->highlightVis->SetMaterial("Gazebo/RedTransparent");
  this->dataPtr->highlightVis->SetVisible(false);
  this->dataPtr->highlightVis->SetTransparency(0.5);
  this->dataPtr->highlightVis->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  this->dataPtr->highlightVis->GetSceneNode()->setInheritScale(false);

  // Snap highlight visual
  for (int i = 0; i < 6; ++i)
  {
    std::string name = "_TAPEMEASURE_SNAP_" + std::to_string(i) + "_";
    rendering::PointVisualPtr pointVis;
    pointVis.reset(new rendering::PointVisual(name, worldVis));

    pointVis->Load();
    pointVis->SetScale(scale);
    pointVis->SetMaterial("Gazebo/YellowTransparent");
    pointVis->SetVisible(false);
    pointVis->SetTransparency(0.8);
    pointVis->SetVisibilityFlags(GZ_VISIBILITY_GUI);
    pointVis->GetSceneNode()->setInheritScale(false);
    this->dataPtr->snapVisuals.push_back(pointVis);
  }

  // Point visuals
  for (int i = 0; i < 2; ++i)
  {
    std::string name = "_TAPEMEASURE_POINT_" + std::to_string(i) + "_";
    rendering::PointVisualPtr pointVis;
    pointVis.reset(new rendering::PointVisual(name, worldVis));

    pointVis->Load();
    pointVis->SetScale(scale);
    pointVis->SetMaterial("Gazebo/BlueTransparent");
    pointVis->SetVisible(false);
    pointVis->SetTransparency(0.5);
    pointVis->SetVisibilityFlags(GZ_VISIBILITY_GUI);
    pointVis->GetSceneNode()->setInheritScale(false);
    this->dataPtr->pointVisuals.push_back(pointVis);
  }

  // Line
  this->dataPtr->lineVisual.reset(new rendering::Visual(
      "_TAPEMEASURE_LINE_", this->dataPtr->scene, false));
  this->dataPtr->lineVisual->SetVisible(false);
  this->dataPtr->lineVisual->GetSceneNode()->setInheritScale(false);
  this->dataPtr->lineVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);

  this->dataPtr->lines =
      this->dataPtr->lineVisual->CreateDynamicLine(
      rendering::RENDERING_LINE_LIST);
  this->dataPtr->lines->AddPoint(ignition::math::Vector3d::Zero);
  this->dataPtr->lines->AddPoint(ignition::math::Vector3d::Zero);
  this->dataPtr->lines->setMaterial("Gazebo/Blue");

  // Text
  this->dataPtr->textVisual.reset(new rendering::Visual(
      "_TAPEMEASURE_TEXT_", this->dataPtr->scene, false));
  this->dataPtr->textVisual->GetSceneNode()->setInheritScale(false);
  this->dataPtr->textVisual->SetVisibilityFlags(GZ_VISIBILITY_GUI);

  this->dataPtr->text.Load("_TAPEMEASURE_TEXT_",
      "0m", "Arial", 0.1, common::Color::Blue);
  this->dataPtr->text.SetShowOnTop(true);

  auto textNode =
      this->dataPtr->textVisual->GetSceneNode()->createChildSceneNode(
      "_TAPEMEASURE_TEXT_NODE_");
  textNode->attachObject(&(this->dataPtr->text));
  textNode->setInheritScale(false);
  this->dataPtr->textVisual->SetVisible(false);

  this->dataPtr->initialized = true;
}

/////////////////////////////////////////////////
void TapeMeasure::Reset()
{
  std::unique_lock<std::recursive_mutex> lock(this->dataPtr->updateMutex);

  this->dataPtr->selectedVis.reset();
  this->dataPtr->selectedPt = ignition::math::Vector3d::Zero;

  this->dataPtr->hoverVis.reset();
  this->dataPtr->hoverPt = ignition::math::Vector3d::Zero;

  this->dataPtr->hoverPtDirty = false;
  this->dataPtr->selectedPtDirty = false;

  for (auto &vis : this->dataPtr->pointVisuals)
  {
    if (vis->GetVisible())
      vis->SetVisible(false);

    if (vis->GetParent())
    {
      vis->GetParent()->DetachVisual(vis);
      this->dataPtr->scene->WorldVisual()->AttachVisual(vis);
    }
  }

  if (this->dataPtr->highlightVis)
  {
    this->dataPtr->highlightVis->SetVisible(false);
    if (this->dataPtr->highlightVis->GetParent())
    {
      this->dataPtr->highlightVis->GetParent()->DetachVisual(
          this->dataPtr->highlightVis);
    }
  }

  for (auto &vis : this->dataPtr->snapVisuals)
  {
    vis->SetVisible(false);
  }

  if (this->dataPtr->lineVisual->GetVisible())
    this->dataPtr->lineVisual->SetVisible(false);

  if (this->dataPtr->textVisual->GetVisible())
    this->dataPtr->textVisual->SetVisible(false);

  event::Events::DisconnectRender(this->dataPtr->renderConnection);
  this->dataPtr->renderConnection.reset();
}

/////////////////////////////////////////////////
bool TapeMeasure::OnMousePress(const common::MouseEvent &/*_event*/)
{
  return false;
}

/////////////////////////////////////////////////
bool TapeMeasure::OnMouseMove(const common::MouseEvent &_event)
{
  // Get the first visual
  auto vis = this->dataPtr->userCamera->GetVisual(_event.Pos());

  // Get the first triangle intercepted
  ignition::math::Vector3d pt;
  std::vector<ignition::math::Vector3d> triangle;
  if (vis)
  {
    this->dataPtr->rayQuery->SelectMeshTriangle(_event.Pos().X(),
        _event.Pos().Y(), vis->GetRootVisual(), pt, triangle);
  }

  // If holding Shift, get the closest triangle vertex
  if (!triangle.empty() &&
      QApplication::keyboardModifiers() & Qt::ShiftModifier)
  {
    this->dataPtr->snapPts.clear();
    this->dataPtr->snapPts.push_back(triangle[0]);
    this->dataPtr->snapPts.push_back(triangle[1]);
    this->dataPtr->snapPts.push_back(triangle[2]);
    this->dataPtr->snapPts.push_back((triangle[0] + triangle[1])/2.0);
    this->dataPtr->snapPts.push_back((triangle[0] + triangle[2])/2.0);
    this->dataPtr->snapPts.push_back((triangle[1] + triangle[2])/2.0);

    std::vector<double> diff;
    for (unsigned int i = 0; i < 6; ++i)
    {
      diff.push_back((this->dataPtr->snapPts[i] - pt).Length());
    }

    auto closestIdx = std::min_element(std::begin(diff), std::end(diff)) -
        diff.begin();

    pt = this->dataPtr->snapPts[closestIdx];
  }
  else
  {
    this->dataPtr->snapPts.clear();
  }

  if (pt != ignition::math::Vector3d::Zero)
  {
    this->dataPtr->hoverPt = pt;
    this->dataPtr->hoverVis = vis;

    if (!this->dataPtr->renderConnection)
    {
      this->dataPtr->renderConnection = event::Events::ConnectRender(
          boost::bind(&TapeMeasure::Update, this));
    }
  }
  else
  {
    std::unique_lock<std::recursive_mutex> lock(this->dataPtr->updateMutex);

    this->dataPtr->hoverVis.reset();
    this->dataPtr->hoverPt = ignition::math::Vector3d::Zero;
  }
  this->dataPtr->hoverPtDirty = true;

  return false;
}

//////////////////////////////////////////////////
bool TapeMeasure::OnMouseRelease(const common::MouseEvent &_event)
{
  if (_event.Button() == common::MouseEvent::LEFT)
  {
    this->dataPtr->selectedPt = this->dataPtr->hoverPt;
    this->dataPtr->selectedVis = this->dataPtr->hoverVis;
    this->dataPtr->selectedPtDirty = true;
  }

  return false;
}

/////////////////////////////////////////////////
void TapeMeasure::Update()
{
  std::unique_lock<std::recursive_mutex> lock(this->dataPtr->updateMutex);

  bool pt0Chosen = this->dataPtr->pointVisuals[0]->GetDepth() > 1;
  bool pt1Chosen = this->dataPtr->pointVisuals[1]->GetDepth() > 1;

  // Highlight visuals
  if (this->dataPtr->hoverPtDirty)
  {
    if (this->dataPtr->hoverPt != ignition::math::Vector3d::Zero &&
        this->dataPtr->hoverVis && !pt1Chosen)
    {
      // Highlight vis
      auto pt =
          this->dataPtr->hoverVis->GetWorldPose().Ign().Rot().Inverse() *
          (this->dataPtr->hoverPt -
          this->dataPtr->hoverVis->GetWorldPose().Ign().Pos());

      if (this->dataPtr->highlightVis->GetParent())
      {
        this->dataPtr->highlightVis->GetParent()->DetachVisual(
            this->dataPtr->highlightVis);
      }
      this->dataPtr->hoverVis->AttachVisual(
          this->dataPtr->highlightVis);

      this->dataPtr->highlightVis->SetPosition(
          pt / this->dataPtr->hoverVis->GetScale().Ign());
      this->dataPtr->lines->SetPoint(1, pt);
      this->dataPtr->highlightVis->SetVisible(true);

      // Snap visuals
      if (!this->dataPtr->snapPts.empty())
      {
        for (unsigned int i = 0; i < 6; ++i)
        {
          if (this->dataPtr->snapPts[i] != this->dataPtr->hoverPt)
          {
            auto snapPt =
                this->dataPtr->hoverVis->GetWorldPose().Ign().Rot().Inverse() *
                (this->dataPtr->snapPts[i] -
                this->dataPtr->hoverVis->GetWorldPose().Ign().Pos());

            if (this->dataPtr->snapVisuals[i]->GetParent())
            {
              this->dataPtr->snapVisuals[i]->GetParent()->DetachVisual(
                  this->dataPtr->snapVisuals[i]);
            }
            this->dataPtr->hoverVis->AttachVisual(
                this->dataPtr->snapVisuals[i]);

            this->dataPtr->snapVisuals[i]->SetPosition(
                snapPt / this->dataPtr->hoverVis->GetScale().Ign());
            this->dataPtr->snapVisuals[i]->SetVisible(true);
          }
          else
            this->dataPtr->snapVisuals[i]->SetVisible(false);
        }
      }
      else
      {
        for (unsigned int i = 0; i < 6; ++i)
          this->dataPtr->snapVisuals[i]->SetVisible(false);
      }
    }
    else
    {
      // turn off visualization if no visuals are hovered
      this->dataPtr->highlightVis->SetVisible(false);
      for (unsigned int i = 0; i < 6; ++i)
        this->dataPtr->snapVisuals[i]->SetVisible(false);
    }

    this->dataPtr->hoverPtDirty = false;
  }

  // Point visuals
  if (this->dataPtr->selectedPtDirty && this->dataPtr->selectedVis &&
      this->dataPtr->selectedPt != ignition::math::Vector3d::Zero)
  {
    // convert to local coordinates relative to parent visual
    auto selectedPt =
        this->dataPtr->selectedVis->GetWorldPose().Ign().Rot().Inverse() *
        (this->dataPtr->selectedPt -
        this->dataPtr->selectedVis->GetWorldPose().Ign().Pos());

    if (!pt0Chosen)
    {
      if (this->dataPtr->selectedVis !=
          this->dataPtr->pointVisuals[0]->GetParent())
      {
        if (this->dataPtr->pointVisuals[0]->GetParent())
        {
          this->dataPtr->pointVisuals[0]->GetParent()->DetachVisual(
              this->dataPtr->pointVisuals[0]);
        }
        this->dataPtr->selectedVis->AttachVisual(
            this->dataPtr->pointVisuals[0]);
      }

      this->dataPtr->pointVisuals[0]->SetVisible(true);
      this->dataPtr->pointVisuals[0]->SetPosition(selectedPt /
          this->dataPtr->selectedVis->GetScale().Ign());

      this->dataPtr->lines->SetPoint(0, this->dataPtr->selectedPt);
    }
    else if (!pt1Chosen)
    {
      if (this->dataPtr->selectedVis !=
          this->dataPtr->pointVisuals[1]->GetParent())
      {
        if (this->dataPtr->pointVisuals[1]->GetParent())
        {
          this->dataPtr->pointVisuals[1]->GetParent()->DetachVisual(
              this->dataPtr->pointVisuals[1]);
        }
        this->dataPtr->selectedVis->AttachVisual(
            this->dataPtr->pointVisuals[1]);
      }

      this->dataPtr->pointVisuals[1]->SetVisible(true);
      this->dataPtr->pointVisuals[1]->SetPosition(selectedPt /
          this->dataPtr->selectedVis->GetScale().Ign());

      this->dataPtr->lines->SetPoint(1, this->dataPtr->selectedPt);
    }

    this->dataPtr->selectedPtDirty = false;
  }

  if (pt0Chosen)
  {
    this->dataPtr->lines->SetPoint(0,
        this->dataPtr->pointVisuals[0]->GetWorldPose().Ign().Pos());
  }

  if (pt1Chosen)
  {
    this->dataPtr->lines->SetPoint(1,
        this->dataPtr->pointVisuals[1]->GetWorldPose().Ign().Pos());
  }

  if (pt0Chosen && !pt1Chosen)
  {
    // Line
    if (!this->dataPtr->lineVisual->GetVisible())
      this->dataPtr->lineVisual->SetVisible(true);

    this->dataPtr->lines->SetPoint(1, this->dataPtr->hoverPt);

    // Text
    if (!this->dataPtr->textVisual->GetVisible())
      this->dataPtr->textVisual->SetVisible(true);

    this->dataPtr->textVisual->SetPosition(
        (this->dataPtr->lines->Point(0) +
         this->dataPtr->lines->Point(1)) / 2.0);

    auto length = (this->dataPtr->lines->Point(0) -
        this->dataPtr->lines->Point(1)).Length();

    std::ostringstream lengthStr;
    lengthStr << std::fixed << std::setprecision(3) << length;

    this->dataPtr->text.SetText(lengthStr.str() + "m");
  }


  if (pt0Chosen && pt1Chosen)
  {
    // Text
    if (!this->dataPtr->textVisual->GetVisible())
      this->dataPtr->textVisual->SetVisible(true);

    this->dataPtr->textVisual->SetPosition(
        (this->dataPtr->lines->Point(0) +
         this->dataPtr->lines->Point(1)) / 2.0);

    auto length = (this->dataPtr->lines->Point(0) -
        this->dataPtr->lines->Point(1)).Length();

    std::ostringstream lengthStr;
    lengthStr << std::fixed << std::setprecision(3) << length;

    this->dataPtr->text.SetText(lengthStr.str() + "m");
  }
}

/////////////////////////////////////////////////
TapeMeasureWidget::TapeMeasureWidget(QWidget *_parent)
  : QMenu(_parent), dataPtr(new TapeMeasureWidgetPrivate)
{
  auto newMeasure = new QAction(tr("New Measure"), this);
  this->addAction(newMeasure);
  this->connect(newMeasure, SIGNAL(triggered()), this, SLOT(OnNewMeasure()));

  auto clearAll = new QAction(tr("Clear All"), this);
  this->addAction(clearAll);
  this->connect(clearAll, SIGNAL(triggered()), this, SLOT(OnClearAll()));
}

/////////////////////////////////////////////////
TapeMeasureWidget::~TapeMeasureWidget()
{
}

/////////////////////////////////////////////////
void TapeMeasureWidget::OnNewMeasure()
{
  TapeMeasure::Instance()->Enable();
}

/////////////////////////////////////////////////
void TapeMeasureWidget::OnClearAll()
{
  TapeMeasure::Instance()->Reset();
}

