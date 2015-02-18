/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
#include "gazebo/rendering/Visual.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/UserCamera.hh"
#include "gazebo/rendering/RayQuery.hh"

#include "gazebo/gui/qt.h"
#include "gazebo/gui/MouseEventHandler.hh"
#include "gazebo/gui/GuiIface.hh"
#include "gazebo/gui/GuiEvents.hh"

#include "gazebo/gui/ModelAlignPrivate.hh"
#include "gazebo/gui/ModelAlign.hh"

using namespace gazebo;
using namespace gui;

/////////////////////////////////////////////////
ModelAlign::ModelAlign()
  : dataPtr(new ModelAlignPrivate)
{
  this->dataPtr->initialized = false;
}

/////////////////////////////////////////////////
ModelAlign::~ModelAlign()
{
  this->dataPtr->modelPub.reset();
  delete this->dataPtr;
  this->dataPtr = NULL;
}

/////////////////////////////////////////////////
void ModelAlign::Init()
{
  if (this->dataPtr->initialized)
    return;

  rendering::UserCameraPtr cam = gui::get_active_camera();
  if (!cam)
    return;

  if (!cam->GetScene())
    return;

  this->dataPtr->userCamera = cam;
  this->dataPtr->scene = cam->GetScene();

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init();
  this->dataPtr->modelPub =
      this->dataPtr->node->Advertise<msgs::Model>("~/model/modify");

  this->dataPtr->initialized = true;
}

/////////////////////////////////////////////////
void ModelAlign::Transform(math::Box _bbox, math::Pose _worldPose,
    std::vector<math::Vector3> &_vertices)
{
  math::Vector3 center = _bbox.GetCenter();

  // Get the 8 corners of the bounding box.
  math::Vector3 v0 = center +
      math::Vector3(-_bbox.GetXLength()/2.0, _bbox.GetYLength()/2.0,
      _bbox.GetZLength()/2.0);
  math::Vector3 v1 = center +
      math::Vector3(_bbox.GetXLength()/2.0, _bbox.GetYLength()/2.0,
      _bbox.GetZLength()/2.0);
  math::Vector3 v2 = center +
      math::Vector3(-_bbox.GetXLength()/2.0, -_bbox.GetYLength()/2.0,
      _bbox.GetZLength()/2.0);
  math::Vector3 v3 = center +
      math::Vector3(_bbox.GetXLength()/2.0, -_bbox.GetYLength()/2.0,
      _bbox.GetZLength()/2.0);

  math::Vector3 v4 = center +
      math::Vector3(-_bbox.GetXLength()/2.0, _bbox.GetYLength()/2.0,
      -_bbox.GetZLength()/2.0);
  math::Vector3 v5 = center +
      math::Vector3(_bbox.GetXLength()/2.0, _bbox.GetYLength()/2.0,
      -_bbox.GetZLength()/2.0);
  math::Vector3 v6 = center +
      math::Vector3(-_bbox.GetXLength()/2.0, -_bbox.GetYLength()/2.0,
      -_bbox.GetZLength()/2.0);
  math::Vector3 v7 = center +
      math::Vector3(_bbox.GetXLength()/2.0, -_bbox.GetYLength()/2.0,
      -_bbox.GetZLength()/2.0);

  // Transform corners into world spacce.
  v0 = _worldPose.rot * v0 + _worldPose.pos;
  v1 = _worldPose.rot * v1 + _worldPose.pos;
  v2 = _worldPose.rot * v2 + _worldPose.pos;
  v3 = _worldPose.rot * v3 + _worldPose.pos;
  v4 = _worldPose.rot * v4 + _worldPose.pos;
  v5 = _worldPose.rot * v5 + _worldPose.pos;
  v6 = _worldPose.rot * v6 + _worldPose.pos;
  v7 = _worldPose.rot * v7 + _worldPose.pos;

  _vertices.clear();
  _vertices.push_back(v0);
  _vertices.push_back(v1);
  _vertices.push_back(v2);
  _vertices.push_back(v3);
  _vertices.push_back(v4);
  _vertices.push_back(v5);
  _vertices.push_back(v6);
  _vertices.push_back(v7);
}

/////////////////////////////////////////////////
void ModelAlign::GetMinMax(std::vector<math::Vector3> _vertices,
    math::Vector3 &_min, math::Vector3 &_max)
{
  if (_vertices.empty())
    return;

  _min = _vertices[0];
  _max = _vertices[0];

  // find min / max in world space;
  for (unsigned int i = 1; i < _vertices.size(); ++i)
  {
    math::Vector3 v = _vertices[i];
    if (_min.x > v.x)
      _min.x = v.x;
    if (_max.x < v.x)
      _max.x = v.x;
    if (_min.y > v.y)
      _min.y = v.y;
    if (_max.y < v.y)
      _max.y = v.y;
    if (_min.z > v.z)
      _min.z = v.z;
    if (_max.z < v.z)
      _max.z = v.z;
  }
}

/////////////////////////////////////////////////
void ModelAlign::AlignVisuals(std::vector<rendering::VisualPtr> _visuals,
    const std::string &_axis, const std::string &_config,
    const std::string &_target, bool _publish)
{
  if (_config == "reset" || _publish)
  {
    std::map<rendering::VisualPtr, math::Pose>::iterator it =
        this->dataPtr->originalVisualPose.begin();
    for (it; it != this->dataPtr->originalVisualPose.end(); ++it)
    {
      if (it->first)
      {
        it->first->SetWorldPose(it->second);
        if (it->first->GetChildCount() != 0)
        {
          for (unsigned int j = 0; j < it->first->GetChildCount(); ++j)
          {
            it->first->GetChild(j)->SetTransparency(std::abs(
                it->first->GetChild(j)->GetTransparency()*2.0-1.0));
          }
        }
        else
        {
          it->first->SetTransparency(std::abs(
                it->first->GetTransparency()*2.0-1.0));
        }
      }
    }
    this->dataPtr->originalVisualPose.clear();
    if (this->dataPtr->scene)
      this->dataPtr->scene->SelectVisual("", "normal");
    if (!_publish)
      return;
  }

  this->dataPtr->selectedVisuals = _visuals;

  if (this->dataPtr->selectedVisuals.size() <= 1)
    return;

  unsigned int start = 0;
  unsigned int end = 0;
  if (_target == "first")
  {
    start = 1;
    end = this->dataPtr->selectedVisuals.size();
    this->dataPtr->targetVis = this->dataPtr->selectedVisuals.front();
  }
  else if (_target == "last")
  {
    start = 0;
    end = this->dataPtr->selectedVisuals.size()-1;
    this->dataPtr->targetVis = this->dataPtr->selectedVisuals.back();
  }

  math::Pose targetWorldPose = this->dataPtr->targetVis->GetWorldPose();
  math::Box targetBbox = this->dataPtr->targetVis->GetBoundingBox();
  targetBbox.min *= this->dataPtr->targetVis->GetScale();
  targetBbox.max *= this->dataPtr->targetVis->GetScale();

  std::vector<math::Vector3> targetVertices;
  this->Transform(targetBbox, targetWorldPose, targetVertices);

  math::Vector3 targetMin;
  math::Vector3 targetMax;
  this->GetMinMax(targetVertices, targetMin, targetMax);

  for (unsigned i = start; i < end; ++i)
  {
    rendering::VisualPtr vis = this->dataPtr->selectedVisuals[i];

    math::Pose worldPose = vis->GetWorldPose();
    math::Box bbox = vis->GetBoundingBox();
    bbox.min *= vis->GetScale();
    bbox.max *= vis->GetScale();

    std::vector<math::Vector3> vertices;
    this->Transform(bbox, worldPose, vertices);

    math::Vector3 min;
    math::Vector3 max;
    this->GetMinMax(vertices, min, max);

    math::Vector3 trans;
    if (_config == "min")
      trans = targetMin - min;
    else if (_config == "center")
      trans = (targetMin + (targetMax-targetMin)/2) - (min + (max-min)/2);
    else if (_config == "max")
      trans = targetMax - max;

    if (!_publish)
    {
      if (this->dataPtr->originalVisualPose.find(vis) ==
          this->dataPtr->originalVisualPose.end())
      {
        this->dataPtr->originalVisualPose[vis] = vis->GetWorldPose();
        // Children might have different transparencies
        if (vis->GetChildCount() != 0)
        {
          for (unsigned int j = 0; j < vis->GetChildCount(); ++j)
          {
            vis->GetChild(j)->SetTransparency((1.0 -
                vis->GetChild(j)->GetTransparency()) * 0.5);
          }
        }
        else
        {
          vis->SetTransparency((1.0 - vis->GetTransparency()) * 0.5);
        }
      }
      // prevent the visual pose from being updated by the server
      if (this->dataPtr->scene)
        this->dataPtr->scene->SelectVisual(vis->GetName(), "move");
    }

    if (_axis == "x")
    {
      trans.y = trans.z = 0;
      vis->SetWorldPosition(vis->GetWorldPose().pos + trans);
    }
    else if (_axis == "y")
    {
      trans.x = trans.z = 0;
      vis->SetWorldPosition(vis->GetWorldPose().pos + trans);
    }
    else if (_axis == "z")
    {
      trans.x = trans.y = 0;
      vis->SetWorldPosition(vis->GetWorldPose().pos + trans);
    }

    if (_publish)
      this->PublishVisualPose(vis);
  }
}

/////////////////////////////////////////////////
void ModelAlign::PublishVisualPose(rendering::VisualPtr _vis)
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
