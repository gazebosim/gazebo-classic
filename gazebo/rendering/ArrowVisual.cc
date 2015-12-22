/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
/* Desc: Arrow Visualization Class
 * Author: Nate Koenig
 */

#include "gazebo/common/MeshManager.hh"

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/ArrowVisualPrivate.hh"
#include "gazebo/rendering/ArrowVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
ArrowVisual::ArrowVisual(const std::string &_name, VisualPtr _vis)
  : Visual(*new ArrowVisualPrivate, _name, _vis, false)
{
  ArrowVisualPrivate *dPtr =
      reinterpret_cast<ArrowVisualPrivate *>(this->dataPtr);

  dPtr->type = VT_GUI;

  dPtr->headNodeVisible = true;
  dPtr->shaftNodeVisible = true;
  dPtr->rotationNodeVisible = false;
}

/////////////////////////////////////////////////
void ArrowVisual::Load()
{
  Visual::Load();

  ArrowVisualPrivate *dPtr =
      reinterpret_cast<ArrowVisualPrivate *>(this->dataPtr);

  // Make sure the meshes are in Ogre
  this->InsertMesh("axis_shaft");
  this->InsertMesh("axis_head");

  dPtr->shaftVis.reset(
      new Visual(this->GetName()+"__SHAFT__", shared_from_this(), false));
  dPtr->shaftVis->Load();
  dPtr->shaftVis->AttachMesh("axis_shaft");
  dPtr->shaftVis->SetPosition(math::Vector3(0, 0, 0.1));
  dPtr->shaftVis->SetCastShadows(false);

  dPtr->headVis.reset(
      new Visual(this->GetName()+"__HEAD__", shared_from_this(), false));
  dPtr->headVis->Load();
  dPtr->headVis->AttachMesh("axis_head");
  dPtr->headVis->SetPosition(math::Vector3(0, 0, 0.24));
  dPtr->headVis->SetCastShadows(false);

  common::MeshManager::Instance()->CreateTube("rotation_tube",
      0.035, 0.04, 0.01, 1, 32);
  this->InsertMesh("rotation_tube");

  dPtr->rotationVis.reset(
      new Visual(this->GetName()+"__ROTATION__", shared_from_this(), false));
  dPtr->rotationVis->Load();
  dPtr->rotationVis->AttachMesh("rotation_tube");
  dPtr->rotationVis->SetPosition(math::Vector3(0, 0, 0.24));
  dPtr->rotationVis->SetCastShadows(false);

  this->ShowRotation(false);

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}

/////////////////////////////////////////////////
void ArrowVisual::ShowShaft(bool _show)
{
  ArrowVisualPrivate *dPtr =
      reinterpret_cast<ArrowVisualPrivate *>(this->dataPtr);

  dPtr->shaftNodeVisible = _show;

  if (dPtr->shaftVis)
    dPtr->shaftVis->SetVisible(_show);
}

/////////////////////////////////////////////////
void ArrowVisual::ShowHead(bool _show)
{
  ArrowVisualPrivate *dPtr =
      reinterpret_cast<ArrowVisualPrivate *>(this->dataPtr);

  dPtr->headNodeVisible = _show;
  if (dPtr->headVis)
    dPtr->headVis->SetVisible(_show);
}

/////////////////////////////////////////////////
void ArrowVisual::ShowRotation(bool _show)
{
  ArrowVisualPrivate *dPtr =
      reinterpret_cast<ArrowVisualPrivate *>(this->dataPtr);

  dPtr->rotationNodeVisible = _show;

  if (dPtr->rotationVis)
    dPtr->rotationVis->SetVisible(_show);
}

/////////////////////////////////////////////////
void ArrowVisual::SetVisible(bool _visible, bool _cascade)
{
  ArrowVisualPrivate *dPtr =
      reinterpret_cast<ArrowVisualPrivate *>(this->dataPtr);

  dPtr->headVis->SetVisible(dPtr->headNodeVisible && _visible, _cascade);
  dPtr->shaftVis->SetVisible(dPtr->shaftNodeVisible && _visible, _cascade);
  dPtr->rotationVis->SetVisible(
      dPtr->rotationNodeVisible && _visible, _cascade);

  this->dataPtr->visible = _visible;
}
