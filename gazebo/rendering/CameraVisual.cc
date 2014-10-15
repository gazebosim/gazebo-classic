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
/* Desc: Camera Visualization Class
 * Author: Nate Koenig
 */

#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/Camera.hh"
#include "gazebo/rendering/CameraVisualPrivate.hh"
#include "gazebo/rendering/CameraVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
CameraVisual::CameraVisual(const std::string &_name, VisualPtr _vis)
: Visual(*new CameraVisualPrivate, _name, _vis)
{
}

/////////////////////////////////////////////////
CameraVisual::~CameraVisual()
{
  CameraVisualPrivate *dPtr =
      reinterpret_cast<CameraVisualPrivate *>(this->dataPtr);

  if (dPtr->scene)
    dPtr->scene->RemoveCamera(dPtr->camera->GetName());

  dPtr->camera.reset();
}

/////////////////////////////////////////////////
void CameraVisual::Load(const msgs::CameraSensor &_msg)
{
  CameraVisualPrivate *dPtr =
      reinterpret_cast<CameraVisualPrivate *>(this->dataPtr);

  math::Vector2d imageSize = msgs::Convert(_msg.image_size());

  double dist = 2.0;
  double width = 1.0;
  double height = imageSize.y / static_cast<double>(imageSize.x);

  dPtr->camera = dPtr->scene->CreateCamera(this->GetName(), false);

  sdf::ElementPtr cameraElem = msgs::CameraSensorToSDF(_msg);
  dPtr->camera->Load(cameraElem);
  dPtr->camera->Init();
  dPtr->camera->CreateRenderTexture(this->GetName() + "_RTT");

  Ogre::MaterialPtr material =
    Ogre::MaterialManager::getSingleton().create(
        this->GetName()+"_RTT_material",

        Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->getTechnique(0)->getPass(0)->createTextureUnitState();
  material->getTechnique(0)->getPass(0)->setDepthCheckEnabled(true);
  material->getTechnique(0)->getPass(0)->setDepthWriteEnabled(true);
  material->getTechnique(0)->getPass(0)->setLightingEnabled(false);
  material->getTechnique(0)->getPass(0)->getTextureUnitState(
      0)->setTextureName(this->GetName()+"_RTT");

  Ogre::Plane plane;
  plane.normal = Ogre::Vector3::NEGATIVE_UNIT_X;
  plane.d = dist;

  Ogre::MeshManager::getSingleton().createPlane(this->GetName() + "__floor",
      Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME,
      plane, width, height, 1, 1, true, 1, 1.0f, 1.0f,
      Ogre::Vector3::UNIT_Z);

  Ogre::Entity* planeEnt =
    dPtr->scene->GetManager()->createEntity(this->GetName() + "__plane",
        this->GetName() + "__floor");
  planeEnt->setMaterialName(this->GetName()+"_RTT_material");
  planeEnt->setCastShadows(false);
  planeEnt->setVisibilityFlags(GZ_VISIBILITY_GUI);

  DynamicLines *line = this->CreateDynamicLine(RENDERING_LINE_LIST);

  line->AddPoint(math::Vector3(0, 0, 0));
  line->AddPoint(math::Vector3(dist, width*0.5, height*0.5));

  line->AddPoint(math::Vector3(0, 0, 0));
  line->AddPoint(math::Vector3(dist, -width*0.5, height*0.5));

  line->AddPoint(math::Vector3(0, 0, 0));
  line->AddPoint(math::Vector3(dist, -width*0.5, -height*0.5));

  line->AddPoint(math::Vector3(0, 0, 0));
  line->AddPoint(math::Vector3(dist, width*0.5, -height*0.5));

  line->setMaterial("Gazebo/WhiteGlow");
  line->setVisibilityFlags(GZ_VISIBILITY_GUI);

  this->AttachObject(planeEnt);
  dPtr->camera->AttachToVisual(this->GetId(), true);

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);

  if (dPtr->parent)
    dPtr->parent->AttachVisual(shared_from_this());

  dPtr->connections.push_back(
      event::Events::ConnectRender(
      boost::bind(&CameraVisual::Update, this)));
}

/////////////////////////////////////////////////
void CameraVisual::Update()
{
  CameraVisualPrivate *dPtr =
      reinterpret_cast<CameraVisualPrivate *>(this->dataPtr);

  if (!dPtr->camera)
    return;

  dPtr->camera->Render();
}
