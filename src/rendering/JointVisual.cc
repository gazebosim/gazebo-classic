/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Joint Visualization Class
 * Author: Nate Koenig
 */

#include "rendering/ogre.h"
#include "rendering/DynamicLines.hh"
#include "rendering/Scene.hh"
#include "rendering/AxisVisual.hh"
#include "rendering/JointVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
JointVisual::JointVisual(const std::string &_name, VisualPtr _vis)
  : Visual(_name, _vis)
{
  std::cout << "New JointVis[" << this->GetName() << "]\n";
}

/////////////////////////////////////////////////
JointVisual::~JointVisual()
{
  this->axisVisual.reset();
}

/////////////////////////////////////////////////
void JointVisual::Load(ConstJointPtr &_msg)
{
  Visual::Load();

  printf("JointVisual::Load\n");
  this->axisVisual.reset(
      new AxisVisual(this->GetName() + "_AXIS", shared_from_this()));
  this->axisVisual->Load();

/*  this->InsertMesh("axis_shaft");
  this->InsertMesh("axis_head");

  Ogre::MovableObject *shaftObj =
    (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
          this->GetName()+"__SHAFT__", "axis_shaft"));

  Ogre::MovableObject *headObj =
    (Ogre::MovableObject*)(this->scene->GetManager()->createEntity(
          this->GetName()+"__HEAD__", "axis_head"));

  this->shaftNode =
    this->sceneNode->createChildSceneNode(this->GetName() + "_shaft");
  this->shaftNode->attachObject(shaftObj);
  this->shaftNode->setPosition(0, 0, 0.1);

  this->headNode =
    this->sceneNode->createChildSceneNode(this->GetName() + "_head");
  this->headNode->attachObject(headObj);
  this->headNode->setPosition(0, 0, 0.2);


  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
  //this->AttachMesh("axis_shaft");
  // this->SetScale(math::Vector3(0.02, 0.02, 0.1));

  std::cout << "JointVis Axis[" 
    << _msg->axis1().xyz().x()
    << " " << _msg->axis1().xyz().y()
    << " " << _msg->axis1().xyz().z() << "]\n";
    */
}
