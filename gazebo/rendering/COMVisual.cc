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
/* Desc: Center of Mass Visualization Class
 * Author: Nate Koenig
 */

#include "gazebo/common/MeshManager.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Quaternion.hh"
#include "gazebo/math/Pose.hh"

#include "gazebo/rendering/DynamicLines.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/Scene.hh"
#include "gazebo/rendering/COMVisualPrivate.hh"
#include "gazebo/rendering/COMVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
COMVisual::COMVisual(const std::string &_name, VisualPtr _vis)
  : Visual(*new COMVisualPrivate, _name, _vis, false)
{
}

/////////////////////////////////////////////////
COMVisual::~COMVisual()
{
}

/////////////////////////////////////////////////
void COMVisual::Load(sdf::ElementPtr _elem)
{
  Visual::Load();
  math::Pose pose = _elem->Get<math::Pose>("origin");
  this->Load(pose);
}

/////////////////////////////////////////////////
void COMVisual::Load(ConstLinkPtr &_msg)
{
  Visual::Load();

  math::Vector3 xyz(_msg->inertial().pose().position().x(),
                    _msg->inertial().pose().position().y(),
                    _msg->inertial().pose().position().z());
  math::Quaternion q(_msg->inertial().pose().orientation().w(),
                     _msg->inertial().pose().orientation().x(),
                     _msg->inertial().pose().orientation().y(),
                     _msg->inertial().pose().orientation().z());

  // Use principal moments of inertia to scale COM visual
  // \todo: rotate COM to match principal axes when product terms are nonzero
  // This can be done with Eigen, or with code from the following paper:
  // A Method for Fast Diagonalization of a 2x2 or 3x3 Real Symmetric Matrix
  // http://arxiv.org/abs/1306.6291v3
  double mass = _msg->inertial().mass();
  double Ixx = _msg->inertial().ixx();
  double Iyy = _msg->inertial().iyy();
  double Izz = _msg->inertial().izz();
  math::Vector3 boxScale;
  if (mass < 0 || Ixx < 0 || Iyy < 0 || Izz < 0 ||
      Ixx + Iyy < Izz || Iyy + Izz < Ixx || Izz + Ixx < Iyy)
  {
    // Unrealistic inertia, load with default scale
    gzlog << "The link " << _msg->name() << " has unrealistic inertia, "
          << "unable to visualize box of equivalent inertia." << std::endl;
    this->Load(math::Pose(xyz, q));
  }
  else
  {
    // Compute dimensions of box with uniform density and equivalent inertia.
    boxScale.x = sqrt(6*(Izz + Iyy - Ixx) / mass);
    boxScale.y = sqrt(6*(Izz + Ixx - Iyy) / mass);
    boxScale.z = sqrt(6*(Ixx + Iyy - Izz) / mass);
    this->Load(math::Pose(xyz, q), boxScale);
  }
}

/////////////////////////////////////////////////
void COMVisual::Load(const math::Pose &_pose,
                     const math::Vector3 &_scale)
{
  COMVisualPrivate *dPtr =
      reinterpret_cast<COMVisualPrivate *>(this->dataPtr);

  math::Vector3 p1(0, 0, -2*_scale.z);
  math::Vector3 p2(0, 0,  2*_scale.z);
  math::Vector3 p3(0, -2*_scale.y, 0);
  math::Vector3 p4(0,  2*_scale.y, 0);
  math::Vector3 p5(-2*_scale.x, 0, 0);
  math::Vector3 p6(2*_scale.x,  0, 0);
  p1 += _pose.pos;
  p2 += _pose.pos;
  p3 += _pose.pos;
  p4 += _pose.pos;
  p5 += _pose.pos;
  p6 += _pose.pos;
  p1 = _pose.rot.RotateVector(p1);
  p2 = _pose.rot.RotateVector(p2);
  p3 = _pose.rot.RotateVector(p3);
  p4 = _pose.rot.RotateVector(p4);
  p5 = _pose.rot.RotateVector(p5);
  p6 = _pose.rot.RotateVector(p6);

  dPtr->crossLines = this->CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  dPtr->crossLines->setMaterial("Gazebo/Green");
  dPtr->crossLines->AddPoint(p1);
  dPtr->crossLines->AddPoint(p2);
  dPtr->crossLines->AddPoint(p3);
  dPtr->crossLines->AddPoint(p4);
  dPtr->crossLines->AddPoint(p5);
  dPtr->crossLines->AddPoint(p6);

  this->InsertMesh("unit_box");

  Ogre::MovableObject *boxObj =
    (Ogre::MovableObject*)(dPtr->scene->GetManager()->createEntity(
          this->GetName()+"__BOX__", "unit_box"));
  boxObj->setVisibilityFlags(GZ_VISIBILITY_GUI);
  ((Ogre::Entity*)boxObj)->setMaterialName("__GAZEBO_TRANS_PURPLE_MATERIAL__");

  dPtr->boxNode =
      dPtr->sceneNode->createChildSceneNode(this->GetName() + "_BOX");

  dPtr->boxNode->attachObject(boxObj);
  dPtr->boxNode->setScale(_scale.x, _scale.y, _scale.z);
  dPtr->boxNode->setPosition(_pose.pos.x, _pose.pos.y, _pose.pos.z);
  dPtr->boxNode->setOrientation(Ogre::Quaternion(_pose.rot.w, _pose.rot.x,
                                                 _pose.rot.y, _pose.rot.z));

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}
