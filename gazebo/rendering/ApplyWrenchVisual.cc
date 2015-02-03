/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

//#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/DynamicLines.hh"
//#include "gazebo/rendering/Scene.hh"
//#include "gazebo/rendering/AxisVisual.hh"
#include "gazebo/rendering/ArrowVisual.hh"
#include "gazebo/rendering/ApplyWrenchVisualPrivate.hh"
#include "gazebo/rendering/ApplyWrenchVisual.hh"

using namespace gazebo;
using namespace rendering;

/////////////////////////////////////////////////
ApplyWrenchVisual::ApplyWrenchVisual(const std::string &_name,
    VisualPtr _parentVis)
    : Visual(*new ApplyWrenchVisualPrivate, _name, _parentVis, false)
{
}

/////////////////////////////////////////////////
ApplyWrenchVisual::~ApplyWrenchVisual()
{
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::Load()
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  math::Vector3 linkSize = dPtr->parent->GetBoundingBox().GetSize();

  // Point visual
  // Adapted from COMVisual, it may be good to generalize it
  math::Vector3 p1(0, 0, -2*linkSize.z);
  math::Vector3 p2(0, 0,  2*linkSize.z);
  math::Vector3 p3(0, -2*linkSize.y, 0);
  math::Vector3 p4(0,  2*linkSize.y, 0);
  math::Vector3 p5(-2*linkSize.x, 0, 0);
  math::Vector3 p6(2*linkSize.x,  0, 0);
//  p1 += _pose.pos;
//  p2 += _pose.pos;
//  p3 += _pose.pos;
//  p4 += _pose.pos;
//  p5 += _pose.pos;
//  p6 += _pose.pos;
//  p1 = _pose.rot.RotateVector(p1);
//  p2 = _pose.rot.RotateVector(p2);
//  p3 = _pose.rot.RotateVector(p3);
//  p4 = _pose.rot.RotateVector(p4);
//  p5 = _pose.rot.RotateVector(p5);
//  p6 = _pose.rot.RotateVector(p6);

  dPtr->crossLines = dPtr->parent->
      CreateDynamicLine(rendering::RENDERING_LINE_LIST);
  dPtr->crossLines->setMaterial("Gazebo/SkyBlue");
  dPtr->crossLines->AddPoint(p1);
  dPtr->crossLines->AddPoint(p2);
  dPtr->crossLines->AddPoint(p3);
  dPtr->crossLines->AddPoint(p4);
  dPtr->crossLines->AddPoint(p5);
  dPtr->crossLines->AddPoint(p6);

  // Force visual
  dPtr->forceArrow.reset(new rendering::ArrowVisual(
      dPtr->parent->GetName() + "__FORCE_VISUAL__", dPtr->parent));
  dPtr->forceArrow->Load();
  dPtr->forceArrow->SetMaterial("Gazebo/RedBright");
  dPtr->forceArrow->SetScale(math::Vector3(2, 2, 2));
  dPtr->forceArrow->GetSceneNode()->setInheritScale(false);

  this->SetVisibilityFlags(GZ_VISIBILITY_GUI);
}

///////////////////////////////////////////////////
void ApplyWrenchVisual::UpdateForce(math::Vector3 _forceVector)
{
  ApplyWrenchVisualPrivate *dPtr =
      reinterpret_cast<ApplyWrenchVisualPrivate *>(this->dataPtr);

  if (dPtr->forceArrow)
  {
    // Set rotation
    math::Vector3 u = _forceVector;
    u = u.Normalize();
    math::Vector3 v = math::Vector3::UnitZ;
    double cosTheta = v.Dot(u);
    double angle = acos(cosTheta);
    math::Quaternion quat;
    if (math::equal(angle, M_PI))
      quat.SetFromAxis(u.GetPerpendicular(), angle);
    else
      quat.SetFromAxis((v.Cross(u)).Normalize(), angle);
    dPtr->forceArrow->SetRotation(quat);

    // Set position
    double linkSize = dPtr->parent->GetBoundingBox().GetDiagonalLength();
    //double arrowSize = this->forceArrow->GetBoundingBox().GetZLength();
    dPtr->forceArrow->SetPosition(-u * (linkSize*0.5 + 0.5));
  }
}
