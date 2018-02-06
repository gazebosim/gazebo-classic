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

#include <sstream>

#include "gazebo/common/Console.hh"

#include "gazebo/physics/SurfaceParams.hh"

#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTLink.hh"
#include "gazebo/physics/dart/DARTCollision.hh"
#include "gazebo/physics/dart/DARTPlaneShape.hh"
#include "gazebo/physics/dart/DARTSurfaceParams.hh"

#include "gazebo/physics/dart/DARTCollisionPrivate.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTCollision::DARTCollision(LinkPtr _link)
  : Collision(_link),
    dataPtr(new DARTCollisionPrivate())
{
  this->SetName("DART_Collision");
  this->surface.reset(new DARTSurfaceParams());
}

//////////////////////////////////////////////////
DARTCollision::~DARTCollision()
{
  delete this->dataPtr;
  this->dataPtr = nullptr;
}

//////////////////////////////////////////////////
void DARTCollision::Load(sdf::ElementPtr _sdf)
{
  Collision::Load(_sdf);

  if (this->IsStatic())
  {
    this->SetCategoryBits(GZ_FIXED_COLLIDE);
    this->SetCollideBits(~GZ_FIXED_COLLIDE);
  }
}

//////////////////////////////////////////////////
void DARTCollision::Init()
{
  Collision::Init();

  // Create the collision shapes. Since DART6, this has to be done in
  // Init(), because the BodyNode will only have been created in Load()
  // and is not guaranteed to exist before.

  // Set the pose offset.
  if (this->dataPtr->dtCollisionShape)
  {
    // TODO: Remove type check once DART completely supports plane shape.
    // Please see: https://github.com/dartsim/dart/issues/114
    const bool isPlaneShape =
        (boost::dynamic_pointer_cast<DARTPlaneShape>(this->shape) != nullptr);

    if (!isPlaneShape)
    {
      Eigen::Isometry3d tf = DARTTypes::ConvPose(this->RelativePose());
      this->dataPtr->dtCollisionShape->setRelativeTransform(tf);
    }
  }
}

//////////////////////////////////////////////////
void DARTCollision::Fini()
{
  Collision::Fini();
}

//////////////////////////////////////////////////
void DARTCollision::OnPoseChange()
{
  // Nothing to do in dart.
}

//////////////////////////////////////////////////
void DARTCollision::SetCategoryBits(unsigned int _bits)
{
  this->dataPtr->categoryBits = _bits;
}

//////////////////////////////////////////////////
void DARTCollision::SetCollideBits(unsigned int _bits)
{
  this->dataPtr->collideBits = _bits;
}

//////////////////////////////////////////////////
unsigned int DARTCollision::GetCategoryBits() const
{
  return this->dataPtr->categoryBits;
}

//////////////////////////////////////////////////
unsigned int DARTCollision::GetCollideBits() const
{
  return this->dataPtr->collideBits;
}


//////////////////////////////////////////////////
// Helper function to prevent following current limitation of
// ignition::math::Vector3:
// operator [] does not return a reference, so can't use the operator for
// writing the i'th value, and instead have to use X()/Y()/Z() functions.
// This function should should achieve the same as this expression should:
// ``v[i] = val;``
// Given that i = [0..2].
// See also https://bitbucket.org/ignitionrobotics/ign-math/issues/73
template<typename Float>
void SetVector(const size_t _i, const Float _val,
               ignition::math::Vector3d &_v)
{
    if (_i == 0)
    {
      _v.X() =  _val;
    }
    else if (_i == 1)
    {
      _v.Y() = _val;
    }
    else
    {
      _v.Z() =  _val;
    }
}

//////////////////////////////////////////////////
// see also: Book "real-time collision detection by Christer Ericson, v. 1,
// p. 86, function UpdateAABB
// FIXME: I reckon this should go to ignition::math, as updating an AABB
// with a transform is generally useful.
ignition::math::Box UpdateAABB(const ignition::math::Box &_a,
                               const ignition::math::Matrix4d &_m)
{
  // construct object to be returned.
  // Attention: If we only use the default ignition::math::Box constructor
  // and then set min and max individually with Min()/Max(), then
  // the extent of the box won't be initialized to BoxPrivate::EXTENT_FINITE,
  // which in turn will lead to failures if this box is used to add onto
  // another box with ignition::math::Box::operator+=.
  // To construct a valid return object, use the constructor which takes two
  // vectors as arguments, the values don't actually matter.
  // See also issue #72:
  // https://bitbucket.org/ignitionrobotics/ign-math/issues/72/
  ignition::math::Box result(_a);

  // for all 3 axes
  for (size_t i = 0; i < 3; ++i)
  {
    // First, add translation.
    // Use SetVector() until maybe an update to ignition allows the
    // following:
    // result.Min()[i] =  _m(i, 3);
    // result.Max()[i] =  _m(i, 3);
    SetVector(i, _m(i, 3), result.Min());
    SetVector(i, _m(i, 3), result.Max());

    // Then, sum in larger and smaller terms
    for (size_t j = 0; j < 3; ++j)
    {
      // minimum and maximum coordinate of this axis rotated
      float minRot = _m(i, j) * _a.Min()[j];
      float maxRot = _m(i, j) * _a.Max()[j];
      if (minRot < maxRot)
      {
        // result.Min()[i] += minRot;
        // result.Max()[i] += maxRot;
        SetVector(i, result.Min()[i] + minRot, result.Min());
        SetVector(i, result.Max()[i] + maxRot, result.Max());
      }
      else
      {
        // result.Min()[i] += maxRot;
        // result.Max()[i] += minRot;
        SetVector(i, result.Min()[i] + maxRot, result.Min());
        SetVector(i, result.Max()[i] + minRot, result.Max());
      }
    }
  }
  return result;
}


//////////////////////////////////////////////////
ignition::math::Box DARTCollision::BoundingBox() const
{
  GZ_ASSERT(this->dataPtr->dtCollisionShape,
            "DARTCollision Must have a collision shape");
  GZ_ASSERT(this->dataPtr->dtCollisionShape->getShape(),
            "DARTCollision has no shape");

  const dart::math::BoundingBox& bb =
    this->dataPtr->dtCollisionShape->getShape()->getBoundingBox();

  ignition::math::Box result(bb.getMin().x(),
                             bb.getMin().y(),
                             bb.getMin().z(),
                             bb.getMax().x(),
                             bb.getMax().y(),
                             bb.getMax().z());

  result = UpdateAABB(result, ignition::math::Matrix4d(this->RelativePose()));

  return result;
}

//////////////////////////////////////////////////
dart::dynamics::BodyNode *DARTCollision::DARTBodyNode() const
{
  return boost::static_pointer_cast<DARTLink>(this->link)->DARTBodyNode();
}

//////////////////////////////////////////////////
void DARTCollision::SetDARTCollisionShapeNode(
                                          dart::dynamics::ShapeNodePtr _shape,
                                          bool _placeable)
{
  Collision::SetCollision(_placeable);
  this->dataPtr->dtCollisionShape = _shape;
}

//////////////////////////////////////////////////
dart::dynamics::ShapeNodePtr DARTCollision::DARTCollisionShapeNode() const
{
  return this->dataPtr->dtCollisionShape;
}

/////////////////////////////////////////////////
DARTSurfaceParamsPtr DARTCollision::DARTSurface() const
{
  return boost::dynamic_pointer_cast<DARTSurfaceParams>(this->surface);
}
