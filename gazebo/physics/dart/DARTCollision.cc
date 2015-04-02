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

#include <sstream>

#include "gazebo/common/Console.hh"
#include "gazebo/math/Box.hh"

#include "gazebo/physics/SurfaceParams.hh"

#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTLink.hh"
#include "gazebo/physics/dart/DARTCollision.hh"
#include "gazebo/physics/dart/DARTPlaneShape.hh"
#include "gazebo/physics/dart/DARTSurfaceParams.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
DARTCollision::DARTCollision(LinkPtr _link)
  : Collision(_link),
    dtCollisionShape(NULL)
{
  this->SetName("DART_Collision");
  this->surface.reset(new DARTSurfaceParams());
  this->dtBodyNode
      = boost::static_pointer_cast<DARTLink>(this->link)->GetDARTBodyNode();
}

//////////////////////////////////////////////////
DARTCollision::~DARTCollision()
{
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

  // Offset
  if (this->dtCollisionShape)
  {
    // TODO: Don't change offset of shape until dart supports plane shape.
    boost::shared_ptr<DARTPlaneShape> planeShape =
        boost::dynamic_pointer_cast<DARTPlaneShape>(this->shape);

    if (!planeShape)
    {
      math::Pose relativePose = this->GetRelativePose();
      this->dtCollisionShape->setOffset(DARTTypes::ConvVec3(relativePose.pos));
    }
    else
    {
      // change ground plane to be near semi-infinite.
      dart::dynamics::BoxShape *dtBoxShape =
        dynamic_cast<dart::dynamics::BoxShape *>(this->dtCollisionShape);
      dtBoxShape->setSize(Eigen::Vector3d(2100, 2100, 2100.0));
      dtBoxShape->setOffset(Eigen::Vector3d(0.0, 0.0, -2100.0/2.0));
      // gzerr << "plane box modified\n";
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
  this->categoryBits = _bits;
}

//////////////////////////////////////////////////
void DARTCollision::SetCollideBits(unsigned int _bits)
{
  this->collideBits = _bits;
}

//////////////////////////////////////////////////
unsigned int DARTCollision::GetCategoryBits() const
{
  return this->categoryBits;
}

//////////////////////////////////////////////////
unsigned int DARTCollision::GetCollideBits() const
{
  return this->collideBits;
}

//////////////////////////////////////////////////
gazebo::math::Box DARTCollision::GetBoundingBox() const
{
  math::Box result;

  gzerr << "DART does not provide bounding box info.\n";

  return result;
}

//////////////////////////////////////////////////
dart::dynamics::BodyNode *DARTCollision::GetDARTBodyNode() const
{
  return dtBodyNode;
}

//////////////////////////////////////////////////
void DARTCollision::SetDARTCollisionShape(dart::dynamics::Shape *_shape,
                                          bool _placeable)
{
  Collision::SetCollision(_placeable);
  this->dtCollisionShape = _shape;
}

//////////////////////////////////////////////////
dart::dynamics::Shape *DARTCollision::GetDARTCollisionShape() const
{
  return dtCollisionShape;
}

/////////////////////////////////////////////////
DARTSurfaceParamsPtr DARTCollision::GetDARTSurface() const
{
  return boost::dynamic_pointer_cast<DARTSurfaceParams>(this->surface);
}
