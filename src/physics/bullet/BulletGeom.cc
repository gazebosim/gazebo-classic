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
/* Desc: BulletCollision class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id:$
 */

#include <sstream>

#include "PhysicsEngine.hh"
#include "BulletPhysics.hh"
#include "rendering/Visual.hh"
#include "common/Console.hh"
#include "World.hh"


using namespace gazebo;
using namespace physics;

using namespace physics;

using namespace physics;


//////////////////////////////////////////////////
// Constructor
BulletCollision::BulletCollision(Link *_body)
    : Collision(_body)
{
  this->SetName("Bullet Collision");
  this->bulletPhysics = dynamic_cast<BulletPhysics*>(this->physicsEngine);
  this->collisionShape = NULL;
}

//////////////////////////////////////////////////
// Destructor
BulletCollision::~BulletCollision()
{
  delete this->collisionShape;
  this->collisionShape = NULL;
}

//////////////////////////////////////////////////
/// Load the collision
void BulletCollision::Load(common::XMLConfigNode *_node)
{
  Collision::Load(_node);
//  this->visualNode->SetPose(this->GetRelativePose());
}

//////////////////////////////////////////////////
// Save the body based on our common::XMLConfig node
void BulletCollision::Save(std::string &_prefix, std::ostream &_stream)
{
  Collision::Save(_prefix, _stream);
}

//////////////////////////////////////////////////
// Update
void BulletCollision::Update()
{
  Collision::Update();
}

//////////////////////////////////////////////////
// On pose change
void BulletCollision::OnPoseChange()
{
  math::Pose pose = this->GetRelativePose();
  BulletLink *bbody = (BulletLink*)(this->body);

  bbody->SetCollisionRelativePose(this, pose);
}

//////////////////////////////////////////////////
/// Set the category bits, used during collision detection
void BulletCollision::SetCategoryBits(unsigned int _bits)
{
}

//////////////////////////////////////////////////
/// Set the collide bits, used during collision detection
void BulletCollision::SetCollideBits(unsigned int _bits)
{
}

//////////////////////////////////////////////////
/// Get the mass of the collision
Mass BulletCollision::GetLinkMassMatrix()
{
  Mass result;
  return result;
}

//////////////////////////////////////////////////
/// Get the bounding box, defined by the physics engine
void BulletCollision::GetBoundingBox(math::Vector3 &_min,
                                     math::Vector3 &_max) const
{
  if (this->collisionShape)
  {
    btmath::Vector3 btMin, btMax;
    this->collisionShape->getAabb(btTransform::getIdentity(), btMin, btMax);

    _min.Set(btMin.x(), btMin.y(), btMin.z());
    _max.Set(btMax.x(), btMax.y(), btMax.z());
  }
}

//////////////////////////////////////////////////
// Set the collision shape
void BulletCollision::SetCollisionShape(btCollisionShape *_shape)
{
  this->collisionShape = _shape;

  /*btmath::Vector3 vec;
  this->collisionShape->calculateLocalInertia(this->mass.GetAsDouble(), vec);
  */

  this->mass.SetCoG(this->GetRelativePose().pos);
}

//////////////////////////////////////////////////
/// Get the bullet collision shape
btCollisionShape *BulletCollision::GetCollisionShape() const
{
  return this->collisionShape;
}

//////////////////////////////////////////////////
// Set the index of the compound shape
void BulletCollision::SetCompoundShapeIndex(int _index)
{
  this->compoundShapeIndex = 0;
}

