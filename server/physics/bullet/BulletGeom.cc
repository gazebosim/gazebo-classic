/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: BulletGeom class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id:$
 */

#include <sstream>

#include "Mass.hh"
#include "PhysicsEngine.hh"
#include "BulletPhysics.hh"
#include "Visual.hh"
#include "Global.hh"
#include "GazeboMessage.hh"
#include "World.hh"
#include "BulletBody.hh"
#include "Simulator.hh"

#include "BulletGeom.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
BulletGeom::BulletGeom( Body *_body )
    : Geom(_body)
{
  this->SetName("Bullet Geom");
  this->bulletPhysics = dynamic_cast<BulletPhysics*>(this->physicsEngine);
  this->collisionShape = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
BulletGeom::~BulletGeom()
{
  if (this->collisionShape)
    delete this->collisionShape;
  this->collisionShape = NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Load the geom
void BulletGeom::Load(XMLConfigNode *node)
{
  Geom::Load(node);
//  this->visualNode->SetPose( this->GetRelativePose() );
}

////////////////////////////////////////////////////////////////////////////////
// Save the body based on our XMLConfig node
void BulletGeom::Save(std::string &prefix, std::ostream &stream)
{
  Geom::Save(prefix, stream);
}

////////////////////////////////////////////////////////////////////////////////
// Update
void BulletGeom::Update()
{
  Geom::Update();
}

////////////////////////////////////////////////////////////////////////////////
// On pose change
void BulletGeom::OnPoseChange()
{
  Pose3d pose = this->GetRelativePose();
  BulletBody *bbody = (BulletBody*)(this->body);

  bbody->SetGeomRelativePose(this, pose);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the category bits, used during collision detection
void BulletGeom::SetCategoryBits(unsigned int bits)
{
}

////////////////////////////////////////////////////////////////////////////////
/// Set the collide bits, used during collision detection
void BulletGeom::SetCollideBits(unsigned int bits)
{
}

////////////////////////////////////////////////////////////////////////////////
/// Get the mass of the geom
Mass BulletGeom::GetBodyMassMatrix()
{
  Mass result;
  return result;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the bounding box, defined by the physics engine
void BulletGeom::GetBoundingBox( Vector3 &min, Vector3 &max ) const
{
  if (this->collisionShape)
  {
    btVector3 btMin, btMax;
    this->collisionShape->getAabb( btTransform::getIdentity(), btMin, btMax );

    min.Set(btMin.x(), btMin.y(), btMin.z());
    max.Set(btMax.x(), btMax.y(), btMax.z());
  }
}

////////////////////////////////////////////////////////////////////////////////
// Set the collision shape
void BulletGeom::SetCollisionShape( btCollisionShape *shape ) 
{
  this->collisionShape = shape;

  /*btVector3 vec;
  this->collisionShape->calculateLocalInertia(this->mass.GetAsDouble(), vec);
  */

  this->mass.SetCoG(this->GetRelativePose().pos);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the bullet collision shape
btCollisionShape *BulletGeom::GetCollisionShape() const
{
  return this->collisionShape;
}

////////////////////////////////////////////////////////////////////////////////
// Set the index of the compound shape
void BulletGeom::SetCompoundShapeIndex( int index )
{
  this->compoundShapeIndex = 0;
}
