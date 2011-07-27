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
/* Desc: Link class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id: Link.cc 7640 2009-05-13 02:06:08Z natepak $
 */

#include <sstream>

#include "common/XMLConfig.hh"
#include "common/Console.hh"

#include "physics/Geom.hh"
#include "BulletGeom.hh"
#include "BulletMotionState.hh"
#include "math/Quaternion.hh"
#include "common/Exception.hh"
#include "BulletPhysics.hh"
#include "PhysicsEngine.hh"
#include "Mass.hh"
#include "rendering/Visual.hh"

#include "BulletLink.hh"

using namespace gazebo;
using namespace physics;

using namespace physics;

using namespace physics;


////////////////////////////////////////////////////////////////////////////////
// Constructor
BulletLink::BulletLink(Entity *parent)
    : Link(parent)
{
  this->bulletPhysics = dynamic_cast<BulletPhysics*>(this->physicsEngine);

  this->rigidLink = NULL;
  this->compoundShape = new btCompoundShape;
  this->motionState = new BulletMotionState(this);

  if (this->bulletPhysics == NULL)
    gzthrow("Not using the bullet physics engine");
}


////////////////////////////////////////////////////////////////////////////////
// Destructor
BulletLink::~BulletLink()
{
  if (this->rigidLink)
    delete this->rigidLink;
}

////////////////////////////////////////////////////////////////////////////////
// Load the body based on an common::XMLConfig node
void BulletLink::Load(common::XMLConfigNode *node)
{
  Link::Load(node);
  btScalar btMass = 0.0;
  btmath::Vector3 fallInertia(0,0,0);

  // Set the initial pose of the body
  this->motionState->SetVisual( this->visualNode );
  this->motionState->SetWorldPose(this->GetWorldPose());

  btTransform principal;
  btmath::Vector3 principalInertia;

  principal.setIdentity();

  if (!this->IsStatic())
  {
    this->UpdateCoM();
    btMass = this->mass.GetAsDouble();

    /*int i;

    btScalar *masses = new btScalar[this->geoms.size()];
    std::map< std::string, Geom* >::iterator iter;

    // Get a list of all the geom masses
    for (iter = this->geoms.begin(), i=0; iter != geoms.end(); iter++, i++)
      masses[i] = iter->second->GetMass().GetAsDouble();

    // Calculate the center of mass of the compound shape
    this->compoundShape->calculatePrincipalAxisTransform(masses, principal, 
                                                         principalInertia);

    // Convert to gazebo poses
    math::Pose princ = BulletPhysics::ConvertPose(principal);
    math::Pose inverse = BulletPhysics::ConvertPose(principal.inverse());

    // Store the Center of Mass offset in the motion state
    this->motionState->SetCoMOffset(princ);

    // Move the body visual to match the center of mass offset
    math::Pose tmp = this->GetRelativePose();
    tmp.pos += princ.pos;
    this->SetRelativePose(tmp, false);

    // Move all the geoms relative to the center of mass
    for (iter = this->geoms.begin(),i = 0; i < this->geoms.size(); i++, iter++)
    {
      math::Pose origPose, newPose;

      // The original pose of the geometry
      origPose = BulletPhysics::ConvertPose(
                                  this->compoundShape->getChildTransform(i));
     
      // Rotate the geometry around it's own axis. This will allow us to 
      // translate the geom so that the CoG is at the body's (0,0,0)
      newPose.rot = origPose.CoordRotationAdd(princ.rot);

      // Translate the geometry according the center of mass
      newPose.pos = origPose.pos;
      newPose.pos = newPose.CoordPositionAdd(inverse.pos);

      // Restore the original rotation of the geom.
      newPose.rot = origPose.rot;

      // Set the pose of the geom in Bullet
      this->compoundShape->updateChildTransform(i, 
          BulletPhysics::ConvertPose(newPose));

      // Tell the Gazebo Geom that it's pose has changed. This will
      // change the visual accordingly
      iter->second->SetRelativePose(newPose);//, false);

      gzmsg << "Orig Pose[" << origPose << "] New Pose[" << newPose << "]\n";
    }
    */

    // Compute the interia vector
    this->compoundShape->calculateLocalInertia(btMass,fallInertia);
  }
    
  // Create a construction info object
  btRigidLink::btRigidLinkConstructionInfo
    rigidLinkCI(btMass, this->motionState, this->compoundShape, fallInertia);
  
  // Create the new rigid body 
  this->rigidLink = new btRigidLink( rigidLinkCI );
  this->rigidLink->setUserPointer(this);

  // before loading child geometry, we have to figure out of selfCollide is true
  // and modify parent class Entity so this body has its own spaceId
  if (**this->selfCollideP)
  {
    // TODO: Implement this if required for bullet
  }

  this->physicsEngine->AddEntity(this);
}

////////////////////////////////////////////////////////////////////////////////
// Init the Bullet body
void BulletLink::Init() 
{
  Link::Init();
}

////////////////////////////////////////////////////////////////////////////////
/// Finalize the body
void BulletLink::Fini()
{
  Link::Fini();
}

////////////////////////////////////////////////////////////////////////////////
// Update the body
void BulletLink::Update()
{
  Link::Update();
}

////////////////////////////////////////////////////////////////////////////////
// Set whether gravity affects this body
void BulletLink::SetGravityMode(bool mode)
{
  if (!this->rigidLink)
    return;

  if (mode == false)
    this->rigidLink->setMassProps(btScalar(0), btmath::Vector3(0,0,0));
  else
  {
    btScalar btMass = this->mass.GetAsDouble();
    btmath::Vector3 fallInertia(0,0,0);

    this->compoundShape->calculateLocalInertia(btMass,fallInertia);
    this->rigidLink->setMassProps(btMass, fallInertia);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get the gravity mode
bool BulletLink::GetGravityMode()
{
  bool result;
  gzerr << "BulletLink::GetGravityMode not implemented, returning spurious result\n";

  return result;
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether this body will collide with others in the model
void BulletLink::SetSelfCollide(bool collide)
{
}

////////////////////////////////////////////////////////////////////////////////
// Attach a geom to this body
void BulletLink::AttachGeom( Geom *geom )
{
  Link::AttachGeom(geom);

  BulletGeom *bgeom = dynamic_cast<BulletGeom*>(geom);

  if (geom == NULL)
    gzthrow("requires BulletGeom");

  btTransform trans;
  math::Pose relativePose = geom->GetRelativePose();
  trans = BulletPhysics::ConvertPose(relativePose);

  bgeom->SetCompoundShapeIndex( this->compoundShape->getNumChildShapes() );
  this->compoundShape->addChildShape(trans, bgeom->GetCollisionShape());
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Called when the pose of the entity (or one of its parents) has
/// changed
void BulletLink::OnPoseChange()
{
  math::Pose pose = this->GetWorldPose();

  this->motionState->SetWorldPose(pose);
  if (this->rigidLink)
    this->rigidLink->setMotionState(this->motionState);
}

////////////////////////////////////////////////////////////////////////////////
// Set whether this body is enabled
void BulletLink::SetEnabled(bool enable) const
{
  if (!this->rigidLink)
    return;

  if (enable)
    this->rigidLink->activate(true);
  else
    this->rigidLink->setActivationState(WANTS_DEACTIVATION);
}

/////////////////////////////////////////////////////////////////////
// Update the CoM and mass matrix
/*
  What's going on here?  In ODE the CoM of a body corresponds to the
  origin of the body-fixed coordinate system.  In Gazebo, however, we
  want to have arbitrary body coordinate systems (i.e., CoM may be
  displaced from the body-fixed cs).  To get around this limitation in
  Bullet, we have an extra fudge-factor (comPose), describing the pose of
  the CoM relative to Gazebo's body-fixed cs.  When using low-level
  Bullet functions, one must use apply this factor appropriately.

  The UpdateCoM() function is used to compute this offset, based on
  the mass distribution of attached geoms.  This function also shifts
  the Bullet-pose of the geoms, to keep everything in the same place in the
  Gazebo cs.  Simple, neh?
*/
void BulletLink::UpdateCoM()
{
  Link::UpdateCoM();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the velocity of the body
void BulletLink::SetLinearVel(const math::Vector3 &vel)
{
  if (!this->rigidLink)
    return;

  this->rigidLink->setLinearVelocity( btmath::Vector3(vel.x, vel.y, vel.z) );
}

////////////////////////////////////////////////////////////////////////////////
/// Get the velocity of the body
math::Vector3 BulletLink::GetWorldLinearVel() const
{
  if (!this->rigidLink)
    return math::Vector3(0,0,0);

  btmath::Vector3 btVec = this->rigidLink->getLinearVelocity();

  return math::Vector3(btVec.x(), btVec.y(), btVec.z());
}

////////////////////////////////////////////////////////////////////////////////
/// Set the velocity of the body
void BulletLink::SetAngularVel(const math::Vector3 &vel)
{
  if (!this->rigidLink)
    return;

  this->rigidLink->setAngularVelocity( btmath::Vector3(vel.x, vel.y, vel.z) );
}

////////////////////////////////////////////////////////////////////////////////
/// Get the velocity of the body
math::Vector3 BulletLink::GetWorldAngularVel() const
{
  if (!this->rigidLink)
    return math::Vector3(0,0,0);

  btmath::Vector3 btVec = this->rigidLink->getAngularVelocity();

  return math::Vector3(btVec.x(), btVec.y(), btVec.z());
}

////////////////////////////////////////////////////////////////////////////////
/// Set the force applied to the body
void BulletLink::SetForce(const math::Vector3 &force)
{
  if (!this->rigidLink)
    return;

  this->rigidLink->applyCentralForce(btmath::Vector3(force.x, force.y, force.z) );
}

////////////////////////////////////////////////////////////////////////////////
/// Get the force applied to the body
math::Vector3 BulletLink::GetWorldForce() const
{
  if (!this->rigidLink)
    return math::Vector3(0,0,0);

  btmath::Vector3 btVec;

  btVec = this->rigidLink->getTotalForce();

  return math::Vector3(btVec.x(), btVec.y(), btVec.z());
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Set the torque applied to the body
void BulletLink::SetTorque(const math::Vector3 &torque)
{
  if (!this->rigidLink)
    return;

  this->rigidLink->applyTorque(btmath::Vector3(torque.x, torque.y, torque.z));

}

////////////////////////////////////////////////////////////////////////////////
/// \brief Get the torque applied to the body
math::Vector3 BulletLink::GetWorldTorque() const
{
  if (!this->rigidLink)
    return math::Vector3(0,0,0);

  btmath::Vector3 btVec;

  btVec = this->rigidLink->getTotalTorque();

  return math::Vector3(btVec.x(), btVec.y(), btVec.z());
}

////////////////////////////////////////////////////////////////////////////////
// Get the bullet rigid body
btRigidLink *BulletLink::GetBulletLink() const
{
  return this->rigidLink;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the linear damping factor
void BulletLink::SetLinearDamping(double damping)
{
  this->rigidLink->setDamping((btScalar)damping, 
      (btScalar)this->rigidLink->getAngularDamping());
}

////////////////////////////////////////////////////////////////////////////////
/// Set the angular damping factor
void BulletLink::SetAngularDamping(double damping)
{
  this->rigidLink->setDamping(
      (btScalar)this->rigidLink->getLinearDamping(), (btScalar)damping);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the relative pose of a child geom.
void BulletLink::SetGeomRelativePose(BulletGeom *geom, const math::Pose &newPose)
{
  std::map<std::string, Geom*>::iterator iter;
  unsigned int i;

  for (iter=this->geoms.begin(), i=0; iter != this->geoms.end(); iter++, i++)
  {
    if (iter->second == geom)
      break;
  }

  if (i < this->geoms.size())
  {
    // Set the pose of the geom in Bullet
    this->compoundShape->updateChildTransform(i, 
        BulletPhysics::ConvertPose(newPose));
  }
}
