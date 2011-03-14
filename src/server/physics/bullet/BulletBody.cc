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
/* Desc: Body class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id: Body.cc 7640 2009-05-13 02:06:08Z natepak $
 */

#include <sstream>

#include "XMLConfig.hh"
#include "GazeboMessage.hh"

#include "Geom.hh"
#include "BulletGeom.hh"
#include "BulletMotionState.hh"
#include "Quatern.hh"
#include "GazeboError.hh"
#include "BulletPhysics.hh"
#include "PhysicsEngine.hh"
#include "Mass.hh"
#include "Visual.hh"

#include "BulletBody.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
BulletBody::BulletBody(Entity *parent)
    : Body(parent)
{
  this->bulletPhysics = dynamic_cast<BulletPhysics*>(this->physicsEngine);

  this->rigidBody = NULL;
  this->compoundShape = new btCompoundShape;
  this->motionState = new BulletMotionState(this);

  if (this->bulletPhysics == NULL)
    gzthrow("Not using the bullet physics engine");
}


////////////////////////////////////////////////////////////////////////////////
// Destructor
BulletBody::~BulletBody()
{
  if (this->rigidBody)
    delete this->rigidBody;
}

////////////////////////////////////////////////////////////////////////////////
// Load the body based on an XMLConfig node
void BulletBody::Load(XMLConfigNode *node)
{
  Body::Load(node);
  btScalar btMass = 0.0;
  btVector3 fallInertia(0,0,0);

  // Set the initial pose of the body
  this->motionState->SetVisual( this->visualNode );
  this->motionState->SetWorldPose(this->GetWorldPose());

  btTransform principal;
  btVector3 principalInertia;

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
    Pose3d princ = BulletPhysics::ConvertPose(principal);
    Pose3d inverse = BulletPhysics::ConvertPose(principal.inverse());

    // Store the Center of Mass offset in the motion state
    this->motionState->SetCoMOffset(princ);

    // Move the body visual to match the center of mass offset
    Pose3d tmp = this->GetRelativePose();
    tmp.pos += princ.pos;
    this->SetRelativePose(tmp, false);

    // Move all the geoms relative to the center of mass
    for (iter = this->geoms.begin(),i = 0; i < this->geoms.size(); i++, iter++)
    {
      Pose3d origPose, newPose;

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

      std::cout << "Orig Pose[" << origPose << "] New Pose[" << newPose << "]\n";
    }
    */

    // Compute the interia vector
    this->compoundShape->calculateLocalInertia(btMass,fallInertia);
  }
    
  // Create a construction info object
  btRigidBody::btRigidBodyConstructionInfo
    rigidBodyCI(btMass, this->motionState, this->compoundShape, fallInertia);
  
  // Create the new rigid body 
  this->rigidBody = new btRigidBody( rigidBodyCI );
  this->rigidBody->setUserPointer(this);

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
void BulletBody::Init() 
{
  Body::Init();
}

////////////////////////////////////////////////////////////////////////////////
/// Finalize the body
void BulletBody::Fini()
{
  Body::Fini();
}

////////////////////////////////////////////////////////////////////////////////
// Update the body
void BulletBody::Update()
{
  Body::Update();
}

////////////////////////////////////////////////////////////////////////////////
// Set whether gravity affects this body
void BulletBody::SetGravityMode(bool mode)
{
  if (!this->rigidBody)
    return;

  if (mode == false)
    this->rigidBody->setMassProps(btScalar(0), btVector3(0,0,0));
  else
  {
    btScalar btMass = this->mass.GetAsDouble();
    btVector3 fallInertia(0,0,0);

    this->compoundShape->calculateLocalInertia(btMass,fallInertia);
    this->rigidBody->setMassProps(btMass, fallInertia);
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get the gravity mode
bool BulletBody::GetGravityMode()
{
  bool result;
  gzerr(0) << "BulletBody::GetGravityMode not implemented, returning spurious result\n";

  return result;
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether this body will collide with others in the model
void BulletBody::SetSelfCollide(bool collide)
{
}

////////////////////////////////////////////////////////////////////////////////
// Attach a geom to this body
void BulletBody::AttachGeom( Geom *geom )
{
  Body::AttachGeom(geom);

  BulletGeom *bgeom = dynamic_cast<BulletGeom*>(geom);

  if (geom == NULL)
    gzthrow("requires BulletGeom");

  btTransform trans;
  Pose3d relativePose = geom->GetRelativePose();
  trans = BulletPhysics::ConvertPose(relativePose);

  bgeom->SetCompoundShapeIndex( this->compoundShape->getNumChildShapes() );
  this->compoundShape->addChildShape(trans, bgeom->GetCollisionShape());
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Called when the pose of the entity (or one of its parents) has
/// changed
void BulletBody::OnPoseChange()
{
  Pose3d pose = this->GetWorldPose();

  this->motionState->SetWorldPose(pose);
  if (this->rigidBody)
    this->rigidBody->setMotionState(this->motionState);
}

////////////////////////////////////////////////////////////////////////////////
// Set whether this body is enabled
void BulletBody::SetEnabled(bool enable) const
{
  if (!this->rigidBody)
    return;

  if (enable)
    this->rigidBody->activate(true);
  else
    this->rigidBody->setActivationState(WANTS_DEACTIVATION);
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
void BulletBody::UpdateCoM()
{
  Body::UpdateCoM();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the velocity of the body
void BulletBody::SetLinearVel(const Vector3 &vel)
{
  if (!this->rigidBody)
    return;

  this->rigidBody->setLinearVelocity( btVector3(vel.x, vel.y, vel.z) );
}

////////////////////////////////////////////////////////////////////////////////
/// Get the velocity of the body
Vector3 BulletBody::GetWorldLinearVel() const
{
  if (!this->rigidBody)
    return Vector3(0,0,0);

  btVector3 btVec = this->rigidBody->getLinearVelocity();

  return Vector3(btVec.x(), btVec.y(), btVec.z());
}

////////////////////////////////////////////////////////////////////////////////
/// Set the velocity of the body
void BulletBody::SetAngularVel(const Vector3 &vel)
{
  if (!this->rigidBody)
    return;

  this->rigidBody->setAngularVelocity( btVector3(vel.x, vel.y, vel.z) );
}

////////////////////////////////////////////////////////////////////////////////
/// Get the velocity of the body
Vector3 BulletBody::GetWorldAngularVel() const
{
  if (!this->rigidBody)
    return Vector3(0,0,0);

  btVector3 btVec = this->rigidBody->getAngularVelocity();

  return Vector3(btVec.x(), btVec.y(), btVec.z());
}

////////////////////////////////////////////////////////////////////////////////
/// Set the force applied to the body
void BulletBody::SetForce(const Vector3 &force)
{
  if (!this->rigidBody)
    return;

  this->rigidBody->applyCentralForce(btVector3(force.x, force.y, force.z) );
}

////////////////////////////////////////////////////////////////////////////////
/// Get the force applied to the body
Vector3 BulletBody::GetWorldForce() const
{
  if (!this->rigidBody)
    return Vector3(0,0,0);

  btVector3 btVec;

  btVec = this->rigidBody->getTotalForce();

  return Vector3(btVec.x(), btVec.y(), btVec.z());
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Set the torque applied to the body
void BulletBody::SetTorque(const Vector3 &torque)
{
  if (!this->rigidBody)
    return;

  this->rigidBody->applyTorque(btVector3(torque.x, torque.y, torque.z));

}

////////////////////////////////////////////////////////////////////////////////
/// \brief Get the torque applied to the body
Vector3 BulletBody::GetWorldTorque() const
{
  if (!this->rigidBody)
    return Vector3(0,0,0);

  btVector3 btVec;

  btVec = this->rigidBody->getTotalTorque();

  return Vector3(btVec.x(), btVec.y(), btVec.z());
}

////////////////////////////////////////////////////////////////////////////////
// Get the bullet rigid body
btRigidBody *BulletBody::GetBulletBody() const
{
  return this->rigidBody;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the linear damping factor
void BulletBody::SetLinearDamping(double damping)
{
  this->rigidBody->setDamping((btScalar)damping, 
      (btScalar)this->rigidBody->getAngularDamping());
}

////////////////////////////////////////////////////////////////////////////////
/// Set the angular damping factor
void BulletBody::SetAngularDamping(double damping)
{
  this->rigidBody->setDamping(
      (btScalar)this->rigidBody->getLinearDamping(), (btScalar)damping);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the relative pose of a child geom.
void BulletBody::SetGeomRelativePose(BulletGeom *geom, const Pose3d &newPose)
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
