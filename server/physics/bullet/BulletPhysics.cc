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
/* Desc: The Bullet physics engine wrapper
 * Author: Nate Koenig
 * Date: 11 June 2007
 * SVN: $Id: BulletPhysics.cc 7714 2009-05-23 18:08:49Z natepak $
 */

#include "BulletBody.hh"
#include "BulletPlaneShape.hh"
#include "BulletSphereShape.hh"
#include "BulletBoxShape.hh"
#include "BulletCylinderShape.hh"
#include "BulletTrimeshShape.hh"
#include "MapShape.hh"

#include "BulletHingeJoint.hh"
#include "BulletHinge2Joint.hh"
#include "BulletSliderJoint.hh"
#include "BulletBallJoint.hh"
#include "BulletUniversalJoint.hh"

#include "PhysicsFactory.hh"
#include "Mass.hh"
#include "Global.hh"
#include "GazeboMessage.hh"
#include "GazeboError.hh"
#include "World.hh"
#include "Vector3.hh"
#include "Entity.hh"

#include "XMLConfig.hh"
#include "Simulator.hh"

#include "BulletPhysics.hh"

using namespace gazebo;

GZ_REGISTER_PHYSICS_ENGINE("bullet", BulletPhysics);

////////////////////////////////////////////////////////////////////////////////
// Constructor
BulletPhysics::BulletPhysics()
    : PhysicsEngine()
{
  this->collisionConfig = new btDefaultCollisionConfiguration();

  this->dispatcher = new btCollisionDispatcher(this->collisionConfig);

  this->broadPhase = new btDbvtBroadphase();

  // Create the dynamics solver
  this->solver = new btSequentialImpulseConstraintSolver;

  // Instantiate the world
  this->dynamicsWorld = new btDiscreteDynamicsWorld(this->dispatcher,
      this->broadPhase, this->solver, this->collisionConfig);

  Param::Begin(&this->parameters);
  Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
BulletPhysics::~BulletPhysics()
{
  delete this->broadPhase;
  delete this->collisionConfig;
  delete this->dispatcher;
  delete this->solver;

  // TODO: Fix this line
  //delete this->dynamicsWorld;

  this->broadPhase = NULL;
  this->collisionConfig = NULL;
  this->dispatcher = NULL;
  this->solver = NULL;
  this->dynamicsWorld = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Load the Bullet engine
void BulletPhysics::Load(XMLConfigNode *node)
{
  XMLConfigNode *cnode = node->GetChild("bullet", "physics");
  if (cnode == NULL)
    gzthrow("Must define a <physics:ode> node in the XML file");

  this->stepTimeP->Load(cnode);

  this->gravityP->Load(cnode);
}

////////////////////////////////////////////////////////////////////////////////
// Save the Bullet engine
void BulletPhysics::Save(std::string &prefix, std::ostream &stream)
{
  stream << prefix << "<physics:bullet>\n";
  stream << prefix << "  " << *(this->gravityP) << "\n";
  stream << prefix << "</physics:bullet>\n";
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the Bullet engine
void BulletPhysics::Init()
{
  Vector3 g = this->gravityP->GetValue();
  this->dynamicsWorld->setGravity(btVector3(g.x, g.y, g.z));
}

////////////////////////////////////////////////////////////////////////////////
/// Init the engine for threads. 
void BulletPhysics::InitForThread()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the Bullet collisions, create joints
void BulletPhysics::UpdateCollision()
{
}

////////////////////////////////////////////////////////////////////////////////
// Update the Bullet engine
void BulletPhysics::UpdatePhysics()
{
  Time time = Simulator::Instance()->GetRealTime() - this->lastUpdateTime;
  int steps = (int) round( (time / **this->stepTimeP).Double() );

  steps = std::max(steps,1);

  //time = 0.000001;
  //steps = 1;
  //this->dynamicsWorld->stepSimulation(time,  steps, (**this->stepTimeP));
  this->dynamicsWorld->stepSimulation((**this->stepTimeP).Double());

  this->lastUpdateTime = Simulator::Instance()->GetRealTime();
}


////////////////////////////////////////////////////////////////////////////////
// Finilize the Bullet engine
void BulletPhysics::Fini()
{
}


////////////////////////////////////////////////////////////////////////////////
// Remove an entity from the physics engine
void BulletPhysics::RemoveEntity(Entity *entity)
{
}

////////////////////////////////////////////////////////////////////////////////
// Add an entity to the world
void BulletPhysics::AddEntity(Entity *entity)
{
  BulletBody *body = dynamic_cast<BulletBody*>(entity);

  this->dynamicsWorld->addRigidBody(body->GetBulletBody());
}

////////////////////////////////////////////////////////////////////////////////
// Create a new body
Body *BulletPhysics::CreateBody(Entity *parent)
{
  BulletBody *body = new BulletBody(parent);

  return body;
}

////////////////////////////////////////////////////////////////////////////////
/// Create a new geom
Geom *BulletPhysics::CreateGeom(std::string type, Body *parent)
{
  BulletGeom *geom = NULL;
  Shape *shape = NULL;
  BulletBody *body = NULL;

  body = dynamic_cast<BulletBody*>(parent);

  if (body == NULL)
    gzthrow("CreateGeom requires an BulletBody as a parent");

  geom = new BulletGeom(parent);

  if (type == "sphere")
    shape = new BulletSphereShape(geom);
  if (type == "box")
    shape = new BulletBoxShape(geom);
  if (type == "cylinder")
    shape = new BulletCylinderShape(geom);
  if (type == "plane")
    shape = new BulletPlaneShape(geom);
  if (type == "trimesh")
    shape = new BulletTrimeshShape(geom);
  if (type == "map")
    shape = new MapShape(geom);
  else 
    gzthrow("Unable to create a geom of type[" << type << "]");

  return geom;
}


////////////////////////////////////////////////////////////////////////////////
// Create a new joint
Joint *BulletPhysics::CreateJoint(std::string type)
{
  if (type == "slider")
    return new BulletSliderJoint(this->dynamicsWorld);
  if (type == "hinge")
    return new BulletHingeJoint(this->dynamicsWorld);
  if (type == "hinge2")
    return new BulletHinge2Joint(this->dynamicsWorld);
  if (type == "ball")
    return new BulletBallJoint(this->dynamicsWorld);
  if (type == "universal")
    return new BulletUniversalJoint(this->dynamicsWorld);
  else 
    gzthrow("Unable to create joint of type[" << type << "]");

  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Convert an odeMass to Mass
void BulletPhysics::ConvertMass(Mass *mass, void *engineMass)
{
}

////////////////////////////////////////////////////////////////////////////////
/// Convert an gazebo Mass to a bullet Mass
void BulletPhysics::ConvertMass(void *engineMass, const Mass &mass)
{
}

////////////////////////////////////////////////////////////////////////////////
// Create an object to hold a set of ray geoms
/*PhysicsRaySensor *BulletPhysics::CreateRaySensor(Body *body)
{
  return NULL;
}*/

////////////////////////////////////////////////////////////////////////////////
Pose3d BulletPhysics::ConvertPose(btTransform bt)
{
  Pose3d pose;
  pose.pos.x = bt.getOrigin().getX();
  pose.pos.y = bt.getOrigin().getY();
  pose.pos.z = bt.getOrigin().getZ();

  pose.rot.u = bt.getRotation().getW();
  pose.rot.x = bt.getRotation().getX();
  pose.rot.y = bt.getRotation().getY();
  pose.rot.z = bt.getRotation().getZ();

  return pose;
}

////////////////////////////////////////////////////////////////////////////////
/// Convert a gazebo pose to a bullet transform
btTransform BulletPhysics::ConvertPose(const Pose3d pose)
{
  btTransform trans;

  trans.setOrigin( btVector3( pose.pos.x, pose.pos.y, pose.pos.z) );
  trans.setRotation( btQuaternion( pose.rot.x, pose.rot.y, 
                                   pose.rot.z, pose.rot.u) );
  return trans;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the gavity vector
void BulletPhysics::SetGravity(const gazebo::Vector3 &gravity)
{
  this->gravityP->SetValue(gravity);
  this->dynamicsWorld->setGravity(btVector3(gravity.x, gravity.y, gravity.z));
}
