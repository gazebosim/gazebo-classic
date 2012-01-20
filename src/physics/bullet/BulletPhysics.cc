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
/* Desc: The Bullet physics engine wrapper
 * Author: Nate Koenig
 * Date: 11 June 2007
 */
/*
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
#include "common/Console.hh"
#include "common/Exception.hh"
#include "World.hh"
#include "math/Vector3.hh"
#include "Entity.hh"

#include "BulletPhysics.hh"
*/

using namespace gazebo;
using namespace physics;


GZ_REGISTER_PHYSICS_ENGINE("bullet", BulletPhysics);

//////////////////////////////////////////////////
BulletPhysics::BulletPhysics(World *_world)
    : PhysicsEngine(world)
{
  this->collisionConfig = new btDefaultCollisionConfiguration();

  this->dispatcher = new btCollisionDispatcher(this->collisionConfig);

  this->broadPhase = new btDbvtBroadphase();

  // Create the dynamics solver
  this->solver = new btSequentialImpulseConstraintSolver;

  // Instantiate the world
  this->dynamicsWorld = new btDiscreteDynamicsWorld(this->dispatcher,
      this->broadPhase, this->solver, this->collisionConfig);

  common::Param::Begin(&this->parameters);
  common::Param::End();
}

//////////////////////////////////////////////////
BulletPhysics::~BulletPhysics()
{
  delete this->broadPhase;
  delete this->collisionConfig;
  delete this->dispatcher;
  delete this->solver;

  // TODO: Fix this line
  // delete this->dynamicsWorld;

  this->broadPhase = NULL;
  this->collisionConfig = NULL;
  this->dispatcher = NULL;
  this->solver = NULL;
  this->dynamicsWorld = NULL;
}

//////////////////////////////////////////////////
void BulletPhysics::Load(common::XMLConfigNode *_node)
{
  common::XMLConfigNode *cnode = _node->GetChild("bullet", "physics");
  if (cnode == NULL)
    gzthrow("Must define a <physics:ode> _node in the XML file");

  this->stepTimeP->Load(cnode);

  this->gravityP->Load(cnode);
}

//////////////////////////////////////////////////
void BulletPhysics::Save(std::string &_prefix, std::ostream &_stream)
{
  _stream << _prefix << "<physics:bullet>\n";
  _stream << _prefix << "  " << *(this->gravityP) << "\n";
  _stream << _prefix << "</physics:bullet>\n";
}

//////////////////////////////////////////////////
void BulletPhysics::Init()
{
  math::Vector3 g = this->gravityP->GetValue();
  this->dynamicsWorld->setGravity(btmath::Vector3(g.x, g.y, g.z));
}

//////////////////////////////////////////////////
void BulletPhysics::InitForThread()
{
}

//////////////////////////////////////////////////
void BulletPhysics::UpdateCollision()
{
}

//////////////////////////////////////////////////
void BulletPhysics::UpdatePhysics()
{
  /*common::Time time =
 Simulator::Instance()->GetRealTime() - this->lastUpdateTime;
  int steps = (int) round((time / **this->stepTimeP).Double());

  steps = std::max(steps, 1);

  // time = 0.000001;
  // steps = 1;
  // this->dynamicsWorld->stepSimulation(time, steps, (**this->stepTimeP));
  this->dynamicsWorld->stepSimulation((**this->stepTimeP).Double());

  this->lastUpdatecommon::Time = Simulator::Instance()->GetRealTime();
  */
}


//////////////////////////////////////////////////
void BulletPhysics::Fini()
{
}


//////////////////////////////////////////////////
void BulletPhysics::RemoveEntity(Entity *_entity)
{
}

//////////////////////////////////////////////////
void BulletPhysics::AddEntity(Entity *_entity)
{
  BulletLink *body = dynamic_cast<BulletLink*>(_entity);

  this->dynamicsWorld->addRigidLink(body->GetBulletLink());
}

//////////////////////////////////////////////////
Link *BulletPhysics::CreateLink(Entity *parent)
{
  BulletLink *body = new BulletLink(parent);

  return body;
}

//////////////////////////////////////////////////
Collision *BulletPhysics::CreateCollision(std::string type, Link *parent)
{
  BulletCollision *collision = NULL;
  BulletLink *body = NULL;

  body = dynamic_cast<BulletLink*>(parent);

  if (body == NULL)
    gzthrow("CreateCollision requires an BulletLink as a parent");

  collision = new BulletCollision(parent);

  if (type == "sphere")
    new BulletSphereShape(collision);
  if (type == "box")
    new BulletBoxShape(collision);
  if (type == "cylinder")
    new BulletCylinderShape(collision);
  if (type == "plane")
    new BulletPlaneShape(collision);
  if (type == "trimesh")
    new BulletTrimeshShape(collision);
  if (type == "map")
    new MapShape(collision);
  else
    gzthrow("Unable to create a collision of type[" << type << "]");

  return collision;
}


//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
void BulletPhysics::ConvertMass(Mass *_mass, void *_engineMass)
{
}

//////////////////////////////////////////////////
void BulletPhysics::ConvertMass(void *_engineMass, const Mass &_mass)
{
}

//////////////////////////////////////////////////
/*PhysicsRaySensor *BulletPhysics::CreateRaySensor(Link *body)
{
  return NULL;
}*/

//////////////////////////////////////////////////
math::Pose BulletPhysics::ConvertPose(btTransform _bt)
{
  math::Pose pose;
  pose.pos.x = _bt.getOrigin().getX();
  pose.pos.y = _bt.getOrigin().getY();
  pose.pos.z = _bt.getOrigin().getZ();

  pose.rot.u = _bt.getRotation().getW();
  pose.rot.x = _bt.getRotation().getX();
  pose.rot.y = _bt.getRotation().getY();
  pose.rot.z = _bt.getRotation().getZ();

  return pose;
}

//////////////////////////////////////////////////
btTransform BulletPhysics::ConvertPose(const math::Pose _pose)
{
  btTransform trans;

  trans.setOrigin(btmath::Vector3(_pose.pos.x, _pose.pos.y, _pose.pos.z));
  trans.setRotation(btQuaternion(_pose.rot.x, _pose.rot.y,
                                   _pose.rot.z, _pose.rot.u));
  return trans;
}

//////////////////////////////////////////////////
void BulletPhysics::SetGravity(const gazebo::math::Vector3 &_gravity)
{
  this->gravityP->SetValue(_gravity);
  this->dynamicsWorld->setGravity(btmath::Vector3(_gravity.x,
        _gravity.y, _gravity.z));
}


