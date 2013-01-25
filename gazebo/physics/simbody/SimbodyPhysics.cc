/*
 * Copyright 2012 Open Source Robotics Foundation
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
/* Desc: The Simbody physics engine wrapper
 * Author: Nate Koenig
 * Date: 11 June 2007
 */

#include "physics/simbody/SimbodyTypes.hh"
#include "physics/simbody/SimbodyLink.hh"
#include "physics/simbody/SimbodyCollision.hh"

#include "physics/simbody/SimbodyPlaneShape.hh"
#include "physics/simbody/SimbodySphereShape.hh"
#include "physics/simbody/SimbodyHeightmapShape.hh"
#include "physics/simbody/SimbodyMultiRayShape.hh"
#include "physics/simbody/SimbodyBoxShape.hh"
#include "physics/simbody/SimbodyCylinderShape.hh"
#include "physics/simbody/SimbodyTrimeshShape.hh"
#include "physics/simbody/SimbodyRayShape.hh"

#include "physics/simbody/SimbodyHingeJoint.hh"
#include "physics/simbody/SimbodyUniversalJoint.hh"
#include "physics/simbody/SimbodyBallJoint.hh"
#include "physics/simbody/SimbodySliderJoint.hh"
#include "physics/simbody/SimbodyHinge2Joint.hh"
#include "physics/simbody/SimbodyScrewJoint.hh"

#include "physics/PhysicsTypes.hh"
#include "physics/PhysicsFactory.hh"
#include "physics/World.hh"
#include "physics/Entity.hh"
#include "physics/Model.hh"
#include "physics/SurfaceParams.hh"
#include "physics/Collision.hh"
#include "physics/MapShape.hh"

#include "common/Console.hh"
#include "common/Exception.hh"
#include "math/Vector3.hh"

#include "SimbodyPhysics.hh"

// include simbody
#include "Simbody.h"

using namespace gazebo;
using namespace physics;
using namespace SimTK;

GZ_REGISTER_PHYSICS_ENGINE("simbody", SimbodyPhysics)

//////////////////////////////////////////////////
bool ContactCallback()
{
  return true;
}

//////////////////////////////////////////////////
bool ContactProcessed()
{
  return true;
}

//////////////////////////////////////////////////
SimbodyPhysics::SimbodyPhysics(WorldPtr _world)
    : PhysicsEngine(_world), system(), matter(system), forces(system), integ(NULL)
{
  // Instantiate the Multibody System
  // Instantiate the Simbody Matter Subsystem
  // Instantiate the Simbody General Force Subsystem

  // Create an integrator
  //this->integ = new SimTK::RungeKuttaMersonIntegrator(system);
  this->integ = new SimTK::ExplicitEulerIntegrator(system);



}

//////////////////////////////////////////////////
SimbodyPhysics::~SimbodyPhysics()
{
}

//////////////////////////////////////////////////
void SimbodyPhysics::Load(sdf::ElementPtr _sdf)
{
  PhysicsEngine::Load(_sdf);

  sdf::ElementPtr simbodyElem = this->sdf->GetElement("simbody");

  this->stepTimeDouble = simbodyElem->GetElement("dt")->GetValueDouble();

  math::Vector3 g = this->sdf->GetValueVector3("gravity");


  // set gravity
  SimTK::Force::UniformGravity(this->forces, this->matter, SimTK::Vec3(g.x, g.y, g.z));


}

//////////////////////////////////////////////////
void SimbodyPhysics::Init()
{
  // add pendulum body
  // default pin joint has axis in the +z direction,
  // rotate to get pin in x direction
  const SimTK::Rotation ZtoY( -SimTK::Pi/2, SimTK::XAxis);
  SimTK::Body::Rigid pendulumBody(MassProperties(1.0, Vec3(-1, 0, 0), UnitInertia::pointMassAt(Vec3(-1,0,0))));
  SimTK::MobilizedBody::Pin pendulum(matter.Ground(), Transform(ZtoY, Vec3(0, 0, 0)), 
                                pendulumBody,    Transform(ZtoY, Vec3(1, 0, 0)));

  SimTK::State state = this->system.realizeTopology();

  // pendulum.setAngle(state, Pi/4);

  this->integ->initialize(state);
}

//////////////////////////////////////////////////
void SimbodyPhysics::InitForThread()
{
}

//////////////////////////////////////////////////
void SimbodyPhysics::UpdateCollision()
{
}

//////////////////////////////////////////////////
void SimbodyPhysics::UpdatePhysics()
{
  // need to lock, otherwise might conflict with world resetting
  boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);

  common::Time currTime =  this->world->GetRealTime();


  while (integ->getTime() < this->world->GetSimTime().Double())
    this->integ->stepTo(this->world->GetSimTime().Double(),
                       this->world->GetSimTime().Double());

  const SimTK::State &s = integ->getState();
  gzerr << "\n\ntime [" << s.getTime()
        << "] q [" << s.getQ()
        << "] u [" << s.getU()
        << "] dt [" << this->stepTimeDouble
        << "] t [" << this->world->GetSimTime().Double()
        << "]\n";

  ModelPtr model = this->world->GetModel("model1");
  LinkPtr link = model->GetLink("link1");
  JointPtr joint = model->GetJoint("model1::joint1");
  joint->SetAngle(0, s.getQ()[0]);

  this->lastUpdateTime = currTime;
}

//////////////////////////////////////////////////
void SimbodyPhysics::Fini()
{
}

//////////////////////////////////////////////////
void SimbodyPhysics::SetStepTime(double _value)
{
  this->sdf->GetElement("simbody")->GetElement(
      "solver")->GetAttribute("min_step_size")->Set(_value);

  this->stepTimeDouble = _value;
}

//////////////////////////////////////////////////
double SimbodyPhysics::GetStepTime()
{
  return this->stepTimeDouble;
}

//////////////////////////////////////////////////
LinkPtr SimbodyPhysics::CreateLink(ModelPtr _parent)
{
  if (_parent == NULL)
    gzthrow("Link must have a parent\n");

  SimbodyLinkPtr link(new SimbodyLink(_parent));
  link->SetWorld(_parent->GetWorld());

  return link;
}

//////////////////////////////////////////////////
CollisionPtr SimbodyPhysics::CreateCollision(const std::string &_type,
                                            LinkPtr _parent)
{
  SimbodyCollisionPtr collision(new SimbodyCollision(_parent));
  ShapePtr shape = this->CreateShape(_type, collision);
  collision->SetShape(shape);
  shape->SetWorld(_parent->GetWorld());
  return collision;
}

//////////////////////////////////////////////////
ShapePtr SimbodyPhysics::CreateShape(const std::string &_type,
                                    CollisionPtr _collision)
{
  ShapePtr shape;
  SimbodyCollisionPtr collision =
    boost::shared_dynamic_cast<SimbodyCollision>(_collision);

  if (_type == "plane")
    shape.reset(new SimbodyPlaneShape(collision));
  else if (_type == "sphere")
    shape.reset(new SimbodySphereShape(collision));
  else if (_type == "box")
    shape.reset(new SimbodyBoxShape(collision));
  else if (_type == "cylinder")
    shape.reset(new SimbodyCylinderShape(collision));
  else if (_type == "mesh" || _type == "trimesh")
    shape.reset(new SimbodyTrimeshShape(collision));
  else if (_type == "heightmap")
    shape.reset(new SimbodyHeightmapShape(collision));
  else if (_type == "multiray")
    shape.reset(new SimbodyMultiRayShape(collision));
  else if (_type == "ray")
    if (_collision)
      shape.reset(new SimbodyRayShape(_collision));
    else
      shape.reset(new SimbodyRayShape(this->world->GetPhysicsEngine()));
  else
    gzerr << "Unable to create collision of type[" << _type << "]\n";

  /*
  else if (_type == "map" || _type == "image")
    shape.reset(new MapShape(collision));
    */
  return shape;
}

//////////////////////////////////////////////////
JointPtr SimbodyPhysics::CreateJoint(const std::string &_type,
                                     ModelPtr _parent)
{
  JointPtr joint;
  if (_type == "revolute")
    joint.reset(new SimbodyHingeJoint(this->dynamicsWorld, _parent));
  else if (_type == "universal")
    joint.reset(new SimbodyUniversalJoint(this->dynamicsWorld, _parent));
  else if (_type == "ball")
    joint.reset(new SimbodyBallJoint(this->dynamicsWorld, _parent));
  else if (_type == "prismatic")
    joint.reset(new SimbodySliderJoint(this->dynamicsWorld, _parent));
  else if (_type == "revolute2")
    joint.reset(new SimbodyHinge2Joint(this->dynamicsWorld, _parent));
  else if (_type == "screw")
    joint.reset(new SimbodyScrewJoint(this->dynamicsWorld, _parent));
  else
    gzthrow("Unable to create joint of type[" << _type << "]");

  return joint;
}

//////////////////////////////////////////////////
void SimbodyPhysics::ConvertMass(InertialPtr /*_inertial*/,
                                void * /*_engineMass*/)
{
}

//////////////////////////////////////////////////
void SimbodyPhysics::ConvertMass(void * /*_engineMass*/,
                                const InertialPtr /*_inertial*/)
{
}

//////////////////////////////////////////////////
void SimbodyPhysics::SetGravity(const gazebo::math::Vector3 &_gravity)
{
  this->sdf->GetElement("gravity")->GetAttribute("xyz")->Set(_gravity);
}

//////////////////////////////////////////////////
void SimbodyPhysics::DebugPrint() const
{
}


