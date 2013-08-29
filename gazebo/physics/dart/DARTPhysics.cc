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

#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/math/Vector3.hh"

#include "gazebo/transport/Publisher.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/PhysicsFactory.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/SurfaceParams.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/MapShape.hh"

#include "gazebo/physics/dart/DARTScrewJoint.hh"
#include "gazebo/physics/dart/DARTHingeJoint.hh"
#include "gazebo/physics/dart/DARTHinge2Joint.hh"
#include "gazebo/physics/dart/DARTSliderJoint.hh"
#include "gazebo/physics/dart/DARTBallJoint.hh"
#include "gazebo/physics/dart/DARTUniversalJoint.hh"

#include "gazebo/physics/dart/DARTRayShape.hh"
#include "gazebo/physics/dart/DARTBoxShape.hh"
#include "gazebo/physics/dart/DARTSphereShape.hh"
#include "gazebo/physics/dart/DARTCylinderShape.hh"
#include "gazebo/physics/dart/DARTPlaneShape.hh"
#include "gazebo/physics/dart/DARTMeshShape.hh"
#include "gazebo/physics/dart/DARTMultiRayShape.hh"
#include "gazebo/physics/dart/DARTHeightmapShape.hh"

#include "gazebo/physics/dart/DARTModel.hh"
#include "gazebo/physics/dart/DARTLink.hh"

#include "DARTPhysics.hh"

using namespace gazebo;
using namespace physics;

GZ_REGISTER_PHYSICS_ENGINE("dart", DARTPhysics)

//////////////////////////////////////////////////
DARTPhysics::DARTPhysics(WorldPtr _world)
    : PhysicsEngine(_world)
{
  this->dartWorld = new simulation::World;

  // TODO: Gazebo does not support design-time and runtime concept now.
  // Therefore, we basically set dart world as runtime and never change it.
  // When gazebo support the concept, we should apply it to dart also.
  // this->dartWorld->changeDesignTime(false);
}

//////////////////////////////////////////////////
DARTPhysics::~DARTPhysics()
{
  delete this->dartWorld;
}

//////////////////////////////////////////////////
void DARTPhysics::Load(sdf::ElementPtr _sdf)
{
  PhysicsEngine::Load(_sdf);

  // Gravity
  math::Vector3 g = this->sdf->Get<math::Vector3>("gravity");
  this->dartWorld->setGravity(Eigen::Vector3d(g.x, g.y, g.z));

  // Time step
  //double timeStep = this->sdf->GetValueDouble("time_step");
  //this->dartWorld->setTimeStep(timeStep);
  
  // TODO: Elements for dart settings
  //sdf::ElementPtr dartElem = this->sdf->GetElement("dart");
  //this->stepTimeDouble = dartElem->GetElement("dt")->GetValueDouble();
}
 
//////////////////////////////////////////////////
void DARTPhysics::Init()
{
  //this->dartWorld->initialize();
}

//////////////////////////////////////////////////
void DARTPhysics::Fini()
{
  PhysicsEngine::Fini();
}

//////////////////////////////////////////////////
void DARTPhysics::Reset()
{
  {
    this->physicsUpdateMutex->lock();

    this->dartWorld->reset();

    this->physicsUpdateMutex->unlock();
  }
}

//////////////////////////////////////////////////
void DARTPhysics::InitForThread()
{
}
 
//////////////////////////////////////////////////
void DARTPhysics::UpdateCollision()
{
}

//////////////////////////////////////////////////
void DARTPhysics::UpdatePhysics()
{
  {
    // need to lock, otherwise might conflict with world resetting
    boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);
    //this->physicsUpdateMutex->lock();

//    std::vector<Eigen::VectorXd> dofs = this->dartWorld->getDofs();
//    Eigen::VectorXd FirstDof = dofs[0];
//    double state = FirstDof[0];

    //common::Time currTime =  this->world->GetRealTime();
    this->dartWorld->step();

//    dofs = this->dartWorld->getDofs();
//    FirstDof = dofs[0];
//    state = FirstDof[0];
    //gzerr << (this->dartWorld->getDofs().at(0))[0];
    //this->dartWorld->updateKinematics();
    //this->lastUpdateTime = currTime;

    // Update all the transformation of DART's links to gazebo's links
    // TODO: How to visit all the links in the world?
    unsigned int modelCount = this->world->GetModelCount();
    ModelPtr modelItr;

    for (unsigned int i = 0; i < modelCount; ++i)
    {
      modelItr = this->world->GetModel(i);
      // TODO: need to improve speed
      Link_V links = modelItr->GetLinks();
      unsigned int linkCount = links.size();
      DARTLinkPtr dartLinkItr;

      for (unsigned int j = 0; j < linkCount; ++j)
      {
        dartLinkItr
            = boost::shared_dynamic_cast<DARTLink>(links.at(j));
        dartLinkItr->updateDirtyPoseFromDARTTransformation();
      }
    }
    //this->physicsUpdateMutex->unlock();
  }
}

//////////////////////////////////////////////////
std::string DARTPhysics::GetType() const
{
  return "dart";
}

//////////////////////////////////////////////////
void DARTPhysics::SetSeed(uint32_t /*_seed*/)
{
  gzerr << "Not implemented yet...\n";
}

//////////////////////////////////////////////////
ModelPtr DARTPhysics::CreateModel(BasePtr _parent)
{
  DARTModelPtr model(new DARTModel(_parent));

  return model;
}

//////////////////////////////////////////////////
LinkPtr DARTPhysics::CreateLink(ModelPtr _parent)
{
  if (_parent == NULL)
	gzthrow("Link must have a parent\n");

  DARTLinkPtr link(new DARTLink(_parent));

  link->SetWorld(_parent->GetWorld());

  return link;
}

//////////////////////////////////////////////////
CollisionPtr DARTPhysics::CreateCollision(const std::string &_type,
                                            LinkPtr _body)
{
  DARTCollisionPtr collision(new DARTCollision(_body));
  
  ShapePtr shape = this->CreateShape(_type, collision);
  
  collision->SetShape(shape);
  
  shape->SetWorld(_body->GetWorld());
  
  return collision;
}

//////////////////////////////////////////////////
ShapePtr DARTPhysics::CreateShape(const std::string &_type,
                                    CollisionPtr _collision)
{
  ShapePtr shape;
  
  DARTCollisionPtr collision =
    boost::shared_dynamic_cast<DARTCollision>(_collision);

  if (_type == "sphere")
    shape.reset(new DARTSphereShape(collision));
  else if (_type == "plane")
    shape.reset(new DARTPlaneShape(collision));
  else if (_type == "box")
    shape.reset(new DARTBoxShape(collision));
  else if (_type == "cylinder")
    shape.reset(new DARTCylinderShape(collision));
  else if (_type == "multiray")
    shape.reset(new DARTMultiRayShape(collision));
  else if (_type == "mesh" || _type == "trimesh")
    shape.reset(new DARTMeshShape(collision));
  else if (_type == "heightmap")
    shape.reset(new DARTHeightmapShape(collision));
  else if (_type == "map" || _type == "image")
    shape.reset(new MapShape(collision));
  else if (_type == "ray")
    if (_collision)
      shape.reset(new DARTRayShape(collision));
    else
      shape.reset(new DARTRayShape(this->world->GetPhysicsEngine()));
  else
    gzerr << "Unable to create collision of type[" << _type << "]\n";

  return shape;
}

//////////////////////////////////////////////////
JointPtr DARTPhysics::CreateJoint(const std::string &_type, ModelPtr _parent)
{
  JointPtr joint;

//  if (_type == "prismatic")
//    joint.reset(new DARTSliderJoint(_parent));
//  else if (_type == "screw")
//    joint.reset(new DARTScrewJoint(_parent));
//  else if (_type == "revolute")
//    joint.reset(new DARTHingeJoint(_parent));
//  else if (_type == "revolute2")
//    joint.reset(new DARTHinge2Joint(_parent));
//  else if (_type == "ball")
//    joint.reset(new DARTBallJoint(_parent));
//  else if (_type == "universal")
//    joint.reset(new DARTUniversalJoint(_parent));
  if (_type == "revolute")
    joint.reset(new DARTHingeJoint(_parent));
  else
    gzthrow("Unable to create joint of type[" << _type << "]");

  return joint;
}

//////////////////////////////////////////////////
void DARTPhysics::SetGravity(const gazebo::math::Vector3& _gravity)
{
  this->sdf->GetElement("gravity")->GetAttribute("xyz")->Set(_gravity);
  this->dartWorld->setGravity(Eigen::Vector3d(_gravity.x, _gravity.y, _gravity.z));
}

//////////////////////////////////////////////////
void DARTPhysics::DebugPrint() const
{
  gzwarn << "Not implemented!\n";
}

void DARTPhysics::OnRequest(ConstRequestPtr &_msg)
{
  msgs::Response response;
  response.set_id(_msg->id());
  response.set_request(_msg->request());
  response.set_response("success");
  std::string *serializedData = response.mutable_serialized_data();

  if (_msg->request() == "physics_info")
  {
    msgs::Physics physicsMsg;
//    physicsMsg.set_type(msgs::Physics::DART);
//    physicsMsg.set_update_rate(this->GetUpdateRate());
//    physicsMsg.set_solver_type(this->stepType);
//    physicsMsg.set_dt(this->stepTimeDouble);
//    physicsMsg.set_iters(this->GetSORPGSIters());
    physicsMsg.set_enable_physics(this->world->GetEnablePhysicsEngine());
//    physicsMsg.set_sor(this->GetSORPGSW());
//    physicsMsg.set_cfm(this->GetWorldCFM());
//    physicsMsg.set_erp(this->GetWorldERP());
//    physicsMsg.set_contact_max_correcting_vel(
//        this->GetContactMaxCorrectingVel());
//    physicsMsg.set_contact_surface_layer(this->GetContactSurfaceLayer());
//    physicsMsg.mutable_gravity()->CopyFrom(msgs::Convert(this->GetGravity()));

//    response.set_type(physicsMsg.GetTypeName());
    physicsMsg.SerializeToString(serializedData);
    this->responsePub->Publish(response);
  }
}

void DARTPhysics::OnPhysicsMsg(ConstPhysicsPtr &/*_msg*/)
{
  /// Make sure all models get at least on update cycle.
  this->world->EnableAllModels();
}

void DARTPhysics::SetMaxStepSize(double _stepSize)
{
  PhysicsEngine::SetMaxStepSize(_stepSize);

  this->dartWorld->setTimeStep(_stepSize);
}

