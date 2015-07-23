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

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/math/Vector3.hh"

#include "gazebo/transport/Publisher.hh"

#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/ContactManager.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/MapShape.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/PhysicsFactory.hh"
#include "gazebo/physics/SurfaceParams.hh"
#include "gazebo/physics/World.hh"

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
#include "gazebo/physics/dart/DARTPolylineShape.hh"
#include "gazebo/physics/dart/DARTMultiRayShape.hh"
#include "gazebo/physics/dart/DARTHeightmapShape.hh"

#include "gazebo/physics/dart/DARTModel.hh"
#include "gazebo/physics/dart/DARTLink.hh"

#include "gazebo/physics/dart/DARTPhysics.hh"

#include "gazebo/physics/dart/DARTPhysicsPrivate.hh"

using namespace gazebo;
using namespace physics;

GZ_REGISTER_PHYSICS_ENGINE("dart", DARTPhysics)

//////////////////////////////////////////////////
DARTPhysics::DARTPhysics(WorldPtr _world)
    : PhysicsEngine(_world), dataPtr(new DARTPhysicsPrivate())
{
}

//////////////////////////////////////////////////
DARTPhysics::~DARTPhysics()
{
  delete this->dataPtr;
}

//////////////////////////////////////////////////
void DARTPhysics::Load(sdf::ElementPtr _sdf)
{
  PhysicsEngine::Load(_sdf);

  // Gravity
  math::Vector3 g = this->sdf->Get<math::Vector3>("gravity");
  // ODEPhysics checks this, so we will too.
  if (g == math::Vector3(0, 0, 0))
    gzwarn << "Gravity vector is (0, 0, 0). Objects will float.\n";
  this->dataPtr->dtWorld->setGravity(Eigen::Vector3d(g.x, g.y, g.z));
}

//////////////////////////////////////////////////
void DARTPhysics::Init()
{
}

//////////////////////////////////////////////////
void DARTPhysics::Fini()
{
  PhysicsEngine::Fini();
}

//////////////////////////////////////////////////
void DARTPhysics::Reset()
{
  boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);

  // Restore state all the models
  unsigned int modelCount = this->world->GetModelCount();
  DARTModelPtr dartModelIt;

  for (unsigned int i = 0; i < modelCount; ++i)
  {
    dartModelIt =
      boost::dynamic_pointer_cast<DARTModel>(this->world->GetModel(i));
    GZ_ASSERT(dartModelIt.get(), "dartModelIt pointer is NULL");

    dartModelIt->RestoreState();
  }
}

//////////////////////////////////////////////////
void DARTPhysics::InitForThread()
{
}

//////////////////////////////////////////////////
void DARTPhysics::UpdateCollision()
{
  this->contactManager->ResetCount();

  dart::constraint::ConstraintSolver *dtConstraintSolver =
      this->dataPtr->dtWorld->getConstraintSolver();
  dart::collision::CollisionDetector *dtCollisionDetector =
      dtConstraintSolver->getCollisionDetector();
  int numContacts = dtCollisionDetector->getNumContacts();

  for (int i = 0; i < numContacts; ++i)
  {
    const dart::collision::Contact &dtContact =
        dtCollisionDetector->getContact(i);
    dart::dynamics::BodyNode *dtBodyNode1 = dtContact.bodyNode1.lock().get();
    dart::dynamics::BodyNode *dtBodyNode2 = dtContact.bodyNode2.lock().get();

    DARTLinkPtr dartLink1 = this->FindDARTLink(dtBodyNode1);
    DARTLinkPtr dartLink2 = this->FindDARTLink(dtBodyNode2);

    GZ_ASSERT(dartLink1.get() != NULL, "dartLink1 in collision pair is NULL");
    GZ_ASSERT(dartLink2.get() != NULL, "dartLink2 in collision pair is NULL");

    unsigned int colIndex = 0;
    CollisionPtr collisionPtr1 = dartLink1->GetCollision(colIndex);
    CollisionPtr collisionPtr2 = dartLink2->GetCollision(colIndex);

    // Add a new contact to the manager. This will return NULL if no one is
    // listening for contact information.
    Contact *contactFeedback = this->GetContactManager()->NewContact(
                                 collisionPtr1.get(), collisionPtr2.get(),
                                 this->world->GetSimTime());

    if (!contactFeedback)
      continue;

    math::Pose body1Pose = dartLink1->GetWorldPose();
    math::Pose body2Pose = dartLink2->GetWorldPose();
    math::Vector3 localForce1;
    math::Vector3 localForce2;
    math::Vector3 localTorque1;
    math::Vector3 localTorque2;

    // calculate force in world frame
    Eigen::Vector3d force = dtContact.force;

    // calculate torque in world frame
    Eigen::Vector3d torqueA =
        (dtContact.point -
         dtBodyNode1->getTransform().translation()).cross(force);
    Eigen::Vector3d torqueB =
        (dtContact.point -
         dtBodyNode2->getTransform().translation()).cross(-force);

    // Convert from world to link frame
    localForce1 = body1Pose.rot.RotateVectorReverse(
        DARTTypes::ConvVec3(force));
    localForce2 = body2Pose.rot.RotateVectorReverse(
        DARTTypes::ConvVec3(-force));
    localTorque1 = body1Pose.rot.RotateVectorReverse(
        DARTTypes::ConvVec3(torqueA));
    localTorque2 = body2Pose.rot.RotateVectorReverse(
        DARTTypes::ConvVec3(torqueB));

    contactFeedback->positions[0] = DARTTypes::ConvVec3(dtContact.point);
    contactFeedback->normals[0] = DARTTypes::ConvVec3(dtContact.normal);
    contactFeedback->depths[0] = dtContact.penetrationDepth;

    if (!dartLink1->IsStatic())
    {
      contactFeedback->wrench[0].body1Force = localForce1;
      contactFeedback->wrench[0].body1Torque = localTorque1;
    }

    if (!dartLink2->IsStatic())
    {
      contactFeedback->wrench[0].body2Force = localForce2;
      contactFeedback->wrench[0].body2Torque = localTorque2;
    }

    ++contactFeedback->count;
  }
}

//////////////////////////////////////////////////
void DARTPhysics::UpdatePhysics()
{
  // need to lock, otherwise might conflict with world resetting
  boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);

  // common::Time currTime =  this->world->GetRealTime();

  this->dataPtr->dtWorld->setTimeStep(this->maxStepSize);
  this->dataPtr->dtWorld->step();

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
          = boost::dynamic_pointer_cast<DARTLink>(links.at(j));
      dartLinkItr->updateDirtyPoseFromDARTTransformation();
    }
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
  gzwarn << "Not implemented yet in DART.\n";
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
  {
    gzerr << "Link must have a parent in DART.\n";
    return LinkPtr();
  }

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
    boost::dynamic_pointer_cast<DARTCollision>(_collision);

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
  else if (_type == "polyline")
    shape.reset(new DARTPolylineShape(collision));
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

  if (_type == "prismatic")
    joint.reset(new DARTSliderJoint(_parent));
  else if (_type == "screw")
    joint.reset(new DARTScrewJoint(_parent));
  else if (_type == "revolute")
    joint.reset(new DARTHingeJoint(_parent));
  else if (_type == "revolute2")
    joint.reset(new DARTHinge2Joint(_parent));
  else if (_type == "ball")
    joint.reset(new DARTBallJoint(_parent));
  else if (_type == "universal")
    joint.reset(new DARTUniversalJoint(_parent));
  else
    gzerr << "Unable to create joint of type[" << _type << "]";

  return joint;
}

//////////////////////////////////////////////////
void DARTPhysics::SetGravity(const gazebo::math::Vector3 &_gravity)
{
  this->sdf->GetElement("gravity")->Set(_gravity);
  this->dataPtr->dtWorld->setGravity(
    Eigen::Vector3d(_gravity.x, _gravity.y, _gravity.z));
}

//////////////////////////////////////////////////
void DARTPhysics::DebugPrint() const
{
  gzwarn << "Not implemented in DART.\n";
}

//////////////////////////////////////////////////
boost::any DARTPhysics::GetParam(const std::string &_key) const
{
  boost::any value;
  this->GetParam(_key, value);
  return value;
}

//////////////////////////////////////////////////
bool DARTPhysics::GetParam(const std::string &_key, boost::any &_value) const
{
  if (!this->sdf->HasElement("dart"))
  {
    return PhysicsEngine::GetParam(_key, _value);
  }
  sdf::ElementPtr dartElem = this->sdf->GetElement("dart");
  // physics dart element not yet added to sdformat
  // GZ_ASSERT(dartElem != NULL, "DART SDF element does not exist");

  if (_key == "max_contacts")
  {
    _value = dartElem->GetElement("max_contacts")->Get<int>();
  }
  else if (_key == "min_step_size")
  {
    _value = dartElem->GetElement("solver")->Get<double>("min_step_size");
  }
  else
  {
    return PhysicsEngine::GetParam(_key, _value);
  }

  return true;
}

//////////////////////////////////////////////////
bool DARTPhysics::SetParam(const std::string &_key, const boost::any &_value)
{
  /// \TODO fill this out, see issue #1115
  try
  {
    if (_key == "max_contacts")
    {
      int value = boost::any_cast<int>(_value);
      gzerr << "Setting [" << _key << "] in DART to [" << value
            << "] not yet supported.\n";
    }
    else if (_key == "min_step_size")
    {
      double value = boost::any_cast<double>(_value);
      gzerr << "Setting [" << _key << "] in DART to [" << value
            << "] not yet supported.\n";
    }
    else
    {
      if (_key == "max_step_size")
      {
        this->dataPtr->dtWorld->setTimeStep(boost::any_cast<double>(_value));
      }
      return PhysicsEngine::SetParam(_key, _value);
    }
  }
  catch(boost::bad_any_cast &e)
  {
    gzerr << "DARTPhysics::SetParam(" << _key << ") boost::any_cast error: "
          << e.what() << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
dart::simulation::World *DARTPhysics::GetDARTWorld()
{
  return this->dataPtr->dtWorld.get();
}

//////////////////////////////////////////////////
dart::simulation::WorldPtr DARTPhysics::GetDARTWorldPtr()
{
  return this->dataPtr->dtWorld;
}

//////////////////////////////////////////////////
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
    physicsMsg.set_type(msgs::Physics::DART);
    physicsMsg.mutable_gravity()->CopyFrom(msgs::Convert(this->GetGravity()));
    physicsMsg.set_enable_physics(this->world->GetEnablePhysicsEngine());
    physicsMsg.set_real_time_update_rate(this->realTimeUpdateRate);
    physicsMsg.set_real_time_factor(this->targetRealTimeFactor);
    physicsMsg.set_max_step_size(this->maxStepSize);

    response.set_type(physicsMsg.GetTypeName());
    physicsMsg.SerializeToString(serializedData);
    this->responsePub->Publish(response);
  }
}

//////////////////////////////////////////////////
void DARTPhysics::OnPhysicsMsg(ConstPhysicsPtr& _msg)
{
  // Parent class handles many generic parameters
  // This should be done first so that the profile settings
  // can be over-ridden by other message parameters.
  PhysicsEngine::OnPhysicsMsg(_msg);

  if (_msg->has_enable_physics())
    this->world->EnablePhysicsEngine(_msg->enable_physics());

  if (_msg->has_gravity())
    this->SetGravity(msgs::Convert(_msg->gravity()));

  if (_msg->has_real_time_factor())
    this->SetTargetRealTimeFactor(_msg->real_time_factor());

  if (_msg->has_real_time_update_rate())
  {
    this->SetRealTimeUpdateRate(_msg->real_time_update_rate());
  }

  if (_msg->has_max_step_size())
  {
    this->SetMaxStepSize(_msg->max_step_size());
  }

  /// Make sure all models get at least on update cycle.
  this->world->EnableAllModels();
}

//////////////////////////////////////////////////
DARTLinkPtr DARTPhysics::FindDARTLink(
    const dart::dynamics::BodyNode *_dtBodyNode)
{
  DARTLinkPtr res;

  const Model_V& models = this->world->GetModels();

  for (Model_V::const_iterator itModel = models.begin();
       itModel != models.end(); ++itModel)
  {
    const Link_V& links = (*itModel)->GetLinks();

    for (Link_V::const_iterator itLink = links.begin();
         itLink != links.end(); ++itLink)
    {
      DARTLinkPtr dartLink = boost::dynamic_pointer_cast<DARTLink>(*itLink);

      if (dartLink->GetDARTBodyNode() == _dtBodyNode)
      {
        res = dartLink;
        break;
      }
    }
  }

  return res;
}

