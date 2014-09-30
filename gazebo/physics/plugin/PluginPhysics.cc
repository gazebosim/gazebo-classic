/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>

#include <sdf/sdf.hh>

#include <algorithm>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include "gazebo/util/Diagnostics.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Rand.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/common/Timer.hh"

#include "gazebo/transport/Publisher.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/PhysicsFactory.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/SurfaceParams.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/MapShape.hh"
#include "gazebo/physics/ContactManager.hh"

#include "gazebo/physics/plugin/PluginCollision.hh"
#include "gazebo/physics/plugin/PluginLink.hh"
#include "gazebo/physics/plugin/PluginScrewJoint.hh"
#include "gazebo/physics/plugin/PluginHingeJoint.hh"
#include "gazebo/physics/plugin/PluginGearboxJoint.hh"
#include "gazebo/physics/plugin/PluginHinge2Joint.hh"
#include "gazebo/physics/plugin/PluginSliderJoint.hh"
#include "gazebo/physics/plugin/PluginBallJoint.hh"
#include "gazebo/physics/plugin/PluginUniversalJoint.hh"

#include "gazebo/physics/plugin/PluginRayShape.hh"
#include "gazebo/physics/plugin/PluginBoxShape.hh"
#include "gazebo/physics/plugin/PluginSphereShape.hh"
#include "gazebo/physics/plugin/PluginCylinderShape.hh"
#include "gazebo/physics/plugin/PluginPlaneShape.hh"
#include "gazebo/physics/plugin/PluginMeshShape.hh"
#include "gazebo/physics/plugin/PluginMultiRayShape.hh"
#include "gazebo/physics/plugin/PluginHeightmapShape.hh"
#include "gazebo/physics/plugin/PluginPolylineShape.hh"

#include "gazebo/physics/plugin/PluginPhysics.hh"
#include "gazebo/physics/plugin/PluginSurfaceParams.hh"

using namespace gazebo;
using namespace physics;

GZ_REGISTER_PHYSICS_ENGINE("plugin", PluginPhysics)

//////////////////////////////////////////////////
PluginPhysics::PluginPhysics(WorldPtr _world)
    : PhysicsEngine(_world), maxContacts(0)
{
  this->worldId = 0;

  this->colliders.resize(100);

  // Set random seed for physics engine based on gazebo's random seed.
  // Note: this was moved from physics::PhysicsEngine constructor.
  this->SetSeed(math::Rand::GetSeed());
}

//////////////////////////////////////////////////
PluginPhysics::~PluginPhysics()
{
  // Delete all the joint feedbacks.
  for (std::vector<PluginJointFeedback*>::iterator iter =
      this->jointFeedbacks.begin(); iter != this->jointFeedbacks.end(); ++iter)
  {
    delete *iter;
  }
  this->jointFeedbacks.clear();

  // destroy world (worldId)

  this->worldId = NULL;
}

//////////////////////////////////////////////////
void PluginPhysics::Load(sdf::ElementPtr _sdf)
{
  PhysicsEngine::Load(_sdf);

}

/////////////////////////////////////////////////
void PluginPhysics::OnRequest(ConstRequestPtr &_msg)
{
  msgs::Response response;
  response.set_id(_msg->id());
  response.set_request(_msg->request());
  response.set_response("success");
  std::string *serializedData = response.mutable_serialized_data();

  if (_msg->request() == "physics_info")
  {
    msgs::Physics physicsMsg;
    physicsMsg.set_min_step_size(
        boost::any_cast<double>(this->GetParam("min_step_size")));
    physicsMsg.set_real_time_update_rate(this->realTimeUpdateRate);
    physicsMsg.set_real_time_factor(this->targetRealTimeFactor);
    physicsMsg.set_max_step_size(this->maxStepSize);

    response.set_type(physicsMsg.GetTypeName());
    physicsMsg.SerializeToString(serializedData);
    this->responsePub->Publish(response);
  }
}

/////////////////////////////////////////////////
void PluginPhysics::OnPhysicsMsg(ConstPhysicsPtr &_msg)
{
  if (_msg->has_min_step_size())
    this->SetParam("min_step_size", _msg->min_step_size());

  if (_msg->has_enable_physics())
    this->world->EnablePhysicsEngine(_msg->enable_physics());

  if (_msg->has_real_time_factor())
    this->SetTargetRealTimeFactor(_msg->real_time_factor());

  if (_msg->has_real_time_update_rate())
    this->SetRealTimeUpdateRate(_msg->real_time_update_rate());

  if (_msg->has_max_step_size())
    this->SetMaxStepSize(_msg->max_step_size());

  /// Make sure all models get at least on update cycle.
  this->world->EnableAllModels();

  // Parent class handles many generic parameters
  PhysicsEngine::OnPhysicsMsg(_msg);
}



//////////////////////////////////////////////////
void PluginPhysics::Init()
{
}

//////////////////////////////////////////////////
void PluginPhysics::InitForThread()
{
}

//////////////////////////////////////////////////
void PluginPhysics::UpdateCollision()
{
  DIAG_TIMER_START("PluginPhysics::UpdateCollision");

  {
    boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);
  }

  DIAG_TIMER_STOP("PluginPhysics::UpdateCollision");
}

//////////////////////////////////////////////////
void PluginPhysics::UpdatePhysics()
{
  DIAG_TIMER_START("PluginPhysics::UpdatePhysics");

  {
    boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);
  }

  DIAG_TIMER_STOP("PluginPhysics::UpdatePhysics");
}

//////////////////////////////////////////////////
void PluginPhysics::Fini()
{
  PhysicsEngine::Fini();
}

//////////////////////////////////////////////////
void PluginPhysics::Reset()
{
  boost::recursive_mutex::scoped_lock lock(*this->physicsUpdateMutex);
}

//////////////////////////////////////////////////
LinkPtr PluginPhysics::CreateLink(ModelPtr _parent)
{
  if (_parent == NULL)
    gzthrow("Link must have a parent\n");

  PluginLinkPtr link(new PluginLink(_parent));

  link->SetWorld(_parent->GetWorld());

  return link;
}

//////////////////////////////////////////////////
CollisionPtr PluginPhysics::CreateCollision(const std::string &_type,
                                         LinkPtr _body)
{
  PluginCollisionPtr collision(new PluginCollision(_body));
  ShapePtr shape = this->CreateShape(_type, collision);
  collision->SetShape(shape);
  shape->SetWorld(_body->GetWorld());
  return collision;
}

//////////////////////////////////////////////////
ShapePtr PluginPhysics::CreateShape(const std::string &_type,
                                 CollisionPtr _collision)
{
  ShapePtr shape;
  PluginCollisionPtr collision =
    boost::dynamic_pointer_cast<PluginCollision>(_collision);

  if (_type == "sphere")
    shape.reset(new PluginSphereShape(collision));
  else if (_type == "plane")
    shape.reset(new PluginPlaneShape(collision));
  else if (_type == "box")
    shape.reset(new PluginBoxShape(collision));
  else if (_type == "cylinder")
    shape.reset(new PluginCylinderShape(collision));
  else if (_type == "polyline")
    shape.reset(new PluginPolylineShape(collision));
  else if (_type == "multiray")
    shape.reset(new PluginMultiRayShape(collision));
  else if (_type == "mesh" || _type == "trimesh")
    shape.reset(new PluginMeshShape(collision));
  else if (_type == "heightmap")
    shape.reset(new PluginHeightmapShape(collision));
  else if (_type == "map" || _type == "image")
    shape.reset(new MapShape(collision));
  else if (_type == "ray")
    if (_collision)
      shape.reset(new PluginRayShape(collision));
    else
      shape.reset(new PluginRayShape(this->world->GetPhysicsEngine()));
  else
    gzerr << "Unable to create collision of type[" << _type << "]\n";

  return shape;
}

//////////////////////////////////////////////////
int PluginPhysics::GetWorldId()
{
  return this->worldId;
}

//////////////////////////////////////////////////
void PluginPhysics::SetMaxContacts(unsigned int _maxContacts)
{
  this->maxContacts = _maxContacts;
  this->sdf->GetElement("max_contacts")->GetValue()->Set(_maxContacts);
}

//////////////////////////////////////////////////
unsigned int PluginPhysics::GetMaxContacts()
{
  return this->maxContacts;
}

//////////////////////////////////////////////////
JointPtr PluginPhysics::CreateJoint(const std::string &_type, ModelPtr _parent)
{
  JointPtr joint;

  if (_type == "prismatic")
    joint.reset(new PluginSliderJoint(this->worldId, _parent));
  else if (_type == "screw")
    joint.reset(new PluginScrewJoint(this->worldId, _parent));
  else if (_type == "revolute")
    joint.reset(new PluginHingeJoint(this->worldId, _parent));
  else if (_type == "gearbox")
    joint.reset(new PluginGearboxJoint(this->worldId, _parent));
  else if (_type == "revolute2")
    joint.reset(new PluginHinge2Joint(this->worldId, _parent));
  else if (_type == "ball")
    joint.reset(new PluginBallJoint(this->worldId, _parent));
  else if (_type == "universal")
    joint.reset(new PluginUniversalJoint(this->worldId, _parent));
  else
    gzthrow("Unable to create joint of type[" << _type << "]");

  return joint;
}

//////////////////////////////////////////////////
void PluginPhysics::SetGravity(const gazebo::math::Vector3 &_gravity)
{
  this->sdf->GetElement("gravity")->Set(_gravity);
  dWorldSetGravity(this->worldId, _gravity.x, _gravity.y, _gravity.z);
}

/////////////////////////////////////////////////
void PluginPhysics::DebugPrint() const
{
}

/////////////////////////////////////////////////
void PluginPhysics::SetSeed(uint32_t _seed)
{
}

//////////////////////////////////////////////////
bool PluginPhysics::SetParam(const std::string &_key, const boost::any &_value)
{
  sdf::ElementPtr odeElem = this->sdf->GetElement("plugin");
  GZ_ASSERT(odeElem != NULL, "Plugin SDF element does not exist");

  if (_key == "my_solver_string_param")
  {
    std::string value;
    try
    {
      value = boost::any_cast<std::string>(_value);
    }
    catch(const boost::bad_any_cast &e)
    {
      gzerr << "boost any_cast error:" << e.what() << "\n";
      return false;
    }
    // set solver string param to value
  }
  else if (_key == "max_step_size")
  {
    this->SetMaxStepSize(boost::any_cast<double>(_value));
  }
  else
  {
    gzwarn << _key << " is not supported in plugin" << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
boost::any PluginPhysics::GetParam(const std::string &_key) const
{
  sdf::ElementPtr pluginElem = this->sdf->GetElement("plugin");
  GZ_ASSERT(pluginElem != NULL, "Plugin SDF element does not exist");

  if (_key == "my_solver_param_key")
  {
    return pluginElem->GetElement("solver")->Get<std::string>("my_param_key");
  }
  else if (_key == "max_contacts")
    return this->sdf->Get<int>("max_contacts");
  else if (_key == "min_step_size")
    return pluginElem->GetElement("solver")->Get<double>("min_step_size");
  else if (_key == "max_step_size")
    return this->GetMaxStepSize();
  else
  {
    gzwarn << _key << " is not supported in plugin" << std::endl;
    return 0;
  }
}
