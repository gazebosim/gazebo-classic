/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include <sdf/sdf.hh>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Events.hh"

#include "gazebo/transport/TransportIface.hh"
#include "gazebo/transport/Node.hh"

#include "gazebo/math/Rand.hh"

#include "gazebo/physics/ContactManager.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsEngine.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
PhysicsEngine::PhysicsEngine(WorldPtr _world)
  : world(_world)
{
  this->sdf.reset(new sdf::Element);
  sdf::initFile("physics.sdf", this->sdf);

  this->targetRealTimeFactor = 0;
  this->realTimeUpdateRate = 0;
  this->maxStepSize = 0;

  this->node = transport::NodePtr(new transport::Node());
  this->node->Init(this->world->GetName());
  this->physicsSub = this->node->Subscribe("~/physics",
      &PhysicsEngine::OnPhysicsMsg, this);

  this->responsePub =
    this->node->Advertise<msgs::Response>("~/response");

  this->requestSub = this->node->Subscribe("~/request",
                                           &PhysicsEngine::OnRequest, this);

  this->physicsUpdateMutex = new boost::recursive_mutex();

  // Create and initialized the contact manager.
  this->contactManager = new ContactManager();
  this->contactManager->Init(this->world);
}

//////////////////////////////////////////////////
void PhysicsEngine::Load(sdf::ElementPtr _sdf)
{
  this->sdf->Copy(_sdf);

  this->realTimeUpdateRate =
      this->sdf->GetElement("real_time_update_rate")->Get<double>();
  this->targetRealTimeFactor =
      this->sdf->GetElement("real_time_factor")->Get<double>();
  this->maxStepSize =
      this->sdf->GetElement("max_step_size")->Get<double>();
}

//////////////////////////////////////////////////
void PhysicsEngine::Fini()
{
  this->world.reset();
  this->node->Fini();
}

//////////////////////////////////////////////////
PhysicsEngine::~PhysicsEngine()
{
  this->sdf->Reset();
  this->sdf.reset();
  delete this->physicsUpdateMutex;
  this->physicsUpdateMutex = NULL;
  this->responsePub.reset();
  this->requestSub.reset();
  this->node.reset();

  delete this->contactManager;
}

//////////////////////////////////////////////////
math::Vector3 PhysicsEngine::GetGravity() const
{
  return this->sdf->Get<math::Vector3>("gravity");
}

//////////////////////////////////////////////////
CollisionPtr PhysicsEngine::CreateCollision(const std::string &_shapeType,
                                            const std::string &_linkName)
{
  CollisionPtr result;
  LinkPtr link =
    boost::dynamic_pointer_cast<Link>(this->world->GetEntity(_linkName));

  if (!link)
    gzerr << "Unable to find link[" << _linkName << "]\n";
  else
    result = this->CreateCollision(_shapeType, link);

  return result;
}

//////////////////////////////////////////////////
double PhysicsEngine::GetUpdatePeriod()
{
  double updateRate = this->GetRealTimeUpdateRate();
  if (updateRate > 0)
    return 1.0/updateRate;
  else
    return 0;
}

//////////////////////////////////////////////////
ModelPtr PhysicsEngine::CreateModel(BasePtr _base)
{
  ModelPtr ret(new Model(_base));
  return ret;
}

//////////////////////////////////////////////////
double PhysicsEngine::GetTargetRealTimeFactor() const
{
  return this->targetRealTimeFactor;
}

//////////////////////////////////////////////////
double PhysicsEngine::GetRealTimeUpdateRate() const
{
  return this->realTimeUpdateRate;
}

//////////////////////////////////////////////////
double PhysicsEngine::GetMaxStepSize() const
{
  return this->maxStepSize;
}

//////////////////////////////////////////////////
void PhysicsEngine::SetTargetRealTimeFactor(double _factor)
{
  this->sdf->GetElement("real_time_factor")->Set(_factor);
  this->targetRealTimeFactor = _factor;
}

//////////////////////////////////////////////////
void PhysicsEngine::SetRealTimeUpdateRate(double _rate)
{
  this->sdf->GetElement("real_time_update_rate")->Set(_rate);
  this->realTimeUpdateRate = _rate;
}

//////////////////////////////////////////////////
void PhysicsEngine::SetMaxStepSize(double _stepSize)
{
  this->sdf->GetElement("max_step_size")->Set(_stepSize);
  this->maxStepSize = _stepSize;
}

//////////////////////////////////////////////////
void PhysicsEngine::SetWorldCFM(double /*_cfm*/)
{
}

//////////////////////////////////////////////////
void PhysicsEngine::SetWorldERP(double /*_erp*/)
{
}

//////////////////////////////////////////////////
void PhysicsEngine::SetAutoDisableFlag(bool /*_autoDisable*/)
{
}

//////////////////////////////////////////////////
void PhysicsEngine::SetContactMaxCorrectingVel(double /*_vel*/)
{
}

//////////////////////////////////////////////////
void PhysicsEngine::SetMaxContacts(unsigned int /*_maxContacts*/)
{
}

//////////////////////////////////////////////////
void PhysicsEngine::OnRequest(ConstRequestPtr &/*_msg*/)
{
}

//////////////////////////////////////////////////
void PhysicsEngine::OnPhysicsMsg(ConstPhysicsPtr &/*_msg*/)
{
}

//////////////////////////////////////////////////
void PhysicsEngine::SetContactSurfaceLayer(double /*_layerDepth*/)
{
}

//////////////////////////////////////////////////
bool PhysicsEngine::SetParam(const std::string &/*_key*/,
    const boost::any &/*_value*/)
{
  return true;
}

//////////////////////////////////////////////////
boost::any PhysicsEngine::GetParam(const std::string &/*_key*/) const
{
  return 0;
}

//////////////////////////////////////////////////
bool PhysicsEngine::GetParam(const std::string &/*_key*/,
    boost::any &/*_value*/) const
{
  return true;
}

//////////////////////////////////////////////////
template <typename Type> bool PhysicsEngine::GetParam(const std::string &_key,
    Type &_value) const
{
  boost::any value;
  if (!this->GetParam(_key, value))
    return false;
  try
  {
    _value = boost::any_cast<Type>(value);
  }
  catch(boost::bad_any_cast &_e)
  {
    gzwarn << "Failed boost::any_cast in GetParam: " << _e.what() << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
ContactManager *PhysicsEngine::GetContactManager() const
{
  return this->contactManager;
}

//////////////////////////////////////////////////
// Template declarations
template bool PhysicsEngine::GetParam<double>(const std::string &_key,
    double &_value) const;
template bool PhysicsEngine::GetParam<int>(const std::string &_key,
    int &_value) const;
template bool PhysicsEngine::GetParam<std::string>(const std::string &_key,
    std::string &_value) const;
template bool PhysicsEngine::GetParam<math::Vector3>(const std::string &_key,
    math::Vector3 &_value) const;
template bool PhysicsEngine::GetParam<bool>(const std::string &_key,
    bool &_value) const;
