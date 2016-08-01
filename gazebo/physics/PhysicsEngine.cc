/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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

#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

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
#include "gazebo/physics/PresetManager.hh"

#include "gazebo/physics/PhysicsEnginePrivate.hh"
#include "gazebo/physics/PhysicsEngine.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
PhysicsEngine::PhysicsEngine(WorldPtr _world)
: physicsEngineDPtr(new PhysicsEnginePrivate)
{
  this->physicsEngineDPtr->world = _world;
  this->ConstructionHelper();
}

//////////////////////////////////////////////////
PhysicsEngine::PhysicsEngine(PhysicsEnginePrivate &_dataPtr, WorldPtr _world)
: physicsEngineDPtr(&_dataPtr)
{
  this->physicsEngineDPtr->world = _world;
  this->ConstructionHelper();
}

//////////////////////////////////////////////////
void PhysicsEngine::ConstructionHelper()
{
  this->physicsEngineDPtr->sdf.reset(new sdf::Element);
  sdf::initFile("physics.sdf", this->physicsEngineDPtr->sdf);

  this->physicsEngineDPtr->targetRealTimeFactor = 0;
  this->physicsEngineDPtr->realTimeUpdateRate = 0;
  this->physicsEngineDPtr->maxStepSize = 0;

  this->physicsEngineDPtr->node = transport::NodePtr(new transport::Node());
  this->physicsEngineDPtr->node->Init(this->physicsEngineDPtr->world->Name());
  this->physicsEngineDPtr->physicsSub =
    this->physicsEngineDPtr->node->Subscribe("~/physics",
      &PhysicsEngine::OnPhysicsMsg, this);

  this->physicsEngineDPtr->responsePub =
    this->physicsEngineDPtr->node->Advertise<msgs::Response>("~/response");

  this->physicsEngineDPtr->requestSub =
    this->physicsEngineDPtr->node->Subscribe("~/request",
        &PhysicsEngine::OnRequest, this);

  // Create and initialized the contact manager.
  this->physicsEngineDPtr->contactManager = new ContactManager();
  this->physicsEngineDPtr->contactManager->Init(this->physicsEngineDPtr->world);
}

//////////////////////////////////////////////////
void PhysicsEngine::Load(sdf::ElementPtr _sdf)
{
  this->physicsEngineDPtr->sdf->Copy(_sdf);

  this->physicsEngineDPtr->realTimeUpdateRate =
      this->physicsEngineDPtr->sdf->GetElement(
          "real_time_update_rate")->Get<double>();
  this->physicsEngineDPtr->targetRealTimeFactor =
      this->physicsEngineDPtr->sdf->GetElement(
          "real_time_factor")->Get<double>();
  this->physicsEngineDPtr->maxStepSize =
      this->physicsEngineDPtr->sdf->GetElement("max_step_size")->Get<double>();
}

//////////////////////////////////////////////////
void PhysicsEngine::Fini()
{
  // Clean up transport
  {
    this->physicsEngineDPtr->responsePub.reset();
    this->physicsEngineDPtr->requestSub.reset();

    this->physicsEngineDPtr->node.reset();
  }

  if (this->physicsEngineDPtr->sdf)
  {
    this->physicsEngineDPtr->sdf->Reset();
    this->physicsEngineDPtr->sdf.reset();
  }

  if (this->physicsEngineDPtr->contactManager)
  {
    delete this->physicsEngineDPtr->contactManager;
    this->physicsEngineDPtr->contactManager = NULL;
  }

  this->physicsEngineDPtr->world.reset();
}

//////////////////////////////////////////////////
PhysicsEngine::~PhysicsEngine()
{
  this->Fini();
}

//////////////////////////////////////////////////
math::Vector3 PhysicsEngine::GetGravity() const
{
  return this->physicsEngineDPtr->world->Gravity();
}

//////////////////////////////////////////////////
ignition::math::Vector3d PhysicsEngine::MagneticField() const
{
  return this->physicsEngineDPtr->world->MagneticField();
}

//////////////////////////////////////////////////
CollisionPtr PhysicsEngine::CreateCollision(const std::string &_shapeType,
                                            const std::string &_linkName)
{
  CollisionPtr result;
  LinkPtr link =
    std::dynamic_pointer_cast<Link>(
        this->physicsEngineDPtr->world->EntityByName(_linkName));

  if (!link)
    gzerr << "Unable to find link[" << _linkName << "]\n";
  else
    result = this->CreateCollision(_shapeType, link);

  return result;
}

//////////////////////////////////////////////////
double PhysicsEngine::GetUpdatePeriod()
{
  return this->UpdatePeriod();
}

//////////////////////////////////////////////////
double PhysicsEngine::UpdatePeriod() const
{
  double updateRate = this->RealTimeUpdateRate();
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
  return this->TargetRealTimeFactor();
}

//////////////////////////////////////////////////
double PhysicsEngine::TargetRealTimeFactor() const
{
  return this->physicsEngineDPtr->targetRealTimeFactor;
}

//////////////////////////////////////////////////
double PhysicsEngine::GetRealTimeUpdateRate() const
{
  return this->RealTimeUpdateRate();
}

//////////////////////////////////////////////////
double PhysicsEngine::RealTimeUpdateRate() const
{
  return this->physicsEngineDPtr->realTimeUpdateRate;
}

//////////////////////////////////////////////////
double PhysicsEngine::GetMaxStepSize() const
{
  return this->MaxStepSize();
}

//////////////////////////////////////////////////
double PhysicsEngine::MaxStepSize() const
{
  return this->physicsEngineDPtr->maxStepSize;
}

//////////////////////////////////////////////////
void PhysicsEngine::SetTargetRealTimeFactor(const double _factor)
{
  this->physicsEngineDPtr->sdf->GetElement("real_time_factor")->Set(_factor);
  this->physicsEngineDPtr->targetRealTimeFactor = _factor;
}

//////////////////////////////////////////////////
void PhysicsEngine::SetRealTimeUpdateRate(const double _rate)
{
  this->physicsEngineDPtr->sdf->GetElement("real_time_update_rate")->Set(_rate);
  this->physicsEngineDPtr->realTimeUpdateRate = _rate;
}

//////////////////////////////////////////////////
void PhysicsEngine::SetMaxStepSize(const double _stepSize)
{
  this->physicsEngineDPtr->sdf->GetElement("max_step_size")->Set(_stepSize);
  this->physicsEngineDPtr->maxStepSize = _stepSize;
}

//////////////////////////////////////////////////
void PhysicsEngine::SetAutoDisableFlag(const bool /*_autoDisable*/)
{
}

//////////////////////////////////////////////////
void PhysicsEngine::SetMaxContacts(const unsigned int /*_maxContacts*/)
{
}

//////////////////////////////////////////////////
void PhysicsEngine::OnRequest(ConstRequestPtr &/*_msg*/)
{
}

//////////////////////////////////////////////////
void PhysicsEngine::OnPhysicsMsg(ConstPhysicsPtr &_msg)
{
  this->physicsEngineDPtr->world->PresetMgr()->CurrentProfile(
      _msg->profile_name());
}

//////////////////////////////////////////////////
bool PhysicsEngine::SetParam(const std::string &_key,
    const boost::any &_value)
{
  try
  {
    if (_key == "type")
    {
      // Cannot set physics engine type from SetParam
      return false;
    }
    if (_key == "max_step_size")
      this->SetMaxStepSize(boost::any_cast<double>(_value));
    else if (_key == "real_time_update_rate")
      this->SetRealTimeUpdateRate(boost::any_cast<double>(_value));
    else if (_key == "real_time_factor")
      this->SetTargetRealTimeFactor(boost::any_cast<double>(_value));
    else if (_key == "gravity")
    {
      boost::any copy = _value;
#ifndef _WIN32
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
      if (_value.type() == typeid(sdf::Vector3))
      {
        copy = boost::lexical_cast<ignition::math::Vector3d>
            (boost::any_cast<sdf::Vector3>(_value));
      }
      else if (_value.type() == typeid(math::Vector3))
      {
        copy = boost::lexical_cast<ignition::math::Vector3d>
            (boost::any_cast<math::Vector3>(_value));
      }
      this->SetGravity(boost::any_cast<ignition::math::Vector3d>(copy));
    }
    else if (_key == "magnetic_field")
    {
      boost::any copy = _value;
      if (_value.type() == typeid(sdf::Vector3))
      {
        copy = boost::lexical_cast<ignition::math::Vector3d>
            (boost::any_cast<sdf::Vector3>(_value));
      }
      else if (_value.type() == typeid(math::Vector3))
      {
        copy = boost::lexical_cast<ignition::math::Vector3d>
            (boost::any_cast<math::Vector3>(_value));
      }
#ifndef _WIN32
#pragma GCC diagnostic pop
#endif
      this->physicsEngineDPtr->world->SetMagneticField(
          boost::any_cast<ignition::math::Vector3d>(copy));
    }
    else
    {
      gzwarn << "SetParam failed for [" << _key << "] in physics engine "
             << this->Type() << std::endl;
      return false;
    }
  }
  catch(boost::bad_any_cast &_e)
  {
    gzerr << "Caught bad any_cast in PhysicsEngine::SetParam: " << _e.what()
          << std::endl;
    return false;
  }
  catch(boost::bad_lexical_cast &_e)
  {
    gzerr << "Caught bad lexical_cast in PhysicsEngine::SetParam: " << _e.what()
          << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
boost::any PhysicsEngine::GetParam(const std::string &_key) const
{
  return this->Param(_key);
}

//////////////////////////////////////////////////
boost::any PhysicsEngine::Param(const std::string &_key) const
{
  boost::any value;
  this->PhysicsEngine::Param(_key, value);
  return value;
}

//////////////////////////////////////////////////
bool PhysicsEngine::GetParam(const std::string &_key,
    boost::any &_value) const
{
  return this->Param(_key, _value);
}

//////////////////////////////////////////////////
bool PhysicsEngine::Param(const std::string &_key,
    boost::any &_value) const
{
  if (_key == "type")
    _value = this->Type();
  else if (_key == "max_step_size")
    _value = this->MaxStepSize();
  else if (_key == "real_time_update_rate")
    _value = this->RealTimeUpdateRate();
  else if (_key == "real_time_factor")
    _value = this->TargetRealTimeFactor();
  else if (_key == "gravity")
    _value = this->physicsEngineDPtr->world->Gravity();
  else if (_key == "magnetic_field")
    _value = this->physicsEngineDPtr->world->MagneticField();
  else
  {
    gzwarn << "Param failed for [" << _key << "] in physics engine "
           << this->Type() << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
ContactManager *PhysicsEngine::GetContactManager() const
{
  return this->ContactMgr();
}

//////////////////////////////////////////////////
ContactManager *PhysicsEngine::ContactMgr() const
{
  return this->physicsEngineDPtr->contactManager;
}

//////////////////////////////////////////////////
sdf::ElementPtr PhysicsEngine::GetSDF() const
{
  return this->SDF();
}

//////////////////////////////////////////////////
sdf::ElementPtr PhysicsEngine::SDF() const
{
  return this->physicsEngineDPtr->sdf;
}

//////////////////////////////////////////////////
WorldPtr PhysicsEngine::World() const
{
  return this->physicsEngineDPtr->world;
}

//////////////////////////////////////////////////
std::string PhysicsEngine::GetType() const
{
  return this->Type();
}

//////////////////////////////////////////////////
void PhysicsEngine::SetGravity(const gazebo::math::Vector3 &_gravity)
{
  return this->SetGravity(_gravity.Ign());
}

//////////////////////////////////////////////////
void PhysicsEngine::Reset()
{
}

//////////////////////////////////////////////////
void PhysicsEngine::UpdatePhysics()
{
}

//////////////////////////////////////////////////
bool PhysicsEngine::GetAutoDisableFlag()
{
  return this->AutoDisableFlag();
}

//////////////////////////////////////////////////
bool PhysicsEngine::AutoDisableFlag() const
{
  return 0;
}

//////////////////////////////////////////////////
boost::recursive_mutex *PhysicsEngine::GetPhysicsUpdateMutex() const
{
  return NULL;
}

//////////////////////////////////////////////////
std::recursive_mutex &PhysicsEngine::PhysicsUpdateMutex() const
{
  return this->physicsEngineDPtr->physicsUpdateMutex;
}
