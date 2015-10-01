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
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/PresetManager.hh"

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

  this->params.Add<double>("max_step_size",
      std::bind(&PhysicsEngine::GetMaxStepSize, this),
      std::bind(&PhysicsEngine::SetMaxStepSize, this, std::placeholders::_1));

  this->params.Add<double>("real_time_update_rate",
      std::bind(&PhysicsEngine::GetRealTimeUpdateRate, this),
      std::bind(&PhysicsEngine::SetRealTimeUpdateRate, this,
        std::placeholders::_1));

  this->params.Add<double>("real_time_factor",
      std::bind(&PhysicsEngine::GetTargetRealTimeFactor, this),
      std::bind(&PhysicsEngine::SetTargetRealTimeFactor, this,
        std::placeholders::_1));

  this->params.Add<gazebo::math::Vector3>("gravity",
      std::bind(&PhysicsEngine::GetGravity, this),
      std::bind(&PhysicsEngine::SetGravity, this, std::placeholders::_1));

  this->params.Add<ignition::math::Vector3d>("magnetic_field",
      std::bind(&PhysicsEngine::MagneticField, this),
      std::bind(&PhysicsEngine::SetMagneticField, this, std::placeholders::_1));
}

//////////////////////////////////////////////////
void PhysicsEngine::SetMagneticField(const ignition::math::Vector3d &_value)
{
  this->sdf->GetElement("magnetic_field")->Set(_value);
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
ignition::math::Vector3d PhysicsEngine::MagneticField() const
{
  if (this->sdf->HasElement("magnetic_field"))
    return this->sdf->Get<ignition::math::Vector3d>("magnetic_field");
  else
    return ignition::math::Vector3d::Zero;
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
void PhysicsEngine::SetMaxStepSize(const double &_stepSize)
{
  this->sdf->GetElement("max_step_size")->Set(_stepSize);
  this->maxStepSize = _stepSize;
}

//////////////////////////////////////////////////
void PhysicsEngine::SetAutoDisableFlag(bool /*_autoDisable*/)
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
void PhysicsEngine::OnPhysicsMsg(ConstPhysicsPtr &_msg)
{
  this->world->GetPresetManager()->CurrentProfile(_msg->profile_name());
}


//////////////////////////////////////////////////
boost::any PhysicsEngine::GetParam(const std::string &_key) const
{
  #ifndef _WIN32
  #pragma GCC diagnostic push
  #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
  #endif
  boost::any value;
  this->PhysicsEngine::GetParam(_key, value);
  return value;
  #ifndef _WIN32
  #pragma GCC diagnostic pop
  #endif
}

//////////////////////////////////////////////////
bool PhysicsEngine::GetParam(const std::string &_key,
    boost::any &_value) const
{
  if (_key == "type")
  {
    std::string value;
    this->Param<std::string>(_key, value);
    _value = value;
  }
  else if (_key == "max_step_size")
  {
    double value = 0.0;
    this->Param<double>(_key, value);
    _value = value;
  }
  else if (_key == "real_time_update_rate")
  {
    double value = 0.0;
    this->Param<double>(_key, value);
    _value = value;
  }
  else if (_key == "real_time_factor")
  {
    double value = 0.0;
    this->Param<double>(_key, value);
    _value = value;
  }
  else if (_key == "gravity")
  {
    gazebo::math::Vector3 value;
    this->Param<gazebo::math::Vector3>(_key, value);
    _value = value;
  }
  else if (_key == "magnetic_field")
  {
    ignition::math::Vector3d value;
    this->Param<ignition::math::Vector3d>(_key, value);
    _value = value;
  }
  else
  {
    gzwarn << "GetParam failed for [" << _key << "] in physics engine "
           << this->GetType() << std::endl;
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
sdf::ElementPtr PhysicsEngine::GetSDF() const
{
  return this->sdf;
}
