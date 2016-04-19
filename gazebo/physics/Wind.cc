/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <functional>
#include <sdf/sdf.hh>

#include <ignition/math/Vector3.hh>

#include "gazebo/transport/Node.hh"
#include "gazebo/transport/TransportTypes.hh"

#include "gazebo/physics/Entity.hh"
#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Wind.hh"
#include "gazebo/physics/World.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data for the Wind class
    class WindPrivate
    {
      /// \brief Class constructor.
      /// \param[in] _world A reference to the world.
      public: WindPrivate(physics::World &_world)
        : world(_world)
      {
      }

      /// \brief Reference to the world.
      public: World &world;

      /// \brief Wind linear velocity.
      public: ignition::math::Vector3d linearVel;

      /// \brief The function used to to calculate the wind velocity at an
      /// entity's location.
      /// It takes as input a reference to an instance of Wind and a pointer to
      /// an Entity.
      public: std::function< ignition::math::Vector3d (
                  const Wind *, const Entity *)> linearVelFunc;

      // Transport is declared last.
      /// \brief Node for communication.
      public: transport::NodePtr node;

      /// \brief Response publisher.
      public: transport::PublisherPtr responsePub;

      /// \brief Subscribe to the wind topic.
      public: transport::SubscriberPtr windSub;

      /// \brief Subscribe to the request topic.
      public: transport::SubscriberPtr requestSub;
    };
  }
}

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
Wind::Wind(World &_world, sdf::ElementPtr _sdf)
  : dataPtr(new WindPrivate(_world))
{
  this->Load(_sdf);

  this->dataPtr->node = transport::NodePtr(new transport::Node());
  this->dataPtr->node->Init(this->dataPtr->world.GetName());
  this->dataPtr->windSub = this->dataPtr->node->Subscribe("~/wind",
      &Wind::OnWindMsg, this);

  this->dataPtr->responsePub =
    this->dataPtr->node->Advertise<msgs::Response>("~/response");

  this->dataPtr->requestSub = this->dataPtr->node->Subscribe("~/request",
                                           &Wind::OnRequest, this);

  this->SetLinearVelFunc(std::bind(&Wind::LinearVelDefault, this,
        std::placeholders::_1, std::placeholders::_2));
}

//////////////////////////////////////////////////
Wind::~Wind()
{
  // Must call fini on node to remove it from topic manager.
  this->dataPtr->node->Fini();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Wind::LinearVelDefault(
    const Wind *_wind, const Entity */*_entity*/)
{
  return _wind->LinearVel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Wind::WorldLinearVel(const Entity *_entity) const
{
  return this->dataPtr->linearVelFunc(this, _entity);
}

//////////////////////////////////////////////////
ignition::math::Vector3d Wind::RelativeLinearVel(const Entity *_entity) const
{
  return _entity->GetWorldPose().Ign().Rot().Inverse().RotateVector(
      this->WorldLinearVel(_entity));
}

//////////////////////////////////////////////////
void Wind::SetLinearVel(const ignition::math::Vector3d& _vel)
{
  this->dataPtr->linearVel = _vel;
}

//////////////////////////////////////////////////
const ignition::math::Vector3d& Wind::LinearVel(void) const
{
  return this->dataPtr->linearVel;
}

//////////////////////////////////////////////////
bool Wind::SetParam(const std::string &_key,
    const boost::any &_value)
{
  try
  {
    if (_key == "linear_velocity")
    {
      ignition::math::Vector3d vel =
          boost::any_cast<ignition::math::Vector3d>(_value);
      this->SetLinearVel(vel);
    }
    else
    {
      gzwarn << "SetParam failed for [" << _key << "] in wind " << std::endl;
      return false;
    }
  }
  catch(boost::bad_any_cast &_e)
  {
    gzerr << "Caught bad any_cast in Wind::SetParam: " << _e.what()
          << std::endl;
    return false;
  }
  catch(boost::bad_lexical_cast &_e)
  {
    gzerr << "Caught bad lexical_cast in Wind::SetParam: " << _e.what()
          << std::endl;
    return false;
  }
  return true;
}

//////////////////////////////////////////////////
boost::any Wind::Param(const std::string &_key) const
{
  boost::any value;
  this->Param(_key, value);
  return value;
}

//////////////////////////////////////////////////
bool Wind::Param(const std::string &_key,
    boost::any &_value) const
{
  if (_key == "linear_velocity")
    _value = this->LinearVel();
  else
  {
    gzwarn << "Param failed for [" << _key << "] in wind " << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
void Wind::Load(sdf::ElementPtr _sdf)
{
  if (_sdf && _sdf->HasElement("linear_velocity"))
    this->SetLinearVel(_sdf->Get<ignition::math::Vector3d>("linear_velocity"));
}

/////////////////////////////////////////////////
void Wind::OnRequest(ConstRequestPtr &_msg)
{
  msgs::Response response;
  response.set_id(_msg->id());
  response.set_request(_msg->request());
  response.set_response("success");
  std::string *serializedData = response.mutable_serialized_data();

  if (_msg->request() == "wind_info")
  {
    msgs::Wind windMsg;
    windMsg.mutable_linear_velocity()->CopyFrom(
      msgs::Convert(this->dataPtr->linearVel));
    windMsg.set_enable_wind(this->dataPtr->world.WindEnabled());

    response.set_type(windMsg.GetTypeName());
    windMsg.SerializeToString(serializedData);
    this->dataPtr->responsePub->Publish(response);
  }
}

/////////////////////////////////////////////////
void Wind::OnWindMsg(ConstWindPtr &_msg)
{
  if (_msg->has_linear_velocity())
    this->SetLinearVel(msgs::ConvertIgn(_msg->linear_velocity()));

  if (_msg->has_enable_wind())
    this->dataPtr->world.SetWindEnabled(_msg->enable_wind());
}

/////////////////////////////////////////////////
void Wind::SetLinearVelFunc(std::function< ignition::math::Vector3d (
    const Wind *, const Entity *_entity) > _linearVelFunc)
{
  this->dataPtr->linearVelFunc = _linearVelFunc;
}
