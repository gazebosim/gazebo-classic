/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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

#include <boost/algorithm/string.hpp>
#include <functional>
#include <mutex>
#include <sstream>

#include "gazebo/transport/TransportIface.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/common/Events.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Battery.hh"

#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/Light.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/ContactManager.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Wind.hh"

#include "gazebo/util/IntrospectionManager.hh"
#include "gazebo/util/OpenAL.hh"
#include "gazebo/util/UtilTypes.hh"

/// \brief Private data for the Link class
class gazebo::physics::LinkPrivate
{
  /// \brief Event used when the link is enabled or disabled.
  public: event::EventT<void (bool)> enabledSignal;

  /// \brief This flag is used to trigger the enabled
  public: bool enabled = false;

  /// \brief Names of all the sensors attached to the link.
  public: std::vector<std::string> sensors;

  /// \brief All the lights attached to the link.
  public: std::vector<LightPtr> lights;

  /// \brief All the parent joints.
  public: std::vector<JointPtr> parentJoints;

  /// \brief All the child joints.
  public: std::vector<JointPtr> childJoints;

  /// \brief All the attached models.
  public: std::vector<ModelPtr> attachedModels;

  /// \brief Link data publisher
  public: transport::PublisherPtr dataPub;

  /// \brief Link data message
  public: msgs::LinkData linkDataMsg;

  /// \brief True to publish data, false otherwise
  public: bool publishData = false;

  /// \brief Mutex to protect the publishData variable
  public: std::recursive_mutex *publishDataMutex;

  /// \brief Cached list of collisions. This is here for performance.
  public: Collision_V collisions;

  /// \brief Wrench subscriber.
  public: transport::SubscriberPtr wrenchSub;

  /// \brief Vector of wrench messages to be processed.
  public: std::vector<msgs::Wrench> wrenchMsgs;

  /// \brief Mutex to protect the wrenchMsgs variable.
  public: std::mutex wrenchMsgMutex;

  /// \brief Wind velocity.
  public: ignition::math::Vector3d windLinearVel;

  /// \brief Update connection to calculate wind velocity.
  public: event::ConnectionPtr updateConnection;

  /// \brief All the attached batteries.
  public: std::vector<common::BatteryPtr> batteries;

#ifdef HAVE_OPENAL
      /// \brief All the audio sources
      public: std::vector<util::OpenALSourcePtr> audioSources;

      /// \brief An audio sink
      public: util::OpenALSinkPtr audioSink;

      /// \brief Subscriber to contacts with this collision. Used for audio
      /// playback.
      public: transport::SubscriberPtr audioContactsSub;
#endif
};

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
Link::Link(EntityPtr _parent)
    : Entity(_parent), dataPtr(new LinkPrivate)
{
  this->AddType(Base::LINK);
  this->inertial.reset(new Inertial);
  this->dataPtr->parentJoints.clear();
  this->dataPtr->childJoints.clear();
  this->dataPtr->publishDataMutex = new std::recursive_mutex();
}

//////////////////////////////////////////////////
Link::~Link()
{
  this->Fini();
}

//////////////////////////////////////////////////
void Link::Load(sdf::ElementPtr _sdf)
{
  Entity::Load(_sdf);

  // before loading child collision, we have to figure out if selfCollide is
  // true and modify parent class Entity so this body has its own spaceId
  if (this->sdf->HasElement("self_collide"))
  {
    this->SetSelfCollide(this->sdf->Get<bool>("self_collide"));
  }
  else
  {
    this->SetSelfCollide(this->GetModel()->GetSelfCollide());
  }
  this->sdf->GetElement("self_collide")->GetValue()->SetUpdateFunc(
      std::bind(&Link::GetSelfCollide, this));

  // Parse visuals from SDF
  this->ParseVisuals();

  // Load the geometries
  if (this->sdf->HasElement("collision"))
  {
    sdf::ElementPtr collisionElem = this->sdf->GetElement("collision");
    while (collisionElem)
    {
      // Create and Load a collision, which will belong to this body.
      this->LoadCollision(collisionElem);
      collisionElem = collisionElem->GetNextElement("collision");
    }
  }

  if (this->sdf->HasElement("sensor"))
  {
    sdf::ElementPtr sensorElem = this->sdf->GetElement("sensor");
    while (sensorElem)
    {
      /// \todo This if statement is a hack to prevent Links from creating
      /// a force torque sensor. We should make this more generic.
      if (sensorElem->Get<std::string>("type") == "force_torque")
      {
        gzerr << "A link cannot load a [" <<
          sensorElem->Get<std::string>("type") << "] sensor.\n";
      }
      else if (sensorElem->Get<std::string>("type") != "__default__")
      {
        // This must match the implementation in Sensors::GetScopedName
        std::string sensorName = this->GetScopedName(true) + "::" +
          sensorElem->Get<std::string>("name");

        // Tell the sensor library to create a sensor.
        event::Events::createSensor(sensorElem,
            this->GetWorld()->Name(), this->GetScopedName(), this->GetId());

        this->dataPtr->sensors.push_back(sensorName);
      }
      sensorElem = sensorElem->GetNextElement("sensor");
    }
  }

  // Load the lights
  if (this->sdf->HasElement("light"))
  {
    sdf::ElementPtr lightElem = this->sdf->GetElement("light");
    while (lightElem)
    {
      // Create and Load a light
      this->LoadLight(lightElem);
      lightElem = lightElem->GetNextElement("light");
    }
  }

  if (!this->IsStatic())
  {
    this->inertial->Load(this->sdf->GetElement("inertial"));
  }
  else
  {
    this->inertial->SetMass(0.0);
    this->inertial->SetInertiaMatrix(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

#ifdef HAVE_OPENAL
  if (_sdf->HasElement("audio_source"))
  {
    // bool onContact = false;
    sdf::ElementPtr audioElem = this->sdf->GetElement("audio_source");
    std::vector<std::string> collisionNames;

    while (audioElem)
    {
      util::OpenALSourcePtr source = util::OpenAL::Instance()->CreateSource(
          audioElem);

      std::vector<std::string> names = source->CollisionNames();
      std::copy(names.begin(), names.end(), std::back_inserter(collisionNames));

      audioElem = audioElem->GetNextElement("audio_source");
      this->dataPtr->audioSources.push_back(source);
    }

    if (!collisionNames.empty())
    {
      for (std::vector<std::string>::iterator iter = collisionNames.begin();
          iter != collisionNames.end(); ++iter)
      {
        (*iter) = this->GetScopedName() + "::" + (*iter);
      }

      std::string topic =
        this->world->Physics()->GetContactManager()->CreateFilter(
            this->GetScopedName() + "/audio_collision", collisionNames);
      this->dataPtr->audioContactsSub = this->node->Subscribe(topic,
          &Link::OnCollision, this);
    }
  }

  if (_sdf->HasElement("audio_sink"))
  {
    this->dataPtr->audioSink = util::OpenAL::Instance()->CreateSink(
        _sdf->GetElement("audio_sink"));
  }
#endif

  if (this->sdf->HasElement("battery"))
  {
    sdf::ElementPtr batteryElem = this->sdf->GetElement("battery");
    while (batteryElem)
    {
      this->LoadBattery(batteryElem);
      batteryElem = batteryElem->GetNextElement("battery");
    }
  }

  if (this->sdf->HasElement("enable_wind"))
  {
    this->SetWindMode(this->sdf->Get<bool>("enable_wind"));
  }
  else
  {
    this->SetWindMode(this->GetModel()->WindMode());
  }
  this->sdf->GetElement("enable_wind")->GetValue()->SetUpdateFunc(
      std::bind(&Link::WindMode, this));

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      std::bind(
      static_cast<void(Link::*)(const common::UpdateInfo &)>(&Link::Update),
      this, std::placeholders::_1)));

  this->SetStatic(this->IsStatic());
}

//////////////////////////////////////////////////
void Link::Init()
{
  this->linearAccel.Set(0, 0, 0);
  this->angularAccel.Set(0, 0, 0);

  this->dataPtr->enabled = true;

  // Set Link pose before setting pose of child collisions
  this->SetRelativePose(this->sdf->Get<ignition::math::Pose3d>("pose"));
  this->SetInitialRelativePose(this->sdf->Get<ignition::math::Pose3d>("pose"));

  // Call Init for child collisions, which whill set their pose
  Base_V::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::COLLISION))
    {
      CollisionPtr collision = boost::static_pointer_cast<Collision>(*iter);
      this->dataPtr->collisions.push_back(collision);
      collision->Init();
    }
    if ((*iter)->HasType(Base::LIGHT))
    {
      LightPtr light= boost::static_pointer_cast<Light>(*iter);
      light->Init();
    }
  }

  // Initialize all the batteries
  for (auto &battery : this->dataPtr->batteries)
  {
    battery->Init();
  }

  if (this->WindMode() && this->world->WindEnabled())
    this->SetWindEnabled(true);

  this->initialized = true;
}

//////////////////////////////////////////////////
void Link::Fini()
{
  this->dataPtr->updateConnection.reset();

  this->dataPtr->attachedModels.clear();
  this->dataPtr->parentJoints.clear();
  this->dataPtr->childJoints.clear();
  this->dataPtr->collisions.clear();
  this->inertial.reset();
  this->dataPtr->batteries.clear();

  // Remove all the sensors attached to the link
  for (auto const &sensor : this->dataPtr->sensors)
  {
    event::Events::removeSensor(sensor);
  }

  this->dataPtr->sensors.clear();

  // Clean up visuals
  // FIXME: Do we really need to send 2 msgs to delete a visual?!
  if (this->visPub && this->requestPub)
  {
    for (auto iter : this->visuals)
    {
      auto deleteMsg = msgs::CreateRequest("entity_delete",
          std::to_string(iter.second.id()));
      this->requestPub->Publish(*deleteMsg, true);
      delete deleteMsg;

      msgs::Visual msg;
      msg.set_name(iter.second.name());
      msg.set_id(iter.second.id());
      if (this->parent)
      {
        msg.set_parent_name(this->parent->GetScopedName());
        msg.set_parent_id(this->parent->GetId());
      }
      else
      {
        msg.set_parent_name("");
        msg.set_parent_id(0);
      }
      msg.set_delete_me(true);
      this->visPub->Publish(msg);
    }
  }
  this->visuals.clear();

#ifdef HAVE_OPENAL
  if (this->world && this->world->Physics() &&
      this->world->Physics()->GetContactManager())
  {
    this->world->Physics()->GetContactManager()->RemoveFilter(
        this->GetScopedName() + "/audio_collision");
  }
  this->dataPtr->audioContactsSub.reset();
  this->dataPtr->audioSink.reset();
  this->dataPtr->audioSources.clear();
#endif

  // Clean transport
  {
    this->dataPtr->dataPub.reset();
    this->visPub.reset();

    this->dataPtr->wrenchSub.reset();
  }
  this->connections.clear();

  delete this->dataPtr->publishDataMutex;
  this->dataPtr->publishDataMutex = NULL;

  Entity::Fini();
}

//////////////////////////////////////////////////
void Link::Reset()
{
  // resets pose
  Entity::Reset();

  // resets velocity, acceleration, wrench
  this->ResetPhysicsStates();
}

//////////////////////////////////////////////////
void Link::ResetPhysicsStates()
{
  this->SetAngularVel(ignition::math::Vector3d::Zero);
  this->SetLinearVel(ignition::math::Vector3d::Zero);
  this->SetForce(ignition::math::Vector3d::Zero);
  this->SetTorque(ignition::math::Vector3d::Zero);
}

//////////////////////////////////////////////////
void Link::UpdateParameters(sdf::ElementPtr _sdf)
{
  Entity::UpdateParameters(_sdf);

  if (this->sdf->HasElement("inertial"))
  {
    sdf::ElementPtr inertialElem = this->sdf->GetElement("inertial");
    this->inertial->UpdateParameters(inertialElem);
  }

  this->sdf->GetElement("gravity")->GetValue()->SetUpdateFunc(
      std::bind(&Link::GetGravityMode, this));
  this->sdf->GetElement("enable_wind")->GetValue()->SetUpdateFunc(
      std::bind(&Link::WindMode, this));
  this->sdf->GetElement("kinematic")->GetValue()->SetUpdateFunc(
      std::bind(&Link::GetKinematic, this));

  if (this->sdf->Get<bool>("gravity") != this->GetGravityMode())
    this->SetGravityMode(this->sdf->Get<bool>("gravity"));

  this->SetWindMode(this->sdf->Get<bool>("enable_wind"));

  // before loading child collision, we have to figure out if
  // selfCollide is true and modify parent class Entity so this
  // body has its own spaceId
  this->SetSelfCollide(this->sdf->Get<bool>("self_collide"));

  // TODO: this shouldn't be in the physics sim
  if (this->sdf->HasElement("visual"))
  {
    sdf::ElementPtr visualElem = this->sdf->GetElement("visual");
    while (visualElem)
    {
      // TODO: Update visuals properly
      msgs::Visual msg = msgs::VisualFromSDF(visualElem);

      msg.set_name(this->GetScopedName() + "::" + msg.name());
      msg.set_parent_name(this->GetScopedName());
      msg.set_is_static(this->IsStatic());
      msg.set_type(msgs::Visual::VISUAL);

      this->visPub->Publish(msg);

      visualElem = visualElem->GetNextElement("visual");
    }
  }

  if (this->sdf->HasElement("collision"))
  {
    sdf::ElementPtr collisionElem = this->sdf->GetElement("collision");
    while (collisionElem)
    {
      CollisionPtr collision = boost::dynamic_pointer_cast<Collision>(
          this->GetChild(collisionElem->Get<std::string>("name")));

      if (collision)
        collision->UpdateParameters(collisionElem);
      collisionElem = collisionElem->GetNextElement("collision");
    }
  }

  // Update the battery information
  if (this->sdf->HasElement("battery"))
  {
    sdf::ElementPtr batteryElem = this->sdf->GetElement("battery");
    while (batteryElem)
    {
      common::BatteryPtr battery = this->Battery(
          batteryElem->Get<std::string>("name"));

      if (battery)
        battery->UpdateParameters(batteryElem);
      batteryElem = batteryElem->GetNextElement("battery");
    }
  }
}

//////////////////////////////////////////////////
void Link::SetCollideMode(const std::string &_mode)
{
  unsigned int categoryBits;
  unsigned int collideBits;

  if (_mode == "all")
  {
    categoryBits =  GZ_ALL_COLLIDE;
    collideBits =  GZ_ALL_COLLIDE;
  }
  else if (_mode == "none")
  {
    categoryBits =  GZ_NONE_COLLIDE;
    collideBits =  GZ_NONE_COLLIDE;
  }
  else if (_mode == "sensors")
  {
    categoryBits = GZ_SENSOR_COLLIDE;
    collideBits = ~GZ_SENSOR_COLLIDE;
  }
  else if (_mode == "fixed")
  {
    categoryBits = GZ_FIXED_COLLIDE;
    collideBits = ~GZ_FIXED_COLLIDE;
  }
  else if (_mode == "ghost")
  {
    categoryBits = GZ_GHOST_COLLIDE;
    collideBits = ~GZ_GHOST_COLLIDE;
  }
  else
  {
    gzerr << "Unknown collide mode[" << _mode << "]\n";
    return;
  }

  for (Collision_V::iterator iter = this->dataPtr->collisions.begin();
       iter != this->dataPtr->collisions.end(); ++iter)
  {
    if ((*iter))
    {
      (*iter)->SetCategoryBits(categoryBits);
      (*iter)->SetCollideBits(collideBits);
    }
  }
}

//////////////////////////////////////////////////
bool Link::GetSelfCollide() const
{
  GZ_ASSERT(this->sdf != NULL, "Link sdf member is NULL");
  if (this->sdf->HasElement("self_collide"))
    return this->sdf->Get<bool>("self_collide");
  else
    return false;
}

//////////////////////////////////////////////////
void Link::SetLaserRetro(float _retro)
{
  for (Collision_V::iterator iter = this->dataPtr->collisions.begin();
       iter != this->dataPtr->collisions.end(); ++iter)
  {
    (*iter)->SetLaserRetro(_retro);
  }
}

//////////////////////////////////////////////////
void Link::Update(const common::UpdateInfo & /*_info*/)
{
#ifdef HAVE_OPENAL
  if (this->dataPtr->audioSink)
  {
    this->dataPtr->audioSink->SetPose(this->WorldPose());
    this->dataPtr->audioSink->SetVelocity(this->WorldLinearVel());
  }

  // Update all the audio sources
  for (std::vector<util::OpenALSourcePtr>::iterator iter =
      this->dataPtr->audioSources.begin(); iter !=
      this->dataPtr->audioSources.end(); ++iter)
  {
    (*iter)->SetPose(this->WorldPose());
    (*iter)->SetVelocity(this->WorldLinearVel());
  }
#endif

  // FIXME: race condition on factory-based model loading!!!!!
   /*if (this->GetEnabled() != this->dataPtr->enabled)
   {
     this->dataPtr->enabled = this->GetEnabled();
     this->dataPtr->enabledSignal(this->dataPtr->enabled);
   }*/

  if (!this->IsStatic() && !this->dataPtr->wrenchMsgs.empty())
  {
    std::vector<msgs::Wrench> messages;
    {
      std::lock_guard<std::mutex> lock(this->dataPtr->wrenchMsgMutex);
      messages = this->dataPtr->wrenchMsgs;
      this->dataPtr->wrenchMsgs.clear();
    }

    for (auto it : messages)
    {
      this->ProcessWrenchMsg(it);
    }
  }

  // Update the batteries.
  for (auto &battery : this->dataPtr->batteries)
  {
    battery->Update();
  }
}

//////////////////////////////////////////////////
void Link::UpdateWind(const common::UpdateInfo & /*_info*/)
{
  this->dataPtr->windLinearVel = this->world->Wind().WorldLinearVel(this);
}

/////////////////////////////////////////////////
Joint_V Link::GetParentJoints() const
{
  return this->dataPtr->parentJoints;
}

/////////////////////////////////////////////////
Joint_V Link::GetChildJoints() const
{
  return this->dataPtr->childJoints;
}

/////////////////////////////////////////////////
Link_V Link::GetChildJointsLinks() const
{
  Link_V links;
  for (std::vector<JointPtr>::const_iterator iter =
      this->dataPtr->childJoints.begin();
      iter != this->dataPtr->childJoints.end(); ++iter)
  {
    if ((*iter)->GetChild())
      links.push_back((*iter)->GetChild());
  }
  return links;
}

/////////////////////////////////////////////////
Link_V Link::GetParentJointsLinks() const
{
  Link_V links;
  for (std::vector<JointPtr>::const_iterator iter =
      this->dataPtr->parentJoints.begin();
      iter != this->dataPtr->parentJoints.end(); ++iter)
  {
    if ((*iter)->GetParent())
      links.push_back((*iter)->GetParent());
  }
  return links;
}

//////////////////////////////////////////////////
void Link::LoadCollision(sdf::ElementPtr _sdf)
{
  CollisionPtr collision;
  std::string geomType =
    _sdf->GetElement("geometry")->GetFirstElement()->GetName();

  if (geomType == "heightmap" || geomType == "map")
    this->SetStatic(true);

  collision = this->GetWorld()->Physics()->CreateCollision(geomType,
      boost::static_pointer_cast<Link>(shared_from_this()));

  if (!collision)
    gzthrow("Unknown Collisionetry Type[" + geomType + "]");

  collision->Load(_sdf);
}

//////////////////////////////////////////////////
CollisionPtr Link::GetCollisionById(unsigned int _id) const
{
  return boost::dynamic_pointer_cast<Collision>(this->GetById(_id));
}

//////////////////////////////////////////////////
CollisionPtr Link::GetCollision(const std::string &_name)
{
  CollisionPtr result;
  Base_V::const_iterator biter;
  for (biter = this->children.begin(); biter != this->children.end(); ++biter)
  {
    if ((*biter)->GetName() == _name)
    {
      result = boost::dynamic_pointer_cast<Collision>(*biter);
      break;
    }
  }

  return result;
}

//////////////////////////////////////////////////
Collision_V Link::GetCollisions() const
{
  return this->dataPtr->collisions;
}

//////////////////////////////////////////////////
CollisionPtr Link::GetCollision(unsigned int _index) const
{
  CollisionPtr collision;
  if (_index <= this->GetChildCount())
    collision = boost::static_pointer_cast<Collision>(this->GetChild(_index));
  else
    gzerr << "Index is out of range\n";

  return collision;
}

//////////////////////////////////////////////////
void Link::SetLinearAccel(const ignition::math::Vector3d &_accel)
{
  gzwarn << "Link::SetLinearAccel() is deprecated and has "
         << "no effect. Use Link::SetForce() instead.\n";
  this->SetEnabled(true);
  this->linearAccel = _accel;
}

//////////////////////////////////////////////////
void Link::SetAngularAccel(const ignition::math::Vector3d &_accel)
{
  gzwarn << "Link::SetAngularAccel() is deprecated and has "
         << "no effect. Use Link::SetTorque() instead.\n";
  this->SetEnabled(true);
  this->angularAccel = _accel;
}

//////////////////////////////////////////////////
ignition::math::Pose3d Link::WorldCoGPose() const
{
  ignition::math::Pose3d pose = this->WorldPose();
  pose.Pos() += pose.Rot().RotateVector(this->inertial->CoG());
  return pose;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Link::RelativeLinearVel() const
{
  return this->WorldPose().Rot().RotateVectorReverse(this->WorldLinearVel());
}

//////////////////////////////////////////////////
ignition::math::Vector3d Link::RelativeAngularVel() const
{
  return this->WorldPose().Rot().RotateVectorReverse(this->WorldAngularVel());
}

//////////////////////////////////////////////////
ignition::math::Vector3d Link::RelativeLinearAccel() const
{
  return this->RelativeForce() / this->inertial->Mass();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Link::WorldLinearAccel() const
{
  // Developer note (MXG): The meaning of WorldForce seems to vary between
  // engines. In particular, ODE sees it as the net force on a body, whereas
  // DART sees it as the net of the external forces (which excludes internal
  // forces caused by joint constraints). These will be equivalent for bodies
  // that have no joints, but they will not be equivalent in general.
  return this->WorldForce() / this->inertial->Mass();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Link::RelativeAngularAccel() const
{
  return this->WorldPose().Rot().RotateVectorReverse(this->WorldAngularAccel());
}

//////////////////////////////////////////////////
ignition::math::Vector3d Link::WorldAngularAccel() const
{
  // I: inertia matrix in world frame
  // T: sum of external torques in world frame
  // L: angular momentum of CoG in world frame
  // w: angular velocity in world frame
  // return I^-1 * (T - w x L)
  return this->WorldInertiaMatrix().Inverse() *
    (this->WorldTorque() -
     this->WorldAngularVel().Cross(this->WorldAngularMomentum()));
}

//////////////////////////////////////////////////
ignition::math::Vector3d Link::WorldAngularMomentum() const
{
  return this->WorldInertiaMatrix() * this->WorldAngularVel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Link::RelativeForce() const
{
  return this->WorldPose().Rot().RotateVectorReverse(this->WorldForce());
}

//////////////////////////////////////////////////
ignition::math::Vector3d Link::RelativeTorque() const
{
  return this->WorldPose().Rot().RotateVectorReverse(this->WorldTorque());
}

//////////////////////////////////////////////////
ModelPtr Link::GetModel() const
{
  return boost::dynamic_pointer_cast<Model>(this->GetParent());
}

//////////////////////////////////////////////////
ignition::math::Box Link::BoundingBox() const
{
  ignition::math::Box box;

  box.Min().Set(ignition::math::MAX_D, ignition::math::MAX_D,
      ignition::math::MAX_D);
  box.Max().Set(-ignition::math::MAX_D, -ignition::math::MAX_D,
     -ignition::math::MAX_D);

  for (Collision_V::const_iterator iter = this->dataPtr->collisions.begin();
       iter != this->dataPtr->collisions.end(); ++iter)
  {
    box += (*iter)->BoundingBox();
  }

  return box;
}

//////////////////////////////////////////////////
void Link::SetWindMode(const bool _mode)
{
  this->sdf->GetElement("enable_wind")->Set(_mode);

  if (!this->WindMode() && this->dataPtr->updateConnection)
    this->SetWindEnabled(false);
  else if (this->WindMode() && !this->dataPtr->updateConnection)
    this->SetWindEnabled(true);
}

/////////////////////////////////////////////////
void Link::SetWindEnabled(const bool _enable)
{
  if (_enable)
  {
    this->dataPtr->updateConnection = event::Events::ConnectWorldUpdateBegin(
        std::bind(&Link::UpdateWind, this, std::placeholders::_1));
  }
  else
  {
    this->dataPtr->updateConnection.reset();
    // Make sure wind velocity is null
    this->dataPtr->windLinearVel.Set(0, 0, 0);
  }
}

//////////////////////////////////////////////////
const ignition::math::Vector3d Link::WorldWindLinearVel() const
{
  return this->dataPtr->windLinearVel;
}

//////////////////////////////////////////////////
bool Link::WindMode() const
{
  return this->sdf->Get<bool>("enable_wind");
}

//////////////////////////////////////////////////
bool Link::SetSelected(bool _s)
{
  Entity::SetSelected(_s);

  if (_s == false)
    this->SetEnabled(true);

  return true;
}

//////////////////////////////////////////////////
void Link::SetInertial(const InertialPtr &/*_inertial*/)
{
  gzwarn << "Link::SetMass is empty\n";
}

//////////////////////////////////////////////////
ignition::math::Pose3d Link::WorldInertialPose() const
{
  ignition::math::Pose3d inertialPose;
  if (this->inertial)
    inertialPose = this->inertial->Pose();
  return inertialPose + this->WorldPose();
}

//////////////////////////////////////////////////
ignition::math::Matrix3d Link::WorldInertiaMatrix() const
{
  ignition::math::Matrix3d moi;
  if (this->inertial)
  {
    ignition::math::Vector3d pos = this->inertial->Pose().Pos();
    ignition::math::Quaterniond rot = this->WorldPose().Rot().Inverse();
    moi = this->inertial->MOI(ignition::math::Pose3d(pos, rot));
  }
  return moi;
}

//////////////////////////////////////////////////
void Link::AddParentJoint(JointPtr _joint)
{
  this->dataPtr->parentJoints.push_back(_joint);
}

//////////////////////////////////////////////////
void Link::AddChildJoint(JointPtr _joint)
{
  this->dataPtr->childJoints.push_back(_joint);
}

//////////////////////////////////////////////////
void Link::RemoveParentJoint(const std::string &_jointName)
{
  for (std::vector<JointPtr>::iterator iter =
      this->dataPtr->parentJoints.begin();
      iter != this->dataPtr->parentJoints.end(); ++iter)
  {
    /// @todo: can we assume there are no repeats?
    if ((*iter)->GetName() == _jointName)
    {
      this->dataPtr->parentJoints.erase(iter);
      break;
    }
  }
}

//////////////////////////////////////////////////
void Link::RemoveChildJoint(const std::string &_jointName)
{
  for (std::vector<JointPtr>::iterator iter =
      this->dataPtr->childJoints.begin();
      iter != this->dataPtr->childJoints.end(); ++iter)
  {
    /// @todo: can we assume there are no repeats?
    if ((*iter)->GetName() == _jointName)
    {
      this->dataPtr->childJoints.erase(iter);
      break;
    }
  }
}

//////////////////////////////////////////////////
void Link::FillMsg(msgs::Link &_msg)
{
  ignition::math::Pose3d relPose = this->RelativePose();

  _msg.set_id(this->GetId());
  _msg.set_name(this->GetScopedName());
  _msg.set_self_collide(this->GetSelfCollide());
  _msg.set_gravity(this->GetGravityMode());
  _msg.set_enable_wind(this->WindMode());
  _msg.set_kinematic(this->GetKinematic());
  _msg.set_enabled(this->GetEnabled());
  msgs::Set(_msg.mutable_pose(), relPose);

  // The visual msgs name might not have been set if the link was created
  // dynamically without using SDF.
  if (!this->visualMsg->has_name())
    this->visualMsg->set_name(this->GetScopedName());

  msgs::Set(this->visualMsg->mutable_pose(), relPose);
  _msg.add_visual()->CopyFrom(*this->visualMsg);

  _msg.mutable_inertial()->set_mass(this->inertial->Mass());
  _msg.mutable_inertial()->set_ixx(this->inertial->IXX());
  _msg.mutable_inertial()->set_ixy(this->inertial->IXY());
  _msg.mutable_inertial()->set_ixz(this->inertial->IXZ());
  _msg.mutable_inertial()->set_iyy(this->inertial->IYY());
  _msg.mutable_inertial()->set_iyz(this->inertial->IYZ());
  _msg.mutable_inertial()->set_izz(this->inertial->IZZ());
  msgs::Set(_msg.mutable_inertial()->mutable_pose(), this->inertial->Pose());

  for (auto &child : this->children)
  {
    if (child->HasType(Base::COLLISION))
    {
      CollisionPtr collision = boost::static_pointer_cast<Collision>(child);
      collision->FillMsg(*_msg.add_collision());
    }
  }

  // Add in the sensor data.
  if (this->sdf->HasElement("sensor"))
  {
    sdf::ElementPtr sensorElem = this->sdf->GetElement("sensor");
    while (sensorElem)
    {
      msgs::Sensor *msg = _msg.add_sensor();
      msg->CopyFrom(msgs::SensorFromSDF(sensorElem));
      msg->set_parent(this->GetScopedName());
      msg->set_parent_id(this->GetId());
      sensorElem = sensorElem->GetNextElement("sensor");
    }
  }

  if (this->visuals.empty())
    this->ParseVisuals();
  else
    this->UpdateVisualMsg();

  for (Visuals_M::iterator iter = this->visuals.begin();
      iter != this->visuals.end(); ++iter)
  {
    msgs::Visual *vis = _msg.add_visual();
    vis->CopyFrom(iter->second);
  }

  if (this->sdf->HasElement("projector"))
  {
    sdf::ElementPtr elem = this->sdf->GetElement("projector");

    msgs::Projector *proj = _msg.add_projector();
    proj->set_name(
        this->GetScopedName() + "::" + elem->Get<std::string>("name"));
    proj->set_texture(elem->Get<std::string>("texture"));
    proj->set_fov(elem->Get<double>("fov"));
    proj->set_near_clip(elem->Get<double>("near_clip"));
    proj->set_far_clip(elem->Get<double>("far_clip"));
    msgs::Set(proj->mutable_pose(), elem->Get<ignition::math::Pose3d>("pose"));
  }

  if (this->IsCanonicalLink())
    _msg.set_canonical(true);

  // Fill message with battery information
  for (auto &battery : this->dataPtr->batteries)
  {
    msgs::Battery *bat = _msg.add_battery();
    bat->set_name(battery->Name());
    bat->set_voltage(battery->Voltage());
  }

  for (auto &light : this->dataPtr->lights)
  {
    msgs::Light *lightMsg = _msg.add_light();
    light->FillMsg(*lightMsg);
  }
}

//////////////////////////////////////////////////
void Link::ProcessMsg(const msgs::Link &_msg)
{
  if (_msg.id() != this->GetId())
  {
    return;
  }

  this->SetName(_msg.name());

  if (_msg.has_self_collide())
    this->SetSelfCollide(_msg.self_collide());
  if (_msg.has_gravity())
  {
    this->SetGravityMode(_msg.gravity());
    this->SetEnabled(true);
  }
  if (_msg.has_enable_wind())
    this->SetWindMode(_msg.enable_wind());
  if (_msg.has_kinematic())
  {
    this->SetKinematic(_msg.kinematic());
    this->SetEnabled(true);
  }
  if (_msg.has_inertial())
  {
    this->inertial->ProcessMsg(_msg.inertial());
    this->SetEnabled(true);
    // Only update the Center of Mass if object is dynamic
    if (!this->GetKinematic())
      this->UpdateMass();
  }

  if (_msg.has_pose())
  {
    this->SetEnabled(true);
    this->SetRelativePose(msgs::ConvertIgn(_msg.pose()));
  }

  for (int i = 0; i < _msg.collision_size(); i++)
  {
    CollisionPtr coll = this->GetCollisionById(_msg.collision(i).id());
    if (coll)
      coll->ProcessMsg(_msg.collision(i));
  }
  if (_msg.collision_size()>0)
    this->UpdateSurface();
}

//////////////////////////////////////////////////
unsigned int Link::GetSensorCount() const
{
  return this->dataPtr->sensors.size();
}

//////////////////////////////////////////////////
std::string Link::GetSensorName(unsigned int _i) const
{
  if (_i < this->dataPtr->sensors.size())
    return this->dataPtr->sensors[_i];

  return std::string();
}

//////////////////////////////////////////////////
void Link::AttachStaticModel(ModelPtr &_model,
    const ignition::math::Pose3d &_offset)
{
  if (!_model->IsStatic())
  {
    gzerr << "AttachStaticModel requires a static model\n";
    return;
  }

  this->dataPtr->attachedModels.push_back(_model);
  this->attachedModelsOffset.push_back(_offset);
}

//////////////////////////////////////////////////
void Link::DetachStaticModel(const std::string &_modelName)
{
  for (unsigned int i = 0; i < this->dataPtr->attachedModels.size(); i++)
  {
    if (this->dataPtr->attachedModels[i]->GetName() == _modelName)
    {
      this->dataPtr->attachedModels.erase(
          this->dataPtr->attachedModels.begin()+i);
      this->attachedModelsOffset.erase(
          this->attachedModelsOffset.begin()+i);
      break;
    }
  }
}

//////////////////////////////////////////////////
void Link::DetachAllStaticModels()
{
  this->dataPtr->attachedModels.clear();
  this->attachedModelsOffset.clear();
}

//////////////////////////////////////////////////
void Link::OnPoseChange()
{
  ignition::math::Pose3d p;
  for (unsigned int i = 0; i < this->dataPtr->attachedModels.size(); i++)
  {
    p = this->WorldPose();
    p.Pos() += this->attachedModelsOffset[i].Pos();
    p.Rot() = p.Rot() * this->attachedModelsOffset[i].Rot();

    this->dataPtr->attachedModels[i]->SetWorldPose(p, true);
  }
}

//////////////////////////////////////////////////
void Link::SetState(const LinkState &_state)
{
  this->SetWorldPose(_state.Pose());
  this->SetLinearVel(_state.Velocity().Pos());
  this->SetAngularVel(_state.Velocity().Rot().Euler());
  this->SetForce(_state.Wrench().Pos());
  this->SetTorque(_state.Wrench().Rot().Euler());

  /*
  for (unsigned int i = 0; i < _state.GetCollisionStateCount(); ++i)
  {
    CollisionState collisionState = _state.GetCollisionState(i);
    CollisionPtr collision = this->GetCollision(collisionState.GetName());
    if (collision)
      collision->SetState(collisionState);
    else
      gzerr << "Unable to find collision[" << collisionState.GetName() << "]\n";
  }*/
}

/////////////////////////////////////////////////
double Link::GetLinearDamping() const
{
  if (this->sdf->HasElement("velocity_decay"))
    return this->sdf->GetElement("velocity_decay")->Get<double>("linear");
  else
    return 0.0;
}

/////////////////////////////////////////////////
double Link::GetAngularDamping() const
{
  if (this->sdf->HasElement("velocity_decay"))
    return this->sdf->GetElement("velocity_decay")->Get<double>("angular");
  else
    return 0.0;
}

/////////////////////////////////////////////////
void Link::SetKinematic(const bool &/*_kinematic*/)
{
}

/////////////////////////////////////////////////
void Link::SetPublishData(bool _enable)
{
  // Skip if we're trying to disable after the publisher has already been
  // cleared
  if (!_enable && !this->dataPtr->dataPub)
    return;

  {
    std::lock_guard<std::recursive_mutex> lock(
        *this->dataPtr->publishDataMutex);
    if (this->dataPtr->publishData == _enable)
      return;

    this->dataPtr->publishData = _enable;
  }
  if (_enable)
  {
    std::string topic = "~/" + this->GetScopedName();
    this->dataPtr->dataPub = this->node->Advertise<msgs::LinkData>(topic);
    this->connections.push_back(
      event::Events::ConnectWorldUpdateEnd(
        std::bind(&Link::PublishData, this)));
  }
  else
  {
    this->dataPtr->dataPub.reset();
    // Do we want to clear all of them though?
    this->connections.clear();
  }
}

/////////////////////////////////////////////////
void Link::PublishData()
{
  if (this->dataPtr->publishData && this->dataPtr->dataPub->HasConnections())
  {
    msgs::Set(this->dataPtr->linkDataMsg.mutable_time(),
        this->world->SimTime());
    this->dataPtr->linkDataMsg.set_name(this->GetScopedName());
    msgs::Set(this->dataPtr->linkDataMsg.mutable_linear_velocity(),
        this->WorldLinearVel());
    msgs::Set(this->dataPtr->linkDataMsg.mutable_angular_velocity(),
        this->WorldAngularVel());
    this->dataPtr->dataPub->Publish(this->dataPtr->linkDataMsg);
  }
}

//////////////////////////////////////////////////
common::BatteryPtr Link::Battery(const std::string &_name) const
{
  common::BatteryPtr result;

  for (auto &battery : this->dataPtr->batteries)
  {
    if (battery->Name() == _name)
    {
      result = battery;
      break;
    }
  }

  return result;
}

/////////////////////////////////////////////////
common::BatteryPtr Link::Battery(const size_t _index) const
{
  if (_index < this->dataPtr->batteries.size())
    return this->dataPtr->batteries[_index];
  else
    return common::BatteryPtr();
}

/////////////////////////////////////////////////
size_t Link::BatteryCount() const
{
  return this->dataPtr->batteries.size();
}

//////////////////////////////////////////////////
bool Link::VisualId(const std::string &_visName, uint32_t &_visualId) const
{
  for (auto &iter : this->visuals)
  {
    if (iter.second.name() == _visName ||
        iter.second.name() == this->GetScopedName() + "::" + _visName)
    {
      _visualId = iter.first;
      return true;
    }
  }
  gzerr << "Trying to get unique ID of visual from invalid visual name["
        << _visName << "] for link [" << this->GetScopedName() << "]\n";
  return false;
}

//////////////////////////////////////////////////
bool Link::VisualPose(const uint32_t _id, ignition::math::Pose3d &_pose) const
{
  auto iter = this->visuals.find(_id);
  if (iter == this->visuals.end())
  {
    gzerr << "Trying to get pose of visual from invalid visual id[" << _id
          << "] for link [" << this->GetScopedName() << "]\n";
    return false;
  }
  const msgs::Visual &msg = iter->second;
  if (msg.has_pose())
  {
    _pose = msgs::ConvertIgn(msg.pose());
  }
  else
  {
    // Pose wasn't specified on SDF, use default value
    _pose = ignition::math::Pose3d::Zero;
  }
  return true;
}

//////////////////////////////////////////////////
bool Link::SetVisualPose(const uint32_t _id,
                         const ignition::math::Pose3d &_pose)
{
  auto iter = this->visuals.find(_id);
  if (iter == this->visuals.end())
  {
    gzerr << "Trying to set pose of visual from invalid visual id[" << _id
          << "] for link [" << this->GetScopedName() << "]\n";
    return false;
  }
  msgs::Visual &msg = iter->second;
  msgs::Set(msg.mutable_pose(), _pose);
  std::string linkName = this->GetScopedName();
  if (this->sdf->HasElement("visual"))
  {
    sdf::ElementPtr visualElem = this->sdf->GetElement("visual");
    while (visualElem)
    {
      std::string visName = linkName + "::" +
        visualElem->Get<std::string>("name");

      // update visual msg if it exists
      if (msg.name() == visName)
      {
        visualElem->GetElement("pose")->Set(_pose);
        break;
      }

      visualElem = visualElem->GetNextElement("visual");
    }
  }
  msgs::Visual visual;
  visual.set_name(msg.name());
  visual.set_id(_id);
  visual.set_parent_name(linkName);
  visual.set_parent_id(this->GetId());
  msgs::Set(visual.mutable_pose(), _pose);
  this->visPub->Publish(visual);
  return true;
}

//////////////////////////////////////////////////
void Link::OnCollision(ConstContactsPtr &_msg)
{
  std::string collisionName1;
  std::string collisionName2;
  std::string::size_type pos1, pos2;

  for (int i = 0; i < _msg->contact_size(); ++i)
  {
    collisionName1 = _msg->contact(i).collision1();
    collisionName2 = _msg->contact(i).collision2();
    pos1 = collisionName1.rfind("::");
    pos2 = collisionName2.rfind("::");

    GZ_ASSERT(pos1 != std::string::npos, "Invalid collision name");
    GZ_ASSERT(pos2 != std::string::npos, "Invalid collision name");

    collisionName1 = collisionName1.substr(pos1+2);
    collisionName2 = collisionName2.substr(pos2+2);

#ifdef HAVE_OPENAL
    for (std::vector<util::OpenALSourcePtr>::iterator iter =
        this->dataPtr->audioSources.begin();
        iter != this->dataPtr->audioSources.end(); ++iter)
    {
      if ((*iter)->HasCollisionName(collisionName1) ||
          (*iter)->HasCollisionName(collisionName2))
        (*iter)->Play();
    }
#endif
  }
}

/////////////////////////////////////////////////
void Link::ParseVisuals()
{
  this->UpdateVisualMsg();

  for (auto const it : this->visuals)
    this->visPub->Publish(it.second);
}

/////////////////////////////////////////////////
void Link::RemoveChild(EntityPtr _child)
{
  if (_child->HasType(COLLISION))
  {
    this->RemoveCollision(_child->GetScopedName());
  }

  Entity::RemoveChild(_child->GetId());

  this->SetEnabled(true);
}

/////////////////////////////////////////////////
void Link::RemoveCollision(const std::string &_name)
{
  for (Collision_V::iterator iter = this->dataPtr->collisions.begin();
       iter != this->dataPtr->collisions.end(); ++iter)
  {
    if ((*iter)->GetName() == _name || (*iter)->GetScopedName() == _name)
    {
      this->dataPtr->collisions.erase(iter);
      break;
    }
  }
}

/////////////////////////////////////////////////
void Link::SetScale(const ignition::math::Vector3d &_scale)
{
  Base_V::const_iterator biter;
  for (biter = this->children.begin(); biter != this->children.end(); ++biter)
  {
    if ((*biter)->HasType(Base::COLLISION))
    {
      boost::static_pointer_cast<Collision>(*biter)->SetScale(_scale);
    }
  }

  // update the visual sdf to ensure cloning and saving has the correct values.
  this->UpdateVisualGeomSDF(_scale);

  this->scale = _scale;
}

//////////////////////////////////////////////////
void Link::UpdateVisualGeomSDF(const ignition::math::Vector3d &_scale)
{
  // TODO: this shouldn't be in the physics sim
  if (this->sdf->HasElement("visual"))
  {
    sdf::ElementPtr visualElem = this->sdf->GetElement("visual");
    while (visualElem)
    {
      sdf::ElementPtr geomElem = visualElem->GetElement("geometry");

      if (geomElem->HasElement("box"))
      {
        ignition::math::Vector3d size =
            geomElem->GetElement("box")->Get<ignition::math::Vector3d>("size");
        geomElem->GetElement("box")->GetElement("size")->Set(
            _scale/this->scale*size);
      }
      else if (geomElem->HasElement("sphere"))
      {
        // update radius the same way as collision shapes
        double radius = geomElem->GetElement("sphere")->Get<double>("radius");
        double newRadius = _scale.Max();
        double oldRadius = std::max(this->scale.Z(),
            std::max(this->scale.X(), this->scale.Y()));
        geomElem->GetElement("sphere")->GetElement("radius")->Set(
            newRadius/oldRadius*radius);
      }
      else if (geomElem->HasElement("cylinder"))
      {
        // update radius the same way as collision shapes
        double radius = geomElem->GetElement("cylinder")->Get<double>("radius");
        double newRadius = std::max(_scale.X(), _scale.Y());
        double oldRadius = std::max(this->scale.X(), this->scale.Y());

        double length = geomElem->GetElement("cylinder")->Get<double>("length");
        geomElem->GetElement("cylinder")->GetElement("radius")->Set(
            newRadius/oldRadius*radius);
        geomElem->GetElement("cylinder")->GetElement("length")->Set(
            _scale.Z()/this->scale.Z()*length);
      }
      else if (geomElem->HasElement("mesh"))
        geomElem->GetElement("mesh")->GetElement("scale")->Set(_scale);

      visualElem = visualElem->GetNextElement("visual");
    }
  }
}

//////////////////////////////////////////////////
void Link::UpdateVisualMsg()
{
  // TODO: this shouldn't be in the physics sim
  if (this->sdf->HasElement("visual"))
  {
    sdf::ElementPtr visualElem = this->sdf->GetElement("visual");
    while (visualElem)
    {
      msgs::Visual msg = msgs::VisualFromSDF(visualElem);

      bool newVis = true;
      std::string linkName = this->GetScopedName();

      // update visual msg if it exists
      for (auto &iter : this->visuals)
      {
        std::string visName = linkName + "::" +
            visualElem->Get<std::string>("name");
        if (iter.second.name() == visName)
        {
          iter.second.mutable_geometry()->CopyFrom(msg.geometry());
          newVis = false;
          break;
        }
      }

      // add to visual msgs if not found.
      if (newVis)
      {
        std::string visName = this->GetScopedName() + "::" + msg.name();
        msg.set_name(visName);
        msg.set_id(physics::getUniqueId());
        msg.set_parent_name(this->GetScopedName());
        msg.set_parent_id(this->GetId());
        msg.set_is_static(this->IsStatic());
        msg.set_type(msgs::Visual::VISUAL);
        msgs::Set(msg.mutable_scale(), this->scale);

        auto iter = this->visuals.find(msg.id());
        if (iter != this->visuals.end())
          gzthrow(std::string("Duplicate visual name[")+msg.name()+"]\n");
        this->visuals[msg.id()] = msg;
      }

      visualElem = visualElem->GetNextElement("visual");
    }
  }
}

/////////////////////////////////////////////////
double Link::GetWorldEnergyPotential() const
{
  // compute gravitational potential energy for link CG location
  // use origin as reference position
  // E = -m g^T z
  double m = this->GetInertial()->Mass();
  auto g = this->GetWorld()->Gravity();
  auto z = this->WorldCoGPose().Pos();
  return -m * g.Dot(z);
}

/////////////////////////////////////////////////
double Link::GetWorldEnergyKinetic() const
{
  double energy = 0.0;

  // compute linear kinetic energy
  // E = 1/2 m v^T v
  {
    double m = this->GetInertial()->Mass();
    ignition::math::Vector3d v = this->WorldCoGLinearVel();
    energy += 0.5 * m * v.Dot(v);
  }

  // compute angular kinetic energy
  // E = 1/2 w^T I w
  {
    ignition::math::Vector3d w = this->WorldAngularVel();
    ignition::math::Matrix3d inertia = this->WorldInertiaMatrix();
    energy += 0.5 * w.Dot(inertia * w);
  }

  return energy;
}

/////////////////////////////////////////////////
double Link::GetWorldEnergy() const
{
  return this->GetWorldEnergyPotential() + this->GetWorldEnergyKinetic();
}

/////////////////////////////////////////////////
void Link::MoveFrame(const ignition::math::Pose3d &_worldReferenceFrameSrc,
                     const ignition::math::Pose3d &_worldReferenceFrameDst,
                     const bool _preserveWorldVelocity)
{
  ignition::math::Pose3d targetWorldPose = (this->WorldPose() -
      _worldReferenceFrameSrc) + _worldReferenceFrameDst;
  this->SetWorldPose(targetWorldPose);
  if (!_preserveWorldVelocity)
  {
    this->SetWorldTwist(ignition::math::Vector3d::Zero,
      ignition::math::Vector3d::Zero);
  }
}

/////////////////////////////////////////////////
bool Link::FindAllConnectedLinksHelper(const LinkPtr &_originalParentLink,
  Link_V &_connectedLinks, bool _fistLink)
{
  // debug
  // std::string pn;
  // if (_originalParentLink) pn = _originalParentLink->GetName();
  // gzerr << "subsequent call to find connected links: "
  //       << " parent " << pn
  //       << " this link " << this->GetName() << "\n";

  // get all child joints from this link
  Link_V childLinks = this->GetChildJointsLinks();

  // gzerr << "debug: child links are: ";
  // for (Link_V::iterator li = childLinks.begin();
  //                       li != childLinks.end(); ++li)
  //   std::cout << (*li)->GetName() << " ";
  // std::cout << "\n";

  // loop through all joints where this link is a parent link of the joint
  for (Link_V::iterator li = childLinks.begin();
                        li != childLinks.end(); ++li)
  {
    // gzerr << "debug: checking " << (*li)->GetName() << "\n";

    // check child link of each child joint recursively
    if ((*li).get() == _originalParentLink.get())
    {
      // if parent is a child, failed search to find a nice subset of links
      gzdbg << "we have a loop! cannot find nice subset of connected links,"
            << " this link " << this->GetName() << " connects back to"
            << " parent " << _originalParentLink->GetName() << ".\n";
      _connectedLinks.clear();
      return false;
    }
    else if (this->ContainsLink(_connectedLinks, (*li)))
    {
      // do nothing
      // gzerr << "debug: do nothing with " << (*li)->GetName() << "\n";
    }
    else
    {
      // gzerr << "debug: add and recurse " << (*li)->GetName() << "\n";
      // add child link to list
      _connectedLinks.push_back((*li));

      // recursively check if child link has already been checked
      // if it returns false, it looped back to parent, mark flag and break
      // from current for-loop.
      if (!(*li)->FindAllConnectedLinksHelper(_originalParentLink,
        _connectedLinks))
      {
        // one of the recursed link is the parent link
        return false;
      }
    }
  }

  /// \todo: later we can optimize loop below by merging and using a flag.

  // search parents, but if this is the first search, keep going, otherwise
  // flag failure
  // get all parent joints from this link
  Link_V parentLinks = this->GetParentJointsLinks();

  // loop through all joints where this link is a parent link of the joint
  for (Link_V::iterator li = parentLinks.begin();
                        li != parentLinks.end(); ++li)
  {
    // check child link of each child joint recursively
    if ((*li).get() == _originalParentLink.get())
    {
      if (_fistLink)
      {
        // this is the first child link, simply skip if the parent is
        // the _originalParentLink
      }
      else
      {
        // if parent is a child, failed search to find a nice subset of links
        gzdbg << "we have a loop! cannot find nice subset of connected links,"
              << " this link " << this->GetName() << " connects back to"
              << " parent " << _originalParentLink->GetName() << ".\n";
        _connectedLinks.clear();
        return false;
      }
    }
    else if (this->ContainsLink(_connectedLinks, (*li)))
    {
      // do nothing
    }
    else
    {
      // add parent link to list
      _connectedLinks.push_back((*li));

      // recursively check if parent link has already been checked
      // if it returns false, it looped back to parent, mark flag and break
      // from current for-loop.
      if (!(*li)->FindAllConnectedLinksHelper(_originalParentLink,
        _connectedLinks))
      {
        // one of the recursed link is the parent link
        return false;
      }
    }
  }

  return true;
}

/////////////////////////////////////////////////
bool Link::ContainsLink(const Link_V &_vector, const LinkPtr &_value)
{
  for (Link_V::const_iterator iter = _vector.begin();
       iter != _vector.end(); ++iter)
  {
    if ((*iter).get() == _value.get())
      return true;
  }
  return false;
}

/////////////////////////////////////////////////
msgs::Visual Link::GetVisualMessage(const std::string &_name) const
{
  msgs::Visual result;

  Visuals_M::const_iterator iter;
  for (iter = this->visuals.begin(); iter != this->visuals.end(); ++iter)
    if (iter->second.name() == _name)
      break;

  if (iter != this->visuals.end())
    result = iter->second;

  return result;
}

//////////////////////////////////////////////////
void Link::SetStatic(const bool &_static)
{
  if (!_static && !this->dataPtr->wrenchSub)
  {
    std::string topicName = "~/" + this->GetScopedName() + "/wrench";
    boost::replace_all(topicName, "::", "/");
    this->dataPtr->wrenchSub =
        this->node->Subscribe(topicName, &Link::OnWrenchMsg, this);
  }
  else if (_static)
  {
    this->dataPtr->wrenchSub.reset();
  }

  Entity::SetStatic(_static);
}

//////////////////////////////////////////////////
void Link::OnWrenchMsg(ConstWrenchPtr &_msg)
{
  // Sanity check
  if (this->IsStatic())
  {
    gzerr << "Link [" << this->GetName() <<
        "] received a wrench message, but it is static." << std::endl;
    return;
  }

  std::lock_guard<std::mutex> lock(this->dataPtr->wrenchMsgMutex);
  this->dataPtr->wrenchMsgs.push_back(*_msg);
}

//////////////////////////////////////////////////
void Link::ProcessWrenchMsg(const msgs::Wrench &_msg)
{
  // Sanity check
  if (this->IsStatic())
  {
    gzerr << "Link [" << this->GetName() <<
        "] received a wrench message, but it is static." << std::endl;
    return;
  }

  ignition::math::Vector3d pos;
  if (_msg.has_force_offset())
  {
    pos = msgs::ConvertIgn(_msg.force_offset());
  }

  const ignition::math::Vector3d force = msgs::ConvertIgn(_msg.force());
  this->AddLinkForce(force, pos);

  const ignition::math::Vector3d torque = msgs::ConvertIgn(_msg.torque());
  this->AddRelativeTorque(torque);
}

//////////////////////////////////////////////////
void Link::LoadBattery(sdf::ElementPtr _sdf)
{
  common::BatteryPtr battery(new common::Battery());
  battery->Load(_sdf);
  this->dataPtr->batteries.push_back(battery);
}

/////////////////////////////////////////////////
const ignition::math::Vector3d Link::RelativeWindLinearVel() const
{
  return this->WorldPose().Rot().Inverse().RotateVector(
      this->dataPtr->windLinearVel);
}

/////////////////////////////////////////////////
void Link::RegisterIntrospectionItems()
{
  auto uri = this->URI();

  // Callbacks.
  auto fLinkPose = [this]()
  {
    return this->WorldPose();
  };

  auto fLinkLinVel = [this]()
  {
    return this->WorldLinearVel();
  };

  auto fLinkAngVel = [this]()
  {
    return this->WorldAngularVel();
  };

  auto fLinkLinAcc = [this]()
  {
    return this->WorldLinearAccel();
  };

  auto fLinkAngAcc = [this]()
  {
    return this->WorldAngularAccel();
  };

  // Register items.
  common::URI poseURI(uri);
  poseURI.Query().Insert("p", "pose3d/world_pose");
  this->introspectionItems.push_back(poseURI);
  gazebo::util::IntrospectionManager::Instance()->Register
      <ignition::math::Pose3d>(poseURI.Str(), fLinkPose);

  common::URI linVelURI(uri);
  linVelURI.Query().Insert("p", "vector3d/world_linear_velocity");
  this->introspectionItems.push_back(linVelURI);
  gazebo::util::IntrospectionManager::Instance()->Register
      <ignition::math::Vector3d>(linVelURI.Str(), fLinkLinVel);

  common::URI angVelURI(uri);
  angVelURI.Query().Insert("p", "vector3d/world_angular_velocity");
  this->introspectionItems.push_back(angVelURI);
  gazebo::util::IntrospectionManager::Instance()->Register
      <ignition::math::Vector3d>(angVelURI.Str(), fLinkAngVel);

  common::URI linAccURI(uri);
  linAccURI.Query().Insert("p", "vector3d/world_linear_acceleration");
  this->introspectionItems.push_back(linAccURI);
  gazebo::util::IntrospectionManager::Instance()->Register
      <ignition::math::Vector3d>(linAccURI.Str(), fLinkLinAcc);

  common::URI angAccURI(uri);
  angAccURI.Query().Insert("p", "vector3d/world_angular_acceleration");
  this->introspectionItems.push_back(angAccURI);
  gazebo::util::IntrospectionManager::Instance()->Register
      <ignition::math::Vector3d>(angAccURI.Str(), fLinkAngAcc);
}

/////////////////////////////////////////////////
ignition::math::Vector3d Link::WorldLinearVel() const
{
  return this->WorldLinearVel(ignition::math::Vector3d::Zero);
}

/////////////////////////////////////////////////
event::ConnectionPtr Link::ConnectEnabled(
    std::function<void (bool)> _subscriber)
{
  return this->dataPtr->enabledSignal.Connect(_subscriber);
}

//////////////////////////////////////////////////
void Link::LoadLight(sdf::ElementPtr _sdf)
{
  // Create new light object
  LightPtr light(new physics::Light(shared_from_this()));
  light->SetStatic(true);
  light->ProcessMsg(msgs::LightFromSDF(_sdf));
  light->SetWorld(this->world);
  light->Load(_sdf);
  this->dataPtr->lights.push_back(light);
  // NOTE:
  // The light need to be added to the list on Load (before Init) for the case
  // when a model is created from a factory message. Otherwise the model msg
  // published to the client will not contain an entry of this light
}

//////////////////////////////////////////////////
const Link::Visuals_M &Link::Visuals() const
{
  return this->visuals;
}
