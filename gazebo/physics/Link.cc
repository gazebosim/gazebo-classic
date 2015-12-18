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

#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <sstream>
#include <functional>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/transport/TransportIface.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/util/OpenAL.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/math/Quaternion.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Assert.hh"
#include "gazebo/common/Battery.hh"

#include "gazebo/sensors/SensorsIface.hh"
#include "gazebo/sensors/Sensor.hh"

#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/ContactManager.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/Link.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
Link::Link(EntityPtr _parent)
: Entity(_parent),
  linkDPtr(std::static_pointer_cast<LinkDPtr>(this->entityDPtr)),
  dataPtr(new LinkPrivate),
{
  this->ConstructionHelper();
}

//////////////////////////////////////////////////
Link::Link(LinkProtected &_dataPtr, EntityPtr _parent)
: Entity(_dataPtr, _parent),
  linkDPtr(std::static_pointer_cast<LinkDPtr>(this->entityDPtr)),
  dataPtr(new LinkPrivate),
{
  this->ConstructionHelper();
}

//////////////////////////////////////////////////
void Link::ConstructionHelper()
{
  this->AddType(Base::LINK);
  this->linkDPtr->inertial.reset(new Inertial);
  this->dataPtr->parentJoints.clear();
  this->dataPtr->childJoints.clear();
  this->dataPtr->publishData = false;
  this->dataPtr->publishDataMutex = new boost::recursive_mutex();
}

//////////////////////////////////////////////////
Link::~Link()
{
  this->dataPtr->attachedModels.clear();

  for (Visuals_M::iterator iter = this->linkDPtr->visuals.begin();
      iter != this->linkDPtr->visuals.end(); ++iter)
  {
    msgs::Visual msg;
    msg.set_name(iter->second.name());
    msg.set_id(iter->second.id());
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
  this->linkDPtr->visuals.clear();

  if (this->linkDPtr->cgVisuals.size() > 0)
  {
    for (unsigned int i = 0; i < this->linkDPtr->cgVisuals.size(); i++)
    {
      msgs::Visual msg;
      msg.set_name(this->linkDPtr->cgVisuals[i]);
      if (this->parent)
        msg.set_parent_name(this->parent->GetScopedName());
      else
        msg.set_parent_name("");
      msg.set_delete_me(true);
      this->visPub->Publish(msg);
    }
    this->linkDPtr->cgVisuals.clear();
  }

  this->visPub.reset();
  this->dataPtr->sensors.clear();

  this->requestPub.reset();
  this->dataPtr->dataPub.reset();
  this->dataPtr->wrenchSub.reset();
  this->connections.clear();

  delete this->dataPtr->publishDataMutex;
  this->dataPtr->publishDataMutex = NULL;

  this->dataPtr->collisions.clear();
  this->dataPtr->batteries.clear();
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
      std::bind(&Link::SelfCollide, this));

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
        std::string sensorName =
          sensors::create_sensor(sensorElem, this->GetWorld()->GetName(),
              this->GetScopedName(), this->GetId());
        this->dataPtr->sensors.push_back(sensorName);
      }
      sensorElem = sensorElem->GetNextElement("sensor");
    }
  }

  if (!this->IsStatic())
  {
    this->linkDPtr->inertial->Load(this->sdf->GetElement("inertial"));
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

      std::vector<std::string> names = source->GetCollisionNames();
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
        this->world->GetPhysicsEngine()->GetContactManager()->CreateFilter(
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

  this->connections.push_back(event::Events::ConnectWorldUpdateBegin(
      boost::bind(&Link::Update, this, _1)));

  std::string topicName = "~/" + this->GetScopedName() + "/wrench";
  boost::replace_all(topicName, "::", "/");
  this->dataPtr->wrenchSub = this->node->Subscribe(topicName, &Link::OnWrenchMsg, this);
}

//////////////////////////////////////////////////
void Link::Init()
{
  this->linkDPtr->linearAccel.Set(0, 0, 0);
  this->linkDPtr->angularAccel.Set(0, 0, 0);

  this->dataPtr->enabled = true;

  // Set Link pose before setting pose of child collisions
  this->SetRelativePose(this->sdf->Get<math::Pose>("pose"));
  this->SetInitialRelativePose(this->sdf->Get<math::Pose>("pose"));

  // Call Init for child collisions, which whill set their pose
  Base_V::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::COLLISION))
    {
      CollisionPtr collision = std::static_pointer_cast<Collision>(*iter);
      this->dataPtr->collisions.push_back(collision);
      collision->Init();
    }
  }

  // Initialize all the batteries
  for (auto &battery : this->dataPtr->batteries)
  {
    battery->Init();
  }

  this->linkDPtr->intialized = true;
}

//////////////////////////////////////////////////
void Link::Fini()
{
  this->dataPtr->parentJoints.clear();
  this->dataPtr->childJoints.clear();
  this->dataPtr->collisions.clear();
  this->linkDPtr->inertial.reset();
  this->dataPtr->batteries.clear();

  for (std::vector<std::string>::iterator iter = this->dataPtr->sensors.begin();
       iter != this->dataPtr->sensors.end(); ++iter)
  {
    sensors::remove_sensor(*iter);
  }
  this->dataPtr->sensors.clear();

  for (Visuals_M::iterator iter = this->linkDPtr->visuals.begin();
       iter != this->linkDPtr->visuals.end(); ++iter)
  {
    msgs::Request *msg = msgs::CreateRequest("entity_delete",
        boost::lexical_cast<std::string>(iter->second.id()));
    this->requestPub->Publish(*msg, true);
  }

  for (std::vector<std::string>::iterator iter = this->linkDPtr->cgVisuals.begin();
       iter != this->linkDPtr->cgVisuals.end(); ++iter)
  {
    msgs::Request *msg = msgs::CreateRequest("entity_delete", *iter);
    this->requestPub->Publish(*msg, true);
  }

#ifdef HAVE_OPENAL
  this->world->GetPhysicsEngine()->GetContactManager()->RemoveFilter(
      this->GetScopedName() + "/audio_collision");
  this->dataPtr->audioSink.reset();
#endif

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
  this->SetAngularVel(math::Vector3(0, 0, 0));
  this->SetLinearVel(math::Vector3(0, 0, 0));
  this->SetAngularAccel(math::Vector3(0, 0, 0));
  this->SetLinearAccel(math::Vector3(0, 0, 0));
  this->SetForce(math::Vector3(0, 0, 0));
  this->SetTorque(math::Vector3(0, 0, 0));
}

//////////////////////////////////////////////////
void Link::UpdateParameters(sdf::ElementPtr _sdf)
{
  Entity::UpdateParameters(_sdf);

  if (this->sdf->HasElement("inertial"))
  {
    sdf::ElementPtr inertialElem = this->sdf->GetElement("inertial");
    this->linkDPtr->inertial->UpdateParameters(inertialElem);
  }

  this->sdf->GetElement("gravity")->GetValue()->SetUpdateFunc(
      boost::bind(&Link::GetGravityMode, this));
  this->sdf->GetElement("kinematic")->GetValue()->SetUpdateFunc(
      boost::bind(&Link::GetKinematic, this));

  if (this->sdf->Get<bool>("gravity") != this->GetGravityMode())
    this->SetGravityMode(this->sdf->Get<bool>("gravity"));

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
      CollisionPtr collision = std::dynamic_pointer_cast<Collision>(
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
  return this->SelfCollide();
}

//////////////////////////////////////////////////
bool Link::SelfCollide() const
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
    this->dataPtr->audioSink->SetPose(this->GetWorldPose().Ign());
    this->dataPtr->audioSink->SetVelocity(this->GetWorldLinearVel().Ign());
  }

  // Update all the audio sources
  for (std::vector<util::OpenALSourcePtr>::iterator iter =
      this->dataPtr->audioSources.begin(); iter != this->dataPtr->audioSources.end(); ++iter)
  {
    (*iter)->SetPose(this->GetWorldPose().Ign());
    (*iter)->SetVelocity(this->GetWorldLinearVel().Ign());
  }
#endif

  // FIXME: race condition on factory-based model loading!!!!!
   /*if (this->GetEnabled() != this->dataPtr->enabled)
   {
     this->dataPtr->enabled = this->GetEnabled();
     this->dataPtr->enabledSignal(this->dataPtr->enabled);
   }*/

  if (!this->dataPtr->wrenchMsgs.empty())
  {
    std::vector<msgs::Wrench> messages;
    {
      boost::mutex::scoped_lock lock(this->dataPtr->wrenchMsgMutex);
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

/////////////////////////////////////////////////
Joint_V Link::GetParentJoints() const
{
  return this->ParentJoints();
}

/////////////////////////////////////////////////
Joint_V Link::ParentJoints() const
{
  return this->dataPtr->parentJoints;
}

/////////////////////////////////////////////////
Joint_V Link::GetChildJoints() const
{
  return this->ChildJoints();
}

/////////////////////////////////////////////////
Joint_V Link::ChildJoints() const
{
  return this->dataPtr->childJoints;
}

/////////////////////////////////////////////////
Link_V Link::GetChildJointsLinks() const
{
  return this->ChildJointsLinks();
}

/////////////////////////////////////////////////
Link_V Link::ChildJointsLinks() const
{
  Link_V links;
  for (std::vector<JointPtr>::const_iterator iter = this->dataPtr->childJoints.begin();
                                             iter != this->dataPtr->childJoints.end();
                                             ++iter)
  {
    if ((*iter)->GetChild())
      links.push_back((*iter)->GetChild());
  }
  return links;
}

/////////////////////////////////////////////////
Link_V Link::GetParentJointsLinks() const
{
  return this->ParentJointsLinks();
}

/////////////////////////////////////////////////
Link_V Link::ParentJointsLinks() const
{
  Link_V links;
  for (std::vector<JointPtr>::const_iterator iter = this->dataPtr->parentJoints.begin();
                                             iter != this->dataPtr->parentJoints.end();
                                             ++iter)
  {
    if ((*iter)->Parent())
      links.push_back((*iter)->Parent());
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

  collision = this->GetWorld()->GetPhysicsEngine()->CreateCollision(geomType,
      std::static_pointer_cast<Link>(shared_from_this()));

  if (!collision)
    gzthrow("Unknown Collisionetry Type[" + geomType + "]");

  collision->Load(_sdf);
}

//////////////////////////////////////////////////
CollisionPtr Link::GetCollisionById(unsigned int _id) const
{
  return this->CollisionById(_id);
}

//////////////////////////////////////////////////
CollisionPtr Link::CollisionById(const unsigned int _id) const
{
  return std::dynamic_pointer_cast<Collision>(this->BaseById(_id));
}

//////////////////////////////////////////////////
CollisionPtr Link::GetCollision(const std::string &_name)
{
  return this->Collision(_name);
}

//////////////////////////////////////////////////
CollisionPtr Link::Collision(const std::string &_name) const
{
  CollisionPtr result;
  Base_V::const_iterator biter;
  for (biter = this->children.begin(); biter != this->children.end(); ++biter)
  {
    if ((*biter)->GetName() == _name)
    {
      result = std::dynamic_pointer_cast<Collision>(*biter);
      break;
    }
  }

  return result;
}

//////////////////////////////////////////////////
Collision_V Link::GetCollisions() const
{
  return this->Collisions();
}

//////////////////////////////////////////////////
Collision_V Link::Collisions() const
{
  return this->dataPtr->collisions;
}

//////////////////////////////////////////////////
CollisionPtr Link::GetCollision(unsigned int _index) const
{
  return this->Collision(_index);
}

//////////////////////////////////////////////////
CollisionPtr Link::Collision(const unsigned int _index) const
{
  CollisionPtr collision;
  if (_index <= this->GetChildCount())
    collision = std::static_pointer_cast<Collision>(this->GetChild(_index));
  else
    gzerr << "Index is out of range\n";

  return collision;
}

//////////////////////////////////////////////////
void Link::SetLinearAccel(const math::Vector3 &_accel)
{
  this->SetLinearAccel(_accel.Ign());
}

//////////////////////////////////////////////////
void Link::SetLinearAccel(const ignition::math::Vector3d &_accel)
{
  this->SetEnabled(true);
  this->linkDPtr->linearAccel = _accel;
}

//////////////////////////////////////////////////
void Link::SetAngularAccel(const math::Vector3 &_accel)
{
  this->SetAngularAccel(_accel.Ign());
}

//////////////////////////////////////////////////
void Link::SetAngularAccel(const ignition::math::Vector3d &_accel)
{
  this->SetEnabled(true);
  this->linkDPtr->angularAccel = _accel;
}

//////////////////////////////////////////////////
math::Pose Link::GetWorldCoGPose() const
{
  return this->WorldCogPose();
}

//////////////////////////////////////////////////
ignition::math::Pose3d Link::WorldCoGPose() const
{
  ignition::math::Pose3d pose = this->WorldPose();
  pose.Pos() += pose.rot.RotateVector(this->linkDPtr->inertial->GetCoG());
  return pose;
}

//////////////////////////////////////////////////
math::Vector3 Link::GetRelativeLinearVel() const
{
  return this->RelativeLinearVel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Link::RelativeLinearVel() const
{
  return this->WorldPose().Rot().RotateVectorReverse(this->WorldLinearVel());
}

//////////////////////////////////////////////////
math::Vector3 Link::GetRelativeAngularVel() const
{
  return this->RelativeAngularVel();
}

//////////////////////////////////////////////////
math::Vector3 Link::RelativeAngularVel() const
{
  return this->WorldPose().Rot().RotateVectorReverse(this->WorldAngularVel());
}

//////////////////////////////////////////////////
math::Vector3 Link::GetRelativeLinearAccel() const
{
  return this->RelativeLinearAccel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Link::RelativeLinearAccel() const
{
  return this->RelativeForce() / this->linkDPtr->inertial->GetMass();
}

//////////////////////////////////////////////////
math::Vector3 Link::GetWorldLinearAccel() const
{
  return this->WorldLinearAccel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Link::WorldLinearAccel() const
{
  return this->WorldForce() / this->linkDPtr->inertial->GetMass();
}

//////////////////////////////////////////////////
math::Vector3 Link::GetRelativeAngularAccel() const
{
  return this->RelativeAngularAccel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Link::RelativeAngularAccel() const
{
  return this->WorldPose().Rot().RotateVectorReverse(this->WorldAngularAccel());
}

//////////////////////////////////////////////////
math::Vector3 Link::GetWorldAngularAccel() const
{
  return this->WorldAngularAccel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Link::GetWorldAngularAccel() const
{
  // I: inertia matrix in world frame
  // T: sum of external torques in world frame
  // L: angular momentum of CoG in world frame
  // w: angular velocity in world frame
  // return I^-1 * (T - w x L)
  return this->WorldInertiaMatrix().Inverse() * (this->WorldTorque()
    - this->WorldAngularVel().Cross(this->WorldAngularMomentum()));
}

//////////////////////////////////////////////////
math::Vector3 Link::GetWorldAngularMomentum() const
{
  return this->WorldAngularMomentum();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Link::WorldAngularMomentum() const
{
  return this->WorldInertiaMatrix() * this->WorldAngularVel();
}

//////////////////////////////////////////////////
math::Vector3 Link::GetRelativeForce() const
{
  return this->RelativeForce();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Link::RelativeForce() const
{
  return this->WorldPose().Rot().RotateVectorReverse(this->WorldForce());
}

//////////////////////////////////////////////////
math::Vector3 Link::GetRelativeTorque() const
{
  return this->RelativeTorque();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Link::RelativeTorque() const
{
  return this->WorldPose().Rot().RotateVectorReverse(this->WorldTorque());
}

//////////////////////////////////////////////////
ModelPtr Link::GetModel() const
{
  return this->Model();
}

//////////////////////////////////////////////////
ModelPtr Link::Model() const
{
  return std::dynamic_pointer_cast<Model>(this->Parent());
}

//////////////////////////////////////////////////
ignition::math::Box Link::BoundingBox() const
{
  ignition::math::Box box;

  box.Min().Set(IGN_DBL_MAX, IGN_DBL_MAX, IGn_DBL_MAX);
  box.Max().Set(0, 0, 0);

  for (Collision_V::const_iterator iter = this->dataPtr->collisions.begin();
       iter != this->dataPtr->collisions.end(); ++iter)
  {
    box += (*iter)->BoundingBox();
  }

  return box;
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
math::Pose Link::GetWorldInertialPose() const
{
  return this->WorldInertialPose();
}

//////////////////////////////////////////////////
ignition::math::Pose3d Link::tWorldInertialPose() const
{
  ignition::math::Pose3d inertialPose;
  if (this->linkDPtr->inertial)
    inertialPose = this->linkDPtr->inertial->GetPose();
  return inertialPose + this->WorldPose();
}

//////////////////////////////////////////////////
math::Matrix3 Link::GetWorldInertiaMatrix() const
{
  return this->WorldInertiaMatrix();
}

//////////////////////////////////////////////////
ignition::math::Matrix3d Link::WorldInertiaMatrix() const
{
  ignition::math::Matrix3d moi;
  if (this->linkDPtr->inertial)
  {
    ignition::math::Vector3d pos = this->linkDPtr->inertial->GetPose().pos.Ign();
    ignition::math::Quaterniond rot = this->WorldPose().Rot().Inverse();
    moi = this->linkDPtr->inertial->GetMOI(ignition::math::Pose3d(pos, rot));
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
  for (std::vector<JointPtr>::iterator iter = this->dataPtr->parentJoints.begin();
                                       iter != this->dataPtr->parentJoints.end();
                                       ++iter)
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
  for (std::vector<JointPtr>::iterator iter = this->dataPtr->childJoints.begin();
                                       iter != this->dataPtr->childJoints.end();
                                       ++iter)
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
  math::Pose relPose = this->GetRelativePose();

  _msg.set_id(this->GetId());
  _msg.set_name(this->GetScopedName());
  _msg.set_self_collide(this->GetSelfCollide());
  _msg.set_gravity(this->GetGravityMode());
  _msg.set_kinematic(this->GetKinematic());
  _msg.set_enabled(this->GetEnabled());
  msgs::Set(_msg.mutable_pose(), relPose.Ign());

  msgs::Set(this->visualMsg->mutable_pose(), relPose.Ign());
  _msg.add_visual()->CopyFrom(*this->visualMsg);

  _msg.mutable_inertial()->set_mass(this->linkDPtr->inertial->GetMass());
  _msg.mutable_inertial()->set_ixx(this->linkDPtr->inertial->GetIXX());
  _msg.mutable_inertial()->set_ixy(this->linkDPtr->inertial->GetIXY());
  _msg.mutable_inertial()->set_ixz(this->linkDPtr->inertial->GetIXZ());
  _msg.mutable_inertial()->set_iyy(this->linkDPtr->inertial->GetIYY());
  _msg.mutable_inertial()->set_iyz(this->linkDPtr->inertial->GetIYZ());
  _msg.mutable_inertial()->set_izz(this->linkDPtr->inertial->GetIZZ());
  msgs::Set(_msg.mutable_inertial()->mutable_pose(),
      this->linkDPtr->inertial->GetPose().Ign());

  for (auto &child : this->children)
  {
    if (child->HasType(Base::COLLISION))
    {
      CollisionPtr collision = std::static_pointer_cast<Collision>(child);
      collision->FillMsg(*_msg.add_collision());
    }
  }

  for (std::vector<std::string>::iterator iter = this->dataPtr->sensors.begin();
      iter != this->dataPtr->sensors.end(); ++iter)
  {
    sensors::SensorPtr sensor = sensors::get_sensor(*iter);
    if (sensor)
      sensor->FillMsg(*_msg.add_sensor());
  }

  if (this->linkDPtr->visuals.empty())
    this->ParseVisuals();
  else
    this->UpdateVisualMsg();

  for (Visuals_M::iterator iter = this->linkDPtr->visuals.begin();
      iter != this->linkDPtr->visuals.end(); ++iter)
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
  if (_msg.has_kinematic())
  {
    this->SetKinematic(_msg.kinematic());
    this->SetEnabled(true);
  }
  if (_msg.has_inertial())
  {
    this->linkDPtr->inertial->ProcessMsg(_msg.inertial());
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
    CollisionPtr coll = this->CollisionById(_msg.collision(i).id());
    if (coll)
      coll->ProcessMsg(_msg.collision(i));
  }
  if (_msg.collision_size()>0)
    this->UpdateSurface();
}

//////////////////////////////////////////////////
unsigned int Link::GetSensorCount() const
{
  return this->SensorCount();
}

//////////////////////////////////////////////////
unsigned int Link::SensorCount() const
{
  return this->dataPtr->sensors.size();
}

//////////////////////////////////////////////////
std::string Link::GetSensorName(unsigned int _i) const
{
  return this->SensorName();
}

//////////////////////////////////////////////////
std::string Link::SensorName(const unsigned int _i) const
{
  if (_i < this->dataPtr->sensors.size())
    return this->dataPtr->sensors[_i];

  return std::string();
}

//////////////////////////////////////////////////
void Link::AttachStaticModel(ModelPtr &_model, const math::Pose &_offset)
{
  this->AttachStaticMode(_model, _offset.Ign());
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
  this->linkDPtr->attachedModelsOffset.push_back(_offset);
}

//////////////////////////////////////////////////
void Link::DetachStaticModel(const std::string &_modelName)
{
  for (unsigned int i = 0; i < this->dataPtr->attachedModels.size(); i++)
  {
    if (this->dataPtr->attachedModels[i]->GetName() == _modelName)
    {
      this->dataPtr->attachedModels.erase(this->dataPtr->attachedModels.begin()+i);
      this->linkDPtr->attachedModelsOffset.erase(this->linkDPtr->attachedModelsOffset.begin()+i);
      break;
    }
  }
}

//////////////////////////////////////////////////
void Link::DetachAllStaticModels()
{
  this->dataPtr->attachedModels.clear();
  this->linkDPtr->attachedModelsOffset.clear();
}

//////////////////////////////////////////////////
void Link::OnPoseChange()
{
  math::Pose p;
  for (unsigned int i = 0; i < this->dataPtr->attachedModels.size(); i++)
  {
    p = this->GetWorldPose();
    p.pos += this->linkDPtr->attachedModelsOffset[i].pos;
    p.rot = p.rot * this->linkDPtr->attachedModelsOffset[i].rot;

    this->dataPtr->attachedModels[i]->SetWorldPose(p, true);
  }
}

//////////////////////////////////////////////////
void Link::SetState(const LinkState &_state)
{
  this->SetWorldPose(_state.GetPose());
  this->SetLinearVel(_state.GetVelocity().pos);
  this->SetAngularVel(_state.GetVelocity().rot.GetAsEuler());
  this->SetLinearAccel(_state.GetAcceleration().pos);
  this->SetAngularAccel(_state.GetAcceleration().rot.GetAsEuler());
  this->SetForce(_state.GetWrench().pos);
  this->SetTorque(_state.GetWrench().rot.GetAsEuler());

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
  return this->LinearDamping();
}

/////////////////////////////////////////////////
double Link::LinearDamping() const
{
  if (this->sdf->HasElement("velocity_decay"))
    return this->sdf->GetElement("velocity_decay")->Get<double>("linear");
  else
    return 0.0;
}

/////////////////////////////////////////////////
double Link::GetAngularDamping() const
{
  return this->AngularDamping();
}

/////////////////////////////////////////////////
double Link::AngularDamping() const
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
void Link::SetPublishData(const bool _enable)
{
  {
    boost::recursive_mutex::scoped_lock lock(*this->dataPtr->publishDataMutex);
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
        boost::bind(&Link::PublishData, this)));
  }
  else
  {
    this->dataPtr->dataPub.reset();
    this->connections.clear();
  }
}

/////////////////////////////////////////////////
void Link::PublishData()
{
  if (this->dataPtr->publishData && this->dataPtr->dataPub->HasConnections())
  {
    msgs::Set(this->linkDataMsg.mutable_time(), this->world->GetSimTime());
    linkDataMsg.set_name(this->GetScopedName());
    msgs::Set(this->linkDataMsg.mutable_linear_velocity(),
        this->GetWorldLinearVel().Ign());
    msgs::Set(this->linkDataMsg.mutable_angular_velocity(),
        this->GetWorldAngularVel().Ign());
    this->dataPtr->dataPub->Publish(this->linkDataMsg);
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
  for (auto &iter : this->linkDPtr->visuals)
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
  auto iter = this->linkDPtr->visuals.find(_id);
  if (iter == this->linkDPtr->visuals.end())
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
  auto iter = this->linkDPtr->visuals.find(_id);
  if (iter == this->linkDPtr->visuals.end())
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
        this->dataPtr->audioSources.begin(); iter != this->dataPtr->audioSources.end(); ++iter)
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

  for (auto const it : this->linkDPtr->visuals)
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
void Link::SetScale(const math::Vector3 &_scale)
{
  this->SetScale(_scale.Ign());
}

/////////////////////////////////////////////////
void Link::SetScale(const ignition::math::Vector3d &_scale)
{
  Base_V::const_iterator biter;
  for (biter = this->children.begin(); biter != this->children.end(); ++biter)
  {
    if ((*biter)->HasType(Base::COLLISION))
    {
      std::static_pointer_cast<Collision>(*biter)->SetScale(_scale);
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
        math::Vector3 size =
            geomElem->GetElement("box")->Get<math::Vector3>("size");
        geomElem->GetElement("box")->GetElement("size")->Set(
            _scale/this->scale*size);
      }
      else if (geomElem->HasElement("sphere"))
      {
        // update radius the same way as collision shapes
        double radius = geomElem->GetElement("sphere")->Get<double>("radius");
        double newRadius = _scale.Max();
        double oldRadius = this->scale.Max();
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
      for (auto &iter : this->linkDPtr->visuals)
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

        auto iter = this->linkDPtr->visuals.find(msg.id());
        if (iter != this->linkDPtr->visuals.end())
          gzthrow(std::string("Duplicate visual name[")+msg.name()+"]\n");
        this->linkDPtr->visuals[msg.id()] = msg;
      }

      visualElem = visualElem->GetNextElement("visual");
    }
  }
}

/////////////////////////////////////////////////
double Link::GetWorldEnergyPotential() const
{
  return this->WorldEnergyPotential();
}

/////////////////////////////////////////////////
double Link::WorldEnergyPotential() const
{
  // compute gravitational potential energy for link CG location
  // use origin as reference position
  // E = -m g^T z
  double m = this->Inertial()->GetMass();
  ignition::math::Vector3d g = this->World()->GetPhysicsEngine()->GetGravity();
  ignition::math::Vector3d z = this->WorldCoGPose().Pos();
  return -m * g.Dot(z);
}

/////////////////////////////////////////////////
double Link::GetWorldEnergyKinetic() const
{
  return this->WorldEnergyKinetic();
}

/////////////////////////////////////////////////
double Link::WorldEnergyKinetic() const
{
  double energy = 0.0;

  // compute linear kinetic energy
  // E = 1/2 m v^T v
  {
    double m = this->Inertial()->GetMass();
    ignition::math::Vector3d v = this->WorldCoGLinearVel();
    energy += 0.5 * m * v.Dot(v);
  }

  // compute angular kinetic energy
  // E = 1/2 w^T I w
  {
    ignition::math::Vector3d w = this->WorldAngularVel();
    ignition::math::Matrix3d I = this->WorldInertiaMatrix();
    energy += 0.5 * w.Dot(I * w);
  }

  return energy;
}

/////////////////////////////////////////////////
double Link::GetWorldEnergy() const
{
  return this->WorldEnergy();
}

/////////////////////////////////////////////////
double Link::WorldEnergy() const
{
  return this->WorldEnergyPotential() + this->WorldEnergyKinetic();
}

/////////////////////////////////////////////////
void Link::MoveFrame(const math::Pose &_worldReferenceFrameSrc,
                     const math::Pose &_worldReferenceFrameDst)
{
  this->MoveFrame(_worldReferenceFrameSrc.Ign(),
                  _worldReferenceFrameDst.Ign());
}

/////////////////////////////////////////////////
void Link::MoveFrame(const ignition::math::Pose3d &_worldReferenceFrameSrc,
                     const ignition::math::Pose3d &_worldReferenceFrameDst)
{
  ignition::math::Pose3d targetWorldPose =
    (this->WorldPose() - _worldReferenceFrameSrc) + _worldReferenceFrameDst;
  this->SetWorldPose(targetWorldPose);
  this->SetWorldTwist(ignition::math::Vector3d::Zero,
                      ignition::math::Vector3d::Zero);
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
  Link_V childLinks = this->ChildJointsLinks();

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
  return this->VisualMessage(_name);
}

/////////////////////////////////////////////////
msgs::Visual Link::VisualMessage(const std::string &_name) const
{
  msgs::Visual result;

  Visuals_M::const_iterator iter;
  for (iter = this->linkDPtr->visuals.begin(); iter != this->linkDPtr->visuals.end(); ++iter)
    if (iter->second.name() == _name)
      break;

  if (iter != this->linkDPtr->visuals.end())
    result = iter->second;

  return result;
}

//////////////////////////////////////////////////
void Link::OnWrenchMsg(ConstWrenchPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->dataPtr->wrenchMsgMutex);
  this->dataPtr->wrenchMsgs.push_back(*_msg);
}

//////////////////////////////////////////////////
void Link::ProcessWrenchMsg(const msgs::Wrench &_msg)
{
  math::Vector3 pos = math::Vector3::Zero;
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

//////////////////////////////////////////////////
math::Vector3 Link::GetWorldLinearVel() const
{
  this->WorldLinearVel();
}

//////////////////////////////////////////////////
math::Vector3 Link::WorldLinearVel() const
{
  return this->WorldLinearVel(ignition::math::Vector3d::Zero);
}

//////////////////////////////////////////////////
InertialPtr Link::GetInertial() const
{
  return this->Inertial();
}

//////////////////////////////////////////////////
InertialPtr Link::Inertial() const
{
  return this->linkDPtr->inertial;
}

//////////////////////////////////////////////////
bool Link::GetKinematic() const
{
  return this->Kinematic();
}

//////////////////////////////////////////////////
bool Link::Kinematic() const
{
  return false;
}

//////////////////////////////////////////////////
event::ConnectionPtr Link::ConnectEnabled(std::function<void(bool)> _subscriber)
{
  return this->pdPtr->enabledSignal.Connect(_subscriber);
}

//////////////////////////////////////////////////
void Link::DisconnectEnabled(event::ConnectionPtr &_conn)
{
  enabledSignal.Disconnect(_conn);
}
