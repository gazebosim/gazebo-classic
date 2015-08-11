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

#include <sstream>

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

#include "gazebo/sensors/SensorsIface.hh"
#include "gazebo/sensors/Sensor.hh"

#include "gazebo/physics/PhysicsIface.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/ContactManager.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/Battery.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
Link::Link(EntityPtr _parent)
    : Entity(_parent), initialized(false)
{
  this->AddType(Base::LINK);
  this->inertial.reset(new Inertial);
  this->parentJoints.clear();
  this->childJoints.clear();
  this->publishData = false;
  this->publishDataMutex = new boost::recursive_mutex();
}

//////////////////////////////////////////////////
Link::~Link()
{
  this->attachedModels.clear();

  for (Visuals_M::iterator iter = this->visuals.begin();
      iter != this->visuals.end(); ++iter)
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
  this->visuals.clear();

  if (this->cgVisuals.size() > 0)
  {
    for (unsigned int i = 0; i < this->cgVisuals.size(); i++)
    {
      msgs::Visual msg;
      msg.set_name(this->cgVisuals[i]);
      if (this->parent)
        msg.set_parent_name(this->parent->GetScopedName());
      else
        msg.set_parent_name("");
      msg.set_delete_me(true);
      this->visPub->Publish(msg);
    }
    this->cgVisuals.clear();
  }

  this->visPub.reset();
  this->sensors.clear();

  this->requestPub.reset();
  this->dataPub.reset();
  this->wrenchSub.reset();
  this->connections.clear();

  delete this->publishDataMutex;
  this->publishDataMutex = NULL;

  this->collisions.clear();
  this->batteries.clear();
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
      boost::bind(&Link::GetSelfCollide, this));

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
        this->sensors.push_back(sensorName);
      }
      sensorElem = sensorElem->GetNextElement("sensor");
    }
  }

  if (!this->IsStatic())
  {
    this->inertial->Load(this->sdf->GetElement("inertial"));
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
      this->audioSources.push_back(source);
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
      this->audioContactsSub = this->node->Subscribe(topic,
          &Link::OnCollision, this);
    }
  }

  if (_sdf->HasElement("audio_sink"))
  {
    this->audioSink = util::OpenAL::Instance()->CreateSink(
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
  this->wrenchSub = this->node->Subscribe(topicName, &Link::OnWrenchMsg, this);
}

//////////////////////////////////////////////////
void Link::Init()
{
  this->linearAccel.Set(0, 0, 0);
  this->angularAccel.Set(0, 0, 0);

  this->enabled = true;

  // Set Link pose before setting pose of child collisions
  this->SetRelativePose(this->sdf->Get<math::Pose>("pose"));
  this->SetInitialRelativePose(this->sdf->Get<math::Pose>("pose"));

  // Call Init for child collisions, which whill set their pose
  Base_V::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::COLLISION))
    {
      CollisionPtr collision = boost::static_pointer_cast<Collision>(*iter);
      this->collisions.push_back(collision);
      collision->Init();
    }
  }

  for (auto &battery : this->batteries)
  {
    battery->Init();
  }

  this->initialized = true;
}

//////////////////////////////////////////////////
void Link::Fini()
{
  this->parentJoints.clear();
  this->childJoints.clear();
  this->collisions.clear();
  this->inertial.reset();
  this->batteries.clear();

  for (std::vector<std::string>::iterator iter = this->sensors.begin();
       iter != this->sensors.end(); ++iter)
  {
    sensors::remove_sensor(*iter);
  }
  this->sensors.clear();

  for (Visuals_M::iterator iter = this->visuals.begin();
       iter != this->visuals.end(); ++iter)
  {
    msgs::Request *msg = msgs::CreateRequest("entity_delete",
        boost::lexical_cast<std::string>(iter->second.id()));
    this->requestPub->Publish(*msg, true);
  }

  for (std::vector<std::string>::iterator iter = this->cgVisuals.begin();
       iter != this->cgVisuals.end(); ++iter)
  {
    msgs::Request *msg = msgs::CreateRequest("entity_delete", *iter);
    this->requestPub->Publish(*msg, true);
  }

#ifdef HAVE_OPENAL
  this->world->GetPhysicsEngine()->GetContactManager()->RemoveFilter(
      this->GetScopedName() + "/audio_collision");
  this->audioSink.reset();
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
    this->inertial->UpdateParameters(inertialElem);
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
      CollisionPtr collision = boost::dynamic_pointer_cast<Collision>(
          this->GetChild(collisionElem->Get<std::string>("name")));

      if (collision)
        collision->UpdateParameters(collisionElem);
      collisionElem = collisionElem->GetNextElement("collision");
    }
  }

  if (this->sdf->HasElement("battery"))
  {
    sdf::ElementPtr batteryElem = this->sdf->GetElement("battery");
    while (batteryElem)
    {
      BatteryPtr battery = this->Battery(
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

  for (Collision_V::iterator iter = this->collisions.begin();
       iter != this->collisions.end(); ++iter)
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
  for (Collision_V::iterator iter = this->collisions.begin();
       iter != this->collisions.end(); ++iter)
  {
    (*iter)->SetLaserRetro(_retro);
  }
}

//////////////////////////////////////////////////
void Link::Update(const common::UpdateInfo & /*_info*/)
{
#ifdef HAVE_OPENAL
  if (this->audioSink)
  {
    this->audioSink->SetPose(this->GetWorldPose());
    this->audioSink->SetVelocity(this->GetWorldLinearVel());
  }

  // Update all the audio sources
  for (std::vector<util::OpenALSourcePtr>::iterator iter =
      this->audioSources.begin(); iter != this->audioSources.end(); ++iter)
  {
    (*iter)->SetPose(this->GetWorldPose());
    (*iter)->SetVelocity(this->GetWorldLinearVel());
  }
#endif

  // FIXME: race condition on factory-based model loading!!!!!
   /*if (this->GetEnabled() != this->enabled)
   {
     this->enabled = this->GetEnabled();
     this->enabledSignal(this->enabled);
   }*/

  if (!this->wrenchMsgs.empty())
  {
    std::vector<msgs::Wrench> messages;
    {
      boost::mutex::scoped_lock lock(this->wrenchMsgMutex);
      messages = this->wrenchMsgs;
      this->wrenchMsgs.clear();
    }

    for (auto it : messages)
    {
      this->ProcessWrenchMsg(it);
    }
  }
}

/////////////////////////////////////////////////
Joint_V Link::GetParentJoints() const
{
  return this->parentJoints;
}

/////////////////////////////////////////////////
Joint_V Link::GetChildJoints() const
{
  return this->childJoints;
}

/////////////////////////////////////////////////
Link_V Link::GetChildJointsLinks() const
{
  Link_V links;
  for (std::vector<JointPtr>::const_iterator iter = this->childJoints.begin();
                                             iter != this->childJoints.end();
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
  Link_V links;
  for (std::vector<JointPtr>::const_iterator iter = this->parentJoints.begin();
                                             iter != this->parentJoints.end();
                                             ++iter)
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

  collision = this->GetWorld()->GetPhysicsEngine()->CreateCollision(geomType,
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
  return this->collisions;
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
void Link::SetLinearAccel(const math::Vector3 &_accel)
{
  this->SetEnabled(true);
  this->linearAccel = _accel;
}

//////////////////////////////////////////////////
void Link::SetAngularAccel(const math::Vector3 &_accel)
{
  this->SetEnabled(true);
  this->angularAccel = _accel;
}

//////////////////////////////////////////////////
math::Pose Link::GetWorldCoGPose() const
{
  math::Pose pose = this->GetWorldPose();
  pose.pos += pose.rot.RotateVector(this->inertial->GetCoG());
  return pose;
}

//////////////////////////////////////////////////
math::Vector3 Link::GetRelativeLinearVel() const
{
  return this->GetWorldPose().rot.RotateVectorReverse(
      this->GetWorldLinearVel());
}

//////////////////////////////////////////////////
math::Vector3 Link::GetRelativeAngularVel() const
{
  return this->GetWorldPose().rot.RotateVectorReverse(
         this->GetWorldAngularVel());
}

//////////////////////////////////////////////////
math::Vector3 Link::GetRelativeLinearAccel() const
{
  return this->GetRelativeForce() / this->inertial->GetMass();
}

//////////////////////////////////////////////////
math::Vector3 Link::GetWorldLinearAccel() const
{
  return this->GetWorldForce() / this->inertial->GetMass();
}

//////////////////////////////////////////////////
math::Vector3 Link::GetRelativeAngularAccel() const
{
  return this->GetWorldPose().rot.RotateVectorReverse(
    this->GetWorldAngularAccel());
}

//////////////////////////////////////////////////
math::Vector3 Link::GetWorldAngularAccel() const
{
  // I: inertia matrix in world frame
  // T: sum of external torques in world frame
  // L: angular momentum of CoG in world frame
  // w: angular velocity in world frame
  // return I^-1 * (T - w x L)
  return this->GetWorldInertiaMatrix().Inverse() * (this->GetWorldTorque()
    - this->GetWorldAngularVel().Cross(this->GetWorldAngularMomentum()));
}

//////////////////////////////////////////////////
math::Vector3 Link::GetWorldAngularMomentum() const
{
  return this->GetWorldInertiaMatrix() * this->GetWorldAngularVel();
}

//////////////////////////////////////////////////
math::Vector3 Link::GetRelativeForce() const
{
  return this->GetWorldPose().rot.RotateVectorReverse(this->GetWorldForce());
}

//////////////////////////////////////////////////
math::Vector3 Link::GetRelativeTorque() const
{
  return this->GetWorldPose().rot.RotateVectorReverse(this->GetWorldTorque());
}

//////////////////////////////////////////////////
ModelPtr Link::GetModel() const
{
  return boost::dynamic_pointer_cast<Model>(this->GetParent());
}

//////////////////////////////////////////////////
math::Box Link::GetBoundingBox() const
{
  math::Box box;

  box.min.Set(GZ_DBL_MAX, GZ_DBL_MAX, GZ_DBL_MAX);
  box.max.Set(0, 0, 0);

  for (Collision_V::const_iterator iter = this->collisions.begin();
       iter != this->collisions.end(); ++iter)
  {
    box += (*iter)->GetBoundingBox();
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
  math::Pose inertialPose;
  if (this->inertial)
    inertialPose = this->inertial->GetPose();
  return inertialPose + this->GetWorldPose();
}

//////////////////////////////////////////////////
math::Matrix3 Link::GetWorldInertiaMatrix() const
{
  math::Matrix3 moi;
  if (this->inertial)
  {
    math::Vector3 pos = this->inertial->GetPose().pos;
    math::Quaternion rot = this->GetWorldPose().rot.GetInverse();
    moi = this->inertial->GetMOI(math::Pose(pos, rot));
  }
  return moi;
}

//////////////////////////////////////////////////
void Link::AddParentJoint(JointPtr _joint)
{
  this->parentJoints.push_back(_joint);
}

//////////////////////////////////////////////////
void Link::AddChildJoint(JointPtr _joint)
{
  this->childJoints.push_back(_joint);
}

//////////////////////////////////////////////////
void Link::RemoveParentJoint(const std::string &_jointName)
{
  for (std::vector<JointPtr>::iterator iter = this->parentJoints.begin();
                                       iter != this->parentJoints.end();
                                       ++iter)
  {
    /// @todo: can we assume there are no repeats?
    if ((*iter)->GetName() == _jointName)
    {
      this->parentJoints.erase(iter);
      break;
    }
  }
}

//////////////////////////////////////////////////
void Link::RemoveChildJoint(const std::string &_jointName)
{
  for (std::vector<JointPtr>::iterator iter = this->childJoints.begin();
                                       iter != this->childJoints.end();
                                       ++iter)
  {
    /// @todo: can we assume there are no repeats?
    if ((*iter)->GetName() == _jointName)
    {
      this->childJoints.erase(iter);
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
  msgs::Set(_msg.mutable_pose(), relPose);

  msgs::Set(this->visualMsg->mutable_pose(), relPose);
  _msg.add_visual()->CopyFrom(*this->visualMsg);

  _msg.mutable_inertial()->set_mass(this->inertial->GetMass());
  _msg.mutable_inertial()->set_ixx(this->inertial->GetIXX());
  _msg.mutable_inertial()->set_ixy(this->inertial->GetIXY());
  _msg.mutable_inertial()->set_ixz(this->inertial->GetIXZ());
  _msg.mutable_inertial()->set_iyy(this->inertial->GetIYY());
  _msg.mutable_inertial()->set_iyz(this->inertial->GetIYZ());
  _msg.mutable_inertial()->set_izz(this->inertial->GetIZZ());
  msgs::Set(_msg.mutable_inertial()->mutable_pose(), this->inertial->GetPose());

  for (auto &child : this->children)
  {
    if (child->HasType(Base::COLLISION))
    {
      CollisionPtr collision = boost::static_pointer_cast<Collision>(child);
      collision->FillMsg(*_msg.add_collision());
    }
  }

  for (std::vector<std::string>::iterator iter = this->sensors.begin();
      iter != this->sensors.end(); ++iter)
  {
    sensors::SensorPtr sensor = sensors::get_sensor(*iter);
    if (sensor)
      sensor->FillMsg(*_msg.add_sensor());
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
    msgs::Set(proj->mutable_pose(), elem->Get<math::Pose>("pose"));
  }

  if (this->IsCanonicalLink())
    _msg.set_canonical(true);

  for (auto &battery : this->batteries)
  {
    msgs::Battery *bat = _msg.add_battery();
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
    this->inertial->ProcessMsg(_msg.inertial());
    this->SetEnabled(true);
    this->UpdateMass();
  }

  if (_msg.has_pose())
  {
    this->SetEnabled(true);
    this->SetRelativePose(msgs::Convert(_msg.pose()));
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
  return this->sensors.size();
}

//////////////////////////////////////////////////
std::string Link::GetSensorName(unsigned int _i) const
{
  if (_i < this->sensors.size())
    return this->sensors[_i];

  return std::string();
}

//////////////////////////////////////////////////
void Link::AttachStaticModel(ModelPtr &_model, const math::Pose &_offset)
{
  if (!_model->IsStatic())
  {
    gzerr << "AttachStaticModel requires a static model\n";
    return;
  }

  this->attachedModels.push_back(_model);
  this->attachedModelsOffset.push_back(_offset);
}

//////////////////////////////////////////////////
void Link::DetachStaticModel(const std::string &_modelName)
{
  for (unsigned int i = 0; i < this->attachedModels.size(); i++)
  {
    if (this->attachedModels[i]->GetName() == _modelName)
    {
      this->attachedModels.erase(this->attachedModels.begin()+i);
      this->attachedModelsOffset.erase(this->attachedModelsOffset.begin()+i);
      break;
    }
  }
}

//////////////////////////////////////////////////
void Link::DetachAllStaticModels()
{
  this->attachedModels.clear();
  this->attachedModelsOffset.clear();
}

//////////////////////////////////////////////////
void Link::OnPoseChange()
{
  math::Pose p;
  for (unsigned int i = 0; i < this->attachedModels.size(); i++)
  {
    p = this->GetWorldPose();
    p.pos += this->attachedModelsOffset[i].pos;
    p.rot = p.rot * this->attachedModelsOffset[i].rot;

    this->attachedModels[i]->SetWorldPose(p, true);
  }
}

//////////////////////////////////////////////////
void Link::SetState(const LinkState &_state)
{
  this->SetWorldPose(_state.GetPose());

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
  {
    boost::recursive_mutex::scoped_lock lock(*this->publishDataMutex);
    if (this->publishData == _enable)
      return;

    this->publishData = _enable;
  }
  if (_enable)
  {
    std::string topic = "~/" + this->GetScopedName();
    this->dataPub = this->node->Advertise<msgs::LinkData>(topic);
    this->connections.push_back(
      event::Events::ConnectWorldUpdateEnd(
        boost::bind(&Link::PublishData, this)));
  }
  else
  {
    this->dataPub.reset();
    this->connections.clear();
  }
}

/////////////////////////////////////////////////
void Link::PublishData()
{
  if (this->publishData && this->dataPub->HasConnections())
  {
    msgs::Set(this->linkDataMsg.mutable_time(), this->world->GetSimTime());
    linkDataMsg.set_name(this->GetScopedName());
    msgs::Set(this->linkDataMsg.mutable_linear_velocity(),
        this->GetWorldLinearVel());
    msgs::Set(this->linkDataMsg.mutable_angular_velocity(),
        this->GetWorldAngularVel());
    this->dataPub->Publish(this->linkDataMsg);
  }
}

//////////////////////////////////////////////////
BatteryPtr Link::Battery(const std::string &_name) const
{
  BatteryPtr result;

  for (auto &battery : this->batteries)
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
BatteryPtr Link::Battery(size_t _index) const
{
  if (_index < this->batteries.size())
    return this->batteries[_index];
  else
    return BatteryPtr();
}

/////////////////////////////////////////////////
size_t Link::BatteryCount() const
{
  return this->batteries.size();
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
        this->audioSources.begin(); iter != this->audioSources.end(); ++iter)
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
  for (Collision_V::iterator iter = this->collisions.begin();
       iter != this->collisions.end(); ++iter)
  {
    if ((*iter)->GetName() == _name || (*iter)->GetScopedName() == _name)
    {
      this->collisions.erase(iter);
      break;
    }
  }
}

/////////////////////////////////////////////////
void Link::SetScale(const math::Vector3 &_scale)
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
void Link::UpdateVisualGeomSDF(const math::Vector3 &_scale)
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
        double newRadius = std::max(_scale.z, std::max(_scale.x, _scale.y));
        double oldRadius = std::max(this->scale.z,
            std::max(this->scale.x, this->scale.y));
        geomElem->GetElement("sphere")->GetElement("radius")->Set(
            newRadius/oldRadius*radius);
      }
      else if (geomElem->HasElement("cylinder"))
      {
        // update radius the same way as collision shapes
        double radius = geomElem->GetElement("cylinder")->Get<double>("radius");
        double newRadius = std::max(_scale.x, _scale.y);
        double oldRadius = std::max(this->scale.x, this->scale.y);

        double length = geomElem->GetElement("cylinder")->Get<double>("length");
        geomElem->GetElement("cylinder")->GetElement("radius")->Set(
            newRadius/oldRadius*radius);
        geomElem->GetElement("cylinder")->GetElement("length")->Set(
            _scale.z/this->scale.z*length);
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
  double m = this->GetInertial()->GetMass();
  math::Vector3 g = this->GetWorld()->GetPhysicsEngine()->GetGravity();
  math::Vector3 z = this->GetWorldCoGPose().pos;
  return -m * g.Dot(z);
}

/////////////////////////////////////////////////
double Link::GetWorldEnergyKinetic() const
{
  double energy = 0.0;

  // compute linear kinetic energy
  // E = 1/2 m v^T v
  {
    double m = this->GetInertial()->GetMass();
    math::Vector3 v = this->GetWorldCoGLinearVel();
    energy += 0.5 * m * v.Dot(v);
  }

  // compute angular kinetic energy
  // E = 1/2 w^T I w
  {
    math::Vector3 w = this->GetWorldAngularVel();
    math::Matrix3 I = this->GetWorldInertiaMatrix();
    energy += 0.5 * w.Dot(I * w);
  }

  return energy;
}

/////////////////////////////////////////////////
double Link::GetWorldEnergy() const
{
  return this->GetWorldEnergyPotential() + this->GetWorldEnergyKinetic();
}

/////////////////////////////////////////////////
void Link::MoveFrame(const math::Pose &_worldReferenceFrameSrc,
                     const math::Pose &_worldReferenceFrameDst)
{
  math::Pose targetWorldPose = (this->GetWorldPose() - _worldReferenceFrameSrc)
    + _worldReferenceFrameDst;
  this->SetWorldPose(targetWorldPose);
  this->SetWorldTwist(math::Vector3(0, 0, 0), math::Vector3(0, 0, 0));
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
void Link::OnWrenchMsg(ConstWrenchPtr &_msg)
{
  boost::mutex::scoped_lock lock(this->wrenchMsgMutex);
  this->wrenchMsgs.push_back(*_msg);
}

//////////////////////////////////////////////////
void Link::ProcessWrenchMsg(const msgs::Wrench &_msg)
{
  math::Vector3 pos = math::Vector3::Zero;
  if (_msg.has_force_offset())
  {
    pos = msgs::Convert(_msg.force_offset());
  }

  const math::Vector3 force = msgs::Convert(_msg.force());
  this->AddLinkForce(force, pos);

  const math::Vector3 torque = msgs::Convert(_msg.torque());
  this->AddRelativeTorque(torque);
}

//////////////////////////////////////////////////
void Link::LoadBattery(sdf::ElementPtr _sdf)
{
  BatteryPtr battery(new physics::Battery(
      boost::static_pointer_cast<Link>(shared_from_this())));
  battery->Load(_sdf);
  this->batteries.push_back(battery);
}
