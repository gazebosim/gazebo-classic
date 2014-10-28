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
#include <float.h>

#include <boost/thread/recursive_mutex.hpp>
#include <sstream>

#include "gazebo/util/OpenAL.hh"
#include "gazebo/common/KeyFrame.hh"
#include "gazebo/common/Animation.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/CommonTypes.hh"

#include "gazebo/physics/Gripper.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/JointController.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Contact.hh"

#include "gazebo/sensors/SensorManager.hh"

#include "gazebo/transport/Node.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
Model::Model(BasePtr _parent)
  : Entity(_parent)
{
  this->AddType(MODEL);
}

//////////////////////////////////////////////////
Model::~Model()
{
}

//////////////////////////////////////////////////
void Model::Load(sdf::ElementPtr _sdf)
{
  Entity::Load(_sdf);

  this->jointPub = this->node->Advertise<msgs::Joint>("~/joint");

  this->SetStatic(this->sdf->Get<bool>("static"));
  if (this->sdf->HasElement("static"))
  {
    this->sdf->GetElement("static")->GetValue()->SetUpdateFunc(
        boost::bind(&Entity::IsStatic, this));
  }

  if (this->sdf->HasElement("allow_auto_disable"))
    this->SetAutoDisable(this->sdf->Get<bool>("allow_auto_disable"));
  this->LoadLinks();

  // Load the joints if the world is already loaded. Otherwise, the World
  // has some special logic to load models that takes into account state
  // information.
  if (this->world->IsLoaded())
    this->LoadJoints();
}

//////////////////////////////////////////////////
void Model::LoadLinks()
{
  /// \TODO: check for duplicate model, and raise an error
  /// BasePtr dup = Base::GetByName(this->GetScopedName());

  // Load the bodies
  if (this->sdf->HasElement("link"))
  {
    sdf::ElementPtr linkElem = this->sdf->GetElement("link");
    bool canonicalLinkInitialized = false;
    while (linkElem)
    {
      // Create a new link
      LinkPtr link = this->GetWorld()->GetPhysicsEngine()->CreateLink(
          boost::static_pointer_cast<Model>(shared_from_this()));

      /// \TODO: canonical link is hardcoded to the first link.
      ///        warn users for now, need  to add parsing of
      ///        the canonical tag in sdf
      if (!canonicalLinkInitialized)
      {
        link->SetCanonicalLink(true);
        this->canonicalLink = link;
        canonicalLinkInitialized = true;
      }

      // Load the link using the config node. This also loads all of the
      // bodies collisionetries
      link->Load(linkElem);
      linkElem = linkElem->GetNextElement("link");
      this->links.push_back(link);
    }
  }
}

//////////////////////////////////////////////////
void Model::LoadJoints()
{
  // Load the joints
  if (this->sdf->HasElement("joint"))
  {
    sdf::ElementPtr jointElem = this->sdf->GetElement("joint");
    while (jointElem)
    {
      try
      {
        this->LoadJoint(jointElem);
      }
      catch(...)
      {
        gzerr << "LoadJoint Failed\n";
      }
      jointElem = jointElem->GetNextElement("joint");
    }
  }

  if (this->sdf->HasElement("gripper"))
  {
    sdf::ElementPtr gripperElem = this->sdf->GetElement("gripper");
    while (gripperElem)
    {
      this->LoadGripper(gripperElem);
      gripperElem = gripperElem->GetNextElement("gripper");
    }
  }
}

//////////////////////////////////////////////////
void Model::Init()
{
  // Record the model's initial pose (for reseting)
  this->SetInitialRelativePose(this->GetWorldPose());

  this->SetRelativePose(this->GetWorldPose());

  // Initialize the bodies before the joints
  for (Base_V::iterator iter = this->children.begin();
       iter != this->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::LINK))
    {
      LinkPtr link = boost::static_pointer_cast<Link>(*iter);
      if (link)
        link->Init();
      else
        gzerr << "Child [" << (*iter)->GetName()
              << "] has type Base::LINK, but cannot be dynamically casted\n";
    }
    else if ((*iter)->HasType(Base::MODEL))
      boost::static_pointer_cast<Model>(*iter)->Init();
  }

  // Initialize the joints last.
  for (Joint_V::iterator iter = this->joints.begin();
       iter != this->joints.end(); ++iter)
  {
    try
    {
      (*iter)->Init();
    }
    catch(...)
    {
      gzerr << "Init joint failed" << std::endl;
      return;
    }
    // The following message used to be filled and sent in Model::LoadJoint
    // It is moved here, after Joint::Init, so that the joint properties
    // can be included in the message.
    msgs::Joint msg;
    (*iter)->FillMsg(msg);
    this->jointPub->Publish(msg);
  }

  for (std::vector<GripperPtr>::iterator iter = this->grippers.begin();
       iter != this->grippers.end(); ++iter)
  {
    (*iter)->Init();
  }
}

//////////////////////////////////////////////////
void Model::Update()
{
  if (this->IsStatic())
    return;

  boost::recursive_mutex::scoped_lock lock(this->updateMutex);

  for (Joint_V::iterator jiter = this->joints.begin();
       jiter != this->joints.end(); ++jiter)
    (*jiter)->Update();

  if (this->jointController)
    this->jointController->Update();

  if (!this->jointAnimations.empty())
  {
    common::NumericKeyFrame kf(0);
    std::map<std::string, double> jointPositions;
    std::map<std::string, common::NumericAnimationPtr>::iterator iter;
    iter = this->jointAnimations.begin();
    while (iter != this->jointAnimations.end())
    {
      iter->second->GetInterpolatedKeyFrame(kf);

      iter->second->AddTime(
          (this->world->GetSimTime() - this->prevAnimationTime).Double());

      if (iter->second->GetTime() < iter->second->GetLength())
      {
        iter->second->GetInterpolatedKeyFrame(kf);
        jointPositions[iter->first] = kf.GetValue();
        ++iter;
      }
      else
      {
        this->jointAnimations.erase(iter++);
      }
    }
    if (!jointPositions.empty())
    {
      this->jointController->SetJointPositions(jointPositions);
    }
    else
    {
      if (this->onJointAnimationComplete)
        this->onJointAnimationComplete();
    }
    this->prevAnimationTime = this->world->GetSimTime();
  }
}

//////////////////////////////////////////////////
void Model::SetJointPosition(
  const std::string &_jointName, double _position, int _index)
{
  if (this->jointController)
    this->jointController->SetJointPosition(_jointName, _position, _index);
}

//////////////////////////////////////////////////
void Model::SetJointPositions(
    const std::map<std::string, double> &_jointPositions)
{
  if (this->jointController)
    this->jointController->SetJointPositions(_jointPositions);
}

//////////////////////////////////////////////////
void Model::RemoveChild(EntityPtr _child)
{
  Joint_V::iterator jiter;

  if (_child->HasType(LINK))
  {
    bool done = false;

    while (!done)
    {
      done = true;

      for (jiter = this->joints.begin(); jiter != this->joints.end(); ++jiter)
      {
        if (!(*jiter))
          continue;

        LinkPtr jlink0 = (*jiter)->GetJointLink(0);
        LinkPtr jlink1 = (*jiter)->GetJointLink(1);

        if (!jlink0 || !jlink1 || jlink0->GetName() == _child->GetName() ||
            jlink1->GetName() == _child->GetName() ||
            jlink0->GetName() == jlink1->GetName())
        {
          this->joints.erase(jiter);
          done = false;
          break;
        }
      }
    }

    this->RemoveLink(_child->GetScopedName());
  }

  Entity::RemoveChild(_child->GetId());

  for (Link_V::iterator liter = this->links.begin();
       liter != this->links.end(); ++liter)
  {
    (*liter)->SetEnabled(true);
  }
}

//////////////////////////////////////////////////
void Model::Fini()
{
  Entity::Fini();

  this->attachedModels.clear();
  this->joints.clear();
  this->plugins.clear();
  this->links.clear();
  this->canonicalLink.reset();
}

//////////////////////////////////////////////////
void Model::UpdateParameters(sdf::ElementPtr _sdf)
{
  Entity::UpdateParameters(_sdf);

  if (_sdf->HasElement("link"))
  {
    sdf::ElementPtr linkElem = _sdf->GetElement("link");
    while (linkElem)
    {
      LinkPtr link = boost::dynamic_pointer_cast<Link>(
          this->GetChild(linkElem->Get<std::string>("name")));
      link->UpdateParameters(linkElem);
      linkElem = linkElem->GetNextElement("link");
    }
  }
  /*

  if (_sdf->HasElement("joint"))
  {
    sdf::ElementPtr jointElem = _sdf->GetElement("joint");
    while (jointElem)
    {
      JointPtr joint = boost::dynamic_pointer_cast<Joint>(this->GetChild(jointElem->Get<std::string>("name")));
      joint->UpdateParameters(jointElem);
      jointElem = jointElem->GetNextElement("joint");
    }
  }
  */
}

//////////////////////////////////////////////////
const sdf::ElementPtr Model::GetSDF()
{
  return Entity::GetSDF();
}

//////////////////////////////////////////////////
void Model::Reset()
{
  Entity::Reset();

  this->ResetPhysicsStates();

  for (Joint_V::iterator jiter = this->joints.begin();
       jiter != this->joints.end(); ++jiter)
  {
    (*jiter)->Reset();
  }

  // Reset plugins after links and joints,
  // so that plugins can restore initial conditions
  for (std::vector<ModelPluginPtr>::iterator iter = this->plugins.begin();
       iter != this->plugins.end(); ++iter)
  {
    (*iter)->Reset();
  }
}

//////////////////////////////////////////////////
void Model::ResetPhysicsStates()
{
  // reset link velocities when resetting model
  for (Link_V::iterator liter = this->links.begin();
       liter != this->links.end(); ++liter)
  {
    (*liter)->ResetPhysicsStates();
  }
}

//////////////////////////////////////////////////
void Model::SetLinearVel(const math::Vector3 &_vel)
{
  for (Link_V::iterator iter = this->links.begin();
       iter != this->links.end(); ++iter)
  {
    if (*iter)
    {
      (*iter)->SetEnabled(true);
      (*iter)->SetLinearVel(_vel);
    }
  }
}

//////////////////////////////////////////////////
void Model::SetAngularVel(const math::Vector3 &_vel)
{
  for (Link_V::iterator iter = this->links.begin();
       iter != this->links.end(); ++iter)
  {
    if (*iter)
    {
      (*iter)->SetEnabled(true);
      (*iter)->SetAngularVel(_vel);
    }
  }
}

//////////////////////////////////////////////////
void Model::SetLinearAccel(const math::Vector3 &_accel)
{
  for (Link_V::iterator iter = this->links.begin();
       iter != this->links.end(); ++iter)
  {
    if (*iter)
    {
      (*iter)->SetEnabled(true);
      (*iter)->SetLinearAccel(_accel);
    }
  }
}

//////////////////////////////////////////////////
void Model::SetAngularAccel(const math::Vector3 &_accel)
{
  for (Link_V::iterator iter = this->links.begin();
       iter != this->links.end(); ++iter)
  {
    if (*iter)
    {
      (*iter)->SetEnabled(true);
      (*iter)->SetAngularAccel(_accel);
    }
  }
}

//////////////////////////////////////////////////
math::Vector3 Model::GetRelativeLinearVel() const
{
  if (this->GetLink("canonical"))
    return this->GetLink("canonical")->GetRelativeLinearVel();
  else
    return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
math::Vector3 Model::GetWorldLinearVel() const
{
  if (this->GetLink("canonical"))
    return this->GetLink("canonical")->GetWorldLinearVel();
  else
    return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
math::Vector3 Model::GetRelativeAngularVel() const
{
  if (this->GetLink("canonical"))
    return this->GetLink("canonical")->GetRelativeAngularVel();
  else
    return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
math::Vector3 Model::GetWorldAngularVel() const
{
  if (this->GetLink("canonical"))
    return this->GetLink("canonical")->GetWorldAngularVel();
  else
    return math::Vector3(0, 0, 0);
}


//////////////////////////////////////////////////
math::Vector3 Model::GetRelativeLinearAccel() const
{
  if (this->GetLink("canonical"))
    return this->GetLink("canonical")->GetRelativeLinearAccel();
  else
    return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
math::Vector3 Model::GetWorldLinearAccel() const
{
  if (this->GetLink("canonical"))
    return this->GetLink("canonical")->GetWorldLinearAccel();
  else
    return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
math::Vector3 Model::GetRelativeAngularAccel() const
{
  if (this->GetLink("canonical"))
    return this->GetLink("canonical")->GetRelativeAngularAccel();
  else
    return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
math::Vector3 Model::GetWorldAngularAccel() const
{
  if (this->GetLink("canonical"))
    return this->GetLink("canonical")->GetWorldAngularAccel();
  else
    return math::Vector3(0, 0, 0);
}

//////////////////////////////////////////////////
math::Box Model::GetBoundingBox() const
{
  math::Box box;

  box.min.Set(FLT_MAX, FLT_MAX, FLT_MAX);
  box.max.Set(-FLT_MAX, -FLT_MAX, -FLT_MAX);

  for (Link_V::const_iterator iter = this->links.begin();
       iter != this->links.end(); ++iter)
  {
    if (*iter)
    {
      math::Box linkBox;
      linkBox = (*iter)->GetBoundingBox();
      box += linkBox;
    }
  }

  return box;
}

//////////////////////////////////////////////////
unsigned int Model::GetJointCount() const
{
  return this->joints.size();
}

//////////////////////////////////////////////////
const Joint_V &Model::GetJoints() const
{
  return this->joints;
}

//////////////////////////////////////////////////
JointPtr Model::GetJoint(const std::string &_name)
{
  JointPtr result;
  Joint_V::iterator iter;

  for (iter = this->joints.begin(); iter != this->joints.end(); ++iter)
  {
    if ((*iter)->GetScopedName() == _name || (*iter)->GetName() == _name)
    {
      result = (*iter);
      break;
    }
  }

  return result;
}

//////////////////////////////////////////////////
LinkPtr Model::GetLinkById(unsigned int _id) const
{
  return boost::dynamic_pointer_cast<Link>(this->GetById(_id));
}

//////////////////////////////////////////////////
const Link_V &Model::GetLinks() const
{
  return this->links;
}

//////////////////////////////////////////////////
LinkPtr Model::GetLink(const std::string &_name) const
{
  Link_V::const_iterator iter;
  LinkPtr result;

  if (_name == "canonical")
  {
    result = this->canonicalLink;
  }
  else
  {
    for (iter = this->links.begin(); iter != this->links.end(); ++iter)
    {
      if (((*iter)->GetScopedName() == _name) || ((*iter)->GetName() == _name))
      {
        result = *iter;
        break;
      }
    }
  }

  return result;
}

//////////////////////////////////////////////////
void Model::LoadJoint(sdf::ElementPtr _sdf)
{
  JointPtr joint;

  std::string stype = _sdf->Get<std::string>("type");

  joint = this->GetWorld()->GetPhysicsEngine()->CreateJoint(stype,
     boost::static_pointer_cast<Model>(shared_from_this()));
  if (!joint)
  {
    gzerr << "Unable to create joint of type[" << stype << "]\n";
    // gzthrow("Unable to create joint of type[" + stype + "]\n");
    return;
  }

  joint->SetModel(boost::static_pointer_cast<Model>(shared_from_this()));

  // Load the joint
  joint->Load(_sdf);

  if (this->GetJoint(joint->GetScopedName()) != NULL)
  {
    gzerr << "can't have two joint with the same name\n";
    gzthrow("can't have two joints with the same name ["+
      joint->GetScopedName() + "]\n");
  }

  this->joints.push_back(joint);

  if (!this->jointController)
    this->jointController.reset(new JointController(
        boost::dynamic_pointer_cast<Model>(shared_from_this())));
  this->jointController->AddJoint(joint);
}

//////////////////////////////////////////////////
void Model::LoadGripper(sdf::ElementPtr _sdf)
{
  GripperPtr gripper(new Gripper(
      boost::static_pointer_cast<Model>(shared_from_this())));
  gripper->Load(_sdf);
  this->grippers.push_back(gripper);
}

//////////////////////////////////////////////////
void Model::LoadPlugins()
{
  // Check to see if we need to load any model plugins
  if (this->GetPluginCount() > 0)
  {
    int iterations = 0;

    // Wait for the sensors to be initialized before loading
    // plugins, if there are any sensors
    while (this->GetSensorCount() > 0 &&
        !sensors::SensorManager::Instance()->SensorsInitialized() &&
        iterations < 50)
    {
      common::Time::MSleep(100);
      iterations++;
    }

    // Load the plugins if the sensors have been loaded, or if there
    // are no sensors attached to the model.
    if (iterations < 50)
    {
      // Load the plugins
      sdf::ElementPtr pluginElem = this->sdf->GetElement("plugin");
      while (pluginElem)
      {
        this->LoadPlugin(pluginElem);
        pluginElem = pluginElem->GetNextElement("plugin");
      }
    }
    else
    {
      gzerr << "Sensors failed to initialize when loading model["
        << this->GetName() << "] via the factory mechanism."
        << "Plugins for the model will not be loaded.\n";
    }
  }
}

//////////////////////////////////////////////////
unsigned int Model::GetPluginCount() const
{
  unsigned int result = 0;

  // Count all the plugins specified in SDF
  if (this->sdf->HasElement("plugin"))
  {
    sdf::ElementPtr pluginElem = this->sdf->GetElement("plugin");
    while (pluginElem)
    {
      result++;
      pluginElem = pluginElem->GetNextElement("plugin");
    }
  }

  return result;
}

//////////////////////////////////////////////////
unsigned int Model::GetSensorCount() const
{
  unsigned int result = 0;

  // Count all the sensors on all the links
  for (Link_V::const_iterator iter = this->links.begin();
       iter != this->links.end(); ++iter)
  {
    result += (*iter)->GetSensorCount();
  }

  return result;
}

//////////////////////////////////////////////////
void Model::LoadPlugin(sdf::ElementPtr _sdf)
{
  std::string pluginName = _sdf->Get<std::string>("name");
  std::string filename = _sdf->Get<std::string>("filename");
  gazebo::ModelPluginPtr plugin =
    gazebo::ModelPlugin::Create(filename, pluginName);
  if (plugin)
  {
    if (plugin->GetType() != MODEL_PLUGIN)
    {
      gzerr << "Model[" << this->GetName() << "] is attempting to load "
            << "a plugin, but detected an incorrect plugin type. "
            << "Plugin filename[" << filename << "] name["
            << pluginName << "]\n";
      return;
    }

    ModelPtr myself = boost::static_pointer_cast<Model>(shared_from_this());
    plugin->Load(myself, _sdf);
    plugin->Init();
    this->plugins.push_back(plugin);
  }
}

//////////////////////////////////////////////////
void Model::SetGravityMode(const bool &_v)
{
  for (Link_V::iterator liter = this->links.begin();
      liter != this->links.end(); ++liter)
  {
    if (*liter)
    {
      (*liter)->SetGravityMode(_v);
    }
  }
}

//////////////////////////////////////////////////
void Model::SetCollideMode(const std::string &_m)
{
  for (Link_V::iterator liter = this->links.begin();
      liter != this->links.end(); ++liter)
  {
    if (*liter)
    {
      (*liter)->SetCollideMode(_m);
    }
  }
}

//////////////////////////////////////////////////
void Model::SetLaserRetro(const float _retro)
{
  for (Link_V::iterator liter = this->links.begin();
      liter != this->links.end(); ++liter)
  {
    if (*liter)
    {
      (*liter)->SetLaserRetro(_retro);
    }
  }
}

//////////////////////////////////////////////////
void Model::FillMsg(msgs::Model &_msg)
{
  _msg.set_name(this->GetScopedName());
  _msg.set_is_static(this->IsStatic());
  msgs::Set(_msg.mutable_pose(), this->GetWorldPose());
  _msg.set_id(this->GetId());
  msgs::Set(_msg.mutable_scale(), this->scale);

  msgs::Set(this->visualMsg->mutable_pose(), this->GetWorldPose());
  _msg.add_visual()->CopyFrom(*this->visualMsg);

  for (Link_V::iterator iter = this->links.begin(); iter != this->links.end();
      ++iter)
  {
    (*iter)->FillMsg(*_msg.add_link());
  }

  for (Joint_V::iterator iter = this->joints.begin();
       iter != this->joints.end(); ++iter)
  {
    (*iter)->FillMsg(*_msg.add_joint());
  }
}

//////////////////////////////////////////////////
void Model::ProcessMsg(const msgs::Model &_msg)
{
  if (_msg.has_id() && _msg.id() != this->GetId())
  {
    gzerr << "Incorrect ID[" << _msg.id() << " != " << this->GetId() << "]\n";
    return;
  }
  else if ((_msg.has_id() && _msg.id() != this->GetId()) &&
            _msg.name() != this->GetScopedName())
  {
    gzerr << "Incorrect name[" << _msg.name() << " != " << this->GetName()
      << "]\n";
    return;
  }

  this->SetName(this->world->StripWorldName(_msg.name()));
  if (_msg.has_pose())
    this->SetWorldPose(msgs::Convert(_msg.pose()));
  for (int i = 0; i < _msg.link_size(); i++)
  {
    LinkPtr link = this->GetLinkById(_msg.link(i).id());
    if (link)
      link->ProcessMsg(_msg.link(i));
  }

  if (_msg.has_is_static())
    this->SetStatic(_msg.is_static());

  if (_msg.has_scale())
    this->SetScale(msgs::Convert(_msg.scale()));
}

//////////////////////////////////////////////////
void Model::SetJointAnimation(
    const std::map<std::string, common::NumericAnimationPtr> &_anims,
    boost::function<void()> _onComplete)
{
  boost::recursive_mutex::scoped_lock lock(this->updateMutex);
  std::map<std::string, common::NumericAnimationPtr>::const_iterator iter;
  for (iter = _anims.begin(); iter != _anims.end(); ++iter)
  {
    this->jointAnimations[iter->first] = iter->second;
  }
  this->onJointAnimationComplete = _onComplete;
  this->prevAnimationTime = this->world->GetSimTime();
}

//////////////////////////////////////////////////
void Model::StopAnimation()
{
  boost::recursive_mutex::scoped_lock lock(this->updateMutex);
  Entity::StopAnimation();
  this->onJointAnimationComplete.clear();
  this->jointAnimations.clear();
}

//////////////////////////////////////////////////
void Model::AttachStaticModel(ModelPtr &_model, math::Pose _offset)
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
void Model::DetachStaticModel(const std::string &_modelName)
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
void Model::OnPoseChange()
{
  math::Pose p;
  for (unsigned int i = 0; i < this->attachedModels.size(); i++)
  {
    p = this->GetWorldPose();
    p += this->attachedModelsOffset[i];
    this->attachedModels[i]->SetWorldPose(p, true);
  }
}

//////////////////////////////////////////////////
void Model::SetState(const ModelState &_state)
{
  this->SetWorldPose(_state.GetPose(), true);

  LinkState_M linkStates = _state.GetLinkStates();
  for (LinkState_M::iterator iter = linkStates.begin();
       iter != linkStates.end(); ++iter)
  {
    LinkPtr link = this->GetLink(iter->first);
    if (link)
      link->SetState(iter->second);
    else
      gzerr << "Unable to find link[" << iter->first << "]\n";
  }

  // For now we don't use the joint state values to set the state of
  // simulation.
  // for (unsigned int i = 0; i < _state.GetJointStateCount(); ++i)
  // {
  //   JointState jointState = _state.GetJointState(i);
  //   this->SetJointPosition(this->GetName() + "::" + jointState.GetName(),
  //                          jointState.GetAngle(0).Radian());
  // }
}

/////////////////////////////////////////////////
void Model::SetScale(const math::Vector3 &_scale)
{
  if (this->scale == _scale)
    return;

  this->scale = _scale;

  Base_V::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); ++iter)
  {
    if (*iter && (*iter)->HasType(LINK))
    {
      boost::static_pointer_cast<Link>(*iter)->SetScale(_scale);
    }
  }
}

/////////////////////////////////////////////////
void Model::SetEnabled(bool _enabled)
{
  for (Link_V::iterator liter = this->links.begin();
      liter != this->links.end(); ++liter)
  {
    if (*liter)
      (*liter)->SetEnabled(_enabled);
  }
}

/////////////////////////////////////////////////
void Model::SetLinkWorldPose(const math::Pose &_pose, std::string _linkName)
{
  // look for link matching link name
  LinkPtr link = this->GetLink(_linkName);
  if (link)
    this->SetLinkWorldPose(_pose, link);
  else
    gzerr << "Setting Model Pose by specifying Link failed:"
          << " Link[" << _linkName << "] not found.\n";
}

/////////////////////////////////////////////////
void Model::SetLinkWorldPose(const math::Pose &_pose, const LinkPtr &_link)
{
  math::Pose linkPose = _link->GetWorldPose();
  math::Pose currentModelPose = this->GetWorldPose();
  math::Pose linkRelPose = currentModelPose - linkPose;
  math::Pose targetModelPose =  linkRelPose * _pose;
  this->SetWorldPose(targetModelPose);
}

/////////////////////////////////////////////////
void Model::SetAutoDisable(bool _auto)
{
  if (!this->joints.empty())
    return;

  for (Link_V::iterator liter = this->links.begin();
      liter != this->links.end(); ++liter)
  {
    if (*liter)
    {
      (*liter)->SetAutoDisable(_auto);
    }
  }
}

/////////////////////////////////////////////////
bool Model::GetAutoDisable() const
{
  return this->sdf->Get<bool>("allow_auto_disable");
}

/////////////////////////////////////////////////
void Model::RemoveLink(const std::string &_name)
{
  for (Link_V::iterator iter = this->links.begin();
       iter != this->links.end(); ++iter)
  {
    if ((*iter)->GetName() == _name || (*iter)->GetScopedName() == _name)
    {
      this->links.erase(iter);
      break;
    }
  }
}
/////////////////////////////////////////////////
JointControllerPtr Model::GetJointController()
{
  return this->jointController;
}

/////////////////////////////////////////////////
GripperPtr Model::GetGripper(size_t _index) const
{
  if (_index < this->grippers.size())
    return this->grippers[_index];
  else
    return GripperPtr();
}

/////////////////////////////////////////////////
size_t Model::GetGripperCount() const
{
  return this->grippers.size();
}

/////////////////////////////////////////////////
double Model::GetWorldEnergyPotential() const
{
  double e = 0;
  for (Link_V::const_iterator iter = this->links.begin();
    iter != this->links.end(); ++iter)
  {
    e += (*iter)->GetWorldEnergyPotential();
  }
  for (Joint_V::const_iterator iter = this->joints.begin();
    iter != this->joints.end(); ++iter)
  {
    for (unsigned int j = 0; j < (*iter)->GetAngleCount(); ++j)
    {
      e += (*iter)->GetWorldEnergyPotentialSpring(j);
    }
  }
  return e;
}

/////////////////////////////////////////////////
double Model::GetWorldEnergyKinetic() const
{
  double e = 0;
  for (Link_V::const_iterator iter = this->links.begin();
    iter != this->links.end(); ++iter)
  {
    e += (*iter)->GetWorldEnergyKinetic();
  }
  return e;
}

/////////////////////////////////////////////////
double Model::GetWorldEnergy() const
{
  return this->GetWorldEnergyPotential() + this->GetWorldEnergyKinetic();
}
