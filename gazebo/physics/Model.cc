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

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <float.h>

#include <boost/bind.hpp>
#include <boost/function.hpp>
#include <boost/thread/recursive_mutex.hpp>
#include <ignition/math/Pose3.hh>
#include <ignition/msgs/plugin_v.pb.h>
#include <sstream>

#include "gazebo/common/KeyFrame.hh"
#include "gazebo/common/Animation.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/URI.hh"

#include "gazebo/physics/Gripper.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/JointController.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Model.hh"
#include "gazebo/physics/Contact.hh"

#include "gazebo/transport/Node.hh"

#include "gazebo/util/IgnMsgSdf.hh"
#include "gazebo/util/IntrospectionManager.hh"
#include "gazebo/util/OpenAL.hh"

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
  this->Fini();
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

  if (this->sdf->HasElement("self_collide"))
  {
    this->SetSelfCollide(this->sdf->Get<bool>("self_collide"));
  }

  if (this->sdf->HasElement("enable_wind"))
  {
    this->SetWindMode(this->sdf->Get<bool>("enable_wind"));
  }

  if (this->sdf->HasElement("allow_auto_disable"))
    this->SetAutoDisable(this->sdf->Get<bool>("allow_auto_disable"));

  this->LoadLinks();

  this->LoadModels();

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
    while (linkElem)
    {
      // Create a new link
      LinkPtr link = this->GetWorld()->Physics()->CreateLink(
          boost::static_pointer_cast<Model>(shared_from_this()));

      /// \TODO: canonical link is hardcoded to the first link.
      ///        warn users for now, need  to add parsing of
      ///        the canonical tag in sdf

      // find canonical link - there should only be one within a tree of models
      if (!this->canonicalLink)
      {
        // Get the canonical link from parent, if not found then set the
        // current link as the canonoical link.
        LinkPtr cLink;
        BasePtr entity = this->GetParent();
        while (entity && entity->HasType(MODEL))
        {
          ModelPtr model = boost::static_pointer_cast<Model>(entity);
          LinkPtr tmpLink = model->GetLink();
          if (tmpLink)
          {
            cLink = tmpLink;
            break;
          }
          entity = entity->GetParent();
        }

        if (cLink)
        {
          this->canonicalLink = cLink;
        }
        else
        {
          // first link found, set as canonical link
          link->SetCanonicalLink(true);
          this->canonicalLink = link;

          // notify parent models of this canonical link
          entity = this->GetParent();
          while (entity && entity->HasType(MODEL))
          {
            ModelPtr model = boost::static_pointer_cast<Model>(entity);
            model->canonicalLink = this->canonicalLink;
            entity = entity->GetParent();
          }
        }
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
void Model::LoadModels()
{
  // Load the models
  if (this->sdf->HasElement("model"))
  {
    sdf::ElementPtr modelElem = this->sdf->GetElement("model");
    while (modelElem)
    {
      // Create a new model
      ModelPtr model = this->GetWorld()->Physics()->CreateModel(
          boost::static_pointer_cast<Model>(shared_from_this()));
      model->SetWorld(this->GetWorld());
      model->Load(modelElem);
      this->models.push_back(model);
      modelElem = modelElem->GetNextElement("model");
    }

    for (auto &model : this->models)
      model->SetEnabled(true);
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

  // Load nested model joints if the world is not already loaded. Otherwise,
  // LoadJoints will be called from Model::Load.
  if (!this->world->IsLoaded())
  {
    for (auto model : this->models)
      model->LoadJoints();
  }
}

//////////////////////////////////////////////////
void Model::Init()
{
  // Record the model's initial pose (for reseting)
  ignition::math::Pose3d initPose =
    this->sdf->Get<ignition::math::Pose3d>("pose");
  this->SetInitialRelativePose(initPose);
  this->SetRelativePose(initPose);

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
          (this->world->SimTime() - this->prevAnimationTime).Double());

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
    this->prevAnimationTime = this->world->SimTime();
  }

  for (auto &model : this->models)
    model->Update();
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
boost::shared_ptr<Model> Model::shared_from_this()
{
  return boost::static_pointer_cast<Model>(Entity::shared_from_this());
}

//////////////////////////////////////////////////
void Model::Fini()
{
  // Destroy all attached models
  for (auto &model : this->attachedModels)
  {
    if (model)
      model->Fini();
  }
  this->attachedModels.clear();

  // Destroy all models
  for (auto &model : this->models)
  {
    if (model)
      model->Fini();
  }
  this->models.clear();

  // Destroy all joints
  for (auto &joint : this->joints)
  {
    if (joint)
      joint->Fini();
  }
  this->joints.clear();
  this->jointController.reset();

  // Destroy all links
  for (auto &link : this->links)
  {
    if (link)
      link->Fini();
  }
  this->canonicalLink.reset();
  this->links.clear();

  this->plugins.clear();

  Entity::Fini();
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
      auto linkName = linkElem->Get<std::string>("name");
      LinkPtr link = boost::dynamic_pointer_cast<Link>(
          this->GetChild(linkName));
      if (link)
        link->UpdateParameters(linkElem);
      else
      {
        gzwarn << "Model [" << this->GetName() <<
            "] doesn't have a link called [" << linkName << "]" << std::endl;
      }
      linkElem = linkElem->GetNextElement("link");
    }
  }

  if (_sdf->HasElement("joint"))
  {
    sdf::ElementPtr jointElem = _sdf->GetElement("joint");
    while (jointElem)
    {
      auto jointName = jointElem->Get<std::string>("name");
      JointPtr joint = boost::dynamic_pointer_cast<Joint>(
          this->GetChild(jointName));
      if (joint)
        joint->UpdateParameters(jointElem);
      else
      {
        gzwarn << "Model [" << this->GetName() <<
            "] doesn't have a joint called [" << jointName << "]" << std::endl;
      }
      jointElem = jointElem->GetNextElement("joint");
    }
  }

  if (_sdf->HasElement("model"))
  {
    sdf::ElementPtr modelElem = _sdf->GetElement("model");
    while (modelElem)
    {
      auto modelName = modelElem->Get<std::string>("name");
      ModelPtr model = boost::dynamic_pointer_cast<Model>(
          this->GetChild(modelName));
      if (model)
        model->UpdateParameters(modelElem);
      else
      {
        gzwarn << "Model [" << this->GetName() <<
            "] doesn't have a nested model called [" << modelName << "]"
            << std::endl;
      }
      modelElem = modelElem->GetNextElement("model");
    }
  }
}

//////////////////////////////////////////////////
const sdf::ElementPtr Model::GetSDF()
{
  return Entity::GetSDF();
}

//////////////////////////////////////////////////
const sdf::ElementPtr Model::UnscaledSDF()
{
  GZ_ASSERT(this->sdf != NULL, "Model sdf member is NULL");
  this->sdf->Update();

  sdf::ElementPtr unscaledSdf(this->sdf);

  // Go through all collisions and visuals and divide size by scale
  // See Link::UpdateVisualGeomSDF
  if (!this->sdf->HasElement("link"))
    return unscaledSdf;

  auto linkElem = this->sdf->GetElement("link");
  while (linkElem)
  {
    // Visuals
    if (linkElem->HasElement("visual"))
    {
      auto visualElem = linkElem->GetElement("visual");
      while (visualElem)
      {
        auto geomElem = visualElem->GetElement("geometry");

        if (geomElem->HasElement("box"))
        {
          auto size = geomElem->GetElement("box")->
              Get<ignition::math::Vector3d>("size");
          geomElem->GetElement("box")->GetElement("size")->Set(
              size / this->scale);
        }
        else if (geomElem->HasElement("sphere"))
        {
          double radius = geomElem->GetElement("sphere")->Get<double>("radius");
          geomElem->GetElement("sphere")->GetElement("radius")->Set(
              radius/this->scale.Max());
        }
        else if (geomElem->HasElement("cylinder"))
        {
          double radius =
              geomElem->GetElement("cylinder")->Get<double>("radius");
          double length =
              geomElem->GetElement("cylinder")->Get<double>("length");
          double radiusScale = std::max(this->scale.X(), this->scale.Y());

          geomElem->GetElement("cylinder")->GetElement("radius")->Set(
              radius/radiusScale);
          geomElem->GetElement("cylinder")->GetElement("length")->Set(
              length/this->scale.Z());
        }
        else if (geomElem->HasElement("mesh"))
        {
          // Keep mesh scale because meshes can't be scaled yet (issue #1473)
        }

        visualElem = visualElem->GetNextElement("visual");
      }
    }

    // Collisions
    if (linkElem->HasElement("collision"))
    {
      auto collisionElem = linkElem->GetElement("collision");
      while (collisionElem)
      {
        auto geomElem = collisionElem->GetElement("geometry");

        if (geomElem->HasElement("box"))
        {
          auto size = geomElem->GetElement("box")->
              Get<ignition::math::Vector3d>("size");
          geomElem->GetElement("box")->GetElement("size")->Set(
              size / this->scale);
        }
        else if (geomElem->HasElement("sphere"))
        {
          double radius = geomElem->GetElement("sphere")->Get<double>("radius");
          geomElem->GetElement("sphere")->GetElement("radius")->Set(
              radius/this->scale.Max());
        }
        else if (geomElem->HasElement("cylinder"))
        {
          double radius =
              geomElem->GetElement("cylinder")->Get<double>("radius");
          double length =
              geomElem->GetElement("cylinder")->Get<double>("length");
          double radiusScale = std::max(this->scale.X(), this->scale.Y());

          geomElem->GetElement("cylinder")->GetElement("radius")->Set(
              radius/radiusScale);
          geomElem->GetElement("cylinder")->GetElement("length")->Set(
              length/this->scale.Z());
        }
        else if (geomElem->HasElement("mesh"))
        {
          // Keep mesh scale because meshes can't be scaled yet (issue #1473)
        }

        collisionElem = collisionElem->GetNextElement("collision");
      }
    }

    linkElem = linkElem->GetNextElement("link");
  }

  return unscaledSdf;
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

  // reset nested model physics states
  for (auto &m : this->models)
    m->ResetPhysicsStates();
}

//////////////////////////////////////////////////
void Model::SetLinearVel(const ignition::math::Vector3d &_vel)
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
void Model::SetAngularVel(const ignition::math::Vector3d &_vel)
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
void Model::SetLinearAccel(const ignition::math::Vector3d &_accel)
{
  gzwarn << "Model::SetLinearAccel() is deprecated and has no effect. "
         << "Use Link::SetForce() on the link directly instead. \n";
  for (Link_V::iterator iter = this->links.begin();
       iter != this->links.end(); ++iter)
  {
    if (*iter)
    {
      (*iter)->SetEnabled(true);
#ifndef _WIN32
      #pragma GCC diagnostic push
      #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
      (*iter)->SetLinearAccel(_accel);
#ifndef _WIN32
      #pragma GCC diagnostic pop
#endif
    }
  }
}

//////////////////////////////////////////////////
void Model::SetAngularAccel(const ignition::math::Vector3d &_accel)
{
  gzwarn << "Model::SetAngularAccel() is deprecated and has no effect. "
         << "Use Link::SetTorque() on the link directly instead. \n";
  for (Link_V::iterator iter = this->links.begin();
       iter != this->links.end(); ++iter)
  {
    if (*iter)
    {
      (*iter)->SetEnabled(true);
#ifndef _WIN32
      #pragma GCC diagnostic push
      #pragma GCC diagnostic ignored "-Wdeprecated-declarations"
#endif
      (*iter)->SetAngularAccel(_accel);
#ifndef _WIN32
      #pragma GCC diagnostic pop
#endif
    }
  }
}

//////////////////////////////////////////////////
ignition::math::Vector3d Model::RelativeLinearVel() const
{
  if (this->GetLink("canonical"))
    return this->GetLink("canonical")->RelativeLinearVel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Model::WorldLinearVel() const
{
  if (this->GetLink("canonical"))
  {
    return this->GetLink("canonical")->WorldLinearVel();
  }
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Model::RelativeAngularVel() const
{
  if (this->GetLink("canonical"))
    return this->GetLink("canonical")->RelativeAngularVel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Model::WorldAngularVel() const
{
  if (this->GetLink("canonical"))
    return this->GetLink("canonical")->WorldAngularVel();
  else
    return ignition::math::Vector3d::Zero;
}


//////////////////////////////////////////////////
ignition::math::Vector3d Model::RelativeLinearAccel() const
{
  if (this->GetLink("canonical"))
    return this->GetLink("canonical")->RelativeLinearAccel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Model::WorldLinearAccel() const
{
  if (this->GetLink("canonical"))
    return this->GetLink("canonical")->WorldLinearAccel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Model::RelativeAngularAccel() const
{
  if (this->GetLink("canonical"))
    return this->GetLink("canonical")->RelativeAngularAccel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Vector3d Model::WorldAngularAccel() const
{
  if (this->GetLink("canonical"))
    return this->GetLink("canonical")->WorldAngularAccel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Box Model::BoundingBox() const
{
  ignition::math::Box box;

  box.Min().Set(FLT_MAX, FLT_MAX, FLT_MAX);
  box.Max().Set(-FLT_MAX, -FLT_MAX, -FLT_MAX);

  for (const auto &iter : this->links)
  {
    if (iter)
    {
      ignition::math::Box linkBox;
      linkBox = iter->BoundingBox();
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
const Model_V &Model::NestedModels() const
{
  return this->models;
}

//////////////////////////////////////////////////
ModelPtr Model::NestedModel(const std::string &_name) const
{
  ModelPtr result;

  for (auto &m : this->models)
  {
    if ((m->GetScopedName() == _name) || (m->GetName() == _name))
    {
      result = m;
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

  joint = this->GetWorld()->Physics()->CreateJoint(stype,
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
std::vector<std::string> Model::SensorScopedName(
  const std::string &_name) const
{
  std::vector<std::string> names;
  for (Link_V::const_iterator iter = this->links.begin();
       iter != this->links.end(); ++iter)
  {
    for (unsigned int j = 0; j < (*iter)->GetSensorCount(); ++j)
    {
      const auto sensorName = (*iter)->GetSensorName(j);
      if (sensorName.size() < _name.size())
      {
        continue;
      }
      if (sensorName.substr(sensorName.size() - _name.size(), _name.size()) ==
          _name)
      {
        names.push_back(sensorName);
      }
    }
  }

  return names;
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
    while (this->GetSensorCount() > 0 && !this->world->SensorsInitialized() &&
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
        << " Plugins for the model will not be loaded.\n";
    }
  }

  for (auto &model : this->models)
    model->LoadPlugins();
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

  gazebo::ModelPluginPtr plugin;

  try
  {
    plugin = gazebo::ModelPlugin::Create(filename, pluginName);
  }
  catch(...)
  {
    gzerr << "Exception occured in the constructor of plugin with name["
      << pluginName << "] and filename[" << filename << "]. "
      << "This plugin will not run.\n";

    // Log the message. gzerr has problems with this in 1.9. Remove the
    // gzlog command in gazebo2.
    gzlog << "Exception occured in the constructor of plugin with name["
      << pluginName << "] and filename[" << filename << "]. "
      << "This plugin will not run." << std::endl;
    return;
  }

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

    try
    {
      plugin->Load(myself, _sdf);
    }
    catch(...)
    {
      gzerr << "Exception occured in the Load function of plugin with name["
        << pluginName << "] and filename[" << filename << "]. "
        << "This plugin will not run.\n";

      // Log the message. gzerr has problems with this in 1.9. Remove the
      // gzlog command in gazebo2.
      gzlog << "Exception occured in the Load function of plugin with name["
        << pluginName << "] and filename[" << filename << "]. "
        << "This plugin will not run." << std::endl;
      return;
    }

    try
    {
      plugin->Init();
    }
    catch(...)
    {
      gzerr << "Exception occured in the Init function of plugin with name["
        << pluginName << "] and filename[" << filename << "]. "
        << "This plugin will not run\n";

      // Log the message. gzerr has problems with this in 1.9. Remove the
      // gzlog command in gazebo2.
      gzlog << "Exception occured in the Init function of plugin with name["
        << pluginName << "] and filename[" << filename << "]. "
        << "This plugin will not run." << std::endl;
      return;
    }

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
  ignition::math::Pose3d relPose = this->RelativePose();

  _msg.set_name(this->GetScopedName());
  _msg.set_is_static(this->IsStatic());
  _msg.set_self_collide(this->GetSelfCollide());
  _msg.set_enable_wind(this->WindMode());
  msgs::Set(_msg.mutable_pose(), relPose);
  _msg.set_id(this->GetId());
  msgs::Set(_msg.mutable_scale(), this->scale);

  msgs::Set(this->visualMsg->mutable_pose(), relPose);
  _msg.add_visual()->CopyFrom(*this->visualMsg);

  for (const auto &link : this->links)
  {
    link->FillMsg(*_msg.add_link());
  }

  for (const auto &joint : this->joints)
  {
    joint->FillMsg(*_msg.add_joint());
  }
  for (const auto &model : this->models)
  {
    model->FillMsg(*_msg.add_model());
  }

  if (this->sdf->HasElement("plugin"))
  {
    sdf::ElementPtr pluginElem = this->sdf->GetElement("plugin");
    while (pluginElem)
    {
      auto pluginMsg = _msg.add_plugin();
      pluginMsg->CopyFrom(msgs::PluginFromSDF(pluginElem));

      pluginElem = pluginElem->GetNextElement("plugin");
    }
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
    this->SetWorldPose(msgs::ConvertIgn(_msg.pose()));
  for (int i = 0; i < _msg.link_size(); i++)
  {
    LinkPtr link = this->GetLinkById(_msg.link(i).id());
    if (link)
      link->ProcessMsg(_msg.link(i));
  }

  if (_msg.has_is_static())
    this->SetStatic(_msg.is_static());

  if (_msg.has_scale())
    this->SetScale(msgs::ConvertIgn(_msg.scale()));

  if (_msg.has_enable_wind())
    this->SetWindMode(_msg.enable_wind());
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
  this->prevAnimationTime = this->world->SimTime();
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
void Model::AttachStaticModel(ModelPtr &_model, ignition::math::Pose3d _offset)
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
  ignition::math::Pose3d p;
  for (unsigned int i = 0; i < this->attachedModels.size(); i++)
  {
    p = this->WorldPose();
    p += this->attachedModelsOffset[i];
    this->attachedModels[i]->SetWorldPose(p, true);
  }
}

//////////////////////////////////////////////////
void Model::SetState(const ModelState &_state)
{
  this->SetWorldPose(_state.Pose(), true);
  this->SetScale(_state.Scale(), true);

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

  for (const auto &ms : _state.NestedModelStates())
  {
    ModelPtr model = this->NestedModel(ms.first);
    if (model)
      model->SetState(ms.second);
    else
      gzerr << "Unable to find model[" << ms.first << "]\n";
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
void Model::SetScale(const ignition::math::Vector3d &_scale,
      const bool _publish)
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

  if (_publish)
    this->PublishScale();
}

/////////////////////////////////////////////////
ignition::math::Vector3d Model::Scale() const
{
  return this->scale;
}

//////////////////////////////////////////////////
void Model::PublishScale()
{
  GZ_ASSERT(this->GetParentModel() != NULL,
      "A model without a parent model should not happen");

  this->world->PublishModelScale(this->GetParentModel());
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
void Model::SetLinkWorldPose(const ignition::math::Pose3d &_pose,
    std::string _linkName)
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
void Model::SetLinkWorldPose(const ignition::math::Pose3d &_pose,
    const LinkPtr &_link)
{
  ignition::math::Pose3d linkPose = _link->WorldPose();
  ignition::math::Pose3d currentModelPose = this->WorldPose();
  ignition::math::Pose3d linkRelPose = currentModelPose - linkPose;
  ignition::math::Pose3d targetModelPose =  linkRelPose * _pose;
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
void Model::SetSelfCollide(bool _self_collide)
{
  this->sdf->GetElement("self_collide")->Set(_self_collide);
}

/////////////////////////////////////////////////
bool Model::GetSelfCollide() const
{
  return this->sdf->Get<bool>("self_collide");
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
    for (unsigned int j = 0; j < (*iter)->DOF(); ++j)
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

/////////////////////////////////////////////////
gazebo::physics::JointPtr Model::CreateJoint(
  const std::string &_name, const std::string &_type,
  physics::LinkPtr _parent, physics::LinkPtr _child)
{
  gazebo::physics::JointPtr joint;
  if (this->GetJoint(_name))
  {
    gzwarn << "Model [" << this->GetName()
           << "] already has a joint named [" << _name
           << "], skipping creating joint.\n";
    return joint;
  }
  joint = this->world->Physics()->CreateJoint(_type, shared_from_this());
  joint->SetName(_name);
  joint->Attach(_parent, _child);
  // need to call Joint::Load to clone Joint::sdfJoint into Joint::sdf
  joint->Load(_parent, _child, ignition::math::Pose3d::Zero);
  this->joints.push_back(joint);
  return joint;
}

/////////////////////////////////////////////////
gazebo::physics::JointPtr Model::CreateJoint(sdf::ElementPtr _sdf)
{
  if (_sdf->GetName() != "joint" ||
      !_sdf->HasAttribute("name") ||
      !_sdf->HasAttribute("type"))
  {
    gzerr << "Invalid _sdf passed to Model::CreateJoint" << std::endl;
    return physics::JointPtr();
  }

  std::string jointName(_sdf->Get<std::string>("name"));
  if (this->GetJoint(jointName))
  {
    gzwarn << "Model [" << this->GetName()
           << "] already has a joint named [" << jointName
           << "], skipping creating joint.\n";
    return physics::JointPtr();
  }

  try
  {
    // LoadJoint can throw if the scoped name of the joint already exists.
    this->LoadJoint(_sdf);
  }
  catch(...)
  {
    gzerr << "LoadJoint Failed" << std::endl;
  }
  return this->GetJoint(jointName);
}

/////////////////////////////////////////////////
bool Model::RemoveJoint(const std::string &_name)
{
  bool paused = this->world->IsPaused();
  gazebo::physics::JointPtr joint = this->GetJoint(_name);
  if (joint)
  {
    this->world->SetPaused(true);
    if (this->jointController)
    {
      this->jointController->RemoveJoint(joint.get());
    }
    joint->Detach();
    joint->Fini();

    this->joints.erase(
      std::remove(this->joints.begin(), this->joints.end(), joint),
      this->joints.end());
    this->world->SetPaused(paused);
    return true;
  }
  else
  {
    gzwarn << "Joint [" << _name
           << "] does not exist in model [" << this->GetName()
           << "], not removed.\n";
    return false;
  }
}

/////////////////////////////////////////////////
void Model::SetWindMode(const bool _enable)
{
  this->sdf->GetElement("enable_wind")->Set(_enable);
  for (auto &link : this->links)
    link->SetWindMode(_enable);
}

/////////////////////////////////////////////////
bool Model::WindMode() const
{
  return this->sdf->Get<bool>("enable_wind");
}

/////////////////////////////////////////////////
void Model::RegisterIntrospectionItems()
{
  auto uri = this->URI();

  // Callbacks.
  auto fModelPose = [this]()
  {
    return this->WorldPose();
  };

  auto fModelLinVel = [this]()
  {
    return this->WorldLinearVel();
  };

  auto fModelAngVel = [this]()
  {
    return this->WorldAngularVel();
  };

  auto fModelLinAcc = [this]()
  {
    return this->WorldLinearAccel();
  };

  auto fModelAngAcc = [this]()
  {
    return this->WorldAngularAccel();
  };

  // Register items.
  common::URI poseURI(uri);
  poseURI.Query().Insert("p", "pose3d/world_pose");
  this->introspectionItems.push_back(poseURI);
  gazebo::util::IntrospectionManager::Instance()->Register
      <ignition::math::Pose3d>(poseURI.Str(), fModelPose);

  common::URI linVelURI(uri);
  linVelURI.Query().Insert("p", "vector3d/world_linear_velocity");
  this->introspectionItems.push_back(linVelURI);
  gazebo::util::IntrospectionManager::Instance()->Register
      <ignition::math::Vector3d>(linVelURI.Str(), fModelLinVel);

  common::URI angVelURI(uri);
  angVelURI.Query().Insert("p", "vector3d/world_angular_velocity");
  this->introspectionItems.push_back(angVelURI);
  gazebo::util::IntrospectionManager::Instance()->Register
      <ignition::math::Vector3d>(angVelURI.Str(), fModelAngVel);

  common::URI linAccURI(uri);
  linAccURI.Query().Insert("p", "vector3d/world_linear_acceleration");
  this->introspectionItems.push_back(linAccURI);
  gazebo::util::IntrospectionManager::Instance()->Register
      <ignition::math::Vector3d>(linAccURI.Str(), fModelLinAcc);

  common::URI angAccURI(uri);
  angAccURI.Query().Insert("p", "vector3d/world_angular_acceleration");
  this->introspectionItems.push_back(angAccURI);
  gazebo::util::IntrospectionManager::Instance()->Register
      <ignition::math::Vector3d>(angAccURI.Str(), fModelAngAcc);
}

/////////////////////////////////////////////////
LinkPtr Model::CreateLink(const std::string &_name)
{
  LinkPtr link;
  if (this->GetLink(_name))
  {
    gzwarn << "Model [" << this->GetName()
      << "] already has a link named [" << _name
      << "], skipping creating link.\n";
    return link;
  }

  link = this->world->Physics()->CreateLink(shared_from_this());

  link->SetName(_name);
  this->links.push_back(link);

  return link;
}

//////////////////////////////////////////////////
void Model::PluginInfo(const common::URI &_pluginUri,
    ignition::msgs::Plugin_V &_plugins, bool &_success)
{
  _plugins.clear_plugins();
  _success = false;

  if (!_pluginUri.Valid())
  {
    gzwarn << "URI [" << _pluginUri.Str() << "] is not valid." << std::endl;
    return;
  }

  if (!_pluginUri.Path().Contains(this->URI().Path()))
  {
    gzwarn << "Plugin [" << _pluginUri.Str() << "] does not match world [" <<
        this->URI().Str() << "]" << std::endl;
    return;
  }

  auto parts = common::split(_pluginUri.Path().Str(), "/");
  auto myParts = common::split(this->URI().Path().Str(), "/");

  // Iterate over parts following this model
  for (size_t i = myParts.size(); i < parts.size(); i = i+2)
  {
    if (parts[i] == "model")
    {
      // Look in nested models
      auto model = this->NestedModel(parts[i+1]);

      if (!model)
      {
        gzwarn << "Model [" << parts[i+1] << "] not found in model [" <<
            this->GetName() << "]" << std::endl;
        return;
      }

      model->PluginInfo(_pluginUri, _plugins, _success);
      return;
    }
    // Look for plugin
    else if (parts[i] == "plugin")
    {
      // Return empty vector
      if (!this->sdf->HasElement("plugin"))
      {
        _success = true;
        return;
      }

      // Find correct plugin
      auto pluginElem = this->sdf->GetElement("plugin");
      while (pluginElem)
      {
        auto pluginName = pluginElem->Get<std::string>("name");

        // If asking for a specific plugin, skip all other plugins
        if (i+1 < parts.size() && parts[i+1] != pluginName)
        {
          pluginElem = pluginElem->GetNextElement("plugin");
          continue;
        }

        // Get plugin info from SDF
        auto pluginMsg = _plugins.add_plugins();
        pluginMsg->CopyFrom(util::Convert<ignition::msgs::Plugin>(pluginElem));

        pluginElem = pluginElem->GetNextElement("plugin");
      }

      // If asking for a specific plugin and it wasn't found
      if (i+1 < parts.size() && _plugins.plugins_size() == 0)
      {
        gzwarn << "Plugin [" << parts[i+1] << "] not found in model [" <<
            this->URI().Str() << "]" << std::endl;
        return;
      }
      _success = true;
      return;
    }
    else
    {
      gzwarn << "Segment [" << parts[i] << "] in [" << _pluginUri.Str() <<
         "] cannot be handled." << std::endl;
      return;
    }
  }

  gzwarn << "Couldn't get information for plugin [" << _pluginUri.Str() << "]"
      << std::endl;
}
