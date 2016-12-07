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

#include <float.h>

#include <ignition/msgs/plugin_v.pb.h>
#include <sstream>

#include "gazebo/util/OpenAL.hh"
#include "gazebo/common/KeyFrame.hh"
#include "gazebo/common/Animation.hh"
#include "gazebo/common/Plugin.hh"
#include "gazebo/common/Events.hh"
#include "gazebo/common/Exception.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/common/CommonIface.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/URI.hh"

#include "gazebo/util/IntrospectionManager.hh"

#include "gazebo/physics/Gripper.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/JointController.hh"
#include "gazebo/physics/Link.hh"
#include "gazebo/physics/World.hh"
#include "gazebo/physics/PhysicsEngine.hh"
#include "gazebo/physics/Contact.hh"

#include "gazebo/transport/Node.hh"

#include "gazebo/physics/ModelPrivate.hh"
#include "gazebo/physics/Model.hh"

using namespace gazebo;
using namespace physics;

//////////////////////////////////////////////////
Model::Model(BasePtr _parent)
: Entity(*new ModelPrivate, _parent),
  modelDPtr(static_cast<ModelPrivate*>(this->entityDPtr))
{
  this->AddType(MODEL);
}

//////////////////////////////////////////////////
Model::Model(ModelPrivate &_dataPtr, BasePtr _parent)
: Entity(_dataPtr, _parent),
  modelDPtr(static_cast<ModelPrivate*>(this->entityDPtr))
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

  this->modelDPtr->jointPub =
    this->modelDPtr->node->Advertise<msgs::Joint>("~/joint");

  this->SetStatic(this->modelDPtr->sdf->Get<bool>("static"));
  if (this->modelDPtr->sdf->HasElement("static"))
  {
    this->modelDPtr->sdf->GetElement("static")->GetValue()->SetUpdateFunc(
        std::bind(&Entity::IsStatic, this));
  }

  if (this->modelDPtr->sdf->HasElement("self_collide"))
  {
    this->SetSelfCollide(this->modelDPtr->sdf->Get<bool>("self_collide"));
  }

  if (this->modelDPtr->sdf->HasElement("enable_wind"))
  {
    this->SetWindMode(this->modelDPtr->sdf->Get<bool>("enable_wind"));
  }

  if (this->modelDPtr->sdf->HasElement("allow_auto_disable"))
    this->SetAutoDisable(this->modelDPtr->sdf->Get<bool>("allow_auto_disable"));

  this->LoadLinks();

  this->LoadModels();

  // Load the joints if the world is already loaded. Otherwise, the World
  // has some special logic to load models that takes into account state
  // information.
  if (this->modelDPtr->world->IsLoaded())
    this->LoadJoints();
}

//////////////////////////////////////////////////
void Model::LoadLinks()
{
  /// \TODO: check for duplicate model, and raise an error
  /// BasePtr dup = Base::BaseByName(this->ScopedName());

  // Load the bodies
  if (this->modelDPtr->sdf->HasElement("link"))
  {
    sdf::ElementPtr linkElem = this->modelDPtr->sdf->GetElement("link");
    while (linkElem)
    {
      // Create a new link
      LinkPtr link = this->World()->Physics()->CreateLink(
          std::static_pointer_cast<Model>(shared_from_this()));

      /// \TODO: canonical link is hardcoded to the first link.
      ///        warn users for now, need  to add parsing of
      ///        the canonical tag in sdf

      // find canonical link - there should only be one within a tree of models
      if (!this->modelDPtr->canonicalLink)
      {
        // Get the canonical link from parent, if not found then set the
        // current link as the canonoical link.
        LinkPtr cLink;
        BasePtr entity = this->Parent();
        while (entity && entity->HasType(MODEL))
        {
          ModelPtr model = std::static_pointer_cast<Model>(entity);
          LinkPtr tmpLink = model->LinkByName();
          if (tmpLink)
          {
            cLink = tmpLink;
            break;
          }
          entity = entity->Parent();
        }

        if (cLink)
        {
          this->modelDPtr->canonicalLink = cLink;
        }
        else
        {
          // first link found, set as canonical link
          link->SetCanonicalLink(true);
          this->modelDPtr->canonicalLink = link;

          // notify parent models of this canonical link
          entity = this->Parent();
          while (entity && entity->HasType(MODEL))
          {
            ModelPtr model = std::static_pointer_cast<Model>(entity);
            model->modelDPtr->canonicalLink = this->modelDPtr->canonicalLink;
            entity = entity->Parent();
          }
        }
      }

      // Load the link using the config node. This also loads all of the
      // bodies collisionetries
      link->Load(linkElem);
      linkElem = linkElem->GetNextElement("link");
      this->modelDPtr->links.push_back(link);
    }
  }
}

//////////////////////////////////////////////////
void Model::LoadModels()
{
  // Load the models
  if (this->modelDPtr->sdf->HasElement("model"))
  {
    sdf::ElementPtr modelElem = this->modelDPtr->sdf->GetElement("model");
    while (modelElem)
    {
      // Create a new model
      ModelPtr model = this->World()->Physics()->CreateModel(
          std::static_pointer_cast<Model>(shared_from_this()));
      model->SetWorld(this->World());
      model->Load(modelElem);
      this->modelDPtr->models.push_back(model);
      modelElem = modelElem->GetNextElement("model");
    }

    for (auto &model : this->modelDPtr->models)
      model->SetEnabled(true);
  }
}

//////////////////////////////////////////////////
void Model::LoadJoints()
{
  // Load the joints
  if (this->modelDPtr->sdf->HasElement("joint"))
  {
    sdf::ElementPtr jointElem = this->modelDPtr->sdf->GetElement("joint");
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

  if (this->modelDPtr->sdf->HasElement("gripper"))
  {
    sdf::ElementPtr gripperElem = this->modelDPtr->sdf->GetElement("gripper");
    while (gripperElem)
    {
      this->LoadGripper(gripperElem);
      gripperElem = gripperElem->GetNextElement("gripper");
    }
  }

  // Load nested model joints if the world is not already loaded. Otherwise,
  // LoadJoints will be called from Model::Load.
  if (!this->modelDPtr->world->IsLoaded())
  {
    for (auto model : this->modelDPtr->models)
      model->LoadJoints();
  }
}

//////////////////////////////////////////////////
void Model::Init()
{
  // Record the model's initial pose (for reseting)
  ignition::math::Pose3d initPose =
    this->modelDPtr->sdf->Get<ignition::math::Pose3d>("pose");
  this->SetInitialRelativePose(initPose);
  this->SetRelativePose(initPose);

  // Initialize the bodies before the joints
  for (Base_V::iterator iter = this->modelDPtr->children.begin();
       iter != this->modelDPtr->children.end(); ++iter)
  {
    if ((*iter)->HasType(Base::LINK))
    {
      LinkPtr link = std::static_pointer_cast<Link>(*iter);
      if (link)
        link->Init();
      else
        gzerr << "Child [" << (*iter)->Name()
              << "] has type Base::LINK, but cannot be dynamically casted\n";
    }
    else if ((*iter)->HasType(Base::MODEL))
      std::static_pointer_cast<Model>(*iter)->Init();
  }

  // Initialize the joints last.
  for (Joint_V::iterator iter = this->modelDPtr->joints.begin();
       iter != this->modelDPtr->joints.end(); ++iter)
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
    this->modelDPtr->jointPub->Publish(msg);
  }

  for (std::vector<GripperPtr>::iterator iter =
      this->modelDPtr->grippers.begin();
      iter != this->modelDPtr->grippers.end(); ++iter)
  {
    (*iter)->Init();
  }
}

//////////////////////////////////////////////////
void Model::Update()
{
  if (this->IsStatic())
    return;

  std::lock_guard<std::recursive_mutex> lock(this->modelDPtr->updateMutex);

  for (Joint_V::iterator jiter = this->modelDPtr->joints.begin();
       jiter != this->modelDPtr->joints.end(); ++jiter)
  {
    (*jiter)->Update();
  }

  if (this->modelDPtr->jointController)
    this->modelDPtr->jointController->Update();

  if (!this->modelDPtr->jointAnimations.empty())
  {
    common::NumericKeyFrame kf(0);
    std::map<std::string, double> jointPositions;
    std::map<std::string, common::NumericAnimationPtr>::iterator iter;
    iter = this->modelDPtr->jointAnimations.begin();
    while (iter != this->modelDPtr->jointAnimations.end())
    {
      iter->second->GetInterpolatedKeyFrame(kf);

      iter->second->AddTime(
          (this->modelDPtr->world->SimTime() -
           this->modelDPtr->prevAnimationTime).Double());

      if (iter->second->GetTime() < iter->second->GetLength())
      {
        iter->second->GetInterpolatedKeyFrame(kf);
        jointPositions[iter->first] = kf.GetValue();
        ++iter;
      }
      else
      {
        this->modelDPtr->jointAnimations.erase(iter++);
      }
    }
    if (!jointPositions.empty())
    {
      this->modelDPtr->jointController->SetJointPositions(jointPositions);
    }
    else
    {
      if (this->modelDPtr->onJointAnimationComplete)
        this->modelDPtr->onJointAnimationComplete();
    }
    this->modelDPtr->prevAnimationTime = this->modelDPtr->world->SimTime();
  }

  for (auto &model : this->modelDPtr->models)
    model->Update();
}

//////////////////////////////////////////////////
void Model::SetJointPosition(const std::string &_jointName,
    const double _position, const int _index)
{
  if (this->modelDPtr->jointController)
  {
    this->modelDPtr->jointController->SetJointPosition(
        _jointName, _position, _index);
  }
}

//////////////////////////////////////////////////
void Model::SetJointPositions(
    const std::map<std::string, double> &_jointPositions)
{
  if (this->modelDPtr->jointController)
    this->modelDPtr->jointController->SetJointPositions(_jointPositions);
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

      for (jiter = this->modelDPtr->joints.begin();
          jiter != this->modelDPtr->joints.end(); ++jiter)
      {
        if (!(*jiter))
          continue;

        LinkPtr jlink0 = (*jiter)->JointLink(0);
        LinkPtr jlink1 = (*jiter)->JointLink(1);

        if (!jlink0 || !jlink1 || jlink0->Name() == _child->Name() ||
            jlink1->Name() == _child->Name() ||
            jlink0->Name() == jlink1->Name())
        {
          this->modelDPtr->joints.erase(jiter);
          done = false;
          break;
        }
      }
    }

    this->RemoveLink(_child->ScopedName());
  }

  Entity::RemoveChild(_child->Id());

  for (Link_V::iterator liter = this->modelDPtr->links.begin();
       liter != this->modelDPtr->links.end(); ++liter)
  {
    (*liter)->SetEnabled(true);
  }
}

//////////////////////////////////////////////////
std::shared_ptr<Model> Model::shared_from_this()
{
  return std::static_pointer_cast<Model>(Entity::shared_from_this());
}

//////////////////////////////////////////////////
void Model::Fini()
{
  this->modelDPtr->attachedModels.clear();
  this->modelDPtr->canonicalLink.reset();
  this->modelDPtr->jointController.reset();
  this->modelDPtr->joints.clear();
  this->modelDPtr->links.clear();
  this->modelDPtr->models.clear();
  this->modelDPtr->plugins.clear();

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
      LinkPtr link = std::dynamic_pointer_cast<Link>(
          this->Child(linkElem->Get<std::string>("name")));
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
      JointPtr joint = std::dynamic_pointer_cast<Joint>(this->GetChild(jointElem->Get<std::string>("name")));
      joint->UpdateParameters(jointElem);
      jointElem = jointElem->GetNextElement("joint");
    }
  }
  */
}

//////////////////////////////////////////////////
const sdf::ElementPtr Model::GetSDF()
{
  return Entity::SDF();
}

//////////////////////////////////////////////////
const sdf::ElementPtr Model::UnscaledSDF()
{
  GZ_ASSERT(this->modelDPtr->sdf != NULL, "Model sdf member is NULL");
  this->modelDPtr->sdf->Update();

  sdf::ElementPtr unscaledSdf(this->modelDPtr->sdf);

  // Go through all collisions and visuals and divide size by scale
  // See Link::UpdateVisualGeomSDF
  if (!this->modelDPtr->sdf->HasElement("link"))
    return unscaledSdf;

  auto linkElem = this->modelDPtr->sdf->GetElement("link");
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
              size / this->modelDPtr->scale);
        }
        else if (geomElem->HasElement("sphere"))
        {
          double radius = geomElem->GetElement("sphere")->Get<double>("radius");
          geomElem->GetElement("sphere")->GetElement("radius")->Set(
              radius/this->modelDPtr->scale.Max());
        }
        else if (geomElem->HasElement("cylinder"))
        {
          double radius =
              geomElem->GetElement("cylinder")->Get<double>("radius");
          double length =
              geomElem->GetElement("cylinder")->Get<double>("length");
          double radiusScale = std::max(this->modelDPtr->scale.X(),
              this->modelDPtr->scale.Y());

          geomElem->GetElement("cylinder")->GetElement("radius")->Set(
              radius/radiusScale);
          geomElem->GetElement("cylinder")->GetElement("length")->Set(
              length/this->modelDPtr->scale.Z());
        }
        else if (geomElem->HasElement("mesh"))
        {
          geomElem->GetElement("mesh")->GetElement("scale")->Set(
              ignition::math::Vector3d::One);
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
              size / this->modelDPtr->scale);
        }
        else if (geomElem->HasElement("sphere"))
        {
          double radius = geomElem->GetElement("sphere")->Get<double>("radius");
          geomElem->GetElement("sphere")->GetElement("radius")->Set(
              radius/this->modelDPtr->scale.Max());
        }
        else if (geomElem->HasElement("cylinder"))
        {
          double radius =
              geomElem->GetElement("cylinder")->Get<double>("radius");
          double length =
              geomElem->GetElement("cylinder")->Get<double>("length");
          double radiusScale = std::max(this->modelDPtr->scale.X(),
              this->modelDPtr->scale.Y());

          geomElem->GetElement("cylinder")->GetElement("radius")->Set(
              radius/radiusScale);
          geomElem->GetElement("cylinder")->GetElement("length")->Set(
              length/this->modelDPtr->scale.Z());
        }
        else if (geomElem->HasElement("mesh"))
        {
          geomElem->GetElement("mesh")->GetElement("scale")->Set(
              ignition::math::Vector3d::One);
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

  for (Joint_V::iterator jiter = this->modelDPtr->joints.begin();
       jiter != this->modelDPtr->joints.end(); ++jiter)
  {
    (*jiter)->Reset();
  }

  // Reset plugins after links and joints,
  // so that plugins can restore initial conditions
  for (std::vector<ModelPluginPtr>::iterator iter =
      this->modelDPtr->plugins.begin();
       iter != this->modelDPtr->plugins.end(); ++iter)
  {
    (*iter)->Reset();
  }
}

//////////////////////////////////////////////////
void Model::ResetPhysicsStates()
{
  // reset link velocities when resetting model
  for (Link_V::iterator liter = this->modelDPtr->links.begin();
       liter != this->modelDPtr->links.end(); ++liter)
  {
    (*liter)->ResetPhysicsStates();
  }

  // reset nested model physics states
  for (auto &m : this->modelDPtr->models)
    m->ResetPhysicsStates();
}

//////////////////////////////////////////////////
void Model::SetLinearVel(const math::Vector3 &_vel)
{
  this->SetLinearVel(_vel.Ign());
}

//////////////////////////////////////////////////
void Model::SetLinearVel(const ignition::math::Vector3d &_vel)
{
  for (Link_V::iterator iter = this->modelDPtr->links.begin();
       iter != this->modelDPtr->links.end(); ++iter)
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
  this->SetAngularVel(_vel.Ign());
}

//////////////////////////////////////////////////
void Model::SetAngularVel(const ignition::math::Vector3d &_vel)
{
  for (Link_V::iterator iter = this->modelDPtr->links.begin();
       iter != this->modelDPtr->links.end(); ++iter)
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
  this->SetLinearAccel(_accel.Ign());
}

//////////////////////////////////////////////////
void Model::SetLinearAccel(const ignition::math::Vector3d &_accel)
{
  for (Link_V::iterator iter = this->modelDPtr->links.begin();
       iter != this->modelDPtr->links.end(); ++iter)
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
  this->SetAngularAccel(_accel.Ign());
}

//////////////////////////////////////////////////
void Model::SetAngularAccel(const ignition::math::Vector3d &_accel)
{
  for (Link_V::iterator iter = this->modelDPtr->links.begin();
       iter != this->modelDPtr->links.end(); ++iter)
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
  return this->RelativeLinearVel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Model::RelativeLinearVel() const
{
  if (this->LinkByName("canonical"))
  {
    return this->LinkByName("canonical")->RelativeLinearVel();
  }
  else
  {
    return ignition::math::Vector3d::Zero;
  }
}

//////////////////////////////////////////////////
math::Vector3 Model::GetWorldLinearVel() const
{
  return this->WorldLinearVel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Model::WorldLinearVel() const
{
  if (this->LinkByName("canonical"))
    return this->LinkByName("canonical")->WorldLinearVel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
math::Vector3 Model::GetRelativeAngularVel() const
{
  return this->RelativeAngularVel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Model::RelativeAngularVel() const
{
  if (this->LinkByName("canonical"))
    return this->LinkByName("canonical")->RelativeAngularVel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
math::Vector3 Model::GetWorldAngularVel() const
{
  return this->WorldAngularVel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Model::WorldAngularVel() const
{
  if (this->LinkByName("canonical"))
    return this->LinkByName("canonical")->WorldAngularVel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
math::Vector3 Model::GetRelativeLinearAccel() const
{
  return this->RelativeLinearAccel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Model::RelativeLinearAccel() const
{
  if (this->LinkByName("canonical"))
    return this->LinkByName("canonical")->RelativeLinearAccel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
math::Vector3 Model::GetWorldLinearAccel() const
{
  return this->WorldLinearAccel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Model::WorldLinearAccel() const
{
  if (this->LinkByName("canonical"))
    return this->LinkByName("canonical")->WorldLinearAccel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
math::Vector3 Model::GetRelativeAngularAccel() const
{
  return this->RelativeAngularAccel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Model::RelativeAngularAccel() const
{
  if (this->LinkByName("canonical"))
    return this->LinkByName("canonical")->RelativeAngularAccel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
math::Vector3 Model::GetWorldAngularAccel() const
{
  return this->WorldAngularAccel();
}

//////////////////////////////////////////////////
ignition::math::Vector3d Model::WorldAngularAccel() const
{
  if (this->LinkByName("canonical"))
    return this->LinkByName("canonical")->WorldAngularAccel();
  else
    return ignition::math::Vector3d::Zero;
}

//////////////////////////////////////////////////
ignition::math::Box Model::BoundingBox() const
{
  ignition::math::Box box;

  box.Min().Set(IGN_DBL_MAX, IGN_DBL_MAX, IGN_DBL_MAX);
  box.Max().Set(-IGN_DBL_MAX, -IGN_DBL_MAX, -IGN_DBL_MAX);

  for (Link_V::const_iterator iter = this->modelDPtr->links.begin();
       iter != this->modelDPtr->links.end(); ++iter)
  {
    if (*iter)
    {
      box += (*iter)->BoundingBox();
    }
  }

  return box;
}

//////////////////////////////////////////////////
unsigned int Model::GetJointCount() const
{
  return this->JointCount();
}

//////////////////////////////////////////////////
unsigned int Model::JointCount() const
{
  return this->modelDPtr->joints.size();
}

//////////////////////////////////////////////////
const Joint_V &Model::GetJoints() const
{
  return this->Joints();
}

//////////////////////////////////////////////////
const Joint_V &Model::Joints() const
{
  return this->modelDPtr->joints;
}

//////////////////////////////////////////////////
JointPtr Model::GetJoint(const std::string &_name)
{
  return this->JointByName(_name);
}

//////////////////////////////////////////////////
JointPtr Model::JointByName(const std::string &_name) const
{
  JointPtr result;
  Joint_V::iterator iter;

  for (iter = this->modelDPtr->joints.begin();
      iter != this->modelDPtr->joints.end(); ++iter)
  {
    if ((*iter)->ScopedName() == _name || (*iter)->Name() == _name)
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
  return this->modelDPtr->models;
}

//////////////////////////////////////////////////
ModelPtr Model::NestedModel(const std::string &_name) const
{
  ModelPtr result;

  for (auto &m : this->modelDPtr->models)
  {
    if ((m->ScopedName() == _name) || (m->Name() == _name))
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
  return this->LinkById(_id);
}

//////////////////////////////////////////////////
LinkPtr Model::LinkById(const unsigned int _id) const
{
  return std::dynamic_pointer_cast<Link>(this->BaseById(_id));
}

//////////////////////////////////////////////////
const Link_V &Model::GetLinks() const
{
  return this->Links();
}

//////////////////////////////////////////////////
const Link_V &Model::Links() const
{
  return this->modelDPtr->links;
}

//////////////////////////////////////////////////
LinkPtr Model::GetLink(const std::string &_name) const
{
  return this->LinkByName(_name);
}

//////////////////////////////////////////////////
LinkPtr Model::LinkByName(const std::string &_name) const
{
  Link_V::const_iterator iter;
  LinkPtr result;

  if (_name == "canonical")
  {
    result = this->modelDPtr->canonicalLink;
  }
  else
  {
    for (iter = this->modelDPtr->links.begin();
         iter != this->modelDPtr->links.end(); ++iter)
    {
      if (((*iter)->ScopedName() == _name) || ((*iter)->Name() == _name))
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

  joint = this->World()->Physics()->CreateJoint(stype,
     std::static_pointer_cast<Model>(shared_from_this()));
  if (!joint)
  {
    gzerr << "Unable to create joint of type[" << stype << "]\n";
    // gzthrow("Unable to create joint of type[" + stype + "]\n");
    return;
  }

  joint->SetModel(std::static_pointer_cast<Model>(shared_from_this()));

  // Load the joint
  joint->Load(_sdf);

  if (this->JointByName(joint->ScopedName()) != NULL)
  {
    gzerr << "can't have two joint with the same name\n";
    gzthrow("can't have two joints with the same name ["+
      joint->ScopedName() + "]\n");
  }

  this->modelDPtr->joints.push_back(joint);

  if (!this->modelDPtr->jointController)
  {
    this->modelDPtr->jointController.reset(new JointController(
        std::dynamic_pointer_cast<Model>(shared_from_this())));
  }
  this->modelDPtr->jointController->AddJoint(joint);
}

//////////////////////////////////////////////////
void Model::LoadGripper(sdf::ElementPtr _sdf)
{
  GripperPtr gripper(new Gripper(
      std::static_pointer_cast<Model>(shared_from_this())));
  gripper->Load(_sdf);
  this->modelDPtr->grippers.push_back(gripper);
}

//////////////////////////////////////////////////
void Model::LoadPlugins()
{
  // Check to see if we need to load any model plugins
  if (this->PluginCount() > 0)
  {
    int iterations = 0;

    // Wait for the sensors to be initialized before loading
    // plugins, if there are any sensors
    while (this->SensorCount() > 0 &&
           !this->modelDPtr->world->SensorsInitialized() &&
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
      sdf::ElementPtr pluginElem = this->modelDPtr->sdf->GetElement("plugin");
      while (pluginElem)
      {
        this->LoadPlugin(pluginElem);
        pluginElem = pluginElem->GetNextElement("plugin");
      }
    }
    else
    {
      gzerr << "Sensors failed to initialize when loading model["
        << this->Name() << "] via the factory mechanism."
        << " Plugins for the model will not be loaded.\n";
    }
  }

  for (auto &model : this->modelDPtr->models)
    model->LoadPlugins();
}

//////////////////////////////////////////////////
unsigned int Model::GetPluginCount() const
{
  return this->PluginCount();
}

//////////////////////////////////////////////////
unsigned int Model::PluginCount() const
{
  unsigned int result = 0;

  // Count all the plugins specified in SDF
  if (this->modelDPtr->sdf->HasElement("plugin"))
  {
    sdf::ElementPtr pluginElem = this->modelDPtr->sdf->GetElement("plugin");
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
  return this->SensorCount();
}

//////////////////////////////////////////////////
unsigned int Model::SensorCount() const
{
  unsigned int result = 0;

  // Count all the sensors on all the links
  for (Link_V::const_iterator iter = this->modelDPtr->links.begin();
       iter != this->modelDPtr->links.end(); ++iter)
  {
    result += (*iter)->SensorCount();
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
      gzerr << "Model[" << this->Name() << "] is attempting to load "
            << "a plugin, but detected an incorrect plugin type. "
            << "Plugin filename[" << filename << "] name["
            << pluginName << "]\n";
      return;
    }

    ModelPtr myself = std::static_pointer_cast<Model>(shared_from_this());

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

    this->modelDPtr->plugins.push_back(plugin);
  }
}

//////////////////////////////////////////////////
void Model::SetGravityMode(const bool _v)
{
  for (Link_V::iterator liter = this->modelDPtr->links.begin();
      liter != this->modelDPtr->links.end(); ++liter)
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
  for (Link_V::iterator liter = this->modelDPtr->links.begin();
      liter != this->modelDPtr->links.end(); ++liter)
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
  for (Link_V::iterator liter = this->modelDPtr->links.begin();
      liter != this->modelDPtr->links.end(); ++liter)
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

  _msg.set_name(this->ScopedName());
  _msg.set_is_static(this->IsStatic());
  _msg.set_self_collide(this->SelfCollide());
  _msg.set_enable_wind(this->WindMode());
  msgs::Set(_msg.mutable_pose(), relPose);
  _msg.set_id(this->Id());
  msgs::Set(_msg.mutable_scale(), this->modelDPtr->scale);

  msgs::Set(this->modelDPtr->visualMsg->mutable_pose(), relPose);
  _msg.add_visual()->CopyFrom(*this->modelDPtr->visualMsg);

  for (const auto &link : this->modelDPtr->links)
  {
    link->FillMsg(*_msg.add_link());
  }

  for (const auto &joint : this->modelDPtr->joints)
  {
    joint->FillMsg(*_msg.add_joint());
  }
  for (const auto &model : this->modelDPtr->models)
  {
    model->FillMsg(*_msg.add_model());
  }
}

//////////////////////////////////////////////////
void Model::ProcessMsg(const msgs::Model &_msg)
{
  if (_msg.has_id() && _msg.id() != this->Id())
  {
    gzerr << "Incorrect ID[" << _msg.id() << " != " << this->Id() << "]\n";
    return;
  }
  else if ((_msg.has_id() && _msg.id() != this->Id()) &&
            _msg.name() != this->ScopedName())
  {
    gzerr << "Incorrect name[" << _msg.name() << " != " << this->Name()
      << "]\n";
    return;
  }

  this->SetName(this->modelDPtr->world->StripWorldName(_msg.name()));
  if (_msg.has_pose())
    this->SetWorldPose(msgs::ConvertIgn(_msg.pose()));
  for (int i = 0; i < _msg.link_size(); i++)
  {
    LinkPtr link = this->LinkById(_msg.link(i).id());
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
    std::function<void()> _onComplete)
{
  std::lock_guard<std::recursive_mutex> lock(this->modelDPtr->updateMutex);
  std::map<std::string, common::NumericAnimationPtr>::const_iterator iter;

  for (iter = _anims.begin(); iter != _anims.end(); ++iter)
  {
    this->modelDPtr->jointAnimations[iter->first] = iter->second;
  }

  this->modelDPtr->onJointAnimationComplete = _onComplete;
  this->modelDPtr->prevAnimationTime = this->modelDPtr->world->SimTime();
}

//////////////////////////////////////////////////
void Model::StopAnimation()
{
  std::lock_guard<std::recursive_mutex> lock(this->modelDPtr->updateMutex);
  Entity::StopAnimation();
  this->modelDPtr->onJointAnimationComplete = NULL;
  this->modelDPtr->jointAnimations.clear();
}

//////////////////////////////////////////////////
void Model::AttachStaticModel(ModelPtr &_model, math::Pose _offset)
{
  this->AttachStaticModel(_model, _offset.Ign());
}

//////////////////////////////////////////////////
void Model::AttachStaticModel(ModelPtr &_model,
    const ignition::math::Pose3d &_offset)
{
  if (!_model->IsStatic())
  {
    gzerr << "AttachStaticModel requires a static model\n";
    return;
  }

  this->modelDPtr->attachedModels.push_back(_model);
  this->modelDPtr->attachedModelsOffset.push_back(_offset);
}

//////////////////////////////////////////////////
void Model::DetachStaticModel(const std::string &_modelName)
{
  for (unsigned int i = 0; i < this->modelDPtr->attachedModels.size(); i++)
  {
    if (this->modelDPtr->attachedModels[i]->Name() == _modelName)
    {
      this->modelDPtr->attachedModels.erase(
          this->modelDPtr->attachedModels.begin()+i);
      this->modelDPtr->attachedModelsOffset.erase(
          this->modelDPtr->attachedModelsOffset.begin()+i);
      break;
    }
  }
}

//////////////////////////////////////////////////
void Model::OnPoseChange()
{
  ignition::math::Pose3d p;
  for (unsigned int i = 0; i < this->modelDPtr->attachedModels.size(); i++)
  {
    p = this->WorldPose();
    p += this->modelDPtr->attachedModelsOffset[i];
    this->modelDPtr->attachedModels[i]->SetWorldPose(p, true);
  }
}

//////////////////////////////////////////////////
void Model::SetState(const ModelState &_state)
{
  this->SetWorldPose(_state.Pose(), true);
  this->SetScale(_state.Scale(), true);

  LinkState_M linkStates = _state.LinkStates();
  for (LinkState_M::iterator iter = linkStates.begin();
       iter != linkStates.end(); ++iter)
  {
    LinkPtr link(this->LinkByName(iter->first));
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
  //   this->SetJointPosition(this->Name() + "::" + jointState.GetName(),
  //                          jointState.GetAngle(0).Radian());
  // }
}

/////////////////////////////////////////////////
void Model::SetScale(const ignition::math::Vector3d &_scale,
      const bool _publish)
{
  if (this->modelDPtr->scale == _scale)
    return;

  this->modelDPtr->scale = _scale;

  for (Base_V::iterator iter = this->modelDPtr->children.begin();
       iter != this->modelDPtr->children.end(); ++iter)
  {
    if (*iter && (*iter)->HasType(LINK))
    {
      std::static_pointer_cast<Link>(*iter)->SetScale(_scale);
    }
  }

  if (_publish)
    this->PublishScale();
}

/////////////////////////////////////////////////
ignition::math::Vector3d Model::Scale() const
{
  return this->modelDPtr->scale;
}

//////////////////////////////////////////////////
void Model::PublishScale()
{
  GZ_ASSERT(this->ParentModel() != NULL,
      "A model without a parent model should not happen");

  this->modelDPtr->world->PublishModelScale(ModelPtr(this->ParentModel()));
}

/////////////////////////////////////////////////
void Model::SetEnabled(const bool _enabled)
{
  for (Link_V::iterator liter = this->modelDPtr->links.begin();
      liter != this->modelDPtr->links.end(); ++liter)
  {
    if (*liter)
      (*liter)->SetEnabled(_enabled);
  }
}

/////////////////////////////////////////////////
void Model::SetLinkWorldPose(const math::Pose &_pose, std::string _linkName)
{
  this->SetLinkWorldPose(_pose.Ign(), _linkName);
}

/////////////////////////////////////////////////
void Model::SetLinkWorldPose(const ignition::math::Pose3d &_pose,
    const std::string &_linkName)
{
  // look for link matching link name
  LinkPtr link = this->LinkByName(_linkName);
  if (link)
  {
    this->SetLinkWorldPose(_pose, link);
  }
  else
  {
    gzerr << "Setting Model Pose by specifying Link failed:"
          << " Link[" << _linkName << "] not found.\n";
  }
}

/////////////////////////////////////////////////
void Model::SetLinkWorldPose(const math::Pose &_pose, const LinkPtr &_link)
{
  this->SetLinkWorldPose(_pose.Ign(), _link);
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
void Model::SetAutoDisable(const bool _auto)
{
  if (!this->modelDPtr->joints.empty())
    return;

  for (Link_V::iterator liter = this->modelDPtr->links.begin();
      liter != this->modelDPtr->links.end(); ++liter)
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
  return this->AutoDisable();
}

/////////////////////////////////////////////////
bool Model::AutoDisable() const
{
  return this->modelDPtr->sdf->Get<bool>("allow_auto_disable");
}

/////////////////////////////////////////////////
void Model::SetSelfCollide(const bool _selfCollide)
{
  this->modelDPtr->sdf->GetElement("self_collide")->Set(_selfCollide);
}

/////////////////////////////////////////////////
bool Model::GetSelfCollide() const
{
  return this->SelfCollide();
}

/////////////////////////////////////////////////
bool Model::SelfCollide() const
{
  return this->modelDPtr->sdf->Get<bool>("self_collide");
}

/////////////////////////////////////////////////
void Model::RemoveLink(const std::string &_name)
{
  for (Link_V::iterator iter = this->modelDPtr->links.begin();
       iter != this->modelDPtr->links.end(); ++iter)
  {
    if ((*iter)->Name() == _name || (*iter)->ScopedName() == _name)
    {
      this->modelDPtr->links.erase(iter);
      break;
    }
  }
}

/////////////////////////////////////////////////
JointControllerPtr Model::GetJointController()
{
  return this->JointCtrl();
}

/////////////////////////////////////////////////
JointControllerPtr Model::JointCtrl() const
{
  return this->modelDPtr->jointController;
}

/////////////////////////////////////////////////
GripperPtr Model::GetGripper(size_t _index) const
{
  return this->GripperByIndex(_index);
}

/////////////////////////////////////////////////
GripperPtr Model::GripperByIndex(const size_t _index) const
{
  if (_index < this->modelDPtr->grippers.size())
    return this->modelDPtr->grippers[_index];
  else
    return GripperPtr();
}

/////////////////////////////////////////////////
size_t Model::GetGripperCount() const
{
  return this->GripperCount();
}

/////////////////////////////////////////////////
size_t Model::GripperCount() const
{
  return this->modelDPtr->grippers.size();
}

/////////////////////////////////////////////////
double Model::GetWorldEnergyPotential() const
{
  return this->WorldEnergyPotential();
}

/////////////////////////////////////////////////
double Model::WorldEnergyPotential() const
{
  double e = 0;
  for (Link_V::const_iterator iter = this->modelDPtr->links.begin();
    iter != this->modelDPtr->links.end(); ++iter)
  {
    e += (*iter)->WorldEnergyPotential();
  }
  for (Joint_V::const_iterator iter = this->modelDPtr->joints.begin();
    iter != this->modelDPtr->joints.end(); ++iter)
  {
    for (unsigned int j = 0; j < (*iter)->AngleCount(); ++j)
    {
      e += (*iter)->WorldEnergyPotentialSpring(j);
    }
  }
  return e;
}

/////////////////////////////////////////////////
double Model::GetWorldEnergyKinetic() const
{
  return this->WorldEnergyKinetic();
}

/////////////////////////////////////////////////
double Model::WorldEnergyKinetic() const
{
  double e = 0;
  for (Link_V::const_iterator iter = this->modelDPtr->links.begin();
    iter != this->modelDPtr->links.end(); ++iter)
  {
    e += (*iter)->WorldEnergyKinetic();
  }
  return e;
}

/////////////////////////////////////////////////
double Model::GetWorldEnergy() const
{
  return this->WorldEnergy();
}

/////////////////////////////////////////////////
double Model::WorldEnergy() const
{
  return this->WorldEnergyPotential() + this->WorldEnergyKinetic();
}

/////////////////////////////////////////////////
gazebo::physics::JointPtr Model::CreateJoint(
  const std::string &_name, const std::string &_type,
  physics::LinkPtr _parent, physics::LinkPtr _child)
{
  gazebo::physics::JointPtr joint;
  if (this->JointByName(_name))
  {
    gzwarn << "Model [" << this->Name()
           << "] already has a joint named [" << _name
           << "], skipping creating joint.\n";
    return joint;
  }
  joint =
    this->modelDPtr->world->Physics()->CreateJoint(_type, shared_from_this());
  joint->SetName(_name);
  joint->Attach(_parent, _child);
  // need to call Joint::Load to clone Joint::sdfJoint into Joint::sdf
  joint->Load(_parent, _child, ignition::math::Pose3d());
  this->modelDPtr->joints.push_back(joint);
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
  bool paused = this->modelDPtr->world->IsPaused();
  gazebo::physics::JointPtr joint = this->JointByName(_name);
  if (joint)
  {
    this->modelDPtr->world->SetPaused(true);
    if (this->jointController)
    {
      this->jointController->RemoveJoint(joint.get());
    }
    joint->Detach();
    joint->Fini();

    this->modelDPtr->joints.erase(
      std::remove(this->modelDPtr->joints.begin(),
        this->modelDPtr->joints.end(), joint),
      this->modelDPtr->joints.end());
    this->modelDPtr->world->SetPaused(paused);
    return true;
  }
  else
  {
    gzwarn << "Joint [" << _name
           << "] does not exist in model [" << this->Name()
           << "], not removed.\n";
    return false;
  }
}

/////////////////////////////////////////////////
void Model::SetWindMode(const bool _enable)
{
  this->modelDPtr->sdf->GetElement("enable_wind")->Set(_enable);
  for (auto &link : this->modelDPtr->links)
    link->SetWindMode(_enable);
}

/////////////////////////////////////////////////
bool Model::WindMode() const
{
  return this->modelDPtr->sdf->Get<bool>("enable_wind");
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
  this->modelDPtr->introspectionItems.push_back(poseURI);
  gazebo::util::IntrospectionManager::Instance()->Register
      <ignition::math::Pose3d>(poseURI.Str(), fModelPose);

  common::URI linVelURI(uri);
  linVelURI.Query().Insert("p", "vector3d/world_linear_velocity");
  this->modelDPtr->introspectionItems.push_back(linVelURI);
  gazebo::util::IntrospectionManager::Instance()->Register
      <ignition::math::Vector3d>(linVelURI.Str(), fModelLinVel);

  common::URI angVelURI(uri);
  angVelURI.Query().Insert("p", "vector3d/world_angular_velocity");
  this->modelDPtr->introspectionItems.push_back(angVelURI);
  gazebo::util::IntrospectionManager::Instance()->Register
      <ignition::math::Vector3d>(angVelURI.Str(), fModelAngVel);

  common::URI linAccURI(uri);
  linAccURI.Query().Insert("p", "vector3d/world_linear_acceleration");
  this->modelDPtr->introspectionItems.push_back(linAccURI);
  gazebo::util::IntrospectionManager::Instance()->Register
      <ignition::math::Vector3d>(linAccURI.Str(), fModelLinAcc);

  common::URI angAccURI(uri);
  angAccURI.Query().Insert("p", "vector3d/world_angular_acceleration");
  this->modelDPtr->introspectionItems.push_back(angAccURI);
  gazebo::util::IntrospectionManager::Instance()->Register
      <ignition::math::Vector3d>(angAccURI.Str(), fModelAngAcc);
}

/////////////////////////////////////////////////
LinkPtr Model::CreateLink(const std::string &_name)
{
  LinkPtr link;
  if (this->LinkByName(_name))
  {
    gzwarn << "Model [" << this->Name()
      << "] already has a link named [" << _name
      << "], skipping creating link.\n";
    return link;
  }

  link = this->modelDPtr->world->Physics()->CreateLink(
      shared_from_this());

  link->SetName(_name);
  this->modelDPtr->links.push_back(link);

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
