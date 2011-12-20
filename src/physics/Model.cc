/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
/* Desc: Base class for all models.
 * Author: Nathan Koenig and Andrew Howard
 * Date: 8 May 2003
 */

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <boost/thread/recursive_mutex.hpp>

#include <sstream>
#include <float.h>

#include "common/KeyFrame.hh"
#include "common/Animation.hh"
#include "common/Plugin.hh"
#include "common/Events.hh"
#include "common/Exception.hh"
#include "common/Console.hh"
#include "common/CommonTypes.hh"

#include "physics/Joint.hh"
#include "physics/Link.hh"
#include "physics/World.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/Model.hh"
#include "physics/Contact.hh"

#include "transport/Node.hh"

using namespace gazebo;
using namespace physics;

class LinkUpdate_TBB
{
  public: LinkUpdate_TBB(Link_V *bodies) : bodies(bodies) {}

  public: void operator() (const tbb::blocked_range<size_t> &r) const
  {
    for (size_t i=r.begin(); i != r.end(); i++)
    {
      (*this->bodies)[i]->Update();
    }
  }

  private: Link_V *bodies;
};


////////////////////////////////////////////////////////////////////////////////
// Constructor
Model::Model(BasePtr parent)
  : Entity(parent)
{
  this->AddType(MODEL);
  this->updateMutex = new boost::recursive_mutex();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Model::~Model()
{
  delete this->updateMutex;
}

////////////////////////////////////////////////////////////////////////////////
// Load the model
void Model::Load( sdf::ElementPtr &_sdf )
{
  Entity::Load(_sdf);

  this->jointPub = this->node->Advertise<msgs::Joint>("~/joint");

  this->SetStatic( this->sdf->GetValueBool("static") );
  this->sdf->GetAttribute("static")->SetUpdateFunc(
      boost::bind(&Entity::IsStatic, this));

  // TODO: check for duplicate model, and raise an error
  //BasePtr dup = Base::GetByName(this->GetScopedName());

  // Load the bodies
  if (_sdf->HasElement("link"))
  {
    sdf::ElementPtr linkElem = _sdf->GetElement("link");
    bool first = true;
    while (linkElem)
    {
      // Create a new link
      LinkPtr link = this->GetWorld()->GetPhysicsEngine()->CreateLink(
          boost::shared_static_cast<Model>(shared_from_this()));

      /// FIXME: canonical link is hardcoded to the first link.
      ///        warn users for now, need  to add parsing of
      ///        the canonical tag in sdf
      if (first)
      {
        link->SetCanonicalLink(true);
        this->canonicalLink = link;
        first = false;
      }

      // Load the link using the config node. This also loads all of the
      // bodies collisionetries
      link->Load(linkElem);
      linkElem = linkElem->GetNextElement();
    }
  }

  // Load the joints
  if (_sdf->HasElement("joint"))
  {
    sdf::ElementPtr jointElem = _sdf->GetElement("joint");
    while (jointElem)
    {
      this->LoadJoint(jointElem);
      jointElem = jointElem->GetNextElement();
    }
  }

  // Load the plugins
  if (_sdf->HasElement("plugin"))
  {
    sdf::ElementPtr pluginElem = _sdf->GetElement("plugin");
    while (pluginElem)
    {
      this->LoadPlugin(pluginElem);
      pluginElem = pluginElem->GetNextElement();
    }
  }

}

////////////////////////////////////////////////////////////////////////////////
// Initialize the model
void Model::Init()
{
  math::Pose pose;

  // Get the position and orientation of the model (relative to parent)
  pose = this->sdf->GetOrCreateElement("origin")->GetValuePose("pose");

  // Record the model's initial pose (for reseting)
  this->SetInitialRelativePose(pose);

  this->SetRelativePose( pose );

  // Initialize the bodies before the joints
  for (Base_V::iterator iter = this->children.begin(); 
       iter!=this->children.end(); iter++)
  {
    if ((*iter)->HasType(Base::LINK))
      boost::shared_static_cast<Link>(*iter)->Init();
    else if ((*iter)->HasType(Base::MODEL))
    {
      boost::shared_static_cast<Model>(*iter)->Init();
    }
  }

  // Initialize the joints last.
  for (Joint_V::iterator iter = this->joints.begin(); 
       iter != this->joints.end(); iter++)
  {
    (*iter)->Init();
  }

  for (std::vector<ModelPluginPtr>::iterator iter = this->plugins.begin();
       iter != this->plugins.end(); iter++)
  {
    (*iter)->Init();
  }
}


////////////////////////////////////////////////////////////////////////////////
// Update the model
void Model::Update()
{
  this->updateMutex->lock();

  if (this->jointAnimations.size() > 0)
  {
    common::NumericKeyFrame kf(0);
    std::map<std::string, double> jointPositions;
    std::map<std::string, common::NumericAnimationPtr>::iterator iter;
    iter = this->jointAnimations.begin();
    while (iter !=this->jointAnimations.end())
    {
      iter->second->GetInterpolatedKeyFrame(kf);

      iter->second->AddTime(
          (this->world->GetSimTime() - this->prevAnimationTime).Double());

      if (iter->second->GetTime() < iter->second->GetLength())
      {
        iter->second->GetInterpolatedKeyFrame(kf);
        jointPositions[iter->first] = kf.GetValue();
      }
      else
      {
        this->jointAnimations.erase(iter);
      }
      iter++;
    }
    if (jointPositions.size() > 0)
    {
      this->SetJointPositions(jointPositions);
    }
    else
    {
      if (this->onJointAnimationComplete)
        this->onJointAnimationComplete();
    }
    this->prevAnimationTime = this->world->GetSimTime();
  }

  this->updateMutex->unlock();
}

////////////////////////////////////////////////////////////////////////////////
// Remove a child
void Model::RemoveChild(EntityPtr child)
{
  Joint_V::iterator jiter;

  if (child->HasType(LINK))
  {
    bool done = false;

    while (!done)
    {
      done = true;

      for (jiter = this->joints.begin(); jiter != this->joints.end(); jiter++)
      {
        if (!(*jiter))
          continue;

        LinkPtr jlink0 = (*jiter)->GetJointLink(0);
        LinkPtr jlink1 = (*jiter)->GetJointLink(1);

        if (!jlink0 || !jlink1 || jlink0->GetName() == child->GetName() ||
            jlink1->GetName() == child->GetName() ||
            jlink0->GetName() == jlink1->GetName())
        {
          this->joints.erase( jiter );
          done = false;
          break;
        }
      }
    }
  }

  Entity::RemoveChild(child->GetId());

  Base_V::iterator iter;
  for (iter =this->children.begin(); iter != this->children.end(); iter++)
    if (*iter && (*iter)->HasType(LINK))
      boost::static_pointer_cast<Link>(*iter)->SetEnabled(true);

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the model
void Model::Fini()
{
  Entity::Fini();

  this->attachedModels.clear();
  this->joints.clear();
  this->plugins.clear();
  this->canonicalLink.reset();
}

////////////////////////////////////////////////////////////////////////////////
//// Update the parameters using new sdf values
void Model::UpdateParameters( sdf::ElementPtr &_sdf )
{
  Entity::UpdateParameters( _sdf );

  if (_sdf->HasElement("link"))
  {
    sdf::ElementPtr linkElem = _sdf->GetElement("link");
    while (linkElem)
    {
      LinkPtr link = boost::shared_dynamic_cast<Link>(
          this->GetChild(linkElem->GetValueString("name")));
      link->UpdateParameters(linkElem);
      linkElem = linkElem->GetNextElement();
    }
  }
  /*

  if (_sdf->HasElement("joint"))
  {
    sdf::ElementPtr jointElem = _sdf->GetElement("joint");
    while (jointElem)
    {
      JointPtr joint = boost::shared_dynamic_cast<Joint>(this->GetChild(jointElem->GetValueString("name")));
      joint->UpdateParameters(jointElem);
      jointElem = jointElem->GetNextElement();
    }
  }
  */
}

////////////////////////////////////////////////////////////////////////////////
/// Get the SDF values for the model
const sdf::ElementPtr &Model::GetSDF()
{
  Entity::GetSDF();
  return this->sdf;
}

////////////////////////////////////////////////////////////////////////////////
// Reset the model
void Model::Reset()
{
  Entity::Reset();

  for (Joint_V::iterator jiter=this->joints.begin();
       jiter!=this->joints.end(); jiter++)
  {
    (*jiter)->Reset();
  }

  for (std::vector<ModelPluginPtr>::iterator iter = this->plugins.begin();
       iter != this->plugins.end(); iter++)
  {
    (*iter)->Reset();
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the linear velocity of the model
void Model::SetLinearVel( const math::Vector3 &vel )
{
  for (Base_V::iterator iter = this->children.begin();
      iter != this->children.end(); iter++)
  {
    if (*iter && (*iter)->HasType(LINK))
    {
      LinkPtr link = boost::shared_static_cast<Link>(*iter);
      link->SetEnabled(true);
      link->SetLinearVel( vel );
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the angular velocity of the model
void Model::SetAngularVel( const math::Vector3 &vel )
{
  Base_V::iterator iter;

  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    if (*iter && (*iter)->HasType(LINK))
    {
      LinkPtr link = boost::shared_static_cast<Link>(*iter);
      link->SetEnabled(true);
      link->SetAngularVel( vel );
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the linear acceleration of the model
void Model::SetLinearAccel( const math::Vector3 &accel )
{
  Base_V::iterator iter;

  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    if (*iter && (*iter)->HasType(LINK))
    {
      LinkPtr link = boost::shared_static_cast<Link>(*iter);
      link->SetEnabled(true);
      link->SetLinearAccel( accel );
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the angular acceleration of the model
void Model::SetAngularAccel( const math::Vector3 &accel )
{
  Base_V::iterator iter;

  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    if (*iter && (*iter)->HasType(LINK))
    {
      LinkPtr link = boost::shared_static_cast<Link>(*iter);
      link->SetEnabled(true);
      link->SetAngularAccel( accel );
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear velocity of the model
math::Vector3 Model::GetRelativeLinearVel() const
{
  if (!this->GetLink("canonical"))
    return this->GetLink("canonical")->GetRelativeLinearVel();
  else
    return math::Vector3(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear velocity of the entity in the world frame
math::Vector3 Model::GetWorldLinearVel() const
{
  if (!this->GetLink("canonical"))
    return this->GetLink("canonical")->GetWorldLinearVel();
  else
    return math::Vector3(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular velocity of the model
math::Vector3 Model::GetRelativeAngularVel() const
{
  if (!this->GetLink("canonical"))
    return this->GetLink("canonical")->GetRelativeAngularVel();
  else
    return math::Vector3(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular velocity of the model in the world frame
math::Vector3 Model::GetWorldAngularVel() const
{
  if (!this->GetLink("canonical"))
    return this->GetLink("canonical")->GetWorldAngularVel();
  else
    return math::Vector3(0,0,0);
}


////////////////////////////////////////////////////////////////////////////////
/// Get the linear acceleration of the model
math::Vector3 Model::GetRelativeLinearAccel() const
{
  if (!this->GetLink("canonical"))
    return this->GetLink("canonical")->GetRelativeLinearAccel();
  else
    return math::Vector3(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear acceleration of the model in the world frame
math::Vector3 Model::GetWorldLinearAccel() const
{
  if (!this->GetLink("canonical"))
    return this->GetLink("canonical")->GetWorldLinearAccel();
  else
    return math::Vector3(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the model
math::Vector3 Model::GetRelativeAngularAccel() const
{
  if (!this->GetLink("canonical"))
    return this->GetLink("canonical")->GetRelativeAngularAccel();
  else
    return math::Vector3(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the model in the world frame
math::Vector3 Model::GetWorldAngularAccel() const
{
  if (!this->GetLink("canonical"))
    return this->GetLink("canonical")->GetWorldAngularAccel();
  else
    return math::Vector3(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the size of the bounding box
math::Box Model::GetBoundingBox() const
{
  math::Box box;
  Base_V::const_iterator iter;

  box.min.Set(FLT_MAX, FLT_MAX, FLT_MAX);
  box.max.Set(-FLT_MAX, -FLT_MAX, -FLT_MAX);

  for (iter=this->children.begin(); iter!=this->children.end(); iter++)
  {
    if (*iter && (*iter)->HasType(LINK))
    {
      math::Box linkBox;
      LinkPtr link = boost::shared_static_cast<Link>(*iter);
      linkBox = link->GetBoundingBox();
      box += linkBox;
    }
  }

  return box;
}
 
////////////////////////////////////////////////////////////////////////////////
/// Get the number of joints
unsigned int Model::GetJointCount() const
{
  return this->joints.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a joing by index
JointPtr Model::GetJoint( unsigned int index ) const
{
  if (index >= this->joints.size())
    gzthrow("Invalid joint index[" << index << "]\n");

  return this->joints[index];
}

////////////////////////////////////////////////////////////////////////////////
// Get a joint by name
JointPtr Model::GetJoint(const std::string &name)
{
  JointPtr result;
  Joint_V::iterator iter;

  for (iter = this->joints.begin(); iter != this->joints.end(); iter++)
  {
    if ( (*iter)->GetName() == name)
    {
      result = (*iter);
      break;
    }
  }

  return result;
}

LinkPtr Model::GetLinkById(unsigned int _id) const
{
  return boost::shared_dynamic_cast<Link>(this->GetById(_id));
}

/// Get a link by name
LinkPtr Model::GetLink(const std::string &name) const
{
  Base_V::const_iterator biter;
  LinkPtr result;

  if (name == "canonical")
  {
    result = this->canonicalLink;
  }
  else
  {
    for (biter=this->children.begin(); biter != this->children.end(); biter++)
    {
      if ((*biter)->GetName() == name)
      {
        result = boost::shared_dynamic_cast<Link>(*biter);
        break;
      }
    }
  }
 
  return result;
}

////////////////////////////////////////////////////////////////////////////////
// Load a new joint helper function
void Model::LoadJoint( sdf::ElementPtr &_sdf )
{
  JointPtr joint;

  std::string type = _sdf->GetValueString("type");

  joint = this->GetWorld()->GetPhysicsEngine()->CreateJoint( type );
  if (!joint)
    gzthrow("Unable to create joint of type[" + type + "]\n");

  joint->SetModel( boost::shared_static_cast<Model>(shared_from_this()) );

  // Load the joint
  joint->Load(_sdf);

  if (this->GetJoint( joint->GetName() ) != NULL)
    gzthrow( "can't have two joint with the same name");

  msgs::Joint msg;
  msg.set_name( joint->GetName() );
  msg.set_type( msgs::Joint::REVOLUTE );

  if(joint->GetParent())
    msg.set_parent( joint->GetParent()->GetScopedName() );
  else
    msg.set_parent( "world" );

  if(joint->GetChild())
    msg.set_child( joint->GetChild()->GetScopedName() );
  else
    msg.set_child( "world" );
  
  this->jointPub->Publish(msg);

  this->joints.push_back( joint );
}

////////////////////////////////////////////////////////////////////////////////
// Load a plugin
void Model::LoadPlugin( sdf::ElementPtr &_sdf )
{
  std::string name = _sdf->GetValueString("name");
  std::string filename = _sdf->GetValueString("filename");
  gazebo::ModelPluginPtr plugin = gazebo::ModelPlugin::Create(filename, name);
  if (plugin)
  {
    ModelPtr myself = boost::shared_static_cast<Model>(shared_from_this());
    plugin->Load(myself, _sdf);
    this->plugins.push_back( plugin );
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the gravity mode of the model
void Model::SetGravityMode( const bool &v )
{
  Base_V::iterator iter;

  for (iter=this->children.begin(); iter!=this->children.end(); iter++)
  {
    if (*iter && (*iter)->HasType(LINK))
    {
      boost::shared_static_cast<Link>(*iter)->SetGravityMode( v );
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Set the collide mode of the model
void Model::SetCollideMode( const std::string &m )
{
  Base_V::iterator iter;

  for (iter=this->children.begin(); iter!=this->children.end(); iter++)
  {
    if (*iter && (*iter)->HasType(LINK))
    {
      boost::shared_static_cast<Link>(*iter)->SetCollideMode( m );
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Set the Laser retro property of the model
void Model::SetLaserRetro(const float &retro)
{
  Base_V::iterator iter;

  for (iter=this->children.begin(); iter!=this->children.end(); iter++)
  {
    if (*iter && (*iter)->HasType(LINK))
    {
       boost::shared_static_cast<Link>(*iter)->SetLaserRetro(retro);
    }
  }
}

void Model::FillModelMsg(msgs::Model &_msg)
{
  _msg.set_name(this->GetName());
  _msg.set_is_static(this->IsStatic());
  _msg.mutable_pose()->CopyFrom(
      msgs::Convert(this->GetWorldPose()));
  _msg.set_id(this->GetId());

  _msg.add_visual()->CopyFrom(*this->visualMsg);

  for (unsigned int j=0; j < this->GetChildCount(); j++)
  {
    if (this->GetChild(j)->HasType(Base::LINK))
    {
      LinkPtr link = boost::shared_dynamic_cast<Link>( this->GetChild(j) );
      link->FillLinkMsg( *_msg.add_link() );
    }
    if (this->GetChild(j)->HasType(Base::JOINT))
    {
      JointPtr joint = boost::shared_dynamic_cast<Joint>( this->GetChild(j) );
      joint->FillJointMsg( *_msg.add_joints() );
    }
  }
}

void Model::ProcessMsg(const msgs::Model &_msg)
{
  if (_msg.id() != this->GetId())
  {
    gzerr << "Incorrect ID\n";
    return;
  }

  this->SetName(_msg.name());
  if (_msg.has_pose())
    this->SetWorldPose(msgs::Convert(_msg.pose()));
  for (int i=0; i < _msg.link_size(); i++)
  {
    LinkPtr link = this->GetLinkById(_msg.link(i).id());
    if (link)
      link->ProcessMsg(_msg.link(i));
  }

  if (_msg.has_is_static())
    this->SetStatic(_msg.is_static());
}

void Model::SetJointPositions(
    const std::map<std::string, double> &_jointPositions)

{
  // go through all joints in this model and update each one
  //   for each joint update, recursively update all children
  Joint_V::iterator iter;
  std::map<std::string, double>::const_iterator jiter = _jointPositions.begin();
  for (iter = this->joints.begin(); iter != this->joints.end() ; iter++)
  {
    JointPtr joint = *iter;

    std::map<std::string, double>::const_iterator jiter = 
      _jointPositions.find(joint->GetName());
    unsigned int type = joint->GetType();

    // only deal with hinge and revolute joints in the user 
    // request joint_names list
    if ((type == Base::HINGE_JOINT || type == Base::SLIDER_JOINT) &&
        jiter != _jointPositions.end())
    {
      LinkPtr parentLink = joint->GetParent();
      LinkPtr childLink = joint->GetChild();

      if (parentLink && childLink && 
          parentLink->GetName() != childLink->GetName())
      {
        // transform about the current anchor, about the axis
        switch(type)
        {
          case Base::HINGE_JOINT: 
            {
              // rotate child (childLink) about anchor point, by delta-angle 
              // along axis
              double dangle = jiter->second - joint->GetAngle(0).GetAsRadian();

              math::Vector3 anchor;
              math::Vector3 axis;

              if (this->IsStatic())
              {
                math::Pose worldPose = childLink->GetWorldPose();
                axis = worldPose.rot.RotateVector(joint->GetLocalAxis(0));
                anchor = childLink->GetWorldPose().pos;
              }
              else
              {
                anchor = joint->GetAnchor(0);
                axis = joint->GetGlobalAxis(0);
              }

              this->RotateBodyAndChildren(childLink, anchor,
                                          axis, dangle, true);
              break;
            }
          case Base::SLIDER_JOINT: 
            {
              double dposition = jiter->second - 
                                joint->GetAngle(0).GetAsRadian();


              math::Vector3 anchor;
              math::Vector3 axis;

              if (this->IsStatic())
              {
                math::Pose worldPose = childLink->GetWorldPose();
                axis = worldPose.rot.RotateVector(joint->GetLocalAxis(0));
                anchor = childLink->GetWorldPose().pos;
              }
              else
              {
                anchor = joint->GetAnchor(0);
                axis = joint->GetGlobalAxis(0);
              }

              this->SlideBodyAndChildren(childLink, anchor,
                                         axis, dposition, true);
              break;
            }
          default: 
            {
              gzwarn << "Setting non HINGE/SLIDER joint types not"
                       << "implemented [" << joint->GetName() << "]\n";
              break;
            }
        }
      }
    }
  }

  for (jiter = _jointPositions.begin(); jiter != _jointPositions.end(); jiter++)
  {
    JointPtr joint = this->GetJoint(jiter->first);
    joint->SetAngle(0, jiter->second);
  } 
}

void Model::RotateBodyAndChildren(LinkPtr _body1, const math::Vector3 &_anchor, 
    const math::Vector3 &_axis, double _dangle, bool _updateChildren)
{
  math::Pose worldPose = _body1->GetWorldPose();

  // relative to anchor point
  math::Pose relativePose(worldPose.pos - _anchor, worldPose.rot);

  // take axis rotation and turn it int a quaternion
  math::Quaternion rotation(_axis, _dangle); 

  // rotate relative pose by rotation
  math::Pose newRelativePose;

  newRelativePose.pos = rotation.RotateVector(relativePose.pos);
  newRelativePose.rot = rotation * relativePose.rot;

  math::Pose newWorldPose(newRelativePose.pos + _anchor,
                          newRelativePose.rot);

  _body1->SetWorldPose(newWorldPose);

  // recurse through children bodies
  if (_updateChildren) 
  {
    std::vector<LinkPtr> bodies;
    this->GetAllChildrenBodies(bodies, _body1);

    for (std::vector<LinkPtr>::iterator biter = bodies.begin(); 
        biter != bodies.end(); biter++)
    {
      this->RotateBodyAndChildren((*biter), _anchor, _axis, _dangle, false);
    }
  }
}


void Model::SlideBodyAndChildren(LinkPtr _body1, const math::Vector3 &_anchor, 
    const math::Vector3 &_axis, double _dposition, bool _updateChildren)
{
  math::Pose worldPose = _body1->GetWorldPose();

  // relative to anchor point
  math::Pose relativePose(worldPose.pos - _anchor, worldPose.rot);

  // slide relative pose by dposition along axis
  math::Pose newRelativePose;
  newRelativePose.pos = relativePose.pos + _axis * _dposition;
  newRelativePose.rot = relativePose.rot;

  math::Pose newWorldPose(newRelativePose.pos + _anchor, newRelativePose.rot);
  _body1->SetWorldPose(newWorldPose);

  // recurse through children bodies
  if (_updateChildren) 
  {
    std::vector<LinkPtr> bodies;
    this->GetAllChildrenBodies(bodies, _body1);

    for (std::vector<LinkPtr>::iterator biter = bodies.begin(); 
        biter != bodies.end(); biter++)
    {
      this->SlideBodyAndChildren((*biter), _anchor, _axis, _dposition, false);
    }
  }
}



void Model::GetAllChildrenBodies(std::vector<LinkPtr> &_bodies, 
                                 const LinkPtr &_body)
{
  // strategy, for each child, recursively look for children
  //           for each child, also look for parents to catch multiple roots
  for (unsigned int i = 0; i < this->GetJointCount(); i++)
  {
    gazebo::physics::JointPtr joint = this->GetJoint(i);

    // recurse through children connected by joints
    LinkPtr parentLink = joint->GetParent();
    LinkPtr childLink = joint->GetChild();
    if (parentLink && childLink
        && parentLink->GetName() != childLink->GetName()
        && parentLink->GetName() == _body->GetName()
        && !this->InBodies(childLink, _bodies))
    {
      _bodies.push_back(childLink);
      this->GetAllChildrenBodies(_bodies, childLink);
      this->GetAllParentBodies(_bodies, childLink, _body);
    }
  }
}

void Model::GetAllParentBodies(std::vector<LinkPtr> &_bodies, 
    const LinkPtr &_body, const LinkPtr &_origParentBody)
{
  for (unsigned int i = 0; i < this->GetJointCount(); i++)
  {
    JointPtr joint = this->GetJoint(i);

    // recurse through children connected by joints
    LinkPtr parentLink = joint->GetParent();
    LinkPtr childLink = joint->GetChild();

    if (parentLink && childLink
        && parentLink->GetName() != childLink->GetName()
        && childLink->GetName() == _body->GetName()
        && parentLink->GetName() != _origParentBody->GetName()
        && !this->InBodies(parentLink, _bodies))
    {
      _bodies.push_back(parentLink);
      this->GetAllParentBodies(_bodies, childLink, _origParentBody);
    }
  }
}

bool Model::InBodies(const LinkPtr &_body, const std::vector<LinkPtr> &_bodies)
{
  for (std::vector<LinkPtr>::const_iterator bit = _bodies.begin(); 
       bit != _bodies.end(); bit++)
  {
    if ((*bit)->GetName() == _body->GetName()) 
      return true;
  }

  return false;
}

void Model::SetJointAnimation(
    const std::map<std::string, common::NumericAnimationPtr> _anims,
    boost::function<void()> _onComplete)
{
  this->updateMutex->lock();
  std::map<std::string, common::NumericAnimationPtr>::const_iterator iter;
  for (iter = _anims.begin(); iter != _anims.end(); iter++)
  {
    this->jointAnimations[iter->first] = iter->second;
  }
  this->onJointAnimationComplete = _onComplete;
  this->prevAnimationTime = this->world->GetSimTime();
  this->updateMutex->unlock();
}

void Model::AttachStaticModel(ModelPtr &_model, const std::string &_linkName,
                              math::Pose _offset)
{
  if (!_model->IsStatic())
  {
    gzerr << "AttachStaticModel requires a static model\n";
    return;
  }

  this->attachedModels.push_back(_model);
  this->attachedModelsLinks.push_back(_linkName);
  this->attachedModelsOffset.push_back(_offset);
}

void Model::DetachStaticModel(const std::string &_modelName)
{
  for (unsigned int i=0; i < this->attachedModels.size(); i++)
  {
    if (this->attachedModels[i]->GetName() == _modelName)
    {
      this->attachedModels.erase(this->attachedModels.begin()+i);
      this->attachedModelsLinks.erase(this->attachedModelsLinks.begin()+i);
      this->attachedModelsOffset.erase(this->attachedModelsOffset.begin()+i);
      break;
    }
  }
}

void Model::OnPoseChange()
{
  math::Pose p;
  for (unsigned int i=0; i < this->attachedModels.size(); i++)
  {
    p = this->GetLink(this->attachedModelsLinks[i])->GetWorldPose();
    p += this->attachedModelsOffset[i];
    this->attachedModels[i]->SetWorldPose(p, true);
  }
}
