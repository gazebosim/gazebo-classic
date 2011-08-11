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

#include <sstream>
#include <float.h>

#include "common/Plugin.hh"
#include "common/Events.hh"
#include "common/Global.hh"
#include "common/Exception.hh"
#include "common/Console.hh"
#include "common/CommonTypes.hh"

#include "physics/Joint.hh"
#include "physics/Link.hh"
#include "physics/World.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/Model.hh"

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
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Model::~Model()
{
}

////////////////////////////////////////////////////////////////////////////////
// Load the model
void Model::Load( sdf::ElementPtr &_sdf )
{
  Entity::Load(_sdf);

  this->jointPub = this->node->Advertise<msgs::Joint>("~/joint");

  this->SetStatic( this->sdf->GetValueBool("static") );

  // TODO: check for duplicate model, and raise an error
  //BasePtr dup = Base::GetByName(this->GetScopedName());

  // Load the bodies
  if (_sdf->HasElement("link"))
  {
    sdf::ElementPtr linkElem = _sdf->GetElement("link");
    //bool first = false;
    while (linkElem)
    {
      // Create a new link
      LinkPtr link = this->GetWorld()->GetPhysicsEngine()->CreateLink(
          boost::shared_static_cast<Model>(shared_from_this()));

      // if (first)
      // {
      //   link->SetCanonicalLink(true);
      //   first = false;
      // }

      // Load the link using the config node. This also loads all of the
      // bodies geometries
      link->Load(linkElem);

      linkElem = _sdf->GetNextElement("link", linkElem);
    }
  }

  // Load the joints
  if (_sdf->HasElement("joint"))
  {
    sdf::ElementPtr jointElem = _sdf->GetElement("joint");
    while (jointElem)
    {
      this->LoadJoint(jointElem);
      jointElem = _sdf->GetNextElement("joint", jointElem);
    }
  }

  // Load the plugins
  if (_sdf->HasElement("plugin"))
  {
    sdf::ElementPtr pluginElem = _sdf->GetElement("plugin");
    while (pluginElem)
    {
      this->LoadPlugin(pluginElem);
      pluginElem = _sdf->GetNextElement("plugin", pluginElem);
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
  this->SetInitialRelativePose( pose );


  /// FIXME: canonical link is hardcoded to the first link.
  ///        warn users for now, need  to add parsing of
  ///        the canonical tag in sdf
  for (unsigned int i=0; i < this->children.size(); i++)
  {
    if (this->children[i]->HasType(BODY))
    {
      gzwarn << "Model Canonical Link is presetting to first link for now, ignoring any canonical tag if one exists in your xml\n";
      this->canonicalLink = boost::shared_static_cast<Link>(this->children[i]);
      this->canonicalLink->SetCanonicalLink(true);
      break;
    }
  }

  this->SetRelativePose( pose );

  // Initialize the bodies before the joints
  for (Base_V::iterator iter = this->children.begin(); 
       iter!=this->children.end(); iter++)
  {
    if ((*iter)->HasType(BODY))
      boost::shared_static_cast<Link>(*iter)->Init();
    else if ((*iter)->HasType(MODEL))
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
}


////////////////////////////////////////////////////////////////////////////////
// Update the model
void Model::Update()
{
  // NATY: Make this work without renderstate
  /*
  if (this->controllers.size() == 0 && this->IsStatic())
    return;

  if (!this->IsStatic())
  {
    tbb::parallel_for( tbb::blocked_range<size_t>(0, this->bodies.size(), 10),
        LinkUpdate_TBB(&this->bodies) );
  }

  this->contacts.clear();

  std::map<std::string, Controller* >::iterator contIter;
  for (contIter=this->controllers.begin();
      contIter!=this->controllers.end(); contIter++)
  {
    if (contIter->second)
    {
      contIter->second->Update();
    }
  }

  if (RenderState::GetShowJoints())
  {
    JointContainer::iterator jointIter;
    for (jointIter = this->joints.begin(); 
         jointIter != this->joints.end(); jointIter++)
    {
      (*jointIter)->Update();
    }
  }*/
}

////////////////////////////////////////////////////////////////////////////////
// Remove a child
void Model::RemoveChild(EntityPtr child)
{
  Joint_V::iterator jiter;

  if (child->HasType(BODY))
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
    if (*iter && (*iter)->HasType(BODY))
      boost::static_pointer_cast<Link>(*iter)->SetEnabled(true);

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the model
void Model::Fini()
{
  Entity::Fini();

  this->joints.clear();
  this->plugins.clear();
  this->canonicalLink.reset();
}

////////////////////////////////////////////////////////////////////////////////
// Reset the model
void Model::Reset()
{
  /* TODO: Put back in
  JointContainer::iterator jiter;
  Base_V::iterator biter;
  std::map<std::string, Controller* >::iterator citer;
  math::Vector3 v(0,0,0);

  //this->SetWorldPose(this->initPose);  // this has to be relative for nested models to work
  this->SetRelativePose(this->initPose);  // this has to be relative for nested models to work

  for (citer=this->controllers.begin(); citer!=this->controllers.end(); citer++)
  {
    citer->second->Reset();
  }

  for (jiter=this->joints.begin(); jiter!=this->joints.end(); jiter++)
  {
    (*jiter)->Reset();
  }

  for (biter=this->children.begin(); biter != this->children.end(); biter++)
  {
    if (*biter && (*biter)->HasType(BODY))
    {
      Link *link = (Link*)*biter;
      link->SetLinearVel(v);
      link->SetAngularVel(v);
      link->SetForce(v);
      link->SetTorque(v);
    }
  }
  */
}

////////////////////////////////////////////////////////////////////////////////
/// Set the linear velocity of the model
void Model::SetLinearVel( const math::Vector3 &vel )
{
  Base_V::iterator iter;

  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    if (*iter && (*iter)->HasType(BODY))
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
    if (*iter && (*iter)->HasType(BODY))
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
    if (*iter && (*iter)->HasType(BODY))
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
    if (*iter && (*iter)->HasType(BODY))
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
    if (*iter && (*iter)->HasType(BODY))
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

////////////////////////////////////////////////////////////////////////////////
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
  msgs::Init(msg, joint->GetName() );
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
    if (*iter && (*iter)->HasType(BODY))
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
    if (*iter && (*iter)->HasType(BODY))
    {
      boost::shared_static_cast<Link>(*iter)->SetCollideMode( m );
    }
  }
}


////////////////////////////////////////////////////////////////////////////////
/// Set the Laser retro property of the model
void Model::SetLaserRetro( const float &retro )
{
  Base_V::iterator iter;

  for (iter=this->children.begin(); iter!=this->children.end(); iter++)
  {
    if (*iter && (*iter)->HasType(BODY))
    {
       boost::shared_static_cast<Link>(*iter)->SetLaserRetro(retro);
    }
  }
}
