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

#include "common/Events.hh"
#include "common/Global.hh"
#include "common/Exception.hh"
#include "common/Console.hh"
#include "common/XMLConfig.hh"

#include "physics/Joint.hh"
#include "physics/Body.hh"
#include "physics/World.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/Model.hh"

using namespace gazebo;
using namespace physics;

class BodyUpdate_TBB
{
  public: BodyUpdate_TBB(Body_V *bodies) : bodies(bodies) {}

  public: void operator() (const tbb::blocked_range<size_t> &r) const
  {
    for (size_t i=r.begin(); i != r.end(); i++)
    {
      (*this->bodies)[i]->Update();
    }
  }

  private: Body_V *bodies;
};


////////////////////////////////////////////////////////////////////////////////
// Constructor
Model::Model(BasePtr parent)
  : Entity(parent)
{
  this->AddType(MODEL);

  common::Param::Begin(&this->parameters);
  this->canonicalBodyNameP = new common::ParamT<std::string>("canonicalBody",
                                                   std::string(),0);

  this->xyzP = new common::ParamT<math::Vector3>("xyz", math::Vector3(0,0,0), 0);
  this->xyzP->Callback(&Entity::SetRelativePosition, (Entity*)this);

  this->rpyP = new common::ParamT<math::Quatern>("rpy", math::Quatern(1,0,0,0), 0);
  this->rpyP->Callback( &Entity::SetRelativeRotation, (Entity*)this);

  this->enableGravityP = new common::ParamT<bool>("enable_gravity", true, 0);
  this->enableGravityP->Callback( &Model::SetGravityMode, this );

  this->enableFrictionP = new common::ParamT<bool>("enable_friction", true, 0);
  this->enableFrictionP->Callback( &Model::SetFrictionMode, this );

  this->collideP = new common::ParamT<std::string>("collide", "all", 0);
  this->collideP->Callback( &Model::SetCollideMode, this );

  this->laserFiducialP = new common::ParamT<int>("laser_fiducial_id", -1, 0);
  this->laserFiducialP->Callback( &Model::SetLaserFiducialId, this );

  this->laserRetroP = new common::ParamT<float>("laser_retro", -1, 0);
  this->laserRetroP->Callback( &Model::SetLaserRetro, this );
  common::Param::End();
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Model::~Model()
{
  /*Base_V::iterator eiter;
  for (eiter =this->children.begin(); eiter != this->children.end();)
    if (*eiter && (*eiter)->HasType(BODY))
    {
      delete (*eiter);
      *eiter = NULL;
      this->children.erase(eiter); // effectively remove child
    }
    else
      eiter++;
      */

  /* NATY: Put this back in 
  JointContainer::iterator jiter;
  std::map< std::string, Controller* >::iterator citer;

  if (this->graphicsHandler)
  {
    delete this->graphicsHandler;
    this->graphicsHandler = NULL;
  }

  for (jiter = this->joints.begin(); jiter != this->joints.end(); jiter++)
    if (*jiter)
      delete *jiter;
  this->joints.clear();

  for (citer = this->controllers.begin();
       citer != this->controllers.end(); citer++)
  {
    if (citer->second)
    {
      delete citer->second;
      citer->second = NULL;
    }
  }
  this->controllers.clear();

  if (this->myBodyNameP)
  {
    delete this->myBodyNameP;
    this->myBodyNameP = NULL;
  }
  */
}

////////////////////////////////////////////////////////////////////////////////
// Load the model
void Model::Load(common::XMLConfigNode *node)
{
  Entity::Load(node);

  common::XMLConfigNode *childNode;

  this->staticP->Load(node);
  this->canonicalBodyNameP->Load(node);
  this->enableGravityP->Load(node);
  this->enableFrictionP->Load(node);
  this->collideP->Load(node);
  this->laserFiducialP->Load(node);
  this->laserRetroP->Load(node);


  // TODO: check for duplicate model, and raise an error
  //BasePtr dup = Base::GetByName(this->GetScopedName());
  childNode = node->GetChild("origin");
  this->xyzP->Load(childNode);
  this->rpyP->Load(childNode);

  this->LoadChildrenAndJoints(node);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the model
void Model::Init()
{
  math::Pose3d pose;

  this->SetStatic( **(this->staticP) );

  // Get the position and orientation of the model (relative to parent)
  pose.pos = **this->xyzP;
  pose.rot = **this->rpyP;

  this->SetRelativePose( pose );

  // Record the model's initial pose (for reseting)
  this->SetInitialPose( pose );

  if (this->canonicalBodyNameP->GetValue().empty())
  {
    /// FIXME: Model::pose is set to the pose of first body
    ///        seems like there should be a warning for users
    for (unsigned int i=0; i < this->children.size(); i++)
    {
      if (this->children[i]->HasType(BODY))
      {
        this->canonicalBodyNameP->SetValue( this->children[i]->GetName() );
        break;
      }
    }
  }

  this->canonicalBody = boost::shared_dynamic_cast<Body>(this->GetChild(**this->canonicalBodyNameP));

  // This must be placed after creation of the bodies
  // Static variable overrides the gravity
  if (**this->staticP == false)
    this->SetGravityMode( **this->enableGravityP );

  //global fiducial and retro id
  if (**this->laserFiducialP != -1.0 )
    this->SetLaserFiducialId(**this->laserFiducialP);

  if (**this->laserRetroP != -1.0)
    this->SetLaserRetro(**this->laserRetroP);

  // Initialize the bodies before the joints
  for (Base_V::iterator iter = this->children.begin(); 
       iter!=this->children.end(); iter++)
  {
    if ((*iter)->HasType(BODY))
      boost::shared_static_cast<Body>(*iter)->Init();
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
// Save the model in XML format
void Model::Save(std::string &prefix, std::ostream &stream)
{
  std::string p = prefix + "  ";
  std::string typeName;
  Base_V::iterator bodyIter;
  Joint_V::iterator jointIter;

  this->xyzP->SetValue( this->GetRelativePose().pos );
  this->rpyP->SetValue( this->GetRelativePose().rot );


  stream << prefix << "<model";
  stream << " name=\"" << this->GetName() << "\">\n"; 
  stream << prefix << "  " << *(this->xyzP) << "\n";
  stream << prefix << "  " << *(this->rpyP) << "\n";
  stream << prefix << "  " << *(this->enableGravityP) << "\n";
  stream << prefix << "  " << *(this->enableFrictionP) << "\n";
  stream << prefix << "  " << *(this->collideP) << "\n";

  stream << prefix << "  " << *(this->staticP) << "\n";

  for (bodyIter=this->children.begin(); bodyIter!=this->children.end(); bodyIter++)
  {
    stream << "\n";
    EntityPtr entity = boost::shared_dynamic_cast<Entity>(*bodyIter);
    if (entity && entity->HasType(BODY))
    {
      boost::shared_static_cast<Body>(entity)->Save(p, stream);
    }
  }

  // Save all the joints
  for (jointIter = this->joints.begin(); jointIter != this->joints.end(); jointIter++)
  {
    if (*jointIter)
      (*jointIter)->Save(p, stream);
  }

  // Save all the controllers
  /*for (contIter=this->controllers.begin();
      contIter!=this->controllers.end(); contIter++)
  {
    if (contIter->second)
      contIter->second->Save(p, stream);
  }
  */

  // Save all child models
  Base_V::iterator eiter;
  for (eiter = this->children.begin(); eiter != this->children.end(); eiter++)
  {
    if (*eiter && (*eiter)->HasType(MODEL))
    {
      ModelPtr cmodel = boost::static_pointer_cast<Model>(*eiter);
      cmodel->Save(p, stream);
    }
  }

  stream << prefix << "</model:" << typeName << ">\n";
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
        BodyUpdate_TBB(&this->bodies) );
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

        BodyPtr jbody0 = (*jiter)->GetJointBody(0);
        BodyPtr jbody1 = (*jiter)->GetJointBody(1);

        if (!jbody0 || !jbody1 || jbody0->GetName() == child->GetName() ||
            jbody1->GetName() == child->GetName() ||
            jbody0->GetName() == jbody1->GetName())
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
      boost::static_pointer_cast<Body>(*iter)->SetEnabled(true);

}

////////////////////////////////////////////////////////////////////////////////
// Finalize the model
void Model::Fini()
{
  /* TODO: Put back in
  Base_V::iterator biter;
  std::map<std::string, Controller* >::iterator contIter;

  for (contIter = this->controllers.begin();
       contIter != this->controllers.end(); contIter++)
  {
    contIter->second->Fini();
  }

  for (biter=this->children.begin(); biter != this->children.end(); biter++)
  {
    if (*biter && (*biter)->HasType(BODY))
    {
      Body *body = (Body*)*biter;
      body->Fini();
    }
  }

  if (this->graphicsHandler)
  {
    delete this->graphicsHandler;
    this->graphicsHandler = NULL;
  }

  Base_V::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    if (*iter && (*iter)->HasType(MODEL))
    {
      Model *m = (Model*)*iter;
      m->Fini();
    }
  }

  */
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
      Body *body = (Body*)*biter;
      body->SetLinearVel(v);
      body->SetAngularVel(v);
      body->SetForce(v);
      body->SetTorque(v);
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
      BodyPtr body = boost::shared_static_cast<Body>(*iter);
      body->SetEnabled(true);
      body->SetLinearVel( vel );
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
      BodyPtr body = boost::shared_static_cast<Body>(*iter);
      body->SetEnabled(true);
      body->SetAngularVel( vel );
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
      BodyPtr body = boost::shared_static_cast<Body>(*iter);
      body->SetEnabled(true);
      body->SetLinearAccel( accel );
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
      BodyPtr body = boost::shared_static_cast<Body>(*iter);
      body->SetEnabled(true);
      body->SetAngularAccel( accel );
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear velocity of the model
math::Vector3 Model::GetRelativeLinearVel() const
{
  if (!this->GetBody("canonical"))
    return this->GetBody("canonical")->GetRelativeLinearVel();
  else
    return math::Vector3(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear velocity of the entity in the world frame
math::Vector3 Model::GetWorldLinearVel() const
{
  if (!this->GetBody("canonical"))
    return this->GetBody("canonical")->GetWorldLinearVel();
  else
    return math::Vector3(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular velocity of the model
math::Vector3 Model::GetRelativeAngularVel() const
{
  if (!this->GetBody("canonical"))
    return this->GetBody("canonical")->GetRelativeAngularVel();
  else
    return math::Vector3(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular velocity of the model in the world frame
math::Vector3 Model::GetWorldAngularVel() const
{
  if (!this->GetBody("canonical"))
    return this->GetBody("canonical")->GetWorldAngularVel();
  else
    return math::Vector3(0,0,0);
}


////////////////////////////////////////////////////////////////////////////////
/// Get the linear acceleration of the model
math::Vector3 Model::GetRelativeLinearAccel() const
{
  if (!this->GetBody("canonical"))
    return this->GetBody("canonical")->GetRelativeLinearAccel();
  else
    return math::Vector3(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear acceleration of the model in the world frame
math::Vector3 Model::GetWorldLinearAccel() const
{
  if (!this->GetBody("canonical"))
    return this->GetBody("canonical")->GetWorldLinearAccel();
  else
    return math::Vector3(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the model
math::Vector3 Model::GetRelativeAngularAccel() const
{
  if (!this->GetBody("canonical"))
    return this->GetBody("canonical")->GetRelativeAngularAccel();
  else
    return math::Vector3(0,0,0);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the model in the world frame
math::Vector3 Model::GetWorldAngularAccel() const
{
  if (!this->GetBody("canonical"))
    return this->GetBody("canonical")->GetWorldAngularAccel();
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
      math::Box bodyBox;
      BodyPtr body = boost::shared_static_cast<Body>(*iter);
      bodyBox = body->GetBoundingBox();
      box += bodyBox;
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
/// Get a body by name
BodyPtr Model::GetBody(const std::string &name) const
{
  Base_V::const_iterator biter;
  BodyPtr result;

  if (name == "canonical")
  {
    result = this->canonicalBody;
  }
  else
  {
    for (biter=this->children.begin(); biter != this->children.end(); biter++)
    {
      if ((*biter)->GetName() == name)
      {
        result = boost::shared_dynamic_cast<Body>(*biter);
        break;
      }
    }
  }
 
  return result;
}

////////////////////////////////////////////////////////////////////////////////
// Load a new joint helper function
void Model::LoadJoint(common::XMLConfigNode *node)
{
  if (!node)
    gzthrow("Trying to load a joint with NULL XML information");

  JointPtr joint;

  std::string type = node->GetString("type","hinge",1);

  joint = this->GetWorld()->GetPhysicsEngine()->CreateJoint( type );

  joint->SetModel(boost::shared_static_cast<Model>(shared_from_this()));

  // Load the joint
  joint->Load(node);

  if (this->GetJoint( joint->GetName() ) != NULL)
    gzthrow( "can't have two joint with the same name");

  this->joints.push_back( joint );
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
      boost::shared_static_cast<Body>(*iter)->SetGravityMode( v );
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the gravity mode of the model
void Model::SetFrictionMode( const bool &v )
{
  Base_V::iterator iter;

  for (iter=this->children.begin(); iter!=this->children.end(); iter++)
  {
    if ((*iter) && (*iter)->HasType(BODY))
    {
      boost::shared_static_cast<Body>(*iter)->SetFrictionMode( v );
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
      boost::shared_static_cast<Body>(*iter)->SetCollideMode( m );
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the Fiducial Id of the model
void Model::SetLaserFiducialId( const int &id )
{
  Base_V::iterator iter;

  for (iter=this->children.begin(); iter!=this->children.end(); iter++)
  {
    if (*iter && (*iter)->HasType(BODY))
    {
      boost::shared_static_cast<Body>(*iter)->SetLaserFiducialId( id );
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
       boost::shared_static_cast<Body>(*iter)->SetLaserRetro(retro);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Load a physical model
void Model::LoadChildrenAndJoints(common::XMLConfigNode *node)
{
  common::XMLConfigNode *childNode = NULL;

  // Load the bodies
  childNode = node->GetChild("link");
  while (childNode)
  {
    // Create a new body
    BodyPtr body = this->GetWorld()->GetPhysicsEngine()->CreateBody(
        boost::shared_static_cast<Model>(shared_from_this()));

    // Load the body using the config node. This also loads all of the
    // bodies geometries
    body->Load(childNode);

    childNode = childNode->GetNext("link");
  }

  // Load the joints
  childNode = node->GetChild("joint");
  while (childNode)
  {
    try
    {
      this->LoadJoint(childNode);
    }
    catch (common::Exception e)
    {
      gzerr << "Error Loading Joint[" << childNode->GetString("name", std::string(), 0) << "]\n";
      gzerr <<  e << std::endl;
      childNode = childNode->GetNext("joint");
      continue;
    }
    childNode = childNode->GetNext("joint");
  }
}
