/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */
/* Desc: Base class for all models.
 * Author: Nathan Koenig and Andrew Howard
 * Date: 8 May 2003
 * SVN: $Id$
 */

//#include <boost/python.hpp>

#include <sstream>
#include <iostream>

#include "OgreVisual.hh"
#include "GraphicsIfaceHandler.hh"
#include "Global.hh"
#include "GazeboError.hh"
#include "GazeboMessage.hh"
#include "XMLConfig.hh"
#include "World.hh"
#include "OgreCreator.hh"
#include "Body.hh"
#include "HingeJoint.hh"
#include "Hinge2Joint.hh"
#include "BallJoint.hh"
#include "UniversalJoint.hh"
#include "SliderJoint.hh"
#include "PhysicsEngine.hh"
#include "Controller.hh"
#include "ControllerFactory.hh"
#include "IfaceFactory.hh"
#include "Model.hh"

#ifdef TIMING
#include "Simulator.hh"
#endif

using namespace gazebo;

uint Model::lightNumber = 0;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Model::Model(Model *parent)
    : Entity(parent)
{
  this->type = "";
  this->joint = NULL;

  Param::Begin(&this->parameters);
  this->canonicalBodyNameP = new ParamT<std::string>("canonicalBody",
                                                   std::string(),0);

  this->xyzP = new ParamT<Vector3>("xyz", Vector3(0,0,0), 0);
  this->xyzP->Callback(&Model::SetPosition, this);

  this->rpyP = new ParamT<Quatern>("rpy", Quatern(1,0,0,0), 0);
  this->rpyP->Callback( &Model::SetRotation, this);

  this->enableGravityP = new ParamT<bool>("enableGravity", true, 0);
  this->enableGravityP->Callback( &Model::SetGravityMode, this );

  this->enableFrictionP = new ParamT<bool>("enableFriction", true, 0);
  this->enableFrictionP->Callback( &Model::SetFrictionMode, this );

  this->collideP = new ParamT<std::string>("collide", "all", 0);
  this->collideP->Callback( &Model::SetCollideMode, this );

  this->laserFiducialP = new ParamT<int>("laserFiducialId", -1, 0);
  this->laserFiducialP->Callback( &Model::SetLaserFiducialId, this );

  this->laserRetroP = new ParamT<float>("laserRetro", -1, 0);
  this->laserRetroP->Callback( &Model::SetLaserRetro, this );

  Param::End();

  this->graphicsHandler = NULL;
  this->parentBodyNameP = NULL;
  this->myBodyNameP = NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Model::~Model()
{
  std::map< std::string, Body* >::iterator biter;
  std::map< std::string, Joint* >::iterator jiter;
  std::map< std::string, Controller* >::iterator citer;

  if (this->graphicsHandler)
  {
    delete this->graphicsHandler;
    this->graphicsHandler = NULL;
  }

  for (biter=this->bodies.begin(); biter != this->bodies.end(); biter++)
  {
    if (biter->second)
    {
      delete biter->second;
      biter->second = NULL;
    }
  }
  this->bodies.clear();

  for (jiter = this->joints.begin(); jiter != this->joints.end(); jiter++)
  {
    if (jiter->second)
    {
      delete jiter->second;
      jiter->second=NULL;
    }
  }
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

  if (this->parentBodyNameP)
  {
    delete this->parentBodyNameP;
    this->parentBodyNameP = NULL;
  }

  if (this->myBodyNameP)
  {
    delete this->myBodyNameP;
    this->myBodyNameP = NULL;
  }
}

////////////////////////////////////////////////////////////////////////////////
// Load the model
void Model::Load(XMLConfigNode *node, bool removeDuplicate)
{
  XMLConfigNode *childNode;
  Pose3d pose;
  Model* dup;

  this->nameP->Load(node);
  // Look for existing models by the same name
  if((dup = World::Instance()->GetModelByName(this->nameP->GetValue())) != NULL)
  {
    if(!removeDuplicate)
    {
      gzthrow("Duplicate model name" + this->nameP->GetValue() + "\n");
    }
    else
    {
      // Delete the existing one (this should only be reached when called
      // via the factory interface).
      printf("Queuing duplicate model %s (%p) for deletion\n", this->nameP->GetValue().c_str(), dup);
      World::Instance()->DeleteEntity(this->nameP->GetValue().c_str());
    }
  }

  this->staticP->Load(node);

  this->canonicalBodyNameP->Load(node);
  this->xyzP->Load(node);
  this->rpyP->Load(node);
  this->enableGravityP->Load(node);
  this->enableFrictionP->Load(node);
  this->collideP->Load(node);

  this->xmlNode = node;
  this->type=node->GetName();

  this->SetStatic( **(this->staticP) );

  if (this->type == "physical")
    this->LoadPhysical(node);
  else if (this->type == "renderable")
    this->LoadRenderable(node);
  else if (this->type != "empty")
  {
    gzthrow("Invalid model type[" + this->type + "]\n");
  }

  // Load controllers
  childNode = node->GetChildByNSPrefix("controller");
  while (childNode)
  {
    this->LoadController(childNode);
    childNode = childNode->GetNextByNSPrefix("controller");
  }

  // Create a default body if one does not exist in the XML file
  if (this->bodies.size() <= 0)
  {
    std::ostringstream bodyName;

    bodyName << this->GetName() << "_body";

    // Create an empty body for the model
    Body *body = this->CreateBody();
    body->SetName(bodyName.str());

    // Store the pointer to this body
    this->bodies[body->GetName()] = body;

    this->canonicalBodyNameP->SetValue( bodyName.str() );
  }


  if (this->canonicalBodyNameP->GetValue().empty())
  {
    this->canonicalBodyNameP->SetValue( this->bodies.begin()->first );
  }

  // Get the position and orientation of the model (relative to parent)
  pose.Reset();
  pose.pos = this->xyzP->GetValue();
  pose.rot = this->rpyP->GetValue();

  // Record the model's initial pose (for reseting)
  this->SetInitPose(pose);

  // This must be placed after creation of the bodies
  // Static variable overrides the gravity
  if (**this->staticP == false)
    this->SetGravityMode( **this->enableGravityP );

  this->SetFrictionMode( **this->enableFrictionP );

  this->SetCollideMode( **this->collideP );

  this->SetLaserFiducialId( **this->laserFiducialP);
  //this->SetLaserRetro( **this->laserRetroP);

  // Create the graphics iface handler
  this->graphicsHandler = new GraphicsIfaceHandler();
  this->graphicsHandler->Load(this->GetName(), this);


  // Get the name of the python module
  /*this->pName.reset(PyString_FromString(node->GetString("python","",0).c_str()));
  //this->pName.reset(PyString_FromString("pioneer2dx"));

  // Import the python module
  if (this->pName)
  {
  this->pModule.reset(PyImport_Import(this->pName));
  Py_DECREF(this->pName);
  }

  // Get the Update function from the module
  if (this->pModule)
  {
  this->pFuncUpdate.reset(PyObject_GetAttrString(this->pModule, "Update"));
  if (this->pFuncUpdate && !PyCallable_Check(this->pFuncUpdate))
  this->pFuncUpdate = NULL;
  }
   */
}

////////////////////////////////////////////////////////////////////////////////
// Save the model in XML format
void Model::Save(std::string &prefix, std::ostream &stream)
{
  std::string p = prefix + "  ";
  std::string typeName;
  std::map<std::string, Body* >::iterator bodyIter;
  std::map<std::string, Controller* >::iterator contIter;
  std::map<std::string, Joint* >::iterator jointIter;
  Vector3 pos = this->pose.pos;

  if (this->parent)
  {
    Model *pmodel = dynamic_cast<Model*>(this->parent);
    pos = this->pose.pos - pmodel->GetPose().pos;
  }

  this->xyzP->SetValue( pos );
  this->rpyP->SetValue( this->pose.rot );

  if (this->type=="renderable")
    typeName = "renderable";
  else if (this->type=="physical")
    typeName = "physical";

  stream << prefix << "<model:" << typeName;
  stream << " name=\"" << this->nameP->GetValue() << "\">\n"; 
  stream << prefix << "  " << *(this->xyzP) << "\n";
  stream << prefix << "  " << *(this->rpyP) << "\n";
  stream << prefix << "  " << *(this->enableGravityP) << "\n";
  stream << prefix << "  " << *(this->enableFrictionP) << "\n";
  stream << prefix << "  " << *(this->collideP) << "\n";

  if (this->type == "physical")
  {
    stream << prefix << "  " << *(this->staticP) << "\n";

    // Save all the bodies
    for (bodyIter=this->bodies.begin(); bodyIter!=this->bodies.end(); bodyIter++)
    {
      stream << "\n";
      if (bodyIter->second)
        bodyIter->second->Save(p, stream);
    }

    // Save all the joints
    for (jointIter = this->joints.begin(); jointIter != this->joints.end(); 
        jointIter++)
    {
      if (jointIter->second)
        jointIter->second->Save(p, stream);
    }

    // Save all the controllers
    for (contIter=this->controllers.begin();
        contIter!=this->controllers.end(); contIter++)
    {
      if (contIter->second)
        contIter->second->Save(p, stream);
    }
  }
  else
  {
    if (!this->lightName.empty())
    {
      OgreCreator::SaveLight(p, this->lightName, stream);
    }
  }

  if (this->parentBodyNameP && this->myBodyNameP)
  {
    stream << prefix << "  <attach>\n";
    stream << prefix << "    " << *(this->parentBodyNameP) << "\n";
    stream << prefix << "    " << *(this->myBodyNameP) << "\n";
    stream << prefix << "  </attach>\n";
  }

  // Save all child models
  std::vector< Entity* >::iterator eiter;
  for (eiter = this->children.begin(); eiter != this->children.end(); eiter++)
  {
    Model *cmodel = dynamic_cast<Model*>(*eiter);
    if (cmodel)
      cmodel->Save(p, stream);
  }

  stream << prefix << "</model:" << typeName << ">\n";
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the model
void Model::Init()
{
  std::map<std::string, Body* >::iterator biter;
  std::map<std::string, Controller* >::iterator contIter;

  this->graphicsHandler->Init();

  for (biter = this->bodies.begin(); biter!=this->bodies.end(); biter++)
    biter->second->Init();

  for (contIter=this->controllers.begin();
       contIter!=this->controllers.end(); contIter++)
  {
    contIter->second->Init();
  }

  /*this->mtext = new MovableText(this->GetName(), "this is the caption");
  this->mtext->setTextAlignment(MovableText::H_CENTER, MovableText::V_ABOVE);
  this->mtext->setAdditionalHeight(0.5);
  this->sceneNode->attachObject(this->mtext);
  */

  this->InitChild();
}

////////////////////////////////////////////////////////////////////////////////
// Update the model
void Model::Update()
{
  std::map<std::string, Body* >::iterator bodyIter;
  std::map<std::string, Controller* >::iterator contIter;
  std::map<std::string, Joint* >::iterator jointIter;

  Pose3d bodyPose, newPose, oldPose;

#ifdef TIMING
  double tmpT1 = Simulator::Instance()->GetWallTime();
#endif

  for (bodyIter=this->bodies.begin(); bodyIter!=this->bodies.end(); bodyIter++)
  {
    if (bodyIter->second)
    {
      bodyIter->second->Update();
    }
  }

#ifdef TIMING
  double tmpT2 = Simulator::Instance()->GetWallTime();
  std::cout << "      ALL Bodies DT (" << this->GetName() << ") (" << tmpT2-tmpT1 << ")" << std::endl;
#endif

  for (contIter=this->controllers.begin();
       contIter!=this->controllers.end(); contIter++)
  {

    if (contIter->second)
      contIter->second->Update();
  }

#ifdef TIMING
  double tmpT3 = Simulator::Instance()->GetWallTime();
  std::cout << "      ALL Controllers DT (" << this->GetName() << ") (" << tmpT3-tmpT2 << ")" << std::endl;
#endif

  for (jointIter = this->joints.begin(); jointIter != this->joints.end(); jointIter++)
  {
    jointIter->second->Update();
  }

#ifdef TIMING
  double tmpT4 = Simulator::Instance()->GetWallTime();
  std::cout << "      ALL Joints/canonical body (" << this->GetName() << ") DT (" << tmpT4-tmpT3 << ")" << std::endl;
#endif



  // Call the model's python update function, if one exists
  /*if (this->pFuncUpdate)
  {
    boost::python::call<void>(this->pFuncUpdate, this);
  }*/

  if (!this->canonicalBodyNameP->GetValue().empty())
  {
    this->pose = this->bodies[this->canonicalBodyNameP->GetValue()]->GetPose();
    this->xyzP->SetValue(this->pose.pos);
    this->rpyP->SetValue(this->pose.rot);
  }

  if (this->graphicsHandler)
  {
    this->graphicsHandler->Update();
  }

  
#ifdef TIMING
  this->UpdateChild();
  double tmpT5 = Simulator::Instance()->GetWallTime();
  std::cout << "      ALL child (" << this->GetName() << ") update DT (" 
            << tmpT5-tmpT4 << ")" << std::endl;
  std::cout << "      Models::Update() (" << this->GetName() 
            << ") Total DT (" << tmpT5-tmpT1 << ")" << std::endl;
#else
   this->UpdateChild();
#endif





  this->UpdateChild();
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the model
void Model::Fini()
{
  std::map<std::string, Body* >::iterator biter;
  std::map<std::string, Controller* >::iterator contIter;

  for (contIter = this->controllers.begin();
       contIter != this->controllers.end(); contIter++)
  {
    contIter->second->Fini();
  }

  for (biter=this->bodies.begin(); biter != this->bodies.end(); biter++)
  {
    biter->second->Fini();
  }

  if (this->graphicsHandler)
  {
    delete this->graphicsHandler;
    this->graphicsHandler = NULL;
  }

  this->FiniChild();
}

////////////////////////////////////////////////////////////////////////////////
// Reset the model
void Model::Reset()
{
  std::map<std::string, Joint* >::iterator jiter;
  std::map< std::string, Body* >::iterator biter;
  std::map<std::string, Controller* >::iterator citer;
  Vector3 v(0,0,0);

//  this->SetPose(this->initPose);

  for (citer=this->controllers.begin(); citer!=this->controllers.end(); citer++)
  {
    citer->second->Reset();
  }

  for (jiter=this->joints.begin(); jiter!=this->joints.end(); jiter++)
  {
    jiter->second->Reset();
  }

  /*for (biter=this->bodies.begin(); biter != this->bodies.end(); biter++)
  {
    biter->second->SetLinearVel(v);
    biter->second->SetAngularVel(v);
    biter->second->SetForce(v);
    biter->second->SetTorque(v);
  }*/
}



////////////////////////////////////////////////////////////////////////////////
// Get the name of the model
const std::string &Model::GetType() const
{
  return this->type;
}
////////////////////////////////////////////////////////////////////////////////
// Set the initial pose
void Model::SetInitPose(const Pose3d &pose)
{
  this->initPose = pose;
}

////////////////////////////////////////////////////////////////////////////////
// Get the initial pose
const Pose3d &Model::GetInitPose() const
{
  return this->initPose;
}

////////////////////////////////////////////////////////////////////////////////
// Set the current pose
void Model::SetPose(const Pose3d &setPose)
{
  Body *body;
  std::map<std::string, Body* >::iterator iter;

  Pose3d newPose, origPose;

  origPose = this->pose;
  this->pose = setPose;

  for (iter=this->bodies.begin(); iter!=this->bodies.end(); iter++)
  {
    if (!iter->second)
      continue;

    body = iter->second;

    // Compute the pose relative to the model
    newPose = body->GetPose() - origPose;

    // Compute the new pose
    newPose += this->pose;

    body->SetPose(newPose);
  }

  // Update the child models as well
  std::vector<Entity*>::iterator citer;
  for (citer = this->children.begin(); citer != this->children.end(); citer++)
  {
    Model *childModel = dynamic_cast<Model*>(*citer);
    if (childModel)
    {
      newPose = childModel->GetPose() - origPose;
      newPose += this->pose;

      childModel->SetPose(newPose);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the position of the model
void Model::SetPosition( const Vector3 &pos)
{
  Pose3d pose = this->GetPose();
  pose.pos = pos;

  this->SetPose(pose);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the rotation of the model
void Model::SetRotation( const Quatern &rot)
{
  Pose3d pose = this->GetPose();
  pose.rot = rot;

  this->SetPose(pose);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the linear velocity of the model
void Model::SetLinearVel( const Vector3 &vel )
{
  Body *body;
  std::map<std::string, Body* >::iterator iter;

  for (iter=this->bodies.begin(); iter!=this->bodies.end(); iter++)
  {
    body = iter->second;

    body->SetLinearVel( vel );
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the angular velocity of the model
void Model::SetAngularVel( const Vector3 &vel )
{
  Body *body;
  std::map<std::string, Body* >::iterator iter;

  for (iter=this->bodies.begin(); iter!=this->bodies.end(); iter++)
  {
    body = iter->second;

    body->SetAngularVel( vel );
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the linear acceleration of the model
void Model::SetLinearAccel( const Vector3 &accel )
{
  Body *body;
  std::map<std::string, Body* >::iterator iter;

  for (iter=this->bodies.begin(); iter!=this->bodies.end(); iter++)
  {
    body = iter->second;

    body->SetLinearAccel( accel );
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the angular acceleration of the model
void Model::SetAngularAccel( const Vector3 &accel )
{
  Body *body;
  std::map<std::string, Body* >::iterator iter;

  for (iter=this->bodies.begin(); iter!=this->bodies.end(); iter++)
  {
    body = iter->second;

    body->SetAngularAccel( accel );
  }
}
////////////////////////////////////////////////////////////////////////////////
// Get the current pose
Pose3d Model::GetPose() const
{
  return this->pose;
}

////////////////////////////////////////////////////////////////////////////////
// Create and return a new body
Body *Model::CreateBody()
{
  // Create a new body
  return World::Instance()->GetPhysicsEngine()->CreateBody(this);
}

////////////////////////////////////////////////////////////////////////////////
// Create and return a new joint
Joint *Model::CreateJoint(Joint::Type type)
{
  return World::Instance()->GetPhysicsEngine()->CreateJoint(type);
}

////////////////////////////////////////////////////////////////////////////////
Joint *Model::GetJoint(std::string name)
{
  std::map<std::string, Joint* >::const_iterator iter;
  iter = this->joints.find(name);

  if (iter != this->joints.end())
    return iter->second;
  else
    return NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Load a new body helper function
void Model::LoadBody(XMLConfigNode *node)
{
  if (!node)
    gzthrow("Trying to load a body with NULL XML information");

  // Create a new body
  Body *body = this->CreateBody();

  // Load the body using the config node. This also loads all of the
  // bodies geometries
  body->Load(node);

  // Store this body
  if (this->bodies[body->GetName()])
    gzmsg(0) << "Body with name[" << body->GetName() << "] already exists!!\n";

  // Store the pointer to this body
  this->bodies[body->GetName()] = body;

}

////////////////////////////////////////////////////////////////////////////////
// Load a new joint helper function
void Model::LoadJoint(XMLConfigNode *node)
{
  if (!node)
    gzthrow("Trying to load a joint with NULL XML information");

  Joint *joint;

  // Create a Hinge Joint
  if (node->GetName() == "hinge")
    joint = this->CreateJoint(Joint::HINGE);
  else if (node->GetName() == "ball")
    joint = this->CreateJoint(Joint::BALL);
  else if (node->GetName() == "slider")
    joint = this->CreateJoint(Joint::SLIDER);
  else if (node->GetName() == "hinge2")
    joint = this->CreateJoint(Joint::HINGE2);
  else if (node->GetName() == "universal")
    joint = this->CreateJoint(Joint::UNIVERSAL);
  else
  {
    gzthrow("Uknown joint[" + node->GetName() + "]\n");
  }

  joint->SetModel(this);

  // Load each joint
  joint->Load(node);

  if (this->joints.find(joint->GetName()) != this->joints.end())
    gzthrow( "can't have two joint with the same name");

  this->joints[joint->GetName()] = joint;
}

////////////////////////////////////////////////////////////////////////////////
/// Load a controller helper function
void Model::LoadController(XMLConfigNode *node)
{
  if (!node)
    gzthrow( "node parameter is NULL" );

  Controller *controller=0;
  std::ostringstream stream;

  // Get the controller's type
  std::string controllerType = node->GetName();

  // Get the unique name of the controller
  std::string controllerName = node->GetString("name",std::string(),1);
  
  // See if the controller is in a plugin
  std::string pluginName = node->GetString("plugin","",0);
  if (pluginName != "")
	  ControllerFactory::LoadPlugin(pluginName, controllerType);

  // Create the controller based on it's type
  controller = ControllerFactory::NewController(controllerType, this);

  if (controller)
  {
    // Load the controller
    try
    {
      controller->Load(node);
    }
    catch (GazeboError e)
    {
      std::cerr << "Error Loading Controller[" <<  controllerName
      << "]\n" << e << std::endl;
      delete controller;
      controller = NULL;
      return;
    }

    // Store the controller
    this->controllers[controllerName] = controller;
  }
  else
  {
    gzmsg(0) << "Unknown controller[" << controllerType << "]\n";
  }
}

////////////////////////////////////////////////////////////////////////////////
// Return the default body
Body *Model::GetBody()
{
  return this->bodies.begin()->second;
}

////////////////////////////////////////////////////////////////////////////////
/// Get a map of all the bodies
const std::map<std::string, Body*> *Model::GetBodies() const
{
  return &(this->bodies);
}

////////////////////////////////////////////////////////////////////////////////
/// Get a sensor by name
Sensor *Model::GetSensor(const std::string &name) const
{
  Sensor *sensor = NULL;
  std::map< std::string, Body* >::const_iterator biter;

  for (biter=this->bodies.begin(); biter != this->bodies.end(); biter++)
  {
    if ( (sensor = biter->second->GetSensor(name)) != NULL)
      break;
  }

  return sensor;
}
 
////////////////////////////////////////////////////////////////////////////////
/// Get a geom by name
Geom *Model::GetGeom(const std::string &name) const
{
  Geom *geom = NULL;
  std::map< std::string, Body* >::const_iterator biter;

  for (biter=this->bodies.begin(); biter != this->bodies.end(); biter++)
  {
    if ( (geom = biter->second->GetGeom(name)) != NULL)
      break;
  }

  return geom;
}

////////////////////////////////////////////////////////////////////////////////
/// Get a body by name
Body *Model::GetBody(const std::string &name)
{
  if (this->bodies.find(name) != this->bodies.end())
  {
    return this->bodies[name];
  }
  else if (name == "canonical")
  {
    return this->GetCanonicalBody();
  }
  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
// Attach this model to its parent
void Model::Attach(XMLConfigNode *node)
{
  Model *parentModel = NULL;

  Param::Begin(&this->parameters);
  this->parentBodyNameP = new ParamT<std::string>("parentBody","canonical",1);
  this->myBodyNameP = new ParamT<std::string>("myBody",this->canonicalBodyNameP->GetValue(),1);
  Param::End();

  if (node)
  {
    this->parentBodyNameP->Load(node);
    this->myBodyNameP->Load(node);
  }

  parentModel = dynamic_cast<Model*>(this->parent);

  if (parentModel == NULL)
    gzthrow("Parent cannot be NULL when attaching two models");

  this->joint = (HingeJoint*)this->CreateJoint(Joint::HINGE);

  Body *myBody = this->GetBody(**(this->myBodyNameP));
  Body *pBody = parentModel->GetBody(**(this->parentBodyNameP));

  if (myBody == NULL)
    gzthrow("No canonical body set.");

  if (pBody == NULL)
    gzthrow("Parent has no canonical body");

  this->joint->Attach(myBody, pBody);
  this->joint->SetAnchor( myBody->GetPosition() );
  this->joint->SetAxis( Vector3(0,1,0) );
  this->joint->SetParam( dParamHiStop, 0);
  this->joint->SetParam( dParamLoStop, 0);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the canonical body. Used for connected Model heirarchies
Body *Model::GetCanonicalBody()
{
  return this->bodies[this->canonicalBodyNameP->GetValue()];
}

////////////////////////////////////////////////////////////////////////////////
/// Set the gravity mode of the model
void Model::SetGravityMode( const bool &v )
{
  Body *body;
  std::map<std::string, Body* >::iterator iter;

  for (iter=this->bodies.begin(); iter!=this->bodies.end(); iter++)
  {
    body = iter->second;

    body->SetGravityMode( v );
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the gravity mode of the model
void Model::SetFrictionMode( const bool &v )
{
  Body *body;

  std::map<std::string, Body* >::iterator iter;

  for (iter=this->bodies.begin(); iter!=this->bodies.end(); iter++)
  {
    body = iter->second;

    body->SetFrictionMode( v );
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the collide mode of the model
void Model::SetCollideMode( const std::string &m )
{
  Body *body;

  std::map<std::string, Body* >::iterator iter;

  for (iter=this->bodies.begin(); iter!=this->bodies.end(); iter++)
  {
    body = iter->second;

    body->SetCollideMode( m );
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the collide mode of the model
void Model::SetLaserFiducialId( const int &id )
{
  Body *body;

  std::map<std::string, Body* >::iterator iter;

  for (iter=this->bodies.begin(); iter!=this->bodies.end(); iter++)
  {
    body = iter->second;

    body->SetLaserFiducialId( id );
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the collide mode of the model
void Model::SetLaserRetro( const float &retro )
{
  Body *body;

  std::map<std::string, Body* >::iterator iter;

  for (iter=this->bodies.begin(); iter!=this->bodies.end(); iter++)
  {
    body = iter->second;

    body->SetLaserRetro( retro );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Load a renderable model (like a light source).
void Model::LoadRenderable(XMLConfigNode *node)
{
  XMLConfigNode *childNode = NULL;

  // We still need a canonical body so that this model can be attached to
  // others
  Body *body = this->CreateBody();
  char lightNumBuf[8];
  sprintf(lightNumBuf, "%d", lightNumber++);
  body->SetName(this->GetName() + "_RenderableBody_" + lightNumBuf);
  //body->SetGravityMode(false);
  body->SetPose(Pose3d());
  this->bodies[body->GetName()] = body;

  if ((childNode = node->GetChild("light")))
  {
    this->lightName = OgreCreator::CreateLight(childNode, body->GetVisualNode());
  }

}

////////////////////////////////////////////////////////////////////////////////
// Load a physical model
void Model::LoadPhysical(XMLConfigNode *node)
{
  XMLConfigNode *childNode = NULL;

  // Load the bodies
  if (node->GetChildByNSPrefix("body"))
    childNode = node->GetChildByNSPrefix("body");
  else
    childNode = node->GetChild("body");

  while (childNode)
  {
    this->LoadBody(childNode);

    if (childNode->GetNextByNSPrefix("body"))
      childNode = childNode->GetNextByNSPrefix("body");
    else
      childNode = childNode->GetNext("body");
  }

  // Load the joints
  childNode = node->GetChildByNSPrefix("joint");

  while (childNode)
  {
    try
    {
      this->LoadJoint(childNode);
    }
    catch (GazeboError e)
    {
      std::cerr << "Error Loading Joint[" << childNode->GetString("name", std::string(), 0) << "]\n";
      std::cerr <<  e << std::endl;
      childNode = childNode->GetNextByNSPrefix("joint");
      continue;
    }
    childNode = childNode->GetNextByNSPrefix("joint");
  }

}
