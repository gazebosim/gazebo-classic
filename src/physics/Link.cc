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
/* Desc: Link class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#include <sstream>
#include <float.h>

#include "msgs/msgs.h"

#include "common/Events.hh"
#include "math/Quaternion.hh"
#include "common/Console.hh"
#include "common/Global.hh"
#include "common/Exception.hh"

#include "sensors/SensorFactory.hh"
#include "sensors/Sensor.hh"

#include "physics/Model.hh"
#include "physics/World.hh"
#include "physics/PhysicsEngine.hh"
#include "physics/Geom.hh"
#include "physics/Link.hh"

#include "transport/Publisher.hh"

using namespace gazebo;
using namespace physics;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Link::Link(EntityPtr parent)
    : Entity(parent)
{
  this->AddType(Base::BODY);
  this->inertial.reset(new Inertial);
}


////////////////////////////////////////////////////////////////////////////////
// Destructor
Link::~Link()
{
  std::vector<Entity*>::iterator iter;

  for (unsigned int i=0; i < this->visuals.size(); i++)
  {
    msgs::Visual msg;
    msgs::Init(msg, this->visuals[i]);
    msg.set_action( msgs::Visual::DELETE );
    this->visPub->Publish(msg);
  }
  this->visuals.clear();

  if (this->cgVisuals.size() > 0)
  {
    for (unsigned int i=0; i < this->cgVisuals.size(); i++)
    {
      msgs::Visual msg;
      msgs::Init(msg, this->cgVisuals[i]);
      msg.set_action( msgs::Visual::DELETE );
      this->visPub->Publish(msg);
    }
    this->cgVisuals.clear();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Load the body
void Link::Load( sdf::ElementPtr &_sdf )
{
  Entity::Load(_sdf);

  if (!this->IsStatic())
  {
    if (this->sdf->HasElement("inertial"))
      this->inertial->Load(this->sdf->GetElement("inertial"));
    else
      gzerr << "Non-static body has no interial sdf element.\n";
  }

  // before loading child geometry, we have to figure out of selfCollide is true
  // and modify parent class Entity so this body has its own spaceId
  this->SetSelfCollide( this->sdf->GetValueBool("self_collide") );

  // TODO: this shouldn't be in the physics sim
  if (this->sdf->HasElement("visual"))
  {
    sdf::ElementPtr visualElem = this->sdf->GetElement("visual");
    while (visualElem)
    {
      std::ostringstream visname;
      visname << this->GetCompleteScopedName() << "::VISUAL_" << 
        this->visuals.size();

      msgs::Visual msg = msgs::VisualFromSDF(visualElem);
      msgs::Init(msg, visname.str());
      msg.set_parent_id( this->GetCompleteScopedName() );
      msg.set_is_static( this->IsStatic() );

      this->visPub->Publish(msg);
      this->visuals.push_back(msg.header().str_id());

      visualElem = this->sdf->GetNextElement("visual", visualElem); 
    }
  }
 
  // Load the geometries
  if (this->sdf->HasElement("collision"))
  {
    sdf::ElementPtr collisionElem = this->sdf->GetElement("collision");
    while (collisionElem)
    {
      // Create and Load a geom, which will belong to this body.
      this->LoadGeom(collisionElem);
      collisionElem = this->sdf->GetNextElement("collision", collisionElem); 
    }
  }

  if (this->sdf->HasElement("sensor"))
  {
    sdf::ElementPtr sensorElem = this->sdf->GetElement("sensor");
    while (sensorElem)
    {
      this->LoadSensor(sensorElem);
      sensorElem = this->sdf->GetNextElement("sensor", sensorElem); 
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the body
void Link::Init()
{
  Base_V::iterator iter;
  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    if ((*iter)->HasType(Base::GEOM))
    {
      GeomPtr g = boost::shared_static_cast<Geom>(*iter);
      g->Init();
    }
  }

  sensors::Sensor_V::iterator siter;
  for (siter = this->sensors.begin(); siter != this->sensors.end(); siter++)
  {
    (*siter)->Init();
  }

  // save transform from this Parent Model Frame to this Link Frame
  // this is only used in setting Model pose from canonicalLink
  // the true model pose given a canonical body is
  //   this body's pose - this body's offsetFromModelFrame
  this->initModelOffset = this->GetRelativePose().CoordPoseSolve(math::Pose());

  this->SetKinematic( this->sdf->GetValueBool("kinematic") );

  // If no geoms are attached, then don't let gravity affect the body.
  if (this->children.size()==0 || !this->sdf->GetValueBool("gravity"))
    this->SetGravityMode(false);

  // global-inertial damping is implemented in ode svn trunk
  if (this->inertial)
  {
    this->SetLinearDamping( this->inertial->GetLinearDamping() );
    this->SetAngularDamping( this->inertial->GetAngularDamping() );
  }

  this->linearAccel.Set(0,0,0);
  this->angularAccel.Set(0,0,0);

  /// Attach mesh for CG visualization
  /// Add a renderable visual for CG, make visible in Update()
  /// TODO: this shouldn't be in the physics sim
  /*if (this->mass.GetAsDouble() > 0.0)
  {
    std::ostringstream visname;
    visname << this->GetCompleteScopedName() + ":" + this->GetName() << "_CGVISUAL" ;

    msgs::Visual msg;
    msgs::Init(msg, visname.str());
    msg.set_parent_id( this->comEntity->GetCompleteScopedName() );
    msg.set_render_type( msgs::Visual::MESH_RESOURCE );
    msg.set_mesh( "unit_box" );
    msg.set_material( "Gazebo/RedGlow" );
    msg.set_cast_shadows( false );
    msg.set_attach_axes( true );
    msg.set_visible( false );
    msgs::Set(msg.mutable_scale(), math::Vector3(0.1, 0.1, 0.1));
    this->vis_pub->Publish(msg);
    this->cgVisuals.push_back( msg.header().str_id() );

    if (this->children.size() > 1)
    {
      msgs::Visual g_msg;
      msgs::Init(g_msg, visname.str() + "_connectors");

      g_msg.set_parent_id( this->comEntity->GetCompleteScopedName() );
      g_msg.set_render_type( msgs::Visual::LINE_LIST );
      g_msg.set_attach_axes( false );
      g_msg.set_material( "Gazebo/GreenGlow" );
      g_msg.set_visible( false );

      // Create a line to each geom
      for (Base_V::iterator giter = this->children.begin(); 
           giter != this->children.end(); giter++)
      {
        EntityPtr e = boost::shared_dynamic_cast<Entity>(*giter);

        msgs::Point *pt;
        pt = g_msg.add_points();
        pt->set_x(0);
        pt->set_y(0);
        pt->set_z(0);

        pt = g_msg.add_points();
        pt->set_x(e->GetRelativePose().pos.x);
        pt->set_y(e->GetRelativePose().pos.y);
        pt->set_z(e->GetRelativePose().pos.z);
      }
      this->vis_pub->Publish(msg);
      this->cgVisuals.push_back( g_msg.header().str_id() );
    }
  }*/

  this->enabled = true;

  // DO THIS LAST!
  sdf::ElementPtr originElem = this->sdf->GetOrCreateElement("origin");
  this->SetRelativePose( originElem->GetValuePose("pose") );
}

////////////////////////////////////////////////////////////////////////////////
// Finalize the body
void Link::Fini()
{
  Base_V::iterator giter;

  this->connections.clear();

  for (giter = this->children.begin(); giter != this->children.end(); giter++)
    (*giter)->Fini();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the collide mode of the body
void Link::SetCollideMode( const std::string &m )
{
  Base_V::iterator giter;

  unsigned int collideBits;

  if (m == "all")
    collideBits =  GZ_ALL_COLLIDE;
  else if (m == "none")
    collideBits =  GZ_NONE_COLLIDE;
  else if (m == "sensors")
    collideBits = GZ_SENSOR_COLLIDE;
  else if (m == "ghost")
    collideBits = GZ_GHOST_COLLIDE;
  else
  {
    gzerr << "Unknown collide mode[" << m << "]\n";
    return;
  }

  // TODO: Put this back in
  /*for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
  {
    (*giter)->SetCategoryBits(collideBits);
    (*giter)->SetCollideBits(collideBits);
  }*/
}

////////////////////////////////////////////////////////////////////////////////
// Return Self-Collision Setting
bool Link::GetSelfCollide()
{
  return this->sdf->GetValueBool("self_collide");
}

////////////////////////////////////////////////////////////////////////////////
/// Set the laser retro reflectiveness of this body
void Link::SetLaserRetro(float retro)
{
  Base_V::iterator iter;

  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    if ((*iter)->HasType(Base::GEOM))
      boost::shared_static_cast<Geom>(*iter)->SetLaserRetro( retro );
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the body
void Link::Update()
{
  // Apply our linear accel
  //this->SetForce(this->linearAccel);

  // Apply our angular accel
  //this->SetTorque(this->angularAccel);

  // FIXME: FIXME: @todo: @todo: race condition on factory-based model loading!!!!!
   /*if (this->GetEnabled() != this->enabled)
   {
     this->enabled = this->GetEnabled();
     this->enabledSignal(this->enabled);
   }*/
}

////////////////////////////////////////////////////////////////////////////////
// Load a new sensor helper function
void Link::LoadSensor( sdf::ElementPtr &_sdf )
{
  std::string type = _sdf->GetValueString("type");
  sensors::SensorPtr sensor = sensors::SensorFactory::NewSensor(type);
  if (!sensor)
  {
    gzerr << "Unable to create sensor of type[" << type << "]\n";
    return;
  }

  sensor->Load(_sdf);
  this->sensors.push_back(sensor);
  
}

////////////////////////////////////////////////////////////////////////////////
// Load a new geom helper function
void Link::LoadGeom( sdf::ElementPtr &_sdf )
{
  GeomPtr geom;
  std::string type = _sdf->GetElement("geometry")->GetFirstElement()->GetName();

  /*if (type == "heightmap" || type == "map")
    this->SetStatic(true);
    */

  geom = this->GetWorld()->GetPhysicsEngine()->CreateGeom( type, 
      boost::shared_static_cast<Link>(shared_from_this()) );

  if (!geom)
    gzthrow("Unknown Geometry Type["+type +"]");

  geom->Load(_sdf);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the linear acceleration of the body
void Link::SetLinearAccel(const math::Vector3 &accel)
{
  //this->SetEnabled(true); Disabled this line to make autoDisable work
  this->linearAccel = accel;// * this->GetMass();
}



////////////////////////////////////////////////////////////////////////////////
/// Set the angular acceleration of the body
void Link::SetAngularAccel(const math::Vector3 &accel)
{
  //this->SetEnabled(true); Disabled this line to make autoDisable work
  this->angularAccel = accel * this->inertial->GetMass();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear velocity of the body
math::Vector3 Link::GetRelativeLinearVel() const
{
  return this->GetWorldPose().rot.RotateVectorReverse(this->GetWorldLinearVel());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular velocity of the body
math::Vector3 Link::GetRelativeAngularVel() const
{
  return this->GetWorldPose().rot.RotateVectorReverse(this->GetWorldAngularVel());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the linear acceleration of the body
math::Vector3 Link::GetRelativeLinearAccel() const
{
  return this->GetRelativeForce() / this->inertial->GetMass();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the body
math::Vector3 Link::GetWorldLinearAccel() const
{
  return this->GetWorldForce() / this->inertial->GetMass();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the body
math::Vector3 Link::GetRelativeAngularAccel() const
{
  return this->GetRelativeTorque() / this->inertial->GetMass();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the angular acceleration of the body
math::Vector3 Link::GetWorldAngularAccel() const
{
  return this->GetWorldTorque() / this->inertial->GetMass();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the force applied to the body
math::Vector3 Link::GetRelativeForce() const
{
  return this->GetWorldPose().rot.RotateVectorReverse(this->GetWorldForce());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the torque applied to the body
math::Vector3 Link::GetRelativeTorque() const
{
  return this->GetWorldPose().rot.RotateVectorReverse(this->GetWorldTorque());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the model that this body belongs to
ModelPtr Link::GetModel() const
{
  return boost::shared_dynamic_cast<Model>(this->GetParent());
}

////////////////////////////////////////////////////////////////////////////////
/// Get the size of the body
math::Box Link::GetBoundingBox() const
{
  math::Box box;
  Base_V::const_iterator iter;

  box.min.Set(FLT_MAX, FLT_MAX, FLT_MAX);
  box.max.Set(0,0,0);

  for (iter = this->children.begin(); iter != this->children.end(); iter++)
  {
    if ((*iter)->HasType(Base::GEOM))
      box += boost::shared_static_cast<Geom>(*iter)->GetBoundingBox();
  }

  return box;
}

////////////////////////////////////////////////////////////////////////////////
/// Set whether this entity has been selected by the user through the gui
bool Link::SetSelected( bool s )
{
  Entity::SetSelected(s);

  if (s == false)
    this->SetEnabled(true);

  return true;
}

////////////////////////////////////////////////////////////////////////////////
// Set Mass
void Link::SetInertial(const InertialPtr &/*_inertial*/)
{
  gzwarn << "Link::SetMass is empty\n";
}
