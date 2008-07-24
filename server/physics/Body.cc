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
/* Desc: Body class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id$
 */

#include <sstream>

#include "GazeboMessage.hh"
#include "HeightmapGeom.hh"
#include "MapGeom.hh"
#include "OgreVisual.hh"
#include "Global.hh"
#include "Vector2.hh"
#include "Quatern.hh"
#include "GazeboError.hh"
#include "SensorFactory.hh"
#include "Sensor.hh"
#include "SphereGeom.hh"
#include "TrimeshGeom.hh"
#include "BoxGeom.hh"
#include "CylinderGeom.hh"
#include "PlaneGeom.hh"
#include "Geom.hh"
#include "Body.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Body::Body(Entity *parent, dWorldID worldId)
    : Entity(parent)
{

  if ( !this->IsStatic() )
  {
    this->bodyId = dBodyCreate(worldId);

    dMassSetZero( &this->mass );
  }
  else
  {
    this->bodyId = NULL;
  }
}


////////////////////////////////////////////////////////////////////////////////
// Destructor
Body::~Body()
{
  std::vector< Geom* >::iterator giter;
  std::vector< Sensor* >::iterator siter;

  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
  {
    GZ_DELETE (*giter);
  }
  this->geoms.clear();

  for (siter = this->sensors.begin(); siter != this->sensors.end(); siter++)
  {
    GZ_DELETE (*siter);
  }
  this->sensors.clear();
}

////////////////////////////////////////////////////////////////////////////////
// Load the body based on an XMLConfig node
void Body::Load(XMLConfigNode *node)
{
  XMLConfigNode *childNode;

  this->SetName(node->GetString("name",std::string(),1));
  this->SetPosition(node->GetVector3("xyz",Vector3(0,0,0)));
  this->SetRotation(node->GetRotation("rpy",Quatern(1,0,0,0)));
  this->xmlNode=node;

  childNode = node->GetChildByNSPrefix("geom");

  // Load the geometries
  while (childNode)
  {
    // Create and Load a geom, which will belong to this body.
    this->LoadGeom(childNode);
    childNode = childNode->GetNextByNSPrefix("geom");
  }

  childNode = node->GetChildByNSPrefix("sensor");

  // Load the sensors
  while (childNode)
  {
    // Create and Load a sensor, which will belong to this body.
    this->LoadSensor(childNode);
    childNode = childNode->GetNextByNSPrefix("sensor");
  }

  // If no geoms are attached, then don't let gravity affect the body.
  if (this->geoms.size()==0)
  {
    this->SetGravityMode(false);
  }

}

////////////////////////////////////////////////////////////////////////////////
// Save the body based on our XMLConfig node
void Body::Save()
{
  /*std::vector< Geom* >::iterator giter;
  std::vector< Sensor* >::iterator siter;

  this->xmlNode->SetValue("name", this->GetName());
  this->xmlNode->SetValue("xyz", this->GetPosition());
  this->xmlNode->SetValue("rpy", this->GetRotation());

  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
  {
    //(*giter)->Save();
  }

  for (siter = this->sensors.begin(); siter != this->sensors.end(); siter++)
  {
    //(*siter)->Save();
  }
  */
}


////////////////////////////////////////////////////////////////////////////////
// Finalize the body
void Body::Fini()
{
  std::vector< Sensor* >::iterator siter;

  for (siter = this->sensors.begin(); siter != this->sensors.end(); siter++)
  {
    (*siter)->Fini();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Set whether gravity affects this body
void Body::SetGravityMode(bool mode)
{
  if (this->bodyId)
    dBodySetGravityMode(this->bodyId, mode);
}

////////////////////////////////////////////////////////////////////////////////
// Initialize the body
void Body::Init()
{
  // Set the intial pose. Must do this to handle static models
  this->SetPose(this->GetPose());

  std::vector< Sensor* >::iterator siter;

  for (siter = this->sensors.begin(); siter != this->sensors.end(); siter++)
  {
    (*siter)->Init();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Update the body
void Body::Update()
{
  std::vector< Sensor* >::iterator sensorIter;
  std::vector< Geom* >::iterator geomIter;

  if (!this->IsStatic())
  {
    Pose3d pose = this->GetPose();

    // Set the pose of the scene node
    this->visualNode->SetPose(pose);
  }

  for (geomIter=this->geoms.begin();
       geomIter!=this->geoms.end(); geomIter++)
  {
    (*geomIter)->Update();
    if ((*geomIter)->GetName() == "sphere1_geom")
    {
      std::cout << "Geom pose[" << (*geomIter)->GetPose() << "]\n";
    }
  }

  for (sensorIter=this->sensors.begin();
       sensorIter!=this->sensors.end(); sensorIter++)
  {
    (*sensorIter)->Update();
  }
}

////////////////////////////////////////////////////////////////////////////////
// Attach a geom to this body
void Body::AttachGeom( Geom *geom )
{
  if ( this->bodyId )
  {
    if (geom->IsPlaceable())
    {
      if (geom->GetTransId())
        dGeomSetBody(geom->GetTransId(), this->bodyId);
      else if (geom->GetGeomId())
        dGeomSetBody(geom->GetGeomId(), this->bodyId);
    }
  }

  this->geoms.push_back(geom);
}

////////////////////////////////////////////////////////////////////////////////
// Set the pose of the body
void Body::SetPose(const Pose3d &pose)
{
  Pose3d localPose;

  if (this->IsStatic())
  {
    Pose3d oldPose = this->staticPose;
    Pose3d newPose;
    this->staticPose = pose;

    std::vector<Geom*>::iterator iter;

    //this->SetPosition(this->staticPose.pos);
    //this->SetRotation(this->staticPose.rot);
   
    if (this->GetName() == "map_body")
    {
      std::cout << "Set new pose[" << pose << "]\n";
      std::cout << "Old Pose[ " << oldPose << "]\n";
    }

    for (iter = this->geoms.begin(); iter != this->geoms.end(); iter++)
    {
      newPose = (*iter)->GetPose() - oldPose;
      newPose += this->staticPose;
      (*iter)->SetPose(newPose);
    }
  }
  else
  {
    // Compute pose of CoM
    localPose = this->comPose + pose;

    this->SetPosition(localPose.pos);
    this->SetRotation(localPose.rot);
  }
}

////////////////////////////////////////////////////////////////////////////////
// Return the pose of the body
Pose3d Body::GetPose() const
{
  Pose3d pose;

  if (this->IsStatic())
  {
    pose = this->staticPose;
  }
  else
  {
    pose.pos = this->GetPosition();
    pose.rot = this->GetRotation();

    pose = this->comPose.CoordPoseSolve(pose);
  }

  return pose;
}

////////////////////////////////////////////////////////////////////////////////
// Set the position of the body
void Body::SetPosition(const Vector3 &pos)
{
  if (this->bodyId)
    dBodySetPosition(this->bodyId, pos.x, pos.y, pos.z);

  // Set the position of the scene node
  this->visualNode->SetPosition(pos);
}

////////////////////////////////////////////////////////////////////////////////
// Set the rotation of the body
void Body::SetRotation(const Quatern &rot)
{

  if (this->bodyId)
  {
    dQuaternion q;
    q[0] = rot.u;
    q[1] = rot.x;
    q[2] = rot.y;
    q[3] = rot.z;

    // Set the rotation of the ODE body
    dBodySetQuaternion(this->bodyId, q);
  }

  // Set the orientation of the scene node
  this->visualNode->SetRotation(rot);
}

////////////////////////////////////////////////////////////////////////////////
// Return the position of the body. in global CS
Vector3 Body::GetPosition() const
{
  Vector3 pos;

  if (this->bodyId)
  {
    const dReal *p;

    p = dBodyGetPosition(this->bodyId);

    pos.x = p[0];
    pos.y = p[1];
    pos.z = p[2];
  }
  else
  {
    pos = this->staticPose.pos;
  }

  return pos;
}


////////////////////////////////////////////////////////////////////////////////
// Return the rotation
Quatern Body::GetRotation() const
{
  Quatern rot;

  if (this->bodyId)
  {
    const dReal *r;

    r = dBodyGetQuaternion(this->bodyId);

    rot.u = r[0];
    rot.x = r[1];
    rot.y = r[2];
    rot.z = r[3];
  }
  else
  {
    rot = this->staticPose.rot;
  }

  return rot;
}

////////////////////////////////////////////////////////////////////////////////
// Return the ID of this body
dBodyID Body::GetId() const
{
  return this->bodyId;
}


////////////////////////////////////////////////////////////////////////////////
// Set whether this body is enabled
void Body::SetEnabled(bool enable) const
{
  if (!this->bodyId)
    return;

  if (enable)
    dBodyEnable(this->bodyId);
  else
    dBodyDisable(this->bodyId);
}

////////////////////////////////////////////////////////////////////////////////
// Load a new geom helper function
void Body::LoadGeom(XMLConfigNode *node)
{
  Geom *geom;

  if (node->GetName() == "sphere")
    geom = new SphereGeom(this);
  else if (node->GetName() == "cylinder")
    geom = new CylinderGeom(this);
  else if (node->GetName() == "box")
    geom = new BoxGeom(this);
  else if (node->GetName() == "plane")
    geom = new PlaneGeom(this);
  else if (node->GetName() == "trimesh")
    geom = new TrimeshGeom(this);
  else if (node->GetName() == "heightmap")
  {
    this->SetStatic(true);
    geom = new HeightmapGeom(this);
  }
  else if (node->GetName() == "map")
  {
    //this->SetStatic(true);
    geom = new MapGeom(this);
  }
  else
  {
    gzthrow("Unknown Geometry Type["+node->GetString("name",std::string(),0)+"]");
  }

  geom->Load(node);

}

////////////////////////////////////////////////////////////////////////////////
// Load a sensor
void Body::LoadSensor(XMLConfigNode *node)
{
  Sensor *sensor = NULL;

  if (node==NULL)
  {
    gzthrow("Null node pointer. Invalid sensor in the world file.");
  }

  sensor = SensorFactory::NewSensor(node->GetName(), this);

  if (sensor)
  {
    sensor->Load(node);
    this->sensors.push_back(sensor);
  }
  else
  {
    std::ostringstream stream;
    stream << "Null sensor. Invalid sensor name[" << node->GetString("name",std::string(), 0) << "]";
    gzthrow(stream.str());
  }
}

/////////////////////////////////////////////////////////////////////
// Update the CoM and mass matrix
/*
  What's going on here?  In ODE the CoM of a body corresponds to the
  origin of the body-fixed coordinate system.  In Gazebo, however, we
  want to have arbitrary body coordinate systems (i.e., CoM may be
  displaced from the body-fixed cs).  To get around this limitation in
  ODE, we have an extra fudge-factor (comPose), describing the pose of
  the CoM relative to Gazebo's body-fixed cs.  When using low-level
  ODE functions, one must use apply this factor appropriately.

  The UpdateCoM() function is used to compute this offset, based on
  the mass distribution of attached geoms.  This function also shifts
  the ODE-pose of the geoms, to keep everything in the same place in the
  Gazebo cs.  Simple, neh?

  TODO: messes up if you call it twice; should fix.
*/
void Body::UpdateCoM()
{
  const dMass *lmass;
  Pose3d oldPose, newPose, pose;
  std::vector< Geom* >::iterator giter;

  if (!this->bodyId)
    return;

  // Construct the mass matrix by combining all the geoms
  dMassSetZero( &this->mass );

  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
  {
    lmass = (*giter)->GetBodyMassMatrix();
    if ((*giter)->IsPlaceable() && (*giter)->GetGeomId())
    {
      dMassAdd( &this->mass, lmass );
    }
  }

  // Old pose for the CoM
  oldPose = this->comPose;

  if (isnan(this->mass.c[0]))
    this->mass.c[0] = 0;

  if (isnan(this->mass.c[1]))
    this->mass.c[1] = 0;

  if (isnan(this->mass.c[2]))
    this->mass.c[2] = 0;

  // New pose for the CoM
  newPose.pos.x = this->mass.c[0];
  newPose.pos.y = this->mass.c[1];
  newPose.pos.z = this->mass.c[2];

  // Fixup the poses of the geoms (they are attached to the CoM)
  for (giter = this->geoms.begin(); giter != this->geoms.end(); giter++)
  {
    if ((*giter)->IsPlaceable())
    {
      this->comPose = oldPose;
      pose = (*giter)->GetPose();
      this->comPose = newPose;
      (*giter)->SetPose(pose, false);
    }
  }


  // Fixup the pose of the CoM (ODE body)
  this->comPose = oldPose;
  pose = this->GetPose();
  this->comPose = newPose;
  this->SetPose(pose);


  // Settle on the new CoM pose
  this->comPose = newPose;

  // My Cheap Hack, to put the center of mass at the origin
  this->mass.c[0] = this->mass.c[1] = this->mass.c[2] = 0;

  // Set the mass matrix
  if (this->mass.mass > 0)
    dBodySetMass( this->bodyId, &this->mass );
}

////////////////////////////////////////////////////////////////////////////////
/// Get the Center of Mass pose
const Pose3d &Body::GetCoMPose() const
{
  return this->comPose;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the velocity of the body
void Body::SetLinearVel(const Vector3 &vel)
{
  if (this->bodyId)
    dBodySetLinearVel(this->bodyId, vel.x, vel.y, vel.z);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the velocity of the body
Vector3 Body::GetLinearVel() const
{
  Vector3 vel;

  if (this->bodyId)
  {
    const dReal *dvel;

    dvel = dBodyGetLinearVel(this->bodyId);

    vel.x = dvel[0];
    vel.y = dvel[1];
    vel.z = dvel[2];
  }

  return vel;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the velocity of the body
void Body::SetAngularVel(const Vector3 &vel)
{
  if (this->bodyId)
    dBodySetAngularVel(this->bodyId, vel.x, vel.y, vel.z);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the velocity of the body
Vector3 Body::GetAngularVel() const
{
  Vector3 vel;

  if (this->bodyId)
  {
    const dReal *dvel;

    dvel = dBodyGetAngularVel(this->bodyId);

    vel.x = dvel[0];
    vel.y = dvel[1];
    vel.z = dvel[2];
  }

  return vel;
}
////////////////////////////////////////////////////////////////////////////////
/// \brief Set the force applied to the body
void Body::SetForce(const Vector3 &force)
{
  if (this->bodyId)
    dBodySetForce(this->bodyId, force.x, force.y, force.z);
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Get the force applied to the body
Vector3 Body::GetForce() const
{
  Vector3 force;

  if (this->bodyId)
  {
    const dReal *dforce;

    dforce = dBodyGetForce(this->bodyId);

    force.x = dforce[0];
    force.y = dforce[1];
    force.z = dforce[2];
  }

  return force;
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Set the torque applied to the body
void Body::SetTorque(const Vector3 &torque)
{
  if (this->bodyId)
    dBodySetTorque(this->bodyId, torque.x, torque.y, torque.z);
}

////////////////////////////////////////////////////////////////////////////////
/// \brief Get the torque applied to the body
Vector3 Body::GetTorque() const
{
  Vector3 torque;
  if (this->bodyId)
  {
    const dReal *dtorque;

    dtorque = dBodyGetTorque(this->bodyId);

    torque.x = dtorque[0];
    torque.y = dtorque[1];
    torque.z = dtorque[2];
  }

  return torque;
}
