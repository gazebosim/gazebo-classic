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
/* Desc: Geom class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 * SVN: $Id$
 */

#include <sstream>

#include "OgreVisual.hh"
#include "Global.hh"
#include "GazeboMessage.hh"
#include "ContactParams.hh"
#include "World.hh"
#include "Body.hh"
#include "Geom.hh"

using namespace gazebo;

int Geom::geomIdCounter = 0;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Geom::Geom( Body *body)//, const std::string &name)
    : Entity(body)
{
  //this->SetName(name);
  this->body = body;
  this->spaceId = this->body->spaceId;

  // Create the contact parameters
  this->contact = new ContactParams();
  this->geomId = NULL;
  this->transId = NULL;

  this->laserFiducialId = -1;
  this->laserRetro = 0.0;

  this->bbVisual = NULL;

  // Zero out the mass
  dMassSetZero(&this->mass);
  dMassSetZero(&this->bodyMass);

  this->transparency = 0;

}

////////////////////////////////////////////////////////////////////////////////
// Destructor
Geom::~Geom()
{
  std::vector<OgreVisual*>::iterator iter;

  if (this->geomId)
    dGeomDestroy(this->geomId);

  if (this->transId)
    dGeomDestroy(this->transId);

  for (iter = this->visuals.begin(); iter != this->visuals.end(); iter++)
  {
    GZ_DELETE (*iter)
  }
  this->visuals.clear();
}

////////////////////////////////////////////////////////////////////////////////
/// Load the geom
void Geom::Load(XMLConfigNode *node)
{
  XMLConfigNode *childNode = NULL;

  this->xmlNode=node;

  this->SetName(node->GetString("name","",1));

  // The mesh used for visualization
  this->dblMass = node->GetDouble("mass",0.001,0);

  if (this->dblMass <= 0)
  {
    this->dblMass = 0.001;
  }

  this->contact->Load(node);

  this->LoadChild(node);

  this->body->AttachGeom(this);

  Pose3d pose;

  pose.pos = node->GetVector3("xyz",Vector3(0,0,0));
  pose.rot = node->GetRotation("rpy",Quatern());

  // TODO: This should probably be true....but "true" breaks trimesh postions.
  this->SetPose(pose, true);

  this->SetLaserFiducialId(node->GetInt("laserFiducialId",-1,0));
  this->SetLaserRetro(node->GetDouble("laserRetro",0.0,0));

  childNode = node->GetChild("visual");
  while (childNode)
  {
    OgreVisual *visual = new OgreVisual(this->visualNode, this);
    visual->Load(childNode);
    this->visuals.push_back(visual);
    childNode = childNode->GetNext("visual");
  }

  /*if (this->IsStatic())
  {
    this->visualNode->MakeStatic();
  }*/

  // Create the bounding box
  if (this->geomId && dGeomGetClass(this->geomId) != dPlaneClass)
  {
    dReal aabb[6];
    dGeomGetAABB(this->geomId, aabb);

    Vector3 min(aabb[0], aabb[2], aabb[4]);
    Vector3 max(aabb[1], aabb[3], aabb[5]);

    this->bbVisual = new OgreVisual(this->visualNode);
    this->bbVisual->AttachBoundingBox(min,max);
  }

  if (this->geomId && dGeomGetClass(this->geomId) != dPlaneClass && 
      dGeomGetClass(this->geomId) != dHeightfieldClass)
  {
    World::Instance()->RegisterGeom(this);
    this->ShowPhysics(false);
  }

}



////////////////////////////////////////////////////////////////////////////////
// Save the body based on our XMLConfig node
void Geom::Save()
{
  /*std::vector<OgreVisual*>::iterator iter;
  Pose3d pose= this->GetPose();

  this->xmlNode->SetValue("name", this->GetName());
  this->xmlNode->SetValue("xyz", pose.pos);
  this->xmlNode->SetValue("rpy", pose.rot);

  this->xmlNode->SetValue("mass", this->dblMass);
  this->xmlNode->SetValue("laserFiducialId",GetLaserFiducialId());
  this->xmlNode->SetValue("laserRetro",GetLaserRetro());

  for (iter = this->visuals.begin(); iter != this->visuals.end(); iter++)
  {
    (*iter)->Save();
  }
  */
}

////////////////////////////////////////////////////////////////////////////////
// Set the encapsulated geometry object
void Geom::SetGeom(dGeomID geomId, bool placeable)
{
  this->placeable = placeable;

  this->geomId = geomId;
  this->transId = NULL;

  if (this->placeable && !this->IsStatic())
  {
    if (dGeomGetClass(geomId) != dTriMeshClass)
    {
      this->transId = dCreateGeomTransform( this->spaceId );
      dGeomTransformSetGeom( this->transId, this->geomId );
      dGeomTransformSetInfo( this->transId, 1 );
      assert(dGeomGetSpace(this->geomId) == 0);
    }
  }
  else if ( dGeomGetSpace(this->geomId) == 0 )
  {
    dSpaceAdd(this->spaceId, this->geomId);
    assert(dGeomGetSpace(this->geomId) != 0);
  }

  dGeomSetData(this->geomId, this);

  if (this->IsStatic())
  {
    this->SetCategoryBits(GZ_FIXED_COLLIDE);
    this->SetCollideBits(~GZ_FIXED_COLLIDE);
  }

  // Create a new name of the geom's mesh entity
  //std::ostringstream stream;
  //stream << "Entity[" << (int)this->geomId << "]";
  //this->SetName(stream.str());
}

////////////////////////////////////////////////////////////////////////////////
// Update
void Geom::Update()
{
  this->UpdateChild();
}

////////////////////////////////////////////////////////////////////////////////
// Return the geom id
dGeomID Geom::GetGeomId() const
{
  return this->geomId;
}

////////////////////////////////////////////////////////////////////////////////
// Return the transform id
dGeomID Geom::GetTransId() const
{
  return this->transId;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the ODE geom class
int Geom::GetGeomClass() const
{
  if (this->geomId)
    return dGeomGetClass(this->geomId);
  else
    return 0;
}

////////////////////////////////////////////////////////////////////////////////
// Return whether this is a placeable geom.
bool Geom::IsPlaceable() const
{
  return this->placeable;
}

////////////////////////////////////////////////////////////////////////////////
// Set the pose relative to the body
void Geom::SetPose(const Pose3d &pose, bool updateCoM)
{
  if (this->placeable && this->geomId)
  {
    Pose3d localPose;
    dQuaternion q;

    // Transform into CoM relative Pose
    localPose = pose - this->body->GetCoMPose();


    q[0] = localPose.rot.u;
    q[1] = localPose.rot.x;
    q[2] = localPose.rot.y;
    q[3] = localPose.rot.z;


    // Set the pose of the encapsulated geom; this is always relative
    // to the CoM
    dGeomSetPosition(this->geomId, localPose.pos.x, localPose.pos.y, localPose.pos.z);
    dGeomSetQuaternion(this->geomId, q);

    this->visualNode->SetPose(localPose);

    if (updateCoM)
    {
      this->body->UpdateCoM();
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
// Return the pose of the geom relative to the body
Pose3d Geom::GetPose() const
{
  Pose3d pose;

  if (this->placeable && this->geomId)
  {
    const dReal *p;
    dQuaternion r;

    // Get the pose of the encapsulated geom; this is always relative to
    // the CoM
    p = dGeomGetPosition(this->geomId);
    dGeomGetQuaternion(this->geomId, r);


    pose.pos.x = p[0];
    pose.pos.y = p[1];
    pose.pos.z = p[2];

    pose.rot.u = r[0];
    pose.rot.x = r[1];
    pose.rot.y = r[2];
    pose.rot.z = r[3];

    // Transform into body relative pose
    pose += this->body->GetCoMPose();
  }
  else
    pose = this->body->GetPose();

  return pose;
}

////////////////////////////////////////////////////////////////////////////////
// Set the position
void Geom::SetPosition(const Vector3 &pos)
{
  Pose3d pose;

  pose = this->GetPose();
  pose.pos = pos;
  this->SetPose(pose);
}

////////////////////////////////////////////////////////////////////////////////
// Set the rotation
void Geom::SetRotation(const Quatern &rot)
{
  Pose3d pose;

  pose = this->GetPose();
  pose.rot = rot;
  this->SetPose(pose);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the category bits, used during collision detection
void Geom::SetCategoryBits(unsigned int bits)
{
  dGeomSetCategoryBits(this->geomId, bits);
  dGeomSetCategoryBits((dGeomID)this->spaceId, bits);
}

////////////////////////////////////////////////////////////////////////////////
/// Set the collide bits, used during collision detection
void Geom::SetCollideBits(unsigned int bits)
{
  dGeomSetCollideBits(this->geomId, bits);
  dGeomSetCollideBits((dGeomID)this->spaceId, bits);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the mass of the geom
const dMass *Geom::GetBodyMassMatrix()
{
  Pose3d pose;
  dQuaternion q;
  dMatrix3 r;
  dMass bodyMass;

  if (!this->placeable)
    return NULL;

  pose = this->GetPose();

  q[0] = pose.rot.u;
  q[1] = pose.rot.x;
  q[2] = pose.rot.y;
  q[3] = pose.rot.z;

  dQtoR(q,r);


  this->bodyMass = this->mass;

  if (dMassCheck(&this->bodyMass))
  {
    dMassRotate(&this->bodyMass, r);
    dMassTranslate( &this->bodyMass, pose.pos.x, pose.pos.y, pose.pos.z);
  }

  return &this->bodyMass;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the laser fiducial integer id
void Geom::SetLaserFiducialId(int id)
{
  this->laserFiducialId = id;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the laser fiducial integer id
int Geom::GetLaserFiducialId() const
{
  return this->laserFiducialId;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the laser retro reflectiveness
void Geom::SetLaserRetro(float retro)
{
  this->laserRetro = retro;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the laser retro reflectiveness
float Geom::GetLaserRetro() const
{
  return this->laserRetro;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the visibility of the Bounding box of this geometry
void Geom::ShowBoundingBox(bool show)
{
  if (this->bbVisual)
    this->bbVisual->SetVisible(show);
}

// FIXME: ShowJoints and ShowPhysics will mess with each other and with the user's defined transparency visibility
////////////////////////////////////////////////////////////////////////////////
/// Set the visibility of the joints of this geometry
void Geom::ShowJoints(bool show)
{
  std::vector<OgreVisual*>::iterator iter;

  if (show)
  {
    for (iter = this->visuals.begin(); iter != this->visuals.end(); iter++)
    {
      (*iter)->SetTransparency(0.6);
    }
  }
  else
  {
    for (iter = this->visuals.begin(); iter != this->visuals.end(); iter++)
    {
      (*iter)->SetTransparency(0.0);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the visibility of the physical entity of this geom
void Geom::ShowPhysics(bool show)
{
  std::vector<OgreVisual*>::iterator iter;

  if (show)
  {
    for (iter = this->visuals.begin(); iter != this->visuals.end(); iter++)
    {
      (*iter)->SetVisible(false, false);
    }
    /*this->visualNode->SetVisible(true, false);
    this->visualNode->SetTransparency(0.6);
    */
  }
  else
  {
    for (iter = this->visuals.begin(); iter != this->visuals.end(); iter++)
    {
      (*iter)->SetVisible(true, false);
    }
    /*this->visualNode->SetVisible(false, false);
    this->visualNode->SetTransparency(1.0);
    */
  }
}
