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

#include "PhysicsEngine.hh"
#include "OgreVisual.hh"
#include "OgreCreator.hh"
#include "Global.hh"
#include "GazeboMessage.hh"
#include "ContactParams.hh"
#include "World.hh"
#include "Body.hh"
#include "Geom.hh"
#include "Simulator.hh"

using namespace gazebo;

int Geom::geomIdCounter = 0;

////////////////////////////////////////////////////////////////////////////////
// Constructor
Geom::Geom( Body *body)
    : Entity(body)
{
  this->physicsEngine = World::Instance()->GetPhysicsEngine();

  this->typeName = "unknown";

  this->body = body;
  this->SetSpaceId(this->body->GetSpaceId());

  // Create the contact parameters
  this->contact = new ContactParams();
  this->geomId = NULL;
  this->transId = NULL;

  this->bbVisual = NULL;

  // Zero out the mass
  dMassSetZero(&this->mass);
  dMassSetZero(&this->bodyMass);

  this->transparency = 0;

  Param::Begin(&this->parameters);
  this->massP = new ParamT<double>("mass",0.001,0);
  this->massP->Callback( &Geom::SetMass, this);

  this->xyzP = new ParamT<Vector3>("xyz", Vector3(), 0);
  this->xyzP->Callback( &Geom::SetPosition, this);

  this->rpyP = new ParamT<Quatern>("rpy", Quatern(), 0);
  this->rpyP->Callback( &Geom::SetRotation, this);

  this->laserFiducialIdP = new ParamT<int>("laserFiducialId",-1,0);
  this->laserRetroP = new ParamT<float>("laserRetro",-1,0);
  Param::End();

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

  delete this->massP;
  delete this->xyzP;
  delete this->rpyP;
  delete this->laserFiducialIdP;
  delete this->laserRetroP;
}

////////////////////////////////////////////////////////////////////////////////
/// Load the geom
void Geom::Load(XMLConfigNode *node)
{

  XMLConfigNode *childNode = NULL;

  this->xmlNode=node;

  this->typeName = node->GetName();

  this->nameP->Load(node);
  this->SetName(this->nameP->GetValue());
  this->nameP->Load(node);
  this->massP->Load(node);
  this->xyzP->Load(node);
  this->rpyP->Load(node);
  this->laserFiducialIdP->Load(node);
  this->laserRetroP->Load(node);

  if (this->massP->GetValue() <= 0)
  {
    this->massP->SetValue( 0.001 );
  }

  this->contact->Load(node);

  this->LoadChild(node);

  this->body->AttachGeom(this);

  Pose3d pose;

  pose.pos = this->xyzP->GetValue();
  pose.rot = this->rpyP->GetValue();

  // TODO: This should probably be true....but "true" breaks trimesh postions.
  this->SetPose(pose, false);

  childNode = node->GetChild("visual");
  while (childNode)
  {
    std::ostringstream visname;
    visname << this->GetScopedName() << "_VISUAL_" << this->visuals.size();

    OgreVisual *visual = OgreCreator::Instance()->CreateVisual(
        visname.str(), this->visualNode, this);

    if (visual)
    {
      visual->Load(childNode);
      visual->SetIgnorePoseUpdates(true);

      this->visuals.push_back(visual);
      visual->SetCastShadows(true);
    }
    childNode = childNode->GetNext("visual");
  }

  // Create the bounding box
  if (this->geomId && dGeomGetClass(this->geomId) != dPlaneClass)
  {
    Vector3 min;
    Vector3 max;

    this->GetBoundingBox(min,max);

    std::ostringstream visname;
    visname << this->GetScopedName() << "_BBVISUAL" ;

    this->bbVisual = OgreCreator::Instance()->CreateVisual(
        visname.str(), this->visualNode);

    if (this->bbVisual)
    {
      this->bbVisual->SetCastShadows(false);
      this->bbVisual->AttachBoundingBox(min,max);
      this->bbVisual->SetRotation(pose.rot.GetInverse()); //transform aabb from global frame back to local frame
    }
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
void Geom::Save(std::string &prefix, std::ostream &stream)
{
  if (this->GetGeomClass() == dRayClass)
    return;

  std::string p = prefix + "  ";
  std::vector<OgreVisual*>::iterator iter;

  this->xyzP->SetValue( this->GetPose().pos );
  this->rpyP->SetValue( this->GetPose().rot );

  stream << prefix << "<geom:" << this->typeName << " name=\"" 
         << this->nameP->GetValue() << "\">\n";

  stream << prefix << "  " << *(this->xyzP) << "\n";
  stream << prefix << "  " << *(this->rpyP) << "\n";

  this->SaveChild(p, stream);

  stream << prefix << "  " << *(this->massP) << "\n";

  stream << prefix << "  " << *(this->laserFiducialIdP) << "\n";
  stream << prefix << "  " << *(this->laserRetroP) << "\n";

  for (iter = this->visuals.begin(); iter != this->visuals.end(); iter++)
  {
    if (*iter)
      (*iter)->Save(p, stream);
  }

  stream << prefix << "</geom:" << this->typeName << ">\n";
}

////////////////////////////////////////////////////////////////////////////////
// Set the encapsulated geometry object
void Geom::SetGeom(dGeomID geomId, bool placeable)
{
  this->physicsEngine->LockMutex();

  this->placeable = placeable;

  this->geomId = geomId;
  this->transId = NULL;

  if (this->placeable && !this->IsStatic())
  {
    /// @todo: Not sure why this if statement was here
    /// trimesh loading works fine without it
    /// commenting out for now
    //if (dGeomGetClass(geomId) != dTriMeshClass)
    {
      this->transId = dCreateGeomTransform( this->spaceId );
      dGeomTransformSetGeom( this->transId, this->geomId );
      dGeomTransformSetInfo( this->transId, 1 );
      /// @todo: this assert seems to break when geom is a trimesh, why?
      if (dGeomGetClass(geomId) != dTriMeshClass)
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
  else
  {
    // collide with all
    this->SetCategoryBits(GZ_ALL_COLLIDE);
    this->SetCollideBits(GZ_ALL_COLLIDE);
  }

  this->physicsEngine->UnlockMutex();

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
  int result = 0;

  if (this->geomId)
  {
    this->physicsEngine->LockMutex();
    result= dGeomGetClass(this->geomId);
    this->physicsEngine->UnlockMutex();
  }

  return result;
}

////////////////////////////////////////////////////////////////////////////////
// Return whether this is a placeable geom.
bool Geom::IsPlaceable() const
{
  return this->placeable;
}

////////////////////////////////////////////////////////////////////////////////
// Set the pose relative to the body
void Geom::SetPose(const Pose3d &newPose, bool updateCoM)
{
  this->physicsEngine->LockMutex();

  if (this->placeable && this->geomId)
  {
    Pose3d localPose;
    dQuaternion q;

    // Transform into CoM relative Pose
    localPose = newPose - this->body->GetCoMPose();

    q[0] = localPose.rot.u;
    q[1] = localPose.rot.x;
    q[2] = localPose.rot.y;
    q[3] = localPose.rot.z;

    // Set the pose of the encapsulated geom; this is always relative
    // to the CoM
    dGeomSetPosition(this->geomId, localPose.pos.x, localPose.pos.y, localPose.pos.z);
    dGeomSetQuaternion(this->geomId, q);

    if (updateCoM)
    {
      this->body->UpdateCoM();
    }
  }

  this->physicsEngine->UnlockMutex();
}

////////////////////////////////////////////////////////////////////////////////
// Return the pose of the geom relative to the body
Pose3d Geom::GetPose() const
{
  this->physicsEngine->LockMutex();

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

  this->physicsEngine->UnlockMutex();

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
  this->physicsEngine->LockMutex();

  if (this->geomId)
    dGeomSetCategoryBits(this->geomId, bits);
  if (this->spaceId)
    dGeomSetCategoryBits((dGeomID)this->spaceId, bits);

  this->physicsEngine->UnlockMutex();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the collide bits, used during collision detection
void Geom::SetCollideBits(unsigned int bits)
{
  this->physicsEngine->LockMutex();

  if (this->geomId)
    dGeomSetCollideBits(this->geomId, bits);
  if (this->spaceId)
    dGeomSetCollideBits((dGeomID)this->spaceId, bits);

  this->physicsEngine->UnlockMutex();
}

////////////////////////////////////////////////////////////////////////////////
/// Get the mass of the geom
const dMass *Geom::GetBodyMassMatrix()
{

  Pose3d pose;
  dQuaternion q;
  dMatrix3 r;

  if (!this->placeable)
    return NULL;

  this->physicsEngine->LockMutex();
  pose = this->GetPose(); // get pose of the geometry

  q[0] = pose.rot.u;
  q[1] = pose.rot.x;
  q[2] = pose.rot.y;
  q[3] = pose.rot.z;

  dQtoR(q,r); // turn quaternion into rotation matrix

  // this->mass was init to zero at start,
  // read user specified mass into this->dblMass and dMassAdd in this->mass
  this->bodyMass = this->mass;


  if (dMassCheck(&this->bodyMass))
  {
    dMassRotate(&this->bodyMass, r);
    dMassTranslate( &this->bodyMass, pose.pos.x, pose.pos.y, pose.pos.z);
  }
  this->physicsEngine->UnlockMutex();

  return &this->bodyMass;
}

////////////////////////////////////////////////////////////////////////////////
/// Set the laser fiducial integer id
void Geom::SetLaserFiducialId(int id)
{
  this->laserFiducialIdP->SetValue( id );
}

////////////////////////////////////////////////////////////////////////////////
/// Get the laser fiducial integer id
int Geom::GetLaserFiducialId() const
{
  return this->laserFiducialIdP->GetValue();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the laser retro reflectiveness
void Geom::SetLaserRetro(float retro)
{
  this->laserRetroP->SetValue(retro);
}

////////////////////////////////////////////////////////////////////////////////
/// Get the laser retro reflectiveness
float Geom::GetLaserRetro() const
{
  return this->laserRetroP->GetValue();
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
      if (*iter)
      {
        (*iter)->SetVisible(true, false);
        (*iter)->SetTransparency(0.6);
      }
    }
  } 
  else
  {
    for (iter = this->visuals.begin(); iter != this->visuals.end(); iter++)
    {
      if (*iter)
      {
        (*iter)->SetVisible(true, true);
        (*iter)->SetTransparency(0.0);
      }
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
      if (*iter)
        (*iter)->SetVisible(false, false);
    }
  }
  else
  {
    for (iter = this->visuals.begin(); iter != this->visuals.end(); iter++)
    {
      if (*iter)
        (*iter)->SetVisible(true, false);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////
/// Set the mass
void Geom::SetMass(const double &mass)
{
  this->physicsEngine->LockMutex();
  dMassAdjust(&this->mass, mass);
  this->physicsEngine->UnlockMutex();

  this->body->UpdateCoM();

}

////////////////////////////////////////////////////////////////////////////////
/// Get the number of visuals
unsigned int Geom::GetVisualCount() const
{
  return this->visuals.size();
}

////////////////////////////////////////////////////////////////////////////////
/// Get a visual
OgreVisual *Geom::GetVisual(unsigned int index) const
{
  if (index < this->visuals.size())
    return this->visuals[index];
  else
    return NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Get a visual
OgreVisual *Geom::GetVisualById(int id) const
{
  std::vector<OgreVisual*>::const_iterator iter;

  for (iter = this->visuals.begin(); iter != this->visuals.end(); iter++)
  {
    if ( (*iter) && (*iter)->GetId() == id)
      return *iter;
  }

  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the body this geom belongs to
Body *Geom::GetBody() const
{
  return this->body;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the model this geom belongs to
Model *Geom::GetModel() const
{
  return this->body->GetModel();
}

////////////////////////////////////////////////////////////////////////////////
/// Set the friction mode of the geom
void Geom::SetFrictionMode( const bool &v )
{
  this->contact->enableFriction = v;
}

////////////////////////////////////////////////////////////////////////////////
/// Get the bounding box for this geom
void Geom::GetBoundingBox(Vector3 &min, Vector3 &max) const
{
  dReal aabb[6];
  dGeomGetAABB(this->geomId, aabb);

  min.Set(aabb[0], aabb[2], aabb[4]);
  max.Set(aabb[1], aabb[3], aabb[5]);
}
