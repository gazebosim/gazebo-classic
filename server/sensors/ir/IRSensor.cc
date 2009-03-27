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
/* Desc: IR proximity sensor
 * Author: Wenguo Liu
 * Date: 23 february 2004
 * SVN: $Id: IRSensor.cc 4436 2008-03-24 17:42:45Z robotos $
*/

#include <assert.h>
#include <float.h>
#include <sstream>

#include "SensorFactory.hh"
#include "XMLConfig.hh"
#include "Global.hh"
#include "RayGeom.hh"
#include "World.hh"
#include "PhysicsEngine.hh"
#include "GazeboError.hh"
#include "ODEPhysics.hh"
#include "XMLConfig.hh"

#include "IRSensor.hh"

#include "Vector3.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_SENSOR("ir", IRSensor);

//////////////////////////////////////////////////////////////////////////////
// Constructor
IRSensor::IRSensor(Body *body)
    : Sensor(body)
{
  this->active = false;
  this->rayCount = NULL;
  this->rangeCount = NULL;
  this->maxAngle = NULL;
  this->minAngle = NULL;
  this->minRange = NULL;
  this->maxRange = NULL;
  this->origin = NULL;
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
IRSensor::~IRSensor()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Load the ray using parameter from an XMLConfig node
void IRSensor::LoadChild(XMLConfigNode *node)
{
  if (this->body == NULL)
  {
    std::ostringstream stream;
    stream << "Body is NULL";
    gzthrow(stream.str());
  }
  
  XMLConfigNode *iNode;
  int i =0;
  

  this->IRCount = node->GetInt("irCount",0,1);
  
  
  //Allocate memeorys
  this->rayCount = new int[this->IRCount];
  this->rangeCount = new int[this->IRCount];
  this->minAngle = new double[this->IRCount];
  this->maxAngle = new double[this->IRCount];
  this->minRange = new double[this->IRCount];
  this->maxRange = new double[this->IRCount];
  this->origin = new Vector3[this->IRCount];
  
  
  //Read configuration from XML file
  for(i=0, iNode = node->GetChild("ir"); iNode;i++)
 {
 	if(i >= this->IRCount)
 	{
 		std::ostringstream stream;
 		stream<< "please check the irCount is correct!";
 		gzthrow(stream.str());
 	}
 	
 	std::string name = iNode->GetString("name","",1);
    this->rayCount[i] = iNode->GetInt("rayCount",0,1);
    this->rangeCount[i] = iNode->GetInt("rangeCount",0,1);
    this->minAngle[i] = DTOR(iNode->GetDouble("minAngle",-90,1));
    this->maxAngle[i] = DTOR(iNode->GetDouble("maxAngle",90,1));
    this->minRange[i] = iNode->GetDouble("minRange",0,1);
    this->maxRange[i] = iNode->GetDouble("maxRange",8,1);
    this->origin[i] = iNode->GetVector3("origin", Vector3(0,0,0));
    iNode = iNode->GetNext("ir");
 }

  this->displayRays = node->GetBool("displayRays", true);

  // Create a space to contain the ray space
  this->superSpaceId = dSimpleSpaceCreate( 0 );

  // Create a space to contain all the rays
  this->raySpaceId = dSimpleSpaceCreate( this->superSpaceId );

  // Set collision bits
  dGeomSetCategoryBits((dGeomID) this->raySpaceId, GZ_SENSOR_COLLIDE);
  dGeomSetCollideBits((dGeomID) this->raySpaceId, ~GZ_SENSOR_COLLIDE);

  this->body->SetSpaceId( this->raySpaceId );
}

//////////////////////////////////////////////////////////////////////////////
// Init the ray
void IRSensor::InitChild()
{
  Pose3d bodyPose;
  double angle;
  Vector3 start, end, axis;
  RayGeom *ray;

  bodyPose = this->body->GetPose();
  this->prevPose = bodyPose;

  // Create and array of ray geoms
  //for (int i = 0; i < this->rayCount; i++)
  for(int j=0; j<this->IRCount;j++)
  {
    for (int i = this->rayCount[j]-1; i >= 0; i--)
    {
      if(this->rayCount[j]==1)
        angle=(this->maxAngle[j]+ this->minAngle[j])/2;
      else
        angle = i * (this->maxAngle[j] - this->minAngle[j]) / (this->rayCount[j] - 1) + this->minAngle[j];

      axis.Set(cos(angle), sin(angle),0);
      

      start = (axis * this->minRange[j]) + this->origin[j];
      end = (axis * this->maxRange[j]) + this->origin[j];

      ray = new RayGeom(this->body, displayRays);

      ray->SetPoints(start, end);

      this->rays.push_back(ray);

    }
  }

}

//////////////////////////////////////////////////////////////////////////////
// Init the ray
void IRSensor::FiniChild()
{
  std::vector<RayGeom*>::iterator iter;
  for (iter=this->rays.begin(); iter!=this->rays.end(); iter++)
  {
    delete *iter;
  }
  this->rays.clear();
  if(this->rayCount!=NULL)
  {
  	delete []this->rayCount;
  	this->rayCount=NULL;
  }
  if(this->rangeCount!=NULL)
  {
  	delete []this->rangeCount;
  	this->rangeCount=NULL;
  }
  if(this->minAngle!=NULL)
  {
  	delete []this->minAngle;
  	this->minAngle=NULL;
  }
  if(this->maxAngle!=NULL)
  {
  	delete []this->maxAngle;
  	this->maxAngle=NULL;
  }
  if(this->minRange!=NULL)
  {
  	delete []this->minRange;
  	this->minRange=NULL;
  }
  if(this->maxRange!=NULL)
  {
  	delete []this->maxRange;
  	this->maxRange=NULL;
  }
  if(this->origin!=NULL)
  {
  	delete []this->origin;
  	this->origin=NULL;
  }
}

//////////////////////////////////////////////////////////////////////////////
/// Get the minimum angle
double IRSensor::GetMinAngle() const
{
  return 0;//this->minAngle;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the maximum angle
double IRSensor::GetMaxAngle() const
{
  return 0;//this->maxAngle;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the minimum range
double IRSensor::GetMinRange() const
{
  return 0;//this->minRange;
}

//////////////////////////////////////////////////////////////////////////////
///  Get the maximum range
double IRSensor::GetMaxRange() const
{
  return 0;//this->maxRange;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the ray count
int IRSensor::GetIRCount() const
{
  return  this->IRCount;
}

//////////////////////////////////////////////////////////////////////////////
/// Get the range count
int IRSensor::GetRangeCount() const
{
  return 0;//this->rangeCount;
}

//////////////////////////////////////////////////////////////////////////////
// Get detected range for a ray
double IRSensor::GetRange(int index)
{
  if (index < 0 || index >= this->IRCount)
  {
    std::ostringstream stream;
    stream << "index[" << index << "] out of range[0-"
    << this->IRCount << "]";
    gzthrow(stream.str());
  }
  
  //get the ray index range for this specific IR
  int start_index=0;
  int end_index=0;
  for(int i=0;i<index;i++)
  {
  	start_index +=this->rayCount[i];
  }
  end_index = start_index + this->rayCount[index] - 1;
  
  double range = this->maxRange[index];
  for(int i=start_index;i<=end_index;i++)
  {
      range = std::min(this->rays[i]->GetLength(),range);  
  }
  return range;

}

Pose IRSensor::GetPose(int index)
{
  if (index < 0 || index >= this->IRCount)
  {
    std::ostringstream stream;
    stream << "index[" << index << "] out of range[0-"
    << this->IRCount << "]";
    gzthrow(stream.str());
  }
  
  Pose pose;
  pose.pos.x = this->origin[index].x;
  pose.pos.y = this->origin[index].y;
  pose.pos.z = this->origin[index].z;
  pose.roll= 0;
  pose.pitch = 0;
  pose.yaw = 0.5*(this->minAngle[index]+this->maxAngle[index]);
  return pose;
	
}


//////////////////////////////////////////////////////////////////////////////
// Update the sensor information
void IRSensor::UpdateChild()
{
//  if (this->active)
  {
    std::vector<RayGeom*>::iterator iter;
    Pose3d poseDelta;
    Vector3 a, b;

    // Get the pose of the sensor body (global cs)
    poseDelta = this->body->GetPose() - this->prevPose;
    this->prevPose = this->body->GetPose();

    // Reset the ray lengths and mark the geoms as dirty (so they get
    // redrawn)
    for (iter = this->rays.begin(); iter != this->rays.end(); iter++)
    {
      (*iter)->SetLength( this->maxRange[0] );
      (*iter)->SetRetro( 0.0 );
      (*iter)->SetFiducial( -1 );

      // Get the global points of the line
      (*iter)->Update();
    }

    ODEPhysics *ode = dynamic_cast<ODEPhysics*>(World::Instance()->GetPhysicsEngine());

    if (ode == NULL)
    {
      gzthrow( "Invalid physics engine. Must use ODE." );
    }

    // Do collision detection
    dSpaceCollide2( ( dGeomID ) ( this->superSpaceId ),
                    ( dGeomID ) ( ode->GetSpaceId() ),
                    this, &UpdateCallback );
  }
}



/////////////////////////////////////////////////////////////////////////////
// Callback for ray intersection test
void IRSensor::UpdateCallback( void *data, dGeomID o1, dGeomID o2 )
{
  int n = 0;
  dContactGeom contact;
  dxGeom *geom1, *geom2 = NULL;
  RayGeom *rayGeom = NULL;
  Geom *hitGeom = NULL;
  IRSensor *self = NULL;

  self = (IRSensor*) data;

  // Check space
  if ( dGeomIsSpace( o1 ) || dGeomIsSpace( o2 ) )
  {
    if (dGeomGetSpace(o1) == self->superSpaceId || dGeomGetSpace(o2) == self->superSpaceId)
    {
      dSpaceCollide2( o1, o2, self, &UpdateCallback );
    }
    if (dGeomGetSpace(o1) == self->raySpaceId || dGeomGetSpace(o2) == self->raySpaceId)
    {
      dSpaceCollide2( o1, o2, self, &UpdateCallback );
    }
  }
  else
  {
    geom1 = NULL;
    geom2 = NULL;

    // Get pointers to the underlying geoms
    if (dGeomGetClass(o1) == dGeomTransformClass)
      geom1 = (dxGeom*) dGeomGetData(dGeomTransformGetGeom(o1));
    else
      geom1 = (dxGeom*) dGeomGetData(o1);

    if (dGeomGetClass(o2) == dGeomTransformClass)
      geom2 = (dxGeom*) dGeomGetData(dGeomTransformGetGeom(o2));
    else
      geom2 = (dxGeom*) dGeomGetData(o2);

    assert(geom1 && geom2);

    rayGeom = NULL;
    hitGeom = NULL;

    // Figure out which one is a ray; note that this assumes
    // that the ODE dRayClass is used *soley* by the RayGeom.
    if (dGeomGetClass(o1) == dRayClass)
    {
      rayGeom = (RayGeom*) geom1;
      hitGeom = (Geom*) geom2;
      dGeomRaySetParams(o1, 0, 0);
      dGeomRaySetClosestHit(o1, 1);
    }

    if (dGeomGetClass(o2) == dRayClass)
    {
      assert(rayGeom == NULL);
      rayGeom = (RayGeom*) geom2;
      hitGeom = (Geom* )geom1;
      dGeomRaySetParams(o2, 0, 0);
      dGeomRaySetClosestHit(o2, 1);
    }

    // Check for ray/geom intersections
    if ( rayGeom && hitGeom )
    {

      n = dCollide(o1, o2, 1, &contact, sizeof(contact));

      if ( n > 0 )
      {
        if (contact.depth < rayGeom->GetLength())
        {
          rayGeom->SetLength( contact.depth );
        }
      }
    }
  }
}
