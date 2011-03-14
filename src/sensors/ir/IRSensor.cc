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
/* Desc: IR proximity sensor
 * Author: Wenguo Liu
 * Date: 23 february 2004
 * SVN: $Id: IRSensor.cc 4436 2008-03-24 17:42:45Z robotos $
 */

#include <assert.h>
#include <float.h>
#include <sstream>

#include "SensorFactory.hh"
#include "common/XMLConfig.hh"
#include "common/Global.hh"
#include "RayShape.hh"
#include "MultiRayShape.hh"
#include "RaySensor.hh"
#include "World.hh"
#include "PhysicsEngine.hh"
#include "common/GazeboError.hh"
#include "common/XMLConfig.hh"

#include "IRSensor.hh"

#include "common/Vector3.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_SENSOR("ir", IRSensor);

//////////////////////////////////////////////////////////////////////////////
// Constructor
IRSensor::IRSensor(Body *body)
 : Sensor(body)
{
  this->active = false;
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
  Geom *laserGeom;
  MultiRayShape *laserShape;

  iNode = node->GetChild("ir");
  while (iNode)
  {
    laserGeom = this->GetWorld()->GetPhysicsEngine()->CreateGeom(
                     "multiray", this->body);
    laserGeom->SetName("IR Sensor Geom");

    laserShape = (MultiRayShape*)(laserGeom->GetShape());

    //laserShape->SetDisplayType( (**this->displayRaysP) );
    laserShape->Load(iNode);
    /*laserShape->Load(**verticalRayCountP, **rayCountP, 
        **this->originP, **minRangeP, **maxRangeP, 
        **this->verticalMinAngleP, **this->verticalMaxAngleP, **this->minAngleP, **this->maxAngleP);*/

    iNode = iNode->GetNext("ir");
  }

  /*this->IRCount = node->GetInt("irCount",0,1);

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

  */
  /* BULLET
     this->body->SetSpaceId( this->raySpaceId );
     */

}

//////////////////////////////////////////////////////////////////////////////
// Init the ray
void IRSensor::InitChild()
{
  /*Pose3d bodyPose;
  double angle;
  Vector3 start, end, axis;
  RayGeom *ray;
  */

  /*bodyPose = this->body->GetWorldPose();
  //this->prevPose = bodyPose;

  // Create and array of ray geoms
  //for (int i = 0; i < this->rayCount; i++)
  for(unsigned int j=0; j<this->irBeams.size(); j++)
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

      // BULLET
      //ray = new RayGeom(this->body, displayRays);

      ray->SetPoints(start, end);

      this->rays.push_back(ray);

    }
  }*/

}

//////////////////////////////////////////////////////////////////////////////
// Init the ray
void IRSensor::FiniChild()
{
  std::vector<RaySensor*>::iterator iter;
  for (iter=this->irBeams.begin(); iter!=this->irBeams.end(); iter++)
  {
    delete *iter;
  }
  this->irBeams.clear();
}

//////////////////////////////////////////////////////////////////////////////
/// Get the ray count
unsigned int IRSensor::GetIRCount() const
{
  return this->irBeams.size();
}


//////////////////////////////////////////////////////////////////////////////
// Get detected range for a ray
double IRSensor::GetRange(unsigned int index) const
{
  if (index >= this->irBeams.size())
  {
    std::ostringstream stream;
    stream << "index[" << index << "] out of range[0-"
      << this->irBeams.size() << "]";
    gzthrow(stream.str());
  }

  return this->irBeams[index]->GetMinRange();
}

//////////////////////////////////////////////////////////////////////////////
// Get the pose of an ir beam
Pose3d IRSensor::GetPose(unsigned int index) const
{
  if (index >= this->irBeams.size())
  {
    std::ostringstream stream;
    stream << "index[" << index << "] out of range[0-"
      << this->irBeams.size() << "]";
    gzthrow(stream.str());
  }

  return this->irBeams[index]->GetPose();
}

//////////////////////////////////////////////////////////////////////////////
// Update the sensor information
void IRSensor::UpdateChild()
{
  if (this->active)
  {
    std::vector<RaySensor*>::iterator iter;
    for (iter = this->irBeams.begin(); iter != this->irBeams.end(); iter++)
      (*iter)->Update();
  }
}
