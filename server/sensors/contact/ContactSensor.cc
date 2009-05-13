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
/* Desc: Contact sensor
 * Author: Nate Koenig
 * Date: 09 Sept. 2008
 * SVN: $Id$
*/

#include <assert.h>
#include <float.h>
#include <sstream>

#include "Global.hh"
//#include "World.hh"
//#include "Controller.hh"

#include "GazeboError.hh"
#include "XMLConfig.hh"
#include "Simulator.hh"
#include "SensorFactory.hh"
#include "Geom.hh"
#include "ContactParams.hh"
#include "ContactSensor.hh"

#include "Vector3.hh"

using namespace gazebo;

GZ_REGISTER_STATIC_SENSOR("contact", ContactSensor);

//////////////////////////////////////////////////////////////////////////////
// Constructor
ContactSensor::ContactSensor(Body *body)
    : Sensor(body)
{
  this->active = false;

  this->typeName = "contact";

  this->contactCount = 0;
  this->contactStates = NULL;
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
ContactSensor::~ContactSensor()
{
  std::vector< ParamT<std::string> *>::iterator iter;

  if (this->contactStates)
    delete [] this->contactStates;

  if (this->contactTimes)
    delete [] this->contactTimes;

  for (iter = this->geomNamesP.begin(); iter != this->geomNamesP.end(); iter++)
  {
    delete *iter;
  }
  this->geomNamesP.clear();
}

//////////////////////////////////////////////////////////////////////////////
/// Get the contact time
double ContactSensor::GetContactTime(unsigned int index) const
{
  if (index < this->contactCount)
    return this->contactTimes[ index ];

  return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// Return the number of contacts
unsigned int ContactSensor::GetContactCount() const
{
  return this->contactCount;
}

//////////////////////////////////////////////////////////////////////////////
/// Return the contact states
uint8_t ContactSensor::GetContactState(unsigned int index) const
{
  if (index < this->contactCount)
    return this->contactStates[index];

  return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// Return the contact geom name
std::string ContactSensor::GetContactGeomName(unsigned int index) const
{
  if (index < this->contactCount)
    return this->contactNames[index];

  return std::string("");
}

//////////////////////////////////////////////////////////////////////////////
/// Return the contact feedback forces and torques
dJointFeedback ContactSensor::GetContactFeedback(unsigned int index) const
{
  if (index < this->contactCount)
    return this->contactFeedbacks[index];
  return dJointFeedback();
}


//////////////////////////////////////////////////////////////////////////////
/// Return the self geom name
std::string ContactSensor::GetGeomName(unsigned int index) const
{
  if (index < this->contactCount)
    return this->geomNamesP[index]->GetValue();

  return std::string("");
}

//////////////////////////////////////////////////////////////////////////////
/// Reset the contact states
void ContactSensor::ResetContactStates()
{
  memset(this->contactStates, 0, sizeof(uint8_t) * this->contactCount);
}

//////////////////////////////////////////////////////////////////////////////
/// Load the ray using parameter from an XMLConfig node
void ContactSensor::LoadChild(XMLConfigNode *node)
{
  XMLConfigNode *geomNode = NULL;

  if (this->body == NULL)
  {
    gzthrow("Null body in the contact sensor");
  }

  Param::Begin(&this->parameters);
  geomNode = node->GetChild("geom");
  while (geomNode)
  {
    ParamT<std::string> *geomName = new ParamT<std::string>("geom","",1);
    geomName->SetValue( geomNode->GetValue() );
    this->geomNamesP.push_back(geomName);
    geomNode = geomNode->GetNext("geom");
  }
  Param::End();

  this->contactCount = this->geomNamesP.size();
  this->contactTimes = new double[ this->contactCount ];
  this->contactStates = new uint8_t[ this->contactCount ];
  for (unsigned int i=0; i< this->contactCount; i++)
    this->contactNames.push_back("");

  memset(this->contactTimes,0, sizeof(double) * this->contactCount);
  memset(this->contactStates,0, sizeof(uint8_t) * this->contactCount);
  memset(this->contactTimes,0, sizeof(double) * this->contactCount);
}

//////////////////////////////////////////////////////////////////////////////
/// Save the sensor info in XML format
void ContactSensor::SaveChild(std::string &prefix, std::ostream &stream)
{
  std::vector< ParamT<std::string> *>::iterator iter;

  for (iter = this->geomNamesP.begin(); iter != this->geomNamesP.end(); iter++)
  {
    stream << prefix << "  " << *(iter) << "\n";
  }
}

//////////////////////////////////////////////////////////////////////////////
// Init the contact
void ContactSensor::InitChild()
{
  std::vector< ParamT<std::string> *>::iterator iter;

  for (iter = this->geomNamesP.begin(); iter != this->geomNamesP.end(); iter++)
  {
    // Get the geom from the body
    Geom *geom = this->body->GetGeom( **(*iter) );

    // Set the callback
    if (geom)
    {
      geom->contact->Callback( &ContactSensor::ContactCallback, this );
    }
    else
    {
      gzthrow("Unable to find geom[" + **(*iter) + "]");
    }
  }

}

//////////////////////////////////////////////////////////////////////////////
// Init the ray
void ContactSensor::FiniChild()
{
}

//////////////////////////////////////////////////////////////////////////////
// Update the sensor information
void ContactSensor::UpdateChild()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Contact callback
void ContactSensor::ContactCallback(Geom *g1, Geom *g2)
{

  // somehow here, extract contact information when user requests it
  //


  std::vector< ParamT<std::string> *>::iterator iter;
  int i = 0;


  for (iter = this->geomNamesP.begin(); iter != this->geomNamesP.end(); 
       iter++, i++)
  {
    if ( **(*iter) == g1->GetName() || **(*iter) == g2->GetName() )
    {
      if (i < GAZEBO_MAX_CONTACT_FB_DATA)
      {
        dJointSetFeedback(g1->cID, this->contactFeedbacks+i);

        // std::cout << "contact id:" << i
        //           << " geom1:" << g1->GetName()
        //           << " geom2:" << g2->GetName();
        // std::cout << " contact joint ID:" << g1->cID
        //           << " f1:"<<this->contactFeedbacks[i].f1[0]<<","<<this->contactFeedbacks[i].f1[1]<<","<<this->contactFeedbacks[i].f1[2]<<","<<this->contactFeedbacks[i].f1[3]
        //           << " t1:"<<this->contactFeedbacks[i].t1[0]<<","<<this->contactFeedbacks[i].t1[1]<<","<<this->contactFeedbacks[i].t1[2]<<","<<this->contactFeedbacks[i].t1[3]
        //           << " f1_magnitude:" << dLENGTH(this->contactFeedbacks[i].f1)
        //           << " f2:"<<this->contactFeedbacks[i].f2[0]<<","<<this->contactFeedbacks[i].f2[1]<<","<<this->contactFeedbacks[i].f2[2]<<","<<this->contactFeedbacks[i].f2[3]
        //           << " t2:"<<this->contactFeedbacks[i].t2[0]<<","<<this->contactFeedbacks[i].t2[1]<<","<<this->contactFeedbacks[i].t2[2]<<","<<this->contactFeedbacks[i].t2[3]
        //           << " f2_magnitude:" << dLENGTH(this->contactFeedbacks[i].f2)
        //           << std::endl;
      }

      this->contactStates[i] = 1;
      this->contactTimes[i] = Simulator::Instance()->GetRealTime();
      this->contactNames[i] = **(*iter)==g1->GetName()? g2->GetName() : g1->GetName();
    }
  }

}


