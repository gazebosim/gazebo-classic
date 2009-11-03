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
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
ContactSensor::~ContactSensor()
{
  std::vector< ParamT<std::string> *>::iterator iter;
  std::vector<ContactState>::iterator citer;

  for (iter = this->geomNamesP.begin(); iter != this->geomNamesP.end(); iter++)
    delete *iter;
  this->geomNamesP.clear();

  this->contacts.clear();
}

//////////////////////////////////////////////////////////////////////////////
/// Get the contact time
double ContactSensor::GetContactTime(unsigned int index) const
{
  if (index < this->contacts.size())
    return this->contacts[index].time;

  return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// Return the number of contacts
unsigned int ContactSensor::GetContactCount() const
{
  return this->contacts.size();
}

//////////////////////////////////////////////////////////////////////////////
/// Return the contact states
uint8_t ContactSensor::GetContactState(unsigned int index) const
{
  if (index < this->contacts.size())
    return this->contacts[index].state;

  return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// Return the contact geom name
std::string ContactSensor::GetContactGeomName(unsigned int index) const
{
  if (index < this->contacts.size())
    return this->contacts[index].name;

  return std::string("");
}

//////////////////////////////////////////////////////////////////////////////
/// Return contact force on the first body
Vector3 ContactSensor::GetContactBody1Force(unsigned int index) const
{
  Vector3 result;

  if (index < this->contacts.size())
    result = this->contacts[index].body1Force;
 
  return result; 
}

//////////////////////////////////////////////////////////////////////////////
/// Return contact force on the second body
Vector3 ContactSensor::GetContactBody2Force(unsigned int index) const
{
  Vector3 result;

  if (index < this->contacts.size())
    result = this->contacts[index].body2Force;
 
  return result; 
}

//////////////////////////////////////////////////////////////////////////////
/// Return the self geom name
std::string ContactSensor::GetGeomName(unsigned int index) const
{
  if (index < this->contacts.size())
    return this->geomNamesP[index]->GetValue();

  return std::string("");
}

//////////////////////////////////////////////////////////////////////////////
/// Reset the contact states
void ContactSensor::ResetContactStates()
{
  std::vector<ContactState>::iterator iter;
  for (iter = this->contacts.begin(); iter != this->contacts.end(); iter++)
    (*iter).state = 0;
}

//////////////////////////////////////////////////////////////////////////////
/// Load the ray using parameter from an XMLConfig node
void ContactSensor::LoadChild(XMLConfigNode *node)
{
  XMLConfigNode *geomNode = NULL;

  if (this->body == NULL)
    gzthrow("Null body in the contact sensor");

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
}

//////////////////////////////////////////////////////////////////////////////
/// Save the sensor info in XML format
void ContactSensor::SaveChild(std::string &prefix, std::ostream &stream)
{
  std::vector< ParamT<std::string> *>::iterator iter;

  for (iter = this->geomNamesP.begin(); iter != this->geomNamesP.end(); iter++)
    stream << prefix << "  " << *(iter) << "\n";
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
      geom->contact->Callback( &ContactSensor::ContactCallback, this );
    else
      gzthrow("Unable to find geom[" + **(*iter) + "]");
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
      this->contacts[i].state = 1;
      this->contacts[i].time = Simulator::Instance()->GetRealTime();
      this->contacts[i].name = **(*iter)==g1->GetName()? g2->GetName() : g1->GetName();
      this->contacts[i].body1Force = g1->contact->body1Force;
      this->contacts[i].body2Force = g1->contact->body2Force;
      this->contacts[i].body1Torque = g1->contact->body1Torque;
      this->contacts[i].body2Torque = g1->contact->body2Torque;
    }
  }

}


