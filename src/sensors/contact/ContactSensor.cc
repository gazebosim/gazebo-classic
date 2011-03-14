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
#include "ContactSensor.hh"

#include "common/Vector3.hh"

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
  std::vector<Contact>::iterator citer;

  for (iter = this->geomNamesP.begin(); iter != this->geomNamesP.end(); iter++)
    delete *iter;
  this->geomNamesP.clear();

  this->geoms.clear();
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
// Init the contact
void ContactSensor::InitChild()
{
  std::vector< ParamT<std::string> *>::iterator iter;

  for (iter = this->geomNamesP.begin(); iter != this->geomNamesP.end(); iter++)
  {
    // Get the geom from the body
    Geom *geom = this->body->GetGeom( **(*iter) );
    geom->SetContactsEnabled(true);
    this->geoms.push_back(geom);
  }
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
// Update the sensor information
void ContactSensor::UpdateChild()
{
  //this->contacts.clear();
}

//////////////////////////////////////////////////////////////////////////////
// Init the ray
void ContactSensor::FiniChild()
{
}

//////////////////////////////////////////////////////////////////////////////
/// Get the number of geoms that the sensor is observing
unsigned int ContactSensor::GetGeomCount() const
{
  return this->geoms.size();
}

//////////////////////////////////////////////////////////////////////////////
/// Return the number of contacts for an observed geom
unsigned int ContactSensor::GetGeomContactCount(unsigned int geomIndex) const
{
  if (geomIndex < this->geoms.size())
    return this->geoms[geomIndex]->GetContactCount();

  return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// Get a contact for a geom by index
Contact ContactSensor::GetGeomContact(unsigned int geom, unsigned int index) const
{
  if (geom < this->geoms.size())
    return this->geoms[geom]->GetContact( index );

  return Contact();
}

//////////////////////////////////////////////////////////////////////////////
/// Get a contact for a geom by index
Geom* ContactSensor::GetGeom(unsigned int geom) const
{
  if (geom < this->geoms.size())
    return this->geoms[geom];

  return NULL;
}
