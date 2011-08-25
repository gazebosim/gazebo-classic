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

#include "common/Global.hh"
#include "common/Exception.hh"
#include "physics/Physics.hh"
#include "physics/World.hh"
#include "physics/Model.hh"
#include "physics/Link.hh"
#include "physics/Geom.hh"

#include "sensors/SensorFactory.hh"
#include "sensors/ContactSensor.hh"

#include "math/Vector3.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("contact", ContactSensor)

//////////////////////////////////////////////////////////////////////////////
// Constructor
ContactSensor::ContactSensor()
    : Sensor()
{
  this->active = false;
  this->typeName = "contact";
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
ContactSensor::~ContactSensor()
{
/*
  std::vector< ParamT<std::string> *>::iterator iter;
  std::vector<Contact>::iterator citer;

  for (iter = this->geomNamesP.begin(); iter != this->geomNamesP.end(); iter++)
    delete *iter;
  this->geomNamesP.clear();

  this->geoms.clear();
*/
}


//////////////////////////////////////////////////////////////////////////////
///
void ContactSensor::Load()
{

  Sensor::Load();

  std::string linkName = this->sdf->GetLinkName();
  //gzerr << "parent link name : " << linkName << "\n";

  std::string modelName = this->sdf->GetModelName();
  //gzerr << "parent model name : " << modelName << "\n";

  std::string worldName = this->sdf->GetWorldName();
  //gzerr << "parent world name : " << worldName << "\n";

  // get parent link by looking at real parent
  std::string linkFullyScopedName = worldName + "::" + modelName + "::" + linkName;
  //gzerr << "scoped link name : " << linkFullyScopedName << "\n";


  this->world = gazebo::physics::get_world(worldName);
  this->model = this->world->GetModelByName(modelName);
  gazebo::physics::BasePtr tmp = this->model->GetByName(linkFullyScopedName);
  printf("ok\n");
  this->link = boost::dynamic_pointer_cast<gazebo::physics::Link>(this->model->GetByName(linkFullyScopedName));

  if (this->link == NULL)
    gzthrow("Null link in the contact sensor");

  // get geom name
  std::string geom_name = this->sdf->GetElement("contact")->GetElement("collision")->GetValueString("name");

  this->geom = this->link->GetGeom(geom_name);

  if (this->geom == NULL)
    gzthrow("Null geom in the contact sensor");

}

//////////////////////////////////////////////////////////////////////////////
/// Load the contact using parameter from an XMLConfig node
void ContactSensor::Load(sdf::ElementPtr &_sdf)
{
/*
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
*/

  Sensor::Load(_sdf);
}

//////////////////////////////////////////////////////////////////////////////
// Init the contact
void ContactSensor::InitChild()
{
/*
  std::vector< ParamT<std::string> *>::iterator iter;

  for (iter = this->geomNamesP.begin(); iter != this->geomNamesP.end(); iter++)
  {
    // Get the geom from the body
    Geom *geom = this->body->GetGeom( **(*iter) );
    geom->SetContactsEnabled(true);
    this->geoms.push_back(geom);
  }
*/
}

//////////////////////////////////////////////////////////////////////////////
// Update the sensor information
void ContactSensor::Update(bool force)
{
  //this->contacts.clear();
}

//////////////////////////////////////////////////////////////////////////////
// shutdown the contact
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
/*
  if (geomIndex < this->geoms.size())
    return this->geoms[geomIndex]->GetContactCount();
*/

  return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// Get a contact for a geom by index
physics::Contact ContactSensor::GetGeomContact(unsigned int geom, unsigned int index) const
{
/*
  if (geom < this->geoms.size())
    return this->geoms[geom]->GetContact( index );
*/

  return physics::Contact();
}

//////////////////////////////////////////////////////////////////////////////
/// Get a contact for a geom by index
physics::Geom* ContactSensor::GetGeom(unsigned int geom) const
{
/*
  if (geom < this->geoms.size())
    return this->geoms[geom];
*/

  return NULL;
}
