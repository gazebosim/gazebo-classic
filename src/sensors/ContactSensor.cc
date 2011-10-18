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

#include "common/Exception.hh"
#include "physics/Physics.hh"
#include "physics/World.hh"
#include "physics/Model.hh"
#include "physics/Link.hh"
#include "physics/Collision.hh"

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
}


//////////////////////////////////////////////////////////////////////////////
// Destructor
ContactSensor::~ContactSensor()
{
/*
  std::vector< ParamT<std::string> *>::iterator iter;
  std::vector<Contact>::iterator citer;

  for (iter = this->collisionNamesP.begin(); iter != this->collisionNamesP.end(); iter++)
    delete *iter;
  this->collisionNamesP.clear();

  this->collisions.clear();
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

  // get collision name
  std::string collision_name = this->sdf->GetElement("contact")->GetElement("collision")->GetValueString("name");

  this->collision = this->link->GetCollision(collision_name);

  if (this->collision == NULL)
    gzthrow("Null collision in the contact sensor");

}

//////////////////////////////////////////////////////////////////////////////
/// Load the contact using parameter from an XMLConfig node
void ContactSensor::Load(sdf::ElementPtr &_sdf)
{
/*
  XMLConfigNode *collisionNode = NULL;
  if (this->body == NULL)
    gzthrow("Null body in the contact sensor");

  Param::Begin(&this->parameters);
  collisionNode = node->GetChild("collision");

  while (collisionNode)
  {
    ParamT<std::string> *collisionName = new ParamT<std::string>("collision","",1);
    collisionName->SetValue( collisionNode->GetValue() );
    this->collisionNamesP.push_back(collisionName);
    collisionNode = collisionNode->GetNext("collision");
  }
  Param::End();
*/

  Sensor::Load(_sdf);
}

//////////////////////////////////////////////////////////////////////////////
// Init the contact
void ContactSensor::Init()
{
/*
  std::vector< ParamT<std::string> *>::iterator iter;

  for (iter = this->collisionNamesP.begin(); iter != this->collisionNamesP.end(); iter++)
  {
    // Get the collision from the body
    Collision *collision = this->body->GetCollision( **(*iter) );
    this->collisions.push_back(collision);
  }
*/
  this->collision->SetContactsEnabled(true);

  Sensor::Init();
}

//////////////////////////////////////////////////////////////////////////////
// Update the sensor information
void ContactSensor::Update(bool /*_force*/)
{
  //this->contacts.clear();
}

//////////////////////////////////////////////////////////////////////////////
// shutdown the contact
void ContactSensor::Fini()
{
  Sensor::Fini();
}

//////////////////////////////////////////////////////////////////////////////
/// Get the number of collisions that the sensor is observing
unsigned int ContactSensor::GetCollisionCount() const
{
  return this->collisions.size();
}

//////////////////////////////////////////////////////////////////////////////
/// Return the number of contacts for an observed collision
unsigned int ContactSensor::GetCollisionContactCount(unsigned int _collisionIndex) const
{
  if (_collisionIndex < this->collisions.size())
    return this->collisions[_collisionIndex]->GetContactCount();

  return 0;
}

//////////////////////////////////////////////////////////////////////////////
/// Get a contact for a collision by index
physics::Contact ContactSensor::GetCollisionContact(unsigned int _collisionIndex, unsigned int _index) const
{
  if (_collisionIndex < this->collisions.size())
    return this->collisions[_collisionIndex]->GetContact( _index );

  return physics::Contact();
}

//////////////////////////////////////////////////////////////////////////////
/// Get a contact for a collision by index
physics::Collision* ContactSensor::GetCollision(unsigned int _collisionIndex) const
{
  if (_collisionIndex < this->collisions.size())
    return this->collisions[_collisionIndex];

  return NULL;
}

////////////////////////////////////////////////////////////////////////////////
// retrieve list of contacts for a geom
std::vector<gazebo::physics::Contact> ContactSensor::GetContacts()
{
  if (this->model)
    return this->model->GetContacts(this->collision);
  else
  {
    gzerr << "model not setup yet, are you calling GetContacts during Load?\n";
    return std::vector<gazebo::physics::Contact>();
  }
}
