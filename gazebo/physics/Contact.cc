/*
 * Copyright 2011 Nate Koenig
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
/* Desc: Specification of a contact
 * Author: Nate Koenig
 * Date: 10 Nov 2009
 */

#include "Contact.hh"

using namespace gazebo;
using namespace physics;


//////////////////////////////////////////////////
Contact::Contact()
{
  this->collision1 = NULL;
  this->collision2 = NULL;
  this->count = 0;
}

//////////////////////////////////////////////////
Contact::Contact(const Contact &_c)
{
  *this = _c;
}

//////////////////////////////////////////////////
Contact::~Contact()
{
}

//////////////////////////////////////////////////
Contact Contact::Clone() const
{
  Contact c = *this;
  return c;
}

//////////////////////////////////////////////////
Contact &Contact::operator =(const Contact &contact)
{
  this->collision1 = contact.collision1;
  this->collision2 = contact.collision2;

  this->count = contact.count;
  for (int i = 0; i < MAX_CONTACT_JOINTS; i++)
  {
    this->forces[i] = contact.forces[i];
    this->positions[i] = contact.positions[i];
    this->normals[i] = contact.normals[i];
    this->depths[i] = contact.depths[i];
  }

  return *this;
}

//////////////////////////////////////////////////
void Contact::Reset()
{
  this->count = 0;
}


