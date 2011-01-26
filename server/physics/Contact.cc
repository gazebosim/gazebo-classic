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
/* Desc: Specification of a contact
 * Author: Nate Koenig
 * Date: 10 Nov 2009
 * SVN: $Id$
 */

#include "Contact.hh"

using namespace gazebo;

////////////////////////////////////////////////////////////////////////////////
/// Constructor
Contact::Contact()
{
  this->geom1 = NULL;
  this->geom2 = NULL;
}

////////////////////////////////////////////////////////////////////////////////
/// Copy constructor
Contact::Contact(const Contact &c)
{
  *this = c;
}

////////////////////////////////////////////////////////////////////////////////
/// Destructor
Contact::~Contact()
{
}

////////////////////////////////////////////////////////////////////////////////
// Clone the contact
Contact Contact::Clone() const
{
  Contact c = *this;
  return c;
}

////////////////////////////////////////////////////////////////////////////////
// Operator =
const Contact &Contact::operator=(const Contact &contact) 
{
  this->geom1 = contact.geom1;
  this->geom2 = contact.geom2;

  this->forces.clear();
  this->positions.clear();
  this->normals.clear();

  std::copy(contact.forces.begin(), contact.forces.end(), 
            std::back_inserter(this->forces));
  std::copy(contact.positions.begin(), contact.positions.end(), 
            std::back_inserter(this->positions));
  std::copy(contact.normals.begin(), contact.normals.end(), 
            std::back_inserter(this->normals));
  std::copy(contact.depths.begin(), contact.depths.end(), 
            std::back_inserter(this->depths));

  return *this;
}

////////////////////////////////////////////////////////////////////////////////
/// Reset
void Contact::Reset()
{
  this->depths.clear();
  this->positions.clear();
  this->normals.clear();
  this->forces.clear();
}
