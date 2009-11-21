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
