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

#ifndef CONTACT_HH
#define CONTACT_HH

#include "Vector3.hh"
#include "JointFeedback.hh"

namespace gazebo
{
  class Geom;
  
  class Contact
  {
    /// \brief Constructor
    public: Contact();

    /// \brief Copy constructor
    public: Contact(const Contact &c);

    /// \brief Destructor
    public: virtual ~Contact();

    /// \brief Clone the contact
    public: Contact Clone() const;
  
    /// \brief Operator =
    public: const Contact &operator=(const Contact &contact);
  
    public: Geom *geom1;
    public: Geom *geom2;
 
    public: JointFeedback forces;

    public: Vector3 pos;
    public: Vector3 normal;
  
    public: double depth;
  };
}

#endif
