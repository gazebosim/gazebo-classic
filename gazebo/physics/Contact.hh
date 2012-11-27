/*
 * Copyright 2012 Nate Koenig
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

#ifndef _CONTACT_HH_
#define _CONTACT_HH_

#include <vector>

#include "gazebo/common/Time.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/JointWrench.hh"

// For the sake of efficiency, use fixed size arrays for collision
// MAX_COLLIDE_RETURNS limits contact detection, needs to be large
//                      for proper contact dynamics.
// MAX_CONTACT_JOINTS truncates <max_contacts> specified in SDF
#define MAX_COLLIDE_RETURNS 250
#define MAX_CONTACT_JOINTS 32

namespace gazebo
{
  namespace physics
  {
    class Collision;
    /// \addtogroup gazebo_physics
    /// \{

    /// \class Contact Contact.hh physics/physics.hh
    /// \brief A contact between two collisions. Each contact can consist of
    /// a number of contact points
    class Contact
    {
      /// \brief Constructor.
      public: Contact();

      /// \brief Copy constructor
      /// \param[in] _contact Contact to copy.
      public: Contact(const Contact &_contact);

      /// \brief Destructor.
      public: virtual ~Contact();

      /// \brief Deprecated
      public: Contact Clone() const GAZEBO_DEPRECATED;

      /// \brief Operator =.
      /// \param[in] _contact Contact to copy.
      /// \return Reference to this contact
      public: Contact &operator =(const Contact &_contact);

      /// \brief Reset to default values.
      public: void Reset();

      /// \brief Pointer to the first collision in the contact.
      public: Collision *collision1;

      /// \brief Pointer to the second collision in the contact.
      public: Collision *collision2;

      /// \brief Array of forces for the contact.
      public: JointWrench wrench[MAX_CONTACT_JOINTS];

      /// \brief Array of force positions.
      public: math::Vector3 positions[MAX_CONTACT_JOINTS];

      /// \brief Array of force normals.
      public: math::Vector3 normals[MAX_CONTACT_JOINTS];

      /// \brief Array of contact depths
      public: double depths[MAX_CONTACT_JOINTS];

      /// \brief Length of all the arrays.
      public: int count;

      /// \brief Time at which the contact occured.
      public: common::Time time;
    };
    /// \}
  }
}
#endif
