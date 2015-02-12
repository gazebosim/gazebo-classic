/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#include <string>

#include "gazebo/msgs/msgs.hh"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/math/Vector3.hh"
#include "gazebo/physics/JointWrench.hh"
#include "gazebo/util/system.hh"

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
    class GAZEBO_VISIBLE Contact
    {
      /// \brief Constructor.
      public: Contact();

      /// \brief Copy constructor
      /// \param[in] _contact Contact to copy.
      public: Contact(const Contact &_contact);

      /// \brief Destructor.
      public: virtual ~Contact();

      /// \brief Operator =.
      /// \param[in] _contact Contact to copy.
      /// \return Reference to this contact
      public: Contact &operator =(const Contact &_contact);

      /// \brief Operator =.
      /// \param[in] _contact msgs::Contact to copy.
      /// \return Reference to this contact
      public: Contact &operator =(const msgs::Contact &_contact);

      /// \brief Populate a msgs::Contact with data from this.
      /// \param[out] _msg Contact message the will hold the data.
      public: void FillMsg(msgs::Contact &_msg) const;

      /// \brief Produce a debug string.
      /// \return A string that contains the values of the contact.
      public: std::string DebugString() const;

      /// \brief Reset to default values.
      public: void Reset();

      /// \brief Pointer to the first collision object
      public: Collision *collision1;

      /// \brief Pointer to the second collision object
      public: Collision *collision2;

      /// \brief Array of forces for the contact.
      /// All forces and torques are relative to the center of mass of the
      /// respective links that the collision elments are attached to.
      public: JointWrench wrench[MAX_CONTACT_JOINTS];

      /// \brief Array of force positions.
      public: math::Vector3 positions[MAX_CONTACT_JOINTS];

      /// \brief Array of force normals.
      public: math::Vector3 normals[MAX_CONTACT_JOINTS];

      /// \brief Array of contact depths
      public: double depths[MAX_CONTACT_JOINTS];

      /// \brief Length of all the arrays.
      public: int count;

      /// \brief Time at which the contact occurred.
      public: common::Time time;

      /// \brief World in which the contact occurred
      public: WorldPtr world;
    };
    /// \}
  }
}
#endif
