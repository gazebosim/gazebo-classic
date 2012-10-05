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

#ifndef CONTACT_HH
#define CONTACT_HH

#include <vector>

#include "common/Time.hh"
#include "math/Vector3.hh"
#include "JointFeedback.hh"

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

    /// \brief A contact between two collisions. Each contact can consist of
    ///        a numnber of contact points
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
      public: Contact &operator =(const Contact &contact);

      /// \brief Reset
      public: void Reset();

      public: Collision *collision1;
      public: Collision *collision2;

      public: JointFeedback forces[MAX_CONTACT_JOINTS];

      public: math::Vector3 positions[MAX_CONTACT_JOINTS];
      public: math::Vector3 normals[MAX_CONTACT_JOINTS];

      public: double depths[MAX_CONTACT_JOINTS];

      public: int count;

      public: common::Time time;
    };
    /// \}
  }
}
#endif
