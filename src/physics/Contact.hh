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

#ifndef CONTACT_HH
#define CONTACT_HH

#include <vector>

#include "common/Time.hh"
#include "math/Vector3.hh"
#include "JointFeedback.hh"

namespace gazebo
{
	namespace physics
  {
    class Geom;
    /// \addtogroup gazebo_physics
    /// \{
   
    /// \brief A contact between two geoms. Each contact can consist of
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
      public: const Contact &operator=(const Contact &contact);
  
      /// \brief Reset
      public: void Reset();
    
      public: Geom *geom1;
      public: Geom *geom2;
   
      public: std::vector<JointFeedback> forces;
  
      public: std::vector<math::Vector3> positions;
      public: std::vector<math::Vector3> normals;
    
      public: std::vector<double> depths;
  
      public: common::Time time;
    };
    /// \}
  }
}
#endif
