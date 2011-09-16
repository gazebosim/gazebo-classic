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
/* Desc: A ray
 * Author: Nate Keonig, Andrew Howard
 * Date: 14 Oct 2009
 * SVN: $Id:$
 */

#ifndef ODERAYSHAPE_HH
#define ODERAYSHAPE_HH

#include "physics/RayShape.hh"
#include "physics/Shape.hh"

namespace gazebo
{
	namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{
    
    /// \addtogroup gazebo_physics_ode ODE Physics
    /// \{

    /// \brief Ray collision 
    class ODERayShape : public RayShape
    {
      /// \brief Constructor
      /// \param body Link the ray is attached to
      /// \param displayRays Indicates if the rays should be displayed when rendering images
      public: ODERayShape( CollisionPtr collision, bool displayRays );
    
      /// \brief Destructor
      public: virtual ~ODERayShape();
  
      /// \brief Update the tay collision
      public: virtual void Update();
   
      /// \brief Set the ray based on starting and ending points relative to the 
      ///        body
      /// \param posStart Start position, relative the body
      /// \param posEnd End position, relative to the body
      public: virtual void SetPoints(const math::Vector3 &posStart, 
                                     const math::Vector3 &posEnd);
    };
    /// \}
    /// \}
  }
}
#endif
