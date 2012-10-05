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

#include <boost/shared_ptr.hpp>

/// \file
/// \ingroup gazebo_physics
/// \ingroup gazebo_physics_ode
/// \brief ODE wrapper forward declarations and typedefs
namespace gazebo
{
  namespace physics
  {
    class ODECollision;
    class ODELink;
    class ODEPhysics;
    class ODERayShape;

    typedef boost::shared_ptr<ODEPhysics> ODEPhysicsPtr;
    typedef boost::shared_ptr<ODECollision> ODECollisionPtr;
    typedef boost::shared_ptr<ODELink> ODELinkPtr;
    typedef boost::shared_ptr<ODERayShape> ODERayShapePtr;
  }
}
