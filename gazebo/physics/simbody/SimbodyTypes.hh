/*
 * Copyright 2012 Open Source Robotics Foundation
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
/// \ingroup gazebo_physics_simbody
/// \brief Simbody wrapper forward declarations and typedefs
namespace gazebo
{
  namespace physics
  {
    class SimbodyCollision;
    class SimbodyLink;
    class SimbodyModel;
    class SimbodyPhysics;
    class SimbodyRayShape;

    typedef boost::shared_ptr<SimbodyPhysics> SimbodyPhysicsPtr;
    typedef boost::shared_ptr<SimbodyCollision> SimbodyCollisionPtr;
    typedef boost::shared_ptr<SimbodyLink> SimbodyLinkPtr;
    typedef boost::shared_ptr<SimbodyModel> SimbodyModelPtr;
    typedef boost::shared_ptr<SimbodyRayShape> SimbodyRayShapePtr;
  }
}
