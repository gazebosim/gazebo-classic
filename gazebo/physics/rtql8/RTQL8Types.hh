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

#ifndef _RTQL8TYPES_H_
#define _RTQL8TYPES_H_

#include <boost/shared_ptr.hpp>

/// \file
/// \ingroup gazebo_physics
/// \ingroup gazebo_physics_rtql8
/// \brief RTQL8 wrapper forward declarations and typedefs
namespace gazebo
{
  namespace physics
  {
    //class BulletCollision;
    class RTQL8Link;
    class RTQL8Physics;
    //class BulletRayShape;

    typedef boost::shared_ptr<RTQL8Physics> RTQL8PhysicsPtr;
    //typedef boost::shared_ptr<BulletCollision> BulletCollisionPtr;
    typedef boost::shared_ptr<RTQL8Link> RTQL8LinkPtr;
    //typedef boost::shared_ptr<BulletRayShape> BulletRayShapePtr;
  }
}

#endif
