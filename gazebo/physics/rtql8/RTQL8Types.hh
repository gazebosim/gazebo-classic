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
    class RTQL8Physics;
    class RTQL8Model;
    class RTQL8Link;
    class RTQL8Joint;
    class RTQL8Collision;
    class RTQL8RayShape;

    /// \def RTQL8PhysicsPtr
    /// \brief Boost shared pointer for RTQL8Physics.
    typedef boost::shared_ptr<RTQL8Physics> RTQL8PhysicsPtr;

    /// \def RTQL8LinkPtr
    /// \brief Boost shared point to RTQL8Link
    typedef boost::shared_ptr<RTQL8Model> RTQL8ModelPtr;

    /// \def RTQL8LinkPtr
    /// \brief Boost shared point to RTQL8Link
    typedef boost::shared_ptr<RTQL8Link> RTQL8LinkPtr;

    /// \def RTQL8JointPtr
    /// \brief Boost shared point to RTQL8Joint
    typedef boost::shared_ptr<RTQL8Joint> RTQL8JointPtr;

    /// \def RTQL8CollisionPtr
    /// \brief Boost shared point to RTQL8Collision
    typedef boost::shared_ptr<RTQL8Collision> RTQL8CollisionPtr;

    /// \def RTQL8RayShapePtr
    /// \brief Boost shared point to RTQL8RayShape
    typedef boost::shared_ptr<RTQL8RayShape> RTQL8RayShapePtr;
  }
}

#endif
