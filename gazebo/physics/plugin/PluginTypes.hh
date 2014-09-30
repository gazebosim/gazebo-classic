/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#include "gazebo/util/system.hh"

/// \file
/// \brief Plugin wrapper forward declarations and typedefs
namespace gazebo
{
  namespace physics
  {
    class PluginCollision;
    class PluginJoint;
    class PluginLink;
    class PluginRayShape;
    class PluginSurfaceParams;
    class PluginPhysics;

    /// \def PluginCollisionPtr
    /// \brief Boost shared point to PluginCollision
    typedef boost::shared_ptr<PluginCollision> PluginCollisionPtr;

    /// \def PluginJointPtr
    /// \brief Boost shared point to PluginJoint
    typedef boost::shared_ptr<PluginJoint> PluginJointPtr;

    /// \def PluginLinkPtr
    /// \brief Boost shared point to PluginLink
    typedef boost::shared_ptr<PluginLink> PluginLinkPtr;

    /// \def PluginRayShapePtr
    /// \brief Boost shared point to PluginRayShape
    typedef boost::shared_ptr<PluginRayShape> PluginRayShapePtr;

    /// \def PluginSurfaceParamsPtr
    /// \brief Boost shared pointer to PluginSurfaceParams
    typedef  boost::shared_ptr<PluginSurfaceParams> PluginSurfaceParamsPtr;

    /// \def PluginPhysicsPtr
    /// \brief Boost shared pointer for PluginPhysics.
    typedef boost::shared_ptr<PluginPhysics> PluginPhysicsPtr;
  }
}
