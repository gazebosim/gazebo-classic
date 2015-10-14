/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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

#ifndef _GAZEBO_INERTIAVISUAL_HH_
#define _GAZEBO_INERTIAVISUAL_HH_

#include <string>

#include "gazebo/math/Pose.hh"
#include "gazebo/rendering/Visual.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering Rendering
    /// \{

    /// \class InertiaVisual InertiaVisual.hh rendering/rendering.hh
    /// \brief Basic Inertia visualization
    class GZ_RENDERING_VISIBLE InertiaVisual : public Visual
    {
      /// \brief Constructor
      /// \param[in] _name Name of the Visual
      /// \param[in] _vis Parent Visual
      public: InertiaVisual(const std::string &_name, VisualPtr _vis);

      /// \brief Destructor
      public: ~InertiaVisual();

      // Inherited from parent class
      public: virtual void Fini();

      /// \brief Load the Visual from an SDF pointer
      /// \param[in] _elem SDF Element pointer
      public: virtual void Load(sdf::ElementPtr _elem);
      using Visual::Load;

      /// \brief Load from a message
      /// \param[in] _msg Pointer to the message
      public: virtual void Load(ConstLinkPtr &_msg);

      /// \brief Load based on a math::Pose
      /// \param[in] _pose Pose of the Inertia visual
      /// \param[in] _scale Scale factor for the box visual.
      private: void Load(const math::Pose &_pose,
          const math::Vector3 &_scale = math::Vector3(0.02, 0.02, 0.02));

      /// \brief Destroy all the movable objects attached to a scene node.
      /// \param[in] _sceneNode Pointer to the scene node to process.
      private: void DestroyAllAttachedMovableObjects(
                        Ogre::SceneNode *_sceneNode);
    };
    /// \}
  }
}
#endif
