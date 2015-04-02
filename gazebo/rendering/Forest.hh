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
#ifndef _GAZEBO_FOREST_HH_
#define _GAZEBO_FOREST_HH_

#include "gazebo/common/Event.hh"
#include "gazebo/rendering/ogre_gazebo.h"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

namespace Forests
{
  class PagedGeometry;
}

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class Forest Forest.hh rendering/rendering.hh
    /// \brief Generates a forest using paged geometry
    class GAZEBO_VISIBLE Forest
    {
      /// \brief Constructor
      public: Forest(ScenePtr _scene);

      /// \brief Virtual destructor
      public: virtual ~Forest();

      /// \brief Load forest configurations.
      public: void Load();

      /// \brief Clear all paged geometry objects.
      public: void Clear();

      /// \brief Initializes the scene with forest meshes.
      public: void LoadScene();

      /// \brief Update the scene and camera.
      public: void Update(bool _force);

      /// \brief True if the forest is initialized.
      private: bool initialized;

      /// \brief The forest's event connections.
      private: std::vector<event::ConnectionPtr> connections;

      /// \brief Pointer to a camera
      private: CameraPtr camera;

      /// \brief Pointer to the scene
      private: ScenePtr scene;
           
      /// \brief PagedGeometry tree instances.
      private: Forests::PagedGeometry *trees;

      /// \brief PagedGeometry grass instances.
      private: Forests::PagedGeometry *grass;

      /// \brief PagedGeometry bush instances.
      private: Forests::PagedGeometry *bushes;

    };
    /// \}
  }
}
#endif
