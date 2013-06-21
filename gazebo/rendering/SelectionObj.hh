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
#ifndef _SELECTION_OBJ_
#define _SELECTION_OBJ_

#include <string>

#include "gazebo/math/Vector3.hh"
#include "gazebo/rendering/RenderTypes.hh"

namespace gazebo
{
  namespace rendering
  {
    class Scene;

    /// \addtogroup gazebo_rendering
    /// \{

    /// \class SelectionObj SelectionObj.hh rendering/rendering.hh
    /// \brief A graphical selection object
    ///
    /// Used to draw a visual around a selected object.
    class SelectionObj
    {
      /// \brief Constructor
      /// \param[in] _scene Scene to use.
      public: SelectionObj(Scene *_scene);

      /// \brief Destructor
      public: virtual ~SelectionObj();

      /// \brief Initialize the rendering::SelectionObj object
      public: void Init();

      /// \brief Set the position of the node
      /// \param[in] This draws the selection object around the passed in
      /// visual.
      public: void Attach(VisualPtr _visual);

      /// \brief Clear the rendering::SelectionObj object
      public: void Clear();

      /// \brief Return true if the user is move the selection obj
      /// \return True if something is selected.
      public: bool IsActive() const;

      /// \brief Set true if the user is moving the selection obj
      /// \param[in] _active True if the user is interacting with the
      /// selection object.
      public: void SetActive(bool _active);

      /// \brief Get the name of the visual the selection obj is attached to
      /// \return Name of the selected visual.
      public: std::string GetVisualName() const;

      /// \brief Highlight the selection object based on a modifier
      /// \param[in] _mod Modifier used when highlighting the selection
      /// object.
      public: void SetHighlight(const std::string &_mod);

      /// \brief The visual node for the selection object
      private: VisualPtr node;

      /// \brief Pointer to the scene
      private: Scene *scene;

      /// \brief Name of the visual selected.
      private: std::string visualName;

      /// \brief True if a user is interacting with the selection object.
      private: bool active;

      /// \brief Size of the selection object box.
      private: double boxSize;
    };
    /// \}
  }
}
#endif
