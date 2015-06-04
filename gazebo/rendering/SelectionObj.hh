/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifndef _SELECTIONOBJ_HH_
#define _SELECTIONOBJ_HH_

#include <string>

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class SelectionObj SelectionObj.hh
    /// \brief Interactive selection object for models and links
    class GZ_RENDERING_VISIBLE SelectionObj : public Visual
    {
      /// \enum Manipulation modes
      /// \brief Unique identifiers for manipulation modes.
      public: enum SelectionMode
      {
        /// \brief Translation in x
        SELECTION_NONE = 0,
        /// \brief Translation mode
        TRANS,
        /// \brief Rotation mode
        ROT,
        /// \brief Scale mode
        SCALE,
        /// \brief Translation in x
        TRANS_X,
        /// \brief Translation in y
        TRANS_Y,
        /// \brief Translation in z
        TRANS_Z,
        /// \brief Rotation in x
        ROT_X,
        /// \brief Rotation in y
        ROT_Y,
        /// \brief Rotation in z
        ROT_Z,
        /// \brief Scale in x
        SCALE_X,
        /// \brief Scale in y
        SCALE_Y,
        /// \brief Scale in z
        SCALE_Z
      };

      /// \brief Constructor
      /// \param[in] _name Name of selection object.
      /// \param[in] _vis Parent visual that the selection object is
      /// attached to.
      public: SelectionObj(const std::string &_name, VisualPtr _vis);

      /// \brief Deconstructor
      public: virtual ~SelectionObj();

      /// \brief Load
      public: void Load();

      // Documentation inherited.
      public: virtual void Fini();

      /// \brief Attach the selection object to the given visual
      /// \param[in] _vis Pointer to visual to which the selection object
      /// will be attached.
      public: void Attach(rendering::VisualPtr _vis);

      /// \brief Detach the selection object from the current visual.
      public: void Detach();

      /// \brief Set the manipulation mode.
      /// \param[in] _mode Manipulation mode in string: translate rotate, scale.
      public: void SetMode(const std::string &_mode);

      /// \brief Set the selection mode.
      /// \_name Selection mode: TRANS, ROT, SCALE.
      public: void SetMode(SelectionMode _mode);

      /// \brief Set state by highlighting the corresponding selection object
      /// visual.
      /// \param[in] _state Selection state in string format.
      public: void SetState(const std::string &_state);

      /// \brief Set state by highlighting the corresponding selection object
      /// visual.
      /// \param[in] _state Selection state.
      /// \sa SelectionMode
      public: void SetState(SelectionMode _state);

      /// \brief Get the current selection state.
      public: SelectionMode GetState();

      /// \brief Get the current selection mode.
      public: SelectionMode GetMode();

      /// \brief Set selection object to ignore local transforms.
      /// \param[in] _global True to set the visuals to be in global frame.
      public: void SetGlobal(bool _global);

      /// \brief Update selection object size to match the parent visual.
      public: void UpdateSize();

      /// \brief Set the visibility for a specific handle or handle group.
      /// \param[in] _mode Manipulation mode corresponding to the handle.
      /// \param[in] _visible Whether to show or not.
      public: void SetHandleVisible(SelectionMode _mode, bool _visible);

      /// \brief Get the visibility for a specific handle. If a handle group
      /// is specified, the X component's visibility is returned.
      /// \param[in] _mode Manipulation mode corresponding to the handle.
      /// \return Whether it is visible or not.
      public: bool GetHandleVisible(SelectionMode _mode) const;

      /// \brief Set the material for a specific handle or handle group.
      /// \param[in] _mode Manipulation mode corresponding to the handle.
      /// \param[in] _material Material name.
      /// \param[in] _unique True to make the material unique, which
      /// allows the material to change without changing materials that
      /// originally had the same name.
      public: void SetHandleMaterial(SelectionMode _mode, const std::string
          &_material, bool _unique = true);

      /// \brief Helper function to create scale visuals.
      private: void CreateScaleVisual();

      /// \brief Helper function to create rotate visuals.
      private: void CreateRotateVisual();

      /// \brief Helper function to create translate visuals.
      private: void CreateTranslateVisual();
    };
    /// \}
  }
}

#endif
