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
#ifndef _SELECTIONOBJ_HH_
#define _SELECTIONOBJ_HH_

#include <string>
#include <boost/unordered/unordered_map.hpp>

#include "gazebo/rendering/RenderTypes.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class SelectionObj SelectionObj.hh
    /// \brief Interactive selection object for models and links
    class SelectionObj : public Visual
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
        /// \brief Translation and Rotation mode
        TRANS_ROT,
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
      public: SelectionObj(const std::string &_name, VisualPtr _vis);

      /// \brief Deconstructor
      public: virtual ~SelectionObj();

      /// \brief Load
      public: void Load();

      /// \brief Attach the selection object to the given visual
      /// \param[in] _vis Pointer to visual to which the selection object
      /// will be attached.
      public: void Attach(rendering::VisualPtr _vis);

      /// \brief Detach the selection object from the current visual.
      public: void Detach();

      void SetMode(const std::string &_mode);

      void SetMode(SelectionMode _mode);

      /// \brief Set state by highlighting the corresponding selection object
      /// visual.
      /// \param[in] _state Selection state in string format.
      void SetState(const std::string &_state);

      /// \brief Set state by highlighting the corresponding selection object
      /// visual.
      /// \param[in] _state Selection state.
      void SetState(SelectionMode _state);

      /// \brief Get the current selection state.
      SelectionMode GetState();

      /// \brief Get the current selection mode.
      SelectionMode GetMode();

      /// \brief Set selection object to ignore local transforms.
      void SetGlobal(bool _global);

      private: void CreateScaleVisual();
      private: void CreateRotateVisual();
      private: void CreateTranslateVisual();

      private: VisualPtr transVisual;
      private: VisualPtr transXVisual;
      private: VisualPtr transYVisual;
      private: VisualPtr transZVisual;

      private: VisualPtr scaleVisual;
      private: VisualPtr scaleXVisual;
      private: VisualPtr scaleYVisual;
      private: VisualPtr scaleZVisual;

      private: VisualPtr rotVisual;
      private: VisualPtr rotXVisual;
      private: VisualPtr rotYVisual;
      private: VisualPtr rotZVisual;

      private: AxisVisualPtr axisVisual;

      private: static int counter;

      private: std::string name;

      private: SelectionMode mode;

      private: SelectionMode state;

      private: VisualPtr activeVis;

      private: boost::unordered_map<std::string, std::string>
          highlightMaterials;
    };
  }
}

#endif
