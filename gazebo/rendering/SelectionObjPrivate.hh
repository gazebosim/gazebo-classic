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
#ifndef _SELECTIONOBJ_PRIVATE_HH_
#define _SELECTIONOBJ_PRIVATE_HH_

#include <string>

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/rendering/VisualPrivate.hh"
#include "gazebo/rendering/SelectionObj.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \brief Private data for the Selection Obj class.
    class SelectionObjPrivate : public VisualPrivate
    {
      /// \brief Translation visual.
      public: VisualPtr transVisual;

      /// \brief X translation visual.
      public: VisualPtr transXVisual;

      /// \brief Y translation visual.
      public: VisualPtr transYVisual;

      /// \brief Z translation visual.
      public: VisualPtr transZVisual;

      /// \brief Scale visual.
      public: VisualPtr scaleVisual;

      /// \brief X scale visual.
      public: VisualPtr scaleXVisual;

      /// \brief Y Scale visual.
      public: VisualPtr scaleYVisual;

      /// \brief Z scale visual.
      public: VisualPtr scaleZVisual;

      /// \brief Rotation visual.
      public: VisualPtr rotVisual;

      /// \brief X rotation visual.
      public: VisualPtr rotXVisual;

      /// \brief Y rotation visual.
      public: VisualPtr rotYVisual;

      /// \brief Z rotation visual.
      public: VisualPtr rotZVisual;

      /// \brief Current manipulation mode.
      public: SelectionObj::SelectionMode mode;

      /// \brief Current selection state.
      public: SelectionObj::SelectionMode state;

      /// \brief Pointer to visual that is currently selected.
      public: VisualPtr selectedVis;

      /// \brief Minimum scale of the selection object visual.
      public: double minScale;

      /// \brief Maximum scale of the selection object visual.
      public: double maxScale;

      /// \brief Material name for the x axis.
      public: std::string xAxisMat;

      /// \brief Material name for the y axis.
      public: std::string yAxisMat;

      /// \brief Material name for the z axis.
      public: std::string zAxisMat;

      /// \brief Overlay material name for the x axis.
      public: std::string xAxisMatOverlay;

      /// \brief Overlay material name for the y axis.
      public: std::string yAxisMatOverlay;

      /// \brief Overlay material name for the z axis.
      public: std::string zAxisMatOverlay;
    };
  }
}

#endif
