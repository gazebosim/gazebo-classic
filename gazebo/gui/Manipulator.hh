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
#ifndef _MODELMANIPULATOR_HH_
#define _MODELMANIPULATOR_HH_

#include <string>
#include "gazebo/rendering/RenderTypes.hh"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class Manipulator Manipulator.hh
    /// \brief Interactive manipulator for models and links
    class Manipulator
    {
      /// \brief Constructor
      public: Manipulator();
      /// \brief Deconstructor
      public: virtual ~Manipulator();

      public: void Load();

      public: void Attach(rendering::VisualPtr _vis);

      public: void Attach(std::string _entity);

      private: rendering::VisualPtr transVisual;
      private: rendering::VisualPtr scaleVisual;

      private: rendering::VisualPtr rotVisual;
      private: rendering::VisualPtr rotXVisual;
      private: rendering::VisualPtr rotYVisual;
      private: rendering::VisualPtr rotZVisual;

      private: rendering::VisualPtr manipVisual;

      private: rendering::AxisVisualPtr axisVisual;

      private: static int counter;

      private: std::string name;

      private: rendering::VisualPtr parent;

    };
  }
}

#endif
