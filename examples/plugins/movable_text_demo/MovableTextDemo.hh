/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#ifndef EXAMPLES_PLUGINS_MOVABLETEXTDEMO_HH_
#define EXAMPLES_PLUGINS_MOVABLETEXTDEMO_HH_

#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#include <gazebo/rendering/MovableText.hh>

namespace gazebo
{
    class GAZEBO_VISIBLE MovableTextDemo : public GUIPlugin
    {
      Q_OBJECT

      /// \brief Constructor
      /// \param[in] _parent Parent widget
      public: MovableTextDemo();

      /// \brief Destructor
      public: virtual ~MovableTextDemo();

      // Documentation inherited
      public: virtual void Load(sdf::ElementPtr _elem) override;

      /// \brief Counter used to create unique model names
      private: rendering::MovableText text;
    };
}
#endif
