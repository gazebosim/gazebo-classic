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

#ifndef _BUILDING_ITEM_HH_
#define _BUILDING_ITEM_HH_

#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace gui
  {
    /// \addtogroup gazebo_gui
    /// \{

    /// \class BuildingItem BuildingItem.hh
    /// \brief Base class of a items that have building level properties such as
    /// the level number and level height.
    class GZ_GUI_VISIBLE BuildingItem
    {
        /// \brief Constructor
        public: BuildingItem();

        /// \brief Destructor
        public: ~BuildingItem();
    };
    /// \}
  }
}

#endif
