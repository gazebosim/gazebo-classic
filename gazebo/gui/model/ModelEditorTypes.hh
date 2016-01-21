/*
 * Copyright (C) 2015-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_MODEL_EDITOR_TYPES_HH_
#define _GAZEBO_MODEL_EDITOR_TYPES_HH_

#include <memory>

namespace gazebo
{
  /// \brief GUI model editor forward declarations and type defines
  namespace gui
  {
    class EditorMaterialSwitcher;
    class MEUserCmd;

    /// \def EditorMaterialSwitcherPtr
    /// \brief Shared pointer to a EditorMaterialSwitcherPtr object
    using EditorMaterialSwitcherPtr = std::shared_ptr<EditorMaterialSwitcher>;

    /// \def MEUserCmdPtr
    /// \brief Shared pointer to a MEUserCmdPtr object
    using MEUserCmdPtr = std::shared_ptr<MEUserCmd>;
  }
}

#endif
