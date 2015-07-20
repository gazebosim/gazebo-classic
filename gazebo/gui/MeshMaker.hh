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
#ifndef _GAZEBO_MESHMAKER_HH_
#define _GAZEBO_MESHMAKER_HH_

#include <string>
#include "gazebo/gui/EntityMaker.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace msgs
  {
    class Visual;
  }

  namespace gui
  {
    class GAZEBO_VISIBLE MeshMaker : public EntityMaker
    {
      /// \brief Cosntructor
      public: MeshMaker();

      /// \brief Destructor
      public: virtual ~MeshMaker();

      // Documentation inherited
      public: void Init(const std::string &_filename);

      // Documentation inherited
      public: virtual void Start(const rendering::UserCameraPtr _camera);

      // Documentation inherited
      public: virtual void Stop();

      // Documentation inherited
      public: virtual bool IsActive() const;

      // Documentation inherited
      private: virtual void CreateTheEntity();

      private: int state;
      private: msgs::Visual *visualMsg;

      private: static unsigned int counter;
    };
  }
}
#endif
