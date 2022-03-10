/*
 * Copyright (C) 2021 Open Source Robotics Foundation
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
#ifndef GAZEBO_TRANSPORT_TASKGROUP_HH_
#define GAZEBO_TRANSPORT_TASKGROUP_HH_

#include <utility>

// Emit is both a macro in Qt and a function defined by tbb
#undef emit
#include <tbb/tbb.h>
#define emit

namespace gazebo {
  namespace transport {
    class TaskGroup
    {
      public: ~TaskGroup() noexcept
              {
                // Wait for running tasks to finish
                this->taskGroup.wait();
              }

      public: template<class Functor, class... Args> void run(Args&&... args)
      {
        this->taskGroup.run(Functor(std::forward<Args>(args)...));
      }

      private: tbb::task_group taskGroup;
    };
  }
}

#endif
