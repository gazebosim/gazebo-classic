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
#ifndef _TASK_GROUP_HH_
#define _TASK_GROUP_HH_

#include <utility>
#include <tbb/version.h>

// tbb::task was removed in v2021.01, so we need a workaround
#if TBB_VERSION_MAJOR >= 2021

// Emit is both a macro in Qt and a function defined by tbb
#undef emit
#include <tbb/task_group.h>
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
#else
#include <tbb/task.h>

namespace gazebo {
  namespace transport {
    class TaskGroup
    {
      /// \brief A helper class which provides the requisite execute() method
      /// required by tbb.
      private: template<class T> class TaskWrapper : public tbb::task
      {
        public: template<class... Args> TaskWrapper<T>(Args&&... args)
          : functor(std::forward<Args>(args)...)
        {
        }

        public: tbb::task *execute()
                {
                  this->functor();
                  return nullptr;
                }
        
        private: T functor;
      };

      public: template<class Functor, class... Args> void run(Args&&... args)
              {
                TaskWrapper<Functor> *task = new (tbb::task::allocate_root())
                    TaskWrapper<Functor>(std::forward<Args>(args)...);
                tbb::task::enqueue(*task);
              }
    };
  }
}

#endif
#endif
