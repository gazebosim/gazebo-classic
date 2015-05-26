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
#ifndef _GAZEBO_MODELDATABSE_PRIVATE_HH_
#define _GAZEBO_MODELDATABSE_PRIVATE_HH_

#include <list>
#include <map>
#include <string>

#include <boost/thread.hpp>
#include <boost/thread/mutex.hpp>

#include "gazebo/common/Event.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    /// \brief Private class attributes for ModelDatabase.
    class GZ_COMMON_VISIBLE ModelDatabasePrivate
    {
      /// \brief Thread to update the model cache.
      public: boost::thread *updateCacheThread;

      /// \brief A dictionary of all model names indexed by their uri.
      public: std::map<std::string, std::string> modelCache;

      /// \brief True to stop the background thread
      public: bool stop;

      /// \brief Cache update mutex.
      public: boost::mutex updateMutex;

      /// \brief Protects callback list.
      public: boost::mutex callbacksMutex;

      /// \brief Mutex to protect cache thread status checks.
      public: boost::recursive_mutex startCacheMutex;

      /// \brief Condition variable for the updateCacheThread.
      public: boost::condition_variable updateCacheCondition;

      /// \brief Condition variable for completion of one cache update.
      public: boost::condition_variable updateCacheCompleteCondition;

      /// \def CallbackFunc
      /// \brief Boost function that is used to passback the model cache.
      public: typedef boost::function<
               void (const std::map<std::string, std::string> &)> CallbackFunc;

      /// \brief Triggered when the model data has been updated after
      /// calling ModelDatabase::GetModels()
      public: event::EventT<
               void (std::map<std::string, std::string>)> modelDBUpdated;
    };
  }
}
#endif
