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
#ifndef _GAZEBO_COMMON_FUELMODELDATABASE_HH_
#define _GAZEBO_COMMON_FUELMODELDATABASE_HH_

#include <string>
#include <map>
#include <utility>

#include <boost/function.hpp>
#include "gazebo/common/Event.hh"
#include "gazebo/common/SingletonT.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/util/system.hh"

/// \brief The file name of model XML configuration.
#define GZ_MODEL_MANIFEST_FILENAME "model.config"

/// \brief The file name of model database XML configuration.
#define GZ_MODEL_DB_MANIFEST_FILENAME "database.config"

namespace gazebo
{
  namespace common
  {
    /// \brief Forward declare private data class.
    class FuelModelDatabasePrivate;

    /// \addtogroup gazebo_common Common
    /// \{

    /// \class ModelDatabase ModelDatabase.hh common/common.hh
    /// \brief Connects to model database, and has utility functions to find
    /// models.
    class GZ_COMMON_VISIBLE FuelModelDatabase
      : public SingletonT<FuelModelDatabase>
    {
      /// \brief Constructor. This will update the model cache
      private: FuelModelDatabase();

      /// \brief Destructor
      private: virtual ~FuelModelDatabase();

      /// \brief Start the model database.
      /// \param[in] _fetchImmediately True to fetch the models without
      /// waiting.
      public: void Start(bool _fetchImmediately = false);

      /// \brief Returns the the global model database URI.
      /// \return the URI.
      public: std::string GetURI();

      /// \brief Get the dictionary of all model names via a callback.
      ///
      /// This is the non-blocking version of ModelDatabase::GetModels
      /// \param[in] _func Callback function that receives the list of
      /// models.
      /// \return A boost shared pointer. This pointer must remain valid in
      /// order to receive the callback.
      public: event::ConnectionPtr GetModels(boost::function<
                  void (const std::map<std::string, std::string> &)> _func);

      /// \brief Used by a thread to update the model cache.
      /// \param[in] _fetchImmediately True to fetch the models without
      /// waiting.
      private: void UpdateModelCache(bool _fetchImmediately);

      /// \brief Used by ModelDatabase::UpdateModelCache,
      /// no one else should use this function.
      private: bool UpdateModelCacheImpl();

      /// \brief Private data.
      private: FuelModelDatabasePrivate *dataPtr;

      /// \brief Singleton implementation
      private: friend class SingletonT<FuelModelDatabase>;

      /// \brief Handy trick to automatically call a singleton's
      /// constructor.
      private: static FuelModelDatabase *myself;
    };
  }
}
#endif
