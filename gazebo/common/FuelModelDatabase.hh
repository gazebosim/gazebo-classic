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
#ifndef GAZEBO_COMMON_FUELMODELDATABASE_HH_
#define GAZEBO_COMMON_FUELMODELDATABASE_HH_

#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gazebo/common/SingletonT.hh"
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

    /// \class FuelModelDatabase FuelModelDatabase.hh common/common.hh
    /// \brief Connects to an Igniiton Fuel model database, and has utility
    /// functions to find models.
    class GZ_COMMON_VISIBLE FuelModelDatabase
      : public SingletonT<FuelModelDatabase>
    {
      /// \brief Constructor.
      private: FuelModelDatabase();

      /// \brief Destructor
      private: virtual ~FuelModelDatabase();

      /// \brief Get all the Ignition Fuel servers.
      /// \return A collection of Ignition Fuel servers.
      public: std::vector<std::string> Servers() const;

      /// \brief Get the dictionary of all model names via a callback.
      ///
      /// This is a non-blocking function. Your callback will be executed from
      /// a separate thread.
      /// \param[in] _server The Ignition Fuel server URL.
      /// \param[in] _func Callback function that receives the list of models.
      /// The parameter of the callback is a map, where the key is the unique
      /// name (containing the full path in the server, owner and model name)
      /// and the value is the model name.
      /// E.g.: https://api.ignitionfuel.org/1.0/caguero/models/Beer -> Beer
      public: virtual void Models(const std::string &_server,
        std::function<void(const std::map<std::string, std::string> &)> &_func);

      /// \brief Get the dictionary of all model names.
      ///
      /// This is a blocking function.
      /// \param[in] _server The Ignition Fuel server URL.
      /// \return The list of models.
      /// The key of the returned map is the unique name (containing the full
      /// path in the server, owner and model name) and the value is the
      /// model name.
      /// E.g.: https://api.ignitionfuel.org/1.0/caguero/models/Beer -> Beer
      public: virtual std::map<std::string, std::string> Models(
        const std::string &_server) const;

      /// \brief Get a model's SDF file based on a URI.
      ///
      /// Get a model file based on a URI. If the model is on
      /// a remote server, then the model is fetched and installed locally.
      /// \param[in] _uri The URI of the model.
      /// \return The full path and filename to the SDF file.
      public: std::string ModelFile(const std::string &_uri);

      /// \brief Get the local path to a model.
      ///
      /// Get the path to a model based on a URI. If the model is on
      /// a remote server, then the model fetched and installed locally.
      /// \param[in] _uri the model uri
      /// \param[in] _forceDownload True to skip searching local paths.
      /// \return path to a model directory
      public: std::string ModelPath(const std::string &_uri,
        bool _forceDownload = false);


      /// \brief Private data.
      private: std::unique_ptr<FuelModelDatabasePrivate> dataPtr;

      /// \brief Singleton implementation
      private: friend class SingletonT<FuelModelDatabase>;

      /// \brief Handy trick to automatically call a singleton's constructor.
      private: static FuelModelDatabase *myself;
    };
  }
}
#endif
