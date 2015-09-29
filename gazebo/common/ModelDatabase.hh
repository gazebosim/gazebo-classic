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
#ifndef _GAZEBO_MODELDATABSE_HH_
#define _GAZEBO_MODELDATABSE_HH_

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
    class ModelDatabasePrivate;

    /// \addtogroup gazebo_common Common
    /// \{

    /// \class ModelDatabase ModelDatabase.hh common/common.hh
    /// \brief Connects to model database, and has utility functions to find
    /// models.
    class GAZEBO_VISIBLE ModelDatabase : public SingletonT<ModelDatabase>
    {
      /// \brief Constructor. This will update the model cache
      private: ModelDatabase();

      /// \brief Destructor
      private: virtual ~ModelDatabase();

      /// \brief Start the model database.
      /// \param[in] _fetchImmediately True to fetch the models without
      /// waiting.
      public: void Start(bool _fetchImmediately = false);

      /// \brief Finalize the model database.
      public: void Fini();

      /// \brief Returns the the global model database URI.
      /// \return the URI.
      public: std::string GetURI();

      /// \brief Returns the dictionary of all the model names
      ///
      /// This is a blocking call. Which means it will wait for the
      /// ModelDatabase to download the model list.
      /// \return a map of model names, indexed by their full URI.
      public: std::map<std::string, std::string> GetModels();

      /// \brief Get the dictionary of all model names via a callback.
      ///
      /// This is the non-blocking version of ModelDatabase::GetModels
      /// \param[in] _func Callback function that receives the list of
      /// models.
      /// \return A boost shared pointer. This pointer must remain valid in
      /// order to receive the callback.
      public: event::ConnectionPtr  GetModels(boost::function<
                  void (const std::map<std::string, std::string> &)> _func);

      /// \brief Get the name of a model based on a URI.
      ///
      /// The URI must be fully qualified:
      /// http://gazebosim.org/gazebo_models/ground_plane or
      /// model://gazebo_models
      /// \param[in] _uri the model uri
      /// \return the model's name.
      public: std::string GetModelName(const std::string &_uri);

      /// \brief Return the model.config file as a string.
      /// \return The model config file from the model database.
      public: std::string GetModelConfig(const std::string &_uri);

      /// \brief Return the database.config file as a string.
      /// \return The database config file from the model database.
      public: std::string GetDBConfig(const std::string &_uri);

      /// \brief Get the local path to a model.
      ///
      /// Get the path to a model based on a URI. If the model is on
      /// a remote server, then the model fetched and installed locally.
      /// \param[in] _uri the model uri
      /// \param[in] _forceDownload True to skip searching local paths.
      /// \return path to a model directory
      public: std::string GetModelPath(const std::string &_uri,
                  bool _forceDownload = false);

      /// \brief Get a model's SDF file based on a URI.
      ///
      /// Get a model file based on a URI. If the model is on
      /// a remote server, then the model fetched and installed locally.
      /// \param[in] _uri The URI of the model
      /// \return The full path and filename to the SDF file
      public: std::string GetModelFile(const std::string &_uri);

      /// \brief Download all dependencies for a give model path
      ///
      /// Look's in the model's manifest file (_path/model.config)
      /// for all models listed in the <depend> block, and downloads the
      /// models if necessary.
      /// \param[in] _path Path to a model.
      public: void DownloadDependencies(const std::string &_path);

      /// \brief Returns true if the model exists on the database.
      ///
      /// \param[in] _modelName URI of the model (eg:
      /// model://my_model_name).
      /// \return True if the model was found.
      public: bool HasModel(const std::string &_modelName);

      /// \brief A helper function that uses CURL to get a manifest file.
      /// \param[in] _uri URI of a manifest XML file.
      /// \return The contents of the manifest file.
      private: std::string GetManifestImpl(const std::string &_uri);

      /// \brief Used by a thread to update the model cache.
      /// \param[in] _fetchImmediately True to fetch the models without
      /// waiting.
      private: void UpdateModelCache(bool _fetchImmediately);

      /// \brief Used by ModelDatabase::UpdateModelCache,
      /// no one else should use this function.
      private: bool UpdateModelCacheImpl();

      /// \brief Private data.
      private: ModelDatabasePrivate *dataPtr;

      /// \brief Singleton implementation
      private: friend class SingletonT<ModelDatabase>;

      /// \brief Handy trick to automatically call a singleton's
      /// constructor.
      private: static ModelDatabase *myself;
    };
  }
}
#endif
