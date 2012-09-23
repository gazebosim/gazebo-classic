/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#ifndef __GAZEBO_MODELDATABSE_HH__
#define __GAZEBO_MODELDATABSE_HH__

#include <string>

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{

    /// \class ModelDatabase ModelDatabase.hh common/ModelDatabase.hh
    /// \brief Connects to model database, and has utility functions to find
    /// models
    class ModelDatabase
    {
      /// \brief Get the local path to a model.
      ///
      /// Get the path to a model based on a URI. If the model is on
      /// a remote server, then the model fetched and installed locally.
      /// \return Path to a model directory
      public: static std::string GetModelPath(const std::string &_uri);

      /// \brief Get a model's SDF file based on a URI
      ///
      /// Get a model file based on a URI. If the model is on
      /// a remote server, then the model fetched and installed locally.
      /// \param _uri The URI of the model
      /// \return The full path and filename to the SDF file
      public: static std::string GetModelFile(const std::string &_uri);
    };
  }
}

#endif
