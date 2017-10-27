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

#include <functional>
#include <map>
#include <memory>
#include <string>

#include "gazebo/common/Event.hh"
#include "gazebo/util/system.hh"

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
    {
      /// \brief Constructor. This will update the model cache
      /// \param[in] _server The Ignition Fuel server.
      public: FuelModelDatabase(const std::string &_server);

      /// \brief Destructor
      public: virtual ~FuelModelDatabase();

      /// \brief Get the dictionary of all model names via a callback.
      ///
      /// This is the non-blocking version of ModelDatabase::GetModels
      /// \param[in] _func Callback function that receives the list of models.
      /// \return A boost shared pointer. This pointer must remain valid in
      /// order to receive the callback.
      public: void Models(
        std::function<void(const std::map<std::string, std::string> &)> _func);

      /// \brief Private data.
      private: std::unique_ptr<FuelModelDatabasePrivate> dataPtr;
    };
  }
}
#endif
