/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef GAZEBO_GUI_INSERTMODELWIDGETPRIVATE_HH_
#define GAZEBO_GUI_INSERTMODELWIDGETPRIVATE_HH_

#include <map>
#include <memory>
#include <set>
#include <string>
#include <vector>
#include <boost/thread/mutex.hpp>

#ifdef HAVE_IGNITION_FUEL_TOOLS
  #include <ignition/fuel_tools/FuelClient.hh>
  #include <ignition/fuel_tools/ModelIdentifier.hh>
#endif

#include "gazebo/common/Event.hh"
#include "gazebo/util/system.hh"

class QTreeWidget;
class QTreeWidgetItem;
class QFileSystemWatcher;

namespace gazebo
{
  namespace gui
  {
#ifdef HAVE_IGNITION_FUEL_TOOLS
    /// \brief Details to manage an Ignition Fuel server.
    class FuelDatabaseDetails
    {
      /// \brief Tree item that is populated with models from a Fuel server.
      public: QTreeWidgetItem *modelFuelItem = nullptr;

      /// \brief a buffer of models.
      /// It contains elements which uniquely identify models.
      public: std::vector<ignition::fuel_tools::ModelIdentifier> modelBuffer;
    };
#endif

    /// \brief Private class attributes for InsertModelWidget.
    class InsertModelWidgetPrivate
    {
      /// \brief Widget that display all the models that can be inserted.
      public: QTreeWidget *fileTreeWidget;

      /// \brief Tree item that is populated with models from the ModelDatabase.
      public: QTreeWidgetItem *modelDatabaseItem;

      /// \brief Mutex to protect the modelBuffer.
      public: boost::mutex mutex;

      /// \brief Buffer to hold the results from ModelDatabase::GetModels.
      public: std::map<std::string, std::string> modelBuffer;

      /// \brief A file/directory watcher.
      public: QFileSystemWatcher *watcher;

      /// \brief Callback reference count for retrieving models.
      public: event::ConnectionPtr getModelsConnection;

      /// \brief Cache for the names added to fileTreeWidget
      public: std::set<std::string> localFilenameCache;

#ifdef HAVE_IGNITION_FUEL_TOOLS
      /// \brief Stores details about all Fuel servers providing assets.
      /// The key is the server name and the value is the class that captures
      /// multiple information about the server.
      public: std::map<std::string, FuelDatabaseDetails> fuelDetails;

      /// \brief A client for using Ignition Fuel services.
      public: std::unique_ptr<ignition::fuel_tools::FuelClient> fuelClient;
#endif
    };
  }
}
#endif
