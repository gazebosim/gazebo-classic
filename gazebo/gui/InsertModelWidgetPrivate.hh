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
#ifndef _INSERT_MODEL_WIDGET_PRIVATE_HH_
#define _INSERT_MODEL_WIDGET_PRIVATE_HH_

#include <string>
#include <map>
#include <set>
#include <boost/thread/mutex.hpp>

#include "gazebo/common/Event.hh"
#include "gazebo/util/system.hh"

class QTreeWidget;
class QTreeWidgetItem;
class QFileSystemWatcher;

namespace gazebo
{
  namespace gui
  {
    /// \brief Private class attributes for InsertModelWidget.
    class GAZEBO_VISIBLE InsertModelWidgetPrivate
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
    };
  }
}
#endif
