/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
#ifndef _GAZEBO_SYSTEMPATHS_HH_
#define _GAZEBO_SYSTEMPATHS_HH_

#include <stdio.h>

#define LINUX
#ifdef _WIN32
  #include <direct.h>
  #define GetCurrentDir _getcwd
#else
  #include <unistd.h>
  #define GetCurrentDir getcwd
#endif

#include <boost/filesystem.hpp>
#include <list>
#include <string>

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/common/Event.hh"
#include "gazebo/common/SingletonT.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{

    /// \class SystemPaths SystemPaths.hh common/common.hh
    /// \brief Functions to handle getting system paths, keeps track of:
    ///        \li SystemPaths#gazeboPaths - media paths containing
    ///            worlds, models, sdf descriptions, material scripts,
    ///            textures.
    ///        \li SystemPaths#ogrePaths - ogre library paths.
    ///            Should point to Ogre RenderSystem_GL.so et. al.
    ///        \li SystemPaths#pluginPaths - plugin library paths
    ///            for common::WorldPlugin
    class GZ_COMMON_VISIBLE SystemPaths : public SingletonT<SystemPaths>
    {
      /// Constructor for SystemPaths
      private: SystemPaths();

      /// \brief Get the log path
      /// \return the path
      public: std::string GetLogPath() const;

      /// \brief Get the gazebo install paths
      /// \return a list of paths
      public: const std::list<std::string> &GetGazeboPaths();

      /// \brief Get the ogre install paths
      /// \return a list of paths
      public: const std::list<std::string> &GetOgrePaths();

      /// \brief Get the plugin paths
      /// \return a list of paths
      public: const std::list<std::string> &GetPluginPaths();

      /// \brief Get the model paths
      /// \return a list of paths
      public: const std::list<std::string> &GetModelPaths();

      /// Returns the world path extension.
      /// \return Right now, it just returns "/worlds"
      public: std::string GetWorldPathExtension();

      /// Returns the default path suitable for temporary files.
      /// \return a full path name to directory.
      /// E.g.: /tmp (Linux).
      /// \deprecated See const std::string &TmpPath() const
      public: std::string GetTmpPath() GAZEBO_DEPRECATED(8.0);

      /// Returns the default path suitable for temporary files.
      /// \return a full path name to directory.
      /// E.g.: /tmp (Linux).
      public: const std::string &TmpPath() const;

      /// Returns a unique temporary file for this instance of SystemPath.
      /// \return a full path name to directory.
      /// E.g.: /tmp/gazebo_234123 (Linux).
      /// \deprecated See const std::string &TmpInstancePath() const
      public: std::string GetTmpInstancePath() GAZEBO_DEPRECATED(8.0);

      /// Returns a unique temporary file for this instance of SystemPath.
      /// \return a full path name to directory.
      /// E.g.: /tmp/gazebo_234123 (Linux).
      public: const std::string &TmpInstancePath() const;

      /// Returns the default temporary test path.
      /// \return a full path name to directory.
      /// E.g.: /tmp/gazebo_test (Linux).
      /// \deprecated See std::string DefaultTestPath() const
      public: std::string GetDefaultTestPath() GAZEBO_DEPRECATED(8.0);

      /// Returns the default temporary test path.
      /// \return a full path name to directory.
      /// E.g.: /tmp/gazebo_test (Linux).
      public: std::string DefaultTestPath() const;

      /// \brief Find a file or path using a URI
      /// \param[in] _uri the uniform resource identifier
      /// \return Returns full path name to file
      public: std::string FindFileURI(const std::string &_uri);

      /// \brief Find a file in the gazebo paths
      /// \param[in] _filename Name of the file to find.
      /// \param[in] _searchLocalPath True to search in the current working
      /// directory.
      /// \return Returns full path name to file
      public: std::string FindFile(const std::string &_filename,
                                   bool _searchLocalPath = true);

      /// \brief Add colon delimited paths to Gazebo install
      /// \param[in] _path the directory to add
      public: void AddGazeboPaths(const std::string &_path);

      /// \brief Add colon delimited paths to modelPaths
      /// \param[in] _path the directory to add
      public: void AddModelPaths(const std::string &_path);

      /// \brief Add colon delimited paths to modelPaths and signal the update
      /// to InsertModelWidget
      /// \param[in] _path Path to be added to the current model path
      public: void AddModelPathsUpdate(const std::string &_path);

      /// \brief Add colon delimited paths to ogre install
      /// \param[in] _path the directory to add
      public: void AddOgrePaths(const std::string &_path);

      /// \brief Add colon delimited paths to plugins
      /// \param[in] _path the directory to add
      public: void AddPluginPaths(const std::string &_path);

      /// \brief clear out SystemPaths#gazeboPaths
      public: void ClearGazeboPaths();

      /// \brief clear out SystemPaths#modelPaths
      public: void ClearModelPaths();

      /// \brief clear out SystemPaths#ogrePaths
      public: void ClearOgrePaths();

      /// \brief clear out SystemPaths#pluginPaths
      public: void ClearPluginPaths();

      /// \brief add _suffix to the list of path search suffixes
      /// \param[in] _suffix The suffix to add
      public: void AddSearchPathSuffix(const std::string &_suffix);

      /// \brief re-read SystemPaths#gazeboPaths from environment variable
      private: void UpdateModelPaths();

      /// \brief re-read SystemPaths#gazeboPaths from environment variable
      private: void UpdateGazeboPaths();

      /// \brief re-read SystemPaths#pluginPaths from environment variable
      private: void UpdatePluginPaths();

      /// \brief re-read SystemPaths#ogrePaths from environment variable
      private: void UpdateOgrePaths();

      /// \brief adds a path to the list if not already present
      /// \param[in]_path the path
      /// \param[in]_list the list
      private: void InsertUnique(const std::string &_path,
                                 std::list<std::string> &_list);

      /// \brief Paths to installed gazebo media files
      private: std::list<std::string> gazeboPaths;

      /// \brief Paths to the ogre install
      private: std::list<std::string> ogrePaths;

      /// \brief Paths to plugins
      private: std::list<std::string> pluginPaths;

      private: std::list<std::string> suffixPaths;

      private: std::list<std::string> modelPaths;

      private: std::string logPath;

      /// \brief Event to notify InsertModelWidget that the model paths were
      /// changed.
      public: event::EventT<void (std::string)> updateModelRequest;

      /// \brief if true, call UpdateGazeboPaths() within GetGazeboPaths()
      public: bool modelPathsFromEnv;

      /// \brief if true, call UpdateGazeboPaths() within GetGazeboPaths()
      public: bool gazeboPathsFromEnv;

      /// \brief if true, call UpdatePluginPaths() within GetPluginPaths()
      public: bool pluginPathsFromEnv;

      /// \brief if true, call UpdateOgrePaths() within GetOgrePaths()
      public: bool ogrePathsFromEnv;

      private: friend class SingletonT<SystemPaths>;

      /// \brief Path to the default temporary directory
      private: boost::filesystem::path tmpPath;

      /// \brief Path to the instance temporary directory
      private: boost::filesystem::path tmpInstancePath;
    };
    /// \}
  }
}
#endif
