/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifdef WINDOWS
  #include <direct.h>
  #define GetCurrentDir _getcwd
#else
  #include <unistd.h>
  #define GetCurrentDir getcwd
#endif

#include <string>
#include <list>
#include <ignition/common.hh>

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
    class SystemPaths : public ignition::common::SystemPaths,
                        public ignition::common::SingletonT<SystemPaths>
    {
      /// Constructor for SystemPaths
      private: SystemPaths();

      /// \brief Get the gazebo install paths
      /// \return a list of paths
      public: const std::list<std::string> &GetGazeboPaths();

      /// \brief Get the ogre install paths
      /// \return a list of paths
      public: const std::list<std::string> &GetOgrePaths();

      /// \brief Get the model paths
      /// \return a list of paths
      public: const std::list<std::string> &GetModelPaths();

      /// Returns the world path extension.
      /// \return Right now, it just returns "/worlds"
      public: std::string GetWorldPathExtension();

      /// \brief Add colon delimited paths to Gazebo install
      /// \param[in] _path the directory to add
      public: void AddGazeboPaths(const std::string &_path);

      /// \brief Add colon delimited paths to modelPaths
      /// \param[in] _path the directory to add
      public: void AddModelPaths(const std::string &_path);

      /// \brief Add colon delimited paths to ogre install
      /// \param[in] _path the directory to add
      public: void AddOgrePaths(const std::string &_path);

      /// \brief clear out SystemPaths#gazeboPaths
      public: void ClearGazeboPaths();

      /// \brief clear out SystemPaths#modelPaths
      public: void ClearModelPaths();

      /// \brief clear out SystemPaths#ogrePaths
      public: void ClearOgrePaths();

      /// \brief Find a file or path using a URI
      /// \param[in] _uri the uniform resource identifier
      /// \return Returns full path name to file
      protected: virtual std::string FindFileURIHelper(
                     const std::string &_uri);

      /// \brief Find a file in the gazebo paths
      /// \param[in] _filename Name of the file to find.
      /// \return Returns full path name to file
      protected: virtual std::string FindFileHelper(
                     const std::string &_filename);

      // Documentation inherited
      protected: virtual void UpdatePluginPaths();

      /// \brief re-read SystemPaths#gazeboPaths from environment variable
      private: void UpdateModelPaths();

      /// \brief re-read SystemPaths#gazeboPaths from environment variable
      private: void UpdateGazeboPaths();

      /// \brief re-read SystemPaths#ogrePaths from environment variable
      private: void UpdateOgrePaths();

      /// \brief Paths to installed gazebo media files
      private: std::list<std::string> gazeboPaths;

      /// \brief Paths to the ogre install
      private: std::list<std::string> ogrePaths;

      private: std::list<std::string> modelPaths;

      /// \brief if true, call UpdateGazeboPaths() within GetGazeboPaths()
      private: bool modelPathsFromEnv;

      /// \brief if true, call UpdateGazeboPaths() within GetGazeboPaths()
      private: bool gazeboPathsFromEnv;

      /// \brief if true, call UpdateOgrePaths() within GetOgrePaths()
      private: bool ogrePathsFromEnv;

      private: friend class ignition::common::SingletonT<SystemPaths>;
    };
    /// \}
  }
}
#endif
