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
/* Desc: Gazebo configuration on this computer 
 * Author: Jordi Polo
 * Date: 3 May 2008
 */

#ifndef GAZEBOCONFIG_HH
#define GAZEBOCONFIG_HH

#include <string>
#include <list>

#include "common/SingletonT.hh"

namespace gazebo
{
	namespace common
{

/// \addtogroup gazebo_server
/// \brief Local configuration on this computer about how gazebo server should work 
/// \{

  class GazeboConfig : public SingletonT<GazeboConfig>
  {
    /// \brief Constructor
    private: GazeboConfig();

    /// \brief destructor
    private: ~GazeboConfig();

    /// \brief True if the string is null
    public: void Load();

    /// \brief Get paths to Gazebo install 
    public: std::list<std::string>& GetGazeboPaths();

    /// \brief Get paths to ogre install
    public: std::list<std::string>& GetOgrePaths();
 
    /// \brief Get plugin paths
    public: std::list<std::string>& GetPluginPaths();
 
    /// \brief Add colon delimited paths to Gazebo install 
    public: void AddGazeboPaths(std::string path);

    /// \brief Add colon delimited paths to ogre install
    public: void AddOgrePaths(std::string path);
 
    /// \brief Add colon delimited paths to plugins
    public: void AddPluginPaths(std::string path);

    public: void ClearGazeboPaths();
    public: void ClearOgrePaths();
    public: void ClearPluginPaths();

    /// Paths gazebo install
    private: std::list<std::string> gazeboPaths;
    
    /// Paths to the ogre install
    private: std::list<std::string> ogrePaths;

    /// Paths to the plugins
    private: std::list<std::string> pluginPaths;

    //Singleton implementation
    private: friend class SingletonT<GazeboConfig>;
  };


/// \}
}

}
#endif
