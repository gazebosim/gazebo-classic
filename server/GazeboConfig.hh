/*
 *  Gazebo - Outdoor Multi-Robot Simulator
 *  Copyright (C) 2003  
 *     Nate Koenig & Andrew Howard
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */
/* Desc: Gazebo configuration on this computer 
 * Author: Jordi Polo
 * Date: 3 May 2008
 * SVN: $Id$
 */

#ifndef GAZEBOCONFIG_HH
#define GAZEBOCONFIG_HH

#include <string>
#include <list>

namespace gazebo
{

/// \addtogroup gazebo_server
/// \brief Local configuration on this computer about how gazebo server should work 
/// \{

  class GazeboConfig
  {
    /// \brief Constructor
    public: GazeboConfig();

    /// \brief destructor
    public: ~GazeboConfig();

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

  };


/// \}
}

#endif
