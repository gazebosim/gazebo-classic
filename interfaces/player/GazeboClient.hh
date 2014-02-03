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
/* Desc: Gazebo (simulator) client functions
 * Author: Nate Koenig, Andrew Howard
 * Date: 2 March 2006
 */

#ifndef GAZEBO_PLAYER_CLIENT_HH
#define GAZEBO_PLAYER_CLIENT_HH

#include <string>

/// \addtogroup player
/// \brief Gazebo client handler
/// \{

/// \brief Gazebo client handler
///
/// This class handles the Gazebo client object, and acts as a shared
/// data-structure for all Gazebo related drivers.  Note that there
/// can only be one instance of this class (it is entirely static).
class GazeboClient
{
  /// \brief Initialize
  public: static void Init(int _serverid, const std::string &_worldName);

  /// \brief Finalize
  public: static void Fini();

  /// \brief The prefix used for all gazebo ID's
  public: static std::string worldName;
};

/// \}



#endif


