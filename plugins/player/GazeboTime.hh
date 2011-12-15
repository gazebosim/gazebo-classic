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
/* Desc: Gazebo Time functions 
 * Author: Nate Koenig, Andrew Howard
 * Date: 2 March 2006
 * CVS: $Id$
 */

#ifndef GAZEBOTIME_HH
#define GAZEBOTIME_HH

#include <libplayercore/playercore.h>

namespace libgazebo
{

/// \addtogroup player
/// \brief Gazebo player driver
/// \{


  /// \brief Gazebo access to PlayerTime
class GazeboTime : public PlayerTime
{
  /// \brief Constructor
  public: GazeboTime();

  /// \brief Destructor
  public: virtual ~GazeboTime();

  /// \brief Get the simulator time
  public: int GetTime(struct timeval* time);

  /// \brief Get the simulator time
  public: int GetTimeDouble(double* time);

};

/// \}

}
#endif
