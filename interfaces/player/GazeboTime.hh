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
/* Desc: Gazebo Time functions
 * Author: Nate Koenig, Andrew Howard
 * Date: 2 March 2006
 */

#ifndef GAZEBO_PLAYER_TIME_HH
#define GAZEBO_PLAYER_TIME_HH

#include "player.h"

#include "gazebo/common/CommonTypes.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/msgs/msgs.hh"
#include "gazebo/util/system.hh"

/// \addtogroup player
/// \brief Gazebo player driver
/// \{
  /// \brief Gazebo access to PlayerTime
class GAZEBO_VISIBLE GazeboTime : public PlayerTime
{
  /// \brief Constructor
  public: GazeboTime();

  /// \brief Destructor
  public: virtual ~GazeboTime();

  /// \brief Get the simulator time
  public: int GetTime(struct timeval *_time);

  /// \brief Get the simulator time
  public: int GetTimeDouble(double *_time);

  private: void OnStats(ConstWorldStatisticsPtr &_msg);

  private: gazebo::transport::NodePtr node;
  private: gazebo::transport::SubscriberPtr statsSub;
  private: gazebo::common::Time simTime;
};

/// \}

#endif
