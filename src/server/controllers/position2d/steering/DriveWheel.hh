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
/*
 * Desc: Wheel that can not be steered
 * Author: Jordi Polo
 * Date: 18 Dec 2007
 */
#ifndef DRIVEWHEEL_HH
#define DRIVEWHEEL_HH


#include "Wheel.hh"

namespace gazebo
{
  class Joint;

/// \addtogroup gazebo_controller
/// \{
/** \defgroup steer2_position2d car_position2d

  \brief Ackermann steering of four wheels Position2D controller.

  This is a controller that simulates the steering of most of the commercial cars.
  
  \{
*/

/// \brief Car-like four wheels ackermann Position2D controller.
/// This is a controller that simulates a Pioneer 2DX motion
class DriveWheel: public Wheel
{
  /// \brief Constructor
  public: DriveWheel ( );

   /// \brief Destructor
  public: virtual ~DriveWheel();

   /// \brief Connect this controller with a given joint
  public: void Connect(Joint *joint, int type);

   /// \brief Update this controller 
  public: void Update(float speed, float steer=0);

   /// \brief Set the steering torque of this wheel
  public: void SetSteerTorque(float newTorque);

   /// \brief Set the driving force torque of this wheel
  public: void SetTorque(float newTorque);

   /// \brief Stops this wheel
  public: void Stop();

  /// \brief Set the suspension of this wheel
  public: void SetSuspension(float spring, float damping, float step);

  private: Joint *joint;
  private: float torque;
  private: float steerTorque; 
  private: float speedCmd;
  private: float steerCmd;


};

/** \} */
/// \}

}

#endif
