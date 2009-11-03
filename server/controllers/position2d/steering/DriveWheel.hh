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
