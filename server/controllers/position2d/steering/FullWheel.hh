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
 * Desc: Full (steering and drive) or Steer (steering only) wheel
 * Author: Jordi Polo
 * Date: 18 Dec 2007
 */
#ifndef FULLWHEEL_HH
#define FULLWHEEL_HH

#include "Wheel.hh"

namespace gazebo
{
  class Joint;

// \addtogroup gazebo_controller
// \{
/** \defgroup Wheel FullWeel

  \brief A full fledged wheel

  This wheel can be rotated with its own motor and also can rotate.
  
  \{
*/

/// \brief A full fledged wheel
/// This wheel can be rotated with its own motor and also can rotate.
class FullWheel: public Wheel
{
  /// \brief Constructor
  public: FullWheel ();

  /// \brief Destructor
  public: virtual ~FullWheel();

  ///\brief  Connect this control to a joint
  public:  void Connect(Joint *joint, int type);

  ///\brief  Update the wheel 
  public:  void Update(float speed, float steer);

  ///\brief Set the steering torque
  public:  void SetSteerTorque(float newTorque);

  ///\brief Set the torque
  public:  void SetTorque(float newTorque);
  
  ///\brief Set the Steering control parameters
  public: void SetSteerPD(float kp, float kd);

  ///\brief Set the maximum Angle the steering will rotate
  public: void SetSteerMaxAngle(float maxAngle);

  ///\brief Stops this wheel
  public:  void Stop();

  ///\brief Experimental: Sets the suspension of this wheel
  public: void SetSuspension(float spring, float damping, float step);

  private: Joint *joint;
  private: float steerTorque; 
  private: float cmdSteer;
  private: float steerMaxAngle;
 // private: int type;
  
};

/** \} */
// \}

}

#endif
