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
