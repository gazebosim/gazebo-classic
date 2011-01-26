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
 * Desc: General steering controller for any number of wheels and configuration
 * Author: Jordi Polo
 * Date: 23 Dec 2007
 */
#ifndef STEERING_POSITION2D_HH
#define STEERING_POSITION2D_HH


#include "Controller.hh"
#include "Entity.hh"

namespace gazebo
{
  class Wheel;

/// \addtogroup gazebo_controller
/// \{
/** \defgroup steering_position2d steering_position2d

  \brief General steering controller for any number of wheels

  This is a General steering controller for any number of wheels

  \verbatim
  <controller:steering_position2d name="a car">
    <wheel>
       <joint>front-left-joint-name</joint>
       <type>steer</type>
    </wheel>
    <wheel>
       <joint>front-right-joint-name</joint>
       <type>steer</type>
    </wheel>
    <wheel>
       <joint>rear-left-joint-name</joint>
       <type>drive</type>
    </wheel>
    <wheel>
       <joint>rear-right-joint-name</joint>
       <type>drive</type>
    </wheel>

    <torque>1000</torque>
    <steerTorque>100</steerTorque>
    <steerPD></steerPD>
    <steerMaxAngle></steerMaxAngle>
    <interface:position name="iface-name"/>
  </controller:car_position2d>
  \endverbatim

The following parameters:
  \verbatim
     <torque></torque>
    <steerTorque></steerTorque>
    <steerPD></steerPD>
    <steerMaxAngle></steerMaxAngle> 
  \endverbatim
Can be defined outside any <wheel> tag to make them generic to all the wheels. Each wheel can overwrite them in an individual basis also.

  \{
*/

/// \brief Steering-like four wheels ackermann Position2D controller.
class Steering_Position2d : public Controller
{
  /// Constructor
  public: Steering_Position2d(Entity *parent );

  /// Destructor
  public: virtual ~Steering_Position2d();

  /// Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// Init the controller
  protected: virtual void InitChild();

  /// \brief Reset the controller
  protected: virtual void ResetChild();

  /// Update the controller
  protected: virtual void UpdateChild();

  /// Finalize the controller
  protected: virtual void FiniChild();

  /// Update the data in the interface
  private: void PutPositionData();

  /// Get the position command from libgazebo
  private: void GetPositionCmd();

  ///our wheels
  protected: std::vector<Wheel*> wheels;

  /// The Position interface
  private: libgazebo::PositionIface *myIface;

  /// The parent Model
  private: Model *myParent;


  ///commands to the Steering
  private: float cmdSteer;
  private: float cmdSpeed;

  /// True = enable motors
  private: bool enableMotors;

  private: float odomPose[3];
  private: float odomVel[3];

};

/** \} */
/// \}

}

#endif

