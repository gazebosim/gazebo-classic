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
  class PositionIface;
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
  /// \return 0 on success
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// Init the controller
  /// \return 0 on success
  protected: virtual void InitChild();

  /// Update the controller
  /// \return 0 on success
  protected: virtual void UpdateChild(UpdateParams &params);

  /// Finalize the controller
  /// \return 0 on success
  protected: virtual void FiniChild();

  /// Update the data in the interface
  private: void PutPositionData();

  /// Get the position command from libgazebo
  private: void GetPositionCmd();

  ///our wheels
  protected: std::vector<Wheel*> wheels;

  /// The Position interface
  private: PositionIface *myIface;

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

