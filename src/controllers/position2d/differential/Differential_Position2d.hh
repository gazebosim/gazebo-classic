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
 * Desc: Position2d controller for a Differential drive.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN: $Id$
 */
#ifndef DIFFERENTIAL_POSITION2D_HH
#define DIFFERENTIAL_POSITION2D_HH

#include <map>

#include "common/Param.hh"
#include "Controller.hh"

namespace gazebo
{
  class Joint;
  class Entity;

/// \addtogroup gazebo_controller
/// \{
/** \defgroup pioneer2dx_position2d pioneer2dx_position2d

  \brief Pioneer 2 DX Position2D controller.

  This is a controller that simulates a Pioneer 2DX motion

  \verbatim
  <controller:pioneer2dx_position2d name="controller-name">
    <leftJoint>left-joint-name</leftJoint>
    <rightJoint>right-join-name</rightJoint>
    <wheelDiameter>diameter_in_meters</wheelDiameter>
    <wheelSeparation>separation_in_meters</wheelSeparation>
    <torque></torque>
    <interface:position name="iface-name"/>
  </controller:pioneer2dx_position2d>
  \endverbatim
  
  \{
*/

/// \brief Pioneer 2 DX Position2D controller.
/// This is a controller that simulates a Pioneer 2DX motion
class Differential_Position2d : public Controller
{
  /// Constructor
  public: Differential_Position2d(Entity *parent );

  /// Destructor
  public: virtual ~Differential_Position2d();

  /// Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Save the controller.
  /// \stream Output stream
  protected: void SaveChild(std::string &prefix, std::ostream &stream);

  /// Init the controller
  protected: virtual void InitChild();

  /// \brief Reset the controller
  protected: void ResetChild();

  /// Update the controller
  protected: virtual void UpdateChild();

  /// Finalize the controller
  protected: virtual void FiniChild();

  /// Update the data in the interface
  private: void PutPositionData();

  /// Get the position command from libgazebo
  private: void GetPositionCmd();

  /// The Position interface
  private: libgazebo::PositionIface *myIface;

  /// The parent Model
  private: Model *myParent;

  /// Separation between the wheels
  private: ParamT<float> *wheelSepP;

  /// Diameter of the wheels
  private: ParamT<float> *wheelDiamP;

  ///Torque applied to the wheels
  private: ParamT<float> *torqueP;

  /// Speeds of the wheels
  private: float wheelSpeed[2];

  // Simulation time of the last update
  private: Time prevUpdateTime;

  /// True = enable motors
  private: bool enableMotors;

  private: float odomPose[3];
  private: float odomVel[3];

  private: Joint *joints[2];

  private: PhysicsEngine  *physicsEngine;

  private: ParamT<std::string> *leftJointNameP;
  private: ParamT<std::string> *rightJointNameP;
};

/** \} */
/// \}

}

#endif

