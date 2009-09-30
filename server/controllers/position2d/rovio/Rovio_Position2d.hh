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
 * Desc: Position2d controller for a rovio robot
 * Author: Nathan Koenig
 * Date: 22 Sep 2009
 * SVN: $Id$
 */
#ifndef ROVIO_POSITION2D_HH
#define ROVIO_POSITION2D_HH

#include <map>

#include "Param.hh"
#include "Controller.hh"

namespace gazebo
{
  class HingeJoint;
  class Entity;
  class PositionIface;

/// \addtogroup gazebo_controller
/// \{
/** \defgroup rovio_position2d rovio_position2d

  \brief ROvio Position2D controller.

  This is a controller that simulates a Rovio motion

  \verbatim
  <controller:rovio_position2d name="controller-name">
    <leftJoint>left-joint-name</leftJoint>
    <rightJoint>right-join-name</rightJoint>
    <interface:position name="iface-name"/>
  </controller:rovio_position2d>
  \endverbatim
  
  \{
*/

/// \brief Rovio Position2D controller.
/// This is a controller that simulates a Rovio motion
class Rovio_Position2d : public Controller
{
  /// Constructor
  public: Rovio_Position2d(Entity *parent );

  /// Destructor
  public: virtual ~Rovio_Position2d();

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
  private: PositionIface *myIface;

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
  private: double prevUpdateTime;

  /// True = enable motors
  private: bool enableMotors;

  private: float odomPose[3];
  private: float odomVel[3];

  private: HingeJoint *joints[2];

  private: PhysicsEngine  *physicsEngine;

  private: ParamT<std::string> *leftJointNameP;
  private: ParamT<std::string> *rightJointNameP;
  private: ParamT<std::string> *backJointNameP;
};

/** \} */
/// \}

}

#endif

