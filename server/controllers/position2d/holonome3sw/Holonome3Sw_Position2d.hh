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
 * Desc: Position2d controller for a Holonome3Sw drive.
 * Author: Christian Gagneraud (ch'Gans), based on Differential_Position2d.hh
 * Date: 21 Feb 2007
 * SVN: $Id$
 */

#ifndef HOLONOME3SW_POSITION2D_HH
#define HOLONOME3SW_POSITION2D_HH

#include <map>

#include "Controller.hh"

namespace gazebo
{
  class Joint;
  class Entity;
  class PositionIface;

/// \addtogroup gazebo_controller
/// \{
/** \defgroup holonome3sw_position2d holonome3sw_position2d

  \brief Position2D controller for an holonomous robot using 3 swedish
  wheels.

  This is a controller that simulates the motion of an holonomous
  robot using 3 swedish wheels (such as WizBot).

  Kinematics model:
  \verbatim
  J1.R(Th).XiP + J2.PhiP = 0
  
  J1   = [ -sin(A1)   cos(A1)    L1 ]
         [ -sin(A2)   cos(A2)    L2 ]
         [ -sin(A3)   cos(A3)    L3 ]
  
  J2   = [ R1         0          0  ]
         [ 0          R2         0  ]
         [ 0          0          R3 ]
  
  R(t) = [ cos(t)     sin(t)     0  ]
         [ -sin(t)    cos(t)     0  ]
         [ 0          0          1  ]
  
  With:
  A1 = alpha1+beta1+gamma1;
  L1 = l1*cos(beta1+gamma1)
  R1 = r1*cos(gamma1)
  \endverbatim

  \verbatim
  <controller:holonome3sw_position2d name="controller-name">
    <wheels>
     <-- Global to all wheels, optional -->
     <distance>d</distance>
     <-- Global to all wheels, optional -->
     <radius>r</radius>
     <-- Global to all wheels, optional -->
     <max-torque>t</max-torque>
     <-- Global to all wheels, optional -->
     <beta>b</beta>
     <-- Global to all wheels, optional -->
     <gamma>g</gamma>
     <-- 1st wheel -->
     <swedish0>
      <joint-name>joint-name</joint-name>
      <alpha>a</alpha>
      <-- default to wheels/distance -->
      <distance>d</distance>
      <-- default to wheels/radius  -->
      <radius>r</radius>
      <-- default to wheels/max-torque -->
      <max-torque>max-torque</max-torque>
      <-- default to wheels/beta if exist or to zero  -->
      <beta>b</beta>
      <-- default to wheels/beta if exist or to zero  -->
      <gamma>g</gamma>
     </swedish0>
     <-- 2nd wheel -->     
     <swedish1>
      ...
     </swedish>
     <-- 3rd wheel -->     
     <swedish2>
      ...
     </swedish>
    </wheels>
    <interface:position name="iface-name"/>
  </controller:holonome3sw_position2d>
  \endverbatim

  Where:
    - P: Reference point on the robot frame.
    - A: Point representing the center of a wheel
    - distance, alpha: Polar coordinates of A in the robot frame.
    - beta: Orientation of the plane of the wheel with respect with (PA).
    - gamma: Direction, with respect to the wheel plane, of the zero
             component of the velocity of the contact point.

  \{
*/

/// \brief WizBot Position2D controller.
/// This is a controller that simulates a WizBot motion
class Holonome3Sw_Position2d : public Controller
{
  /// Constructor
  public: Holonome3Sw_Position2d(Entity *parent );

  /// Destructor
  public: virtual ~Holonome3Sw_Position2d();

  /// Load the controller
  /// \param node XML config node
  protected: virtual void LoadChild(XMLConfigNode *node);

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

  /// Reset internal datas
  private: void ResetData();
  
  /// The Position interface
  private: PositionIface *myIface;

  /// The parent Model
  private: Model *myParent;


  // The wheel joints.
  private: Joint *joint[3];

  /// Polar coordinate of the center of the wheels
  private: float DIST[3];

  /// Polar coordinate of the center of the wheels
  private: float ALPHA[3];

  /// Orientation of the plane of the wheel 
  private: float BETA[3];

  /// Direction of the zero component of the velocity of the contact point.
  private: float GAMMA[3];

  /// Radius of the wheels
  private: float RADIUS[3];

  /// Max torque to applied to the wheels
  private: float MAXTORQUE[3];

  // Robot posture coordinates in the world: x, y, theta
  private: float Xi[3];
  // Robot speed coordinates in the world: vx, vy, vtheta
  private: float XiP[3];
  // Rotation speed coordinates (wheels, rad/s)
  private: float PhiP[3];

  /// True = enable motors
  private: bool enableMotors;

private:
  float A[3], L[3], R[3];
  
};

/** \} */
/// \}

}

#endif

