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
 * Desc: Controller for a pioneer2 gripper
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN: $Id$
 */
#ifndef PIONEER2_GRIPPER_HH
#define PIONEER2_GRIPPER_HH


#include "Param.hh"
#include "Controller.hh"
#include "Entity.hh"

namespace gazebo
{
  class GripperIface;
  class ActarrayIface;
  class RaySensor;

/// \addtogroup gazebo_controller
/// \{
/** \defgroup pioneer2_gripper pioneer2_gripper

  \brief Pioneer 2 DX Position2D controller.

  This is a controller that simulates a Pioneer 2 Gripper

  \verbatim
  <controller:pioneer2_gripper name="controller-name">
    <leftJoint>left-joint-name</leftJoint>
    <rightJoint>right-join-name</rightJoint>
    <interface:position name="iface-name"/>
  </controller:pioneer2_gripper>
  \endverbatim
  
  \{
*/

/// \brief Pioneer 2 DX Position2D controller.
/// This is a controller that simulates a Pioneer 2DX motion
class Pioneer2_Gripper : public Controller
{
  /// \brief Constructor
  public: Pioneer2_Gripper(Entity *parent);

  /// \brief Destructor
  public: virtual ~Pioneer2_Gripper();

  /// \brief Left paddle contact callback
  public: void LeftPaddleCB(const Contact &contact);

  /// \brief Right paddle contact callback
  public: void RightPaddleCB(const Contact &contact);

  /// \brief Load the controller
  /// \param node XML config node
  /// \return 0 on success
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// \brief Save the controller.
  /// \stream Output stream
  protected: void SaveChild(std::string &prefix, std::ostream &stream);

  /// Init the controller
  /// \return 0 on success
  protected: virtual void InitChild();

  /// Update the controller
  /// \return 0 on success
  protected: virtual void UpdateChild();

  /// Finalize the controller
  /// \return 0 on success
  protected: virtual void FiniChild();

  /// The gripper interface
  private: GripperIface *gripIface;

  // The interface for the lift
  private: ActarrayIface *actIface;

  private: RaySensor *breakBeams[2];

  /// The parent Model
  private: Model *myParent;

  private: Joint *joints[3];
  private: Joint *holdJoint;

  private: Geom *paddles[2];
  private: Geom *contactGeoms[2];

  private: ParamT<std::string> *jointNamesP[3];
  private: ParamT<float> *gainsP[3];
  private: ParamT<float> *forcesP[3];
  private: ParamT<std::string> *breakBeamNamesP[2];
  private: ParamT<std::string> *paddleNamesP[2];
};

/** \} */
/// \}

}

#endif

