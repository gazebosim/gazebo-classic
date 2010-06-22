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
 * Desc: Actuator array controller for a Carl robot.
 * Author: Benjamin Kloster
 * Date: 13 March 2008
 * SVN: $Id$
 */
#ifndef Generic_Actarray_HH
#define Generic_Actarray_HH

#include "Controller.hh"
#include "Entity.hh"

namespace gazebo
{
  class Joint;

/// \addtogroup gazebo_controller
/// \{
/** \defgroup Generic_Actarray Generic_Actarray

  \brief Carl Actuator Array controller.

  This is a controller that simulates a Carl robot

  \verbatim
  <controller:Generic_Actarray name="controller-name" n_actors="number">
    <interface:actarray name="iface-name"/>
  </controller:Generic_Actarray>
  \endverbatim

  \{
*/

/// \brief Carl actuator array controller
/// This is a controller that simulates a Carl robot
class Generic_Actarray : public Controller
{
  /// Constructor
  public: Generic_Actarray(Entity *parent );

  /// Destructor
  public: virtual ~Generic_Actarray();

  /// Load the controller
  /// \param node XML config node
  /// \return 0 on success
  protected: virtual void LoadChild(XMLConfigNode *node);

  /// Init the controller
  /// \return 0 on success
  protected: virtual void InitChild();

  /// Update the controller
  /// \return 0 on success
  protected: virtual void UpdateChild();

  /// Finalize the controller
  /// \return 0 on success
  protected: virtual void FiniChild();

  /// The actarray interface
  private: libgazebo::ActarrayIface *myIface;

  /// The parent Model
  private: Model *myParent;

  /// Number of joints managed by this controller
  private: int n_joints;

  /// The joints of the robot
  private: Joint** joints;

  /// Maximum forces that can be exerted by the joints
  private: float* forces;

  /// Gains of the joints (i.e. how fast they move depending on the difference between actual and target angle)
  private: float* gains;

  /// The error tolerances of the joints
  private: float* tolerances;

};

/** \} */
/// \}

}

#endif

