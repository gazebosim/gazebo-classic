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
 * Desc: Actuator array controller for a Bandit robot.
 * Author: Nathan Koenig
 * Date: 19 Sept 2007
 * SVN: $Id$
 */
#ifndef BANDIT_ACTARRAY_HH
#define BANDIT_ACTARRAY_HH

#include "Param.hh"
#include "Controller.hh"
#include "Entity.hh"

namespace gazebo
{
  class HingeJoint;
  class PositionIface;

/// \addtogroup gazebo_controller
/// \{
/** \defgroup bandit_actarray bandit_actarray

  \brief Bandit Actuator Array controller.

  This is a controller that simulates a Bandit torso

  \verbatim
  <controller:bandit_actarray name="controller-name">
    <interface:actarray name="iface-name"/>
  </controller:bandit_actarray>
  \endverbatim
  
  \{
*/

/// \brief Bandit actuator array controller
/// This is a controller that simulates a Bandit torso
class Bandit_Actarray : public Controller
{
  /// Constructor
  public: Bandit_Actarray(Entity *parent );

  /// Destructor
  public: virtual ~Bandit_Actarray();

  /// Load the controller
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

  /// The actarray interface
  private: ActarrayIface *myIface;

  /// The parent Model
  private: Model *myParent;

  private: Param<std::string> *jointNamesP[16];
  private: HingeJoint *joints[16];
  private: Param<float> *forcesP[16];
  private: Param<float> *gainsP[16];

};

/** \} */
/// \}

}

#endif

