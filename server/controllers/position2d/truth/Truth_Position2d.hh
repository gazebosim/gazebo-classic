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
 * Desc: Truth Position2d controller.
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN: $Id:$
 */
#ifndef TRUTH_POSITION2D_HH
#define TRUTH_POSITION2D_HH


#include "Controller.hh"
#include "Entity.hh"
#include "gazebo.h"


namespace gazebo
{
  class PositionIface;
  class HingeJoint;

/// \addtogroup gazebo_controller
/// \{
/** \defgroup truth_position2d truth_position2d

  \brief Ground truth Position2D controller.

  This is a controller that reports the absolute position. This controller does not accept commands.

  \verbatim
  <controller:truth_position2d name="controller-name">
    <interface:position name="iface-name"/>
  </controller:truth>
  \endverbatim
 
  \{
*/

/// \brief Ground truth Position2D controller.
/// This is a controller that reports the absolute position
class Truth_Position2d : public Controller
{
  /// Constructor
  public: Truth_Position2d(Entity *parent );

  /// Destructor
  public: virtual ~Truth_Position2d();

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

  /// The Position interface
  private: PositionIface *myIface;

  /// The parent Model
  private: Model *myParent;
};

/** \} */
/// \}

}

#endif

