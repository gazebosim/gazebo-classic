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
 * Desc: Joint Force Controller
 * Author: Benjamin Kloster
 * Date: 13 March 2008
 */
#ifndef JOINTFORCE_CONTROLLER_HH
#define JOINTFORCE_CONTROLLER_HH

/// Maximum number of joints that can be watched by one controller
#define GAZEBO_JOINTFORCE_CONTROLLER_MAX_FEEDBACKS 16

#include "Controller.hh"
#include "Entity.hh"
#include <ode/ode.h>
#include <sys/time.h>


namespace gazebo
{
/// \addtogroup gazebo_controller
/// \{
/** \defgroup jointforce_controller jointforce

  \brief A controller that measures forces and torques exerted by joints

  \{
*/

/// \brief A JointForce controller
class JointForce : public Controller
{
  /// Constructor
    public: JointForce(Entity *parent );

  /// Destructor
    public: virtual ~JointForce();

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

  /// The parent Model
  private: Model *myParent;

  /// The Iface. The dJointFeedback structs are rather arbitrary, so we use an Opaque Interface
  private: OpaqueIface *myIface;
  /// The joint feedbacks
  private: dJointFeedback *jointfeedbacks[GAZEBO_JOINTFORCE_CONTROLLER_MAX_FEEDBACKS];
  /// The number of joints we are watching
  private: int n_joints;
};

/** \} */
/// \}

}

#endif

