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
 * Desc: Stubbed out controller
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN: $Id$
 */
#ifndef CONSTROLLER_STUB_HH
#define CONSTROLLER_STUB_HH

#include "Controller.hh"
#include "Entity.hh"

namespace gazebo
{
/// \addtogroup gazebo_controller
/// \{
/** \defgroup controller_stub controller_stub

  \brief A stubbed out controller.

  Copy this example code when creating a new controller
  \{
*/

/// \brief A stubbed out controller.
class ControllerStub : public Controller
{
  /// Constructor
  public: ControllerStub(Entity *parent );

  /// Destructor
  public: virtual ~ControllerStub();

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

  /// The parent Model
  private: Model *myParent;
};

/** \} */
/// \}

}

#endif

