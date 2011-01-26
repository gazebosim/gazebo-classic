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

