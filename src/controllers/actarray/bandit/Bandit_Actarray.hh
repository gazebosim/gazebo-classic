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
 * Desc: Actuator array controller for a Bandit robot.
 * Author: Nathan Koenig
 * Date: 19 Sept 2007
 * SVN: $Id$
 */
#ifndef BANDIT_ACTARRAY_HH
#define BANDIT_ACTARRAY_HH

#include "common/Param.hh"
#include "Controller.hh"
#include "Entity.hh"

#define JOINTCNT 20

namespace gazebo
{
  class Joint;

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
  private: libgazebo::ActarrayIface *myIface;

  /// The parent Model
  private: Model *myParent;

  private: ParamT<std::string> *jointNamesP[JOINTCNT];
  private: Joint *joints[JOINTCNT];
  private: ParamT<float> *forcesP[JOINTCNT];
  private: ParamT<float> *gainsP[JOINTCNT];

};

/** \} */
/// \}

}

#endif

