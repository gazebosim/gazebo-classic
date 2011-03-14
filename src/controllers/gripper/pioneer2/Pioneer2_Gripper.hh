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
 * Desc: Controller for a pioneer2 gripper
 * Author: Nathan Koenig
 * Date: 01 Feb 2007
 * SVN: $Id$
 */
#ifndef PIONEER2_GRIPPER_HH
#define PIONEER2_GRIPPER_HH


#include "common/Param.hh"
#include "Controller.hh"
#include "Entity.hh"

namespace libgazebo
{
  class GripperIface;
  class ActarrayIface;
}

namespace gazebo
{
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
  private: libgazebo::GripperIface *gripIface;

  // The interface for the lift
  private: libgazebo::ActarrayIface *actIface;

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
  private: event::ConnectionPtr leftConnection, rightConnection;
};

/** \} */
/// \}

}

#endif

