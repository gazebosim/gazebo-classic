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

