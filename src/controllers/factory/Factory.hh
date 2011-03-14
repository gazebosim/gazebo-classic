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
 * Desc: Factory controller.
 * Author: Nathan Koenig
 * Date: 29 July 2007
 * SVN: $Id$
 */
#ifndef FACTORY_HH
#define FACTORY_HH

#include "Controller.hh"
#include "Entity.hh"
#include "gazebo.h"

namespace
{
  class FactoryIface;
}

namespace gazebo
{

/// \addtogroup gazebo_controller
/// \{
/** \defgroup factory factory
  \brief Factory used for dynamic construction of models

  The factory controller allows dynamic addition and deletion of models using libgazebo's \ref factory_iface interface.

  \verbatim
  <model:empty name="factory-model-name">
    <controller:factory name="factory-controller-name">
      <interface:factory name="factory-iface-name"/>
    </controller:factory>
  </model:empty>
  \endverbatim
\{
*/

/// \brief Factory used for dynamic construction of models
class Factory : public Controller
{
  /// Constructor
  public: Factory(Entity *parent );

  /// Destructor
  public: virtual ~Factory();

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

  /// The Position interface
  private: libgazebo::FactoryIface *factoryIface;

  /// The parent Model
  private: Model *myParent;

  private: std::string xmlPrefix;
  private: std::string xmlSuffix;
};

/** \} */
/// \}

}

#endif

