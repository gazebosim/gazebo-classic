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
 * SVN: $Id: Factory.hh 7551 2009-03-27 16:15:13Z natepak $
 */
#ifndef FACTORYIFACEHANDLER_HH
#define FACTORYIFACEHANDLER_HH

#include "Controller.hh"
#include "Entity.hh"
#include "gz.h"


namespace gazebo
{
  class World;

  /// \addtogroup gazebo_server
  /// \{
  /** \defgroup factory factory
    \brief Factory used for dynamic construction of models
  
    The factory controller allows dynamic addition and deletion of models using libgazebo's \ref factory_iface interface.
  
  \{
  */
  
  /// \brief Factory used for dynamic construction of models
  class FactoryIfaceHandler
  {
  
    /// Constructor
    public: FactoryIfaceHandler(World *world);
  
    /// Destructor
    public: virtual ~FactoryIfaceHandler();
  
    /// Init the controller
    /// \return 0 on success
    public: void Init();
  
    /// Update the controller
    /// \return 0 on success
    public: void Update();
  
    /// The Position interface
    private: libgazebo::FactoryIface *factoryIface;
  
    private: World *world;
  
    private: std::string xmlPrefix;
    private: std::string xmlSuffix;
  };
  
  /** \} */
  /// \}

}

#endif

