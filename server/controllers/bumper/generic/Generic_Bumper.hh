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
 * Desc: Bumper Controller
 * Author: Nate Koenig
 * Date: 09 Sept 2008
 */
#ifndef GENERICBUMPER_CONTROLLER_HH
#define GENERICBUMPER_CONTROLLER_HH

#include <sys/time.h>

#include "Controller.hh"
#include "Entity.hh"

namespace gazebo
{
  class ContactSensor;

  /// \addtogroup gazebo_controller
  /// \{
  /** \defgroup bumper bumper
  
    \brief A controller that returns bump contacts
  
    \{
  */
  
  /// \brief A Bumper controller
  class Generic_Bumper : public Controller
  {
    /// Constructor
      public: Generic_Bumper(Entity *parent );
  
    /// Destructor
      public: virtual ~Generic_Bumper();
  
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
    private: ContactSensor *myParent;
  
    /// The Iface. 
    private: libgazebo::BumperIface *myIface;
  };
  
  /** \} */
  /// \}

}

#endif

