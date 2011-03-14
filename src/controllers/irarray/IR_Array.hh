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
 * Desc: IR array controller.
 * Author: Wenguo Liu
 * Date: 24 Apr 2008
 */

#ifndef IR_ARRAY_HH
#define IR_ARRAY_HH

#include "Controller.hh"

namespace libgazebo
{
  class IRIface;
}

namespace gazebo
{
  class IRSensor;


  /// \brief Sick LMS 200 laser controller.
  /// 
  /// This is a controller that simulates a ir array
  class IR_Array : public Controller
  {
    /// \brief Constructor
    /// \param parent The parent entity, must be a Model or a Sensor
    public: IR_Array(Entity *parent);
  
    /// \brief Destructor
    public: virtual ~IR_Array();
  
    /// \brief Load the controller
    /// \param node XML config node
    /// \return 0 on success
    protected: virtual void LoadChild(XMLConfigNode *node);
  
    /// \brief Init the controller
    /// \return 0 on success
    protected: virtual void InitChild();
  
    /// \brief Update the controller
    /// \return 0 on success
    protected: virtual void UpdateChild();
  
    /// \brief Finalize the controller
    /// \return 0 on success
    protected: virtual void FiniChild();
  
    /// \brief Put laser data to the iface
    private: void PutIRData();
  
    /// The ir interface
    private: libgazebo::IRIface *irIface;
  
    /// The parent sensor
    private: IRSensor *myParent;
  
  };
  
  /** /} */
  /// @}
}

#endif
