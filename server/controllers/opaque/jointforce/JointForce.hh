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
 * Desc: Joint Force Controller
 * Author: Benjamin Kloster
 * Date: 13 March 2008
 */
#ifndef JOINTFORCE_CONTROLLER_HH
#define JOINTFORCE_CONTROLLER_HH

/// Maximum number of joints that can be watched by one controller
//#define GAZEBO_JOINTFORCE_CONTROLLER_MAX_FEEDBACKS 16

#include "Controller.hh"
#include "Entity.hh"
#include <vector>

namespace gazebo
{
  class Joint;

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
  
    /// The Iface. The dJointFeedback structs are rather arbitrary, 
    /// so we use an Opaque Interface
    private: libgazebo::OpaqueIface *myIface;
 
    private: std::vector<Joint*> joints; 
  };
  
  /** \} */
  /// \}

}

#endif

