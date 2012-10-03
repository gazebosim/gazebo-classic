/*
 * Copyright 2011 Nate Koenig
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
/* Desc: Specification of a contact
 * Author: Nate Koenig
 * Date: 10 Nov 2009
 */

#ifndef JOINTFEEDBACK_HH
#define JOINTFEEDBACK_HH

#include "math/Vector3.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Feedback information from a joint.  These are
    ///        forces and torques on parent and child Link's
    class JointFeedback
    {
      /// \brief Operator =
      public: JointFeedback &operator =(const JointFeedback &f)
              {
                this->body1Force = f.body1Force;
                this->body2Force = f.body2Force;

                this->body1Torque = f.body1Torque;
                this->body2Torque = f.body2Torque;
                return *this;
              }

      public: math::Vector3 body1Force;
      public: math::Vector3 body2Force;

      public: math::Vector3 body1Torque;
      public: math::Vector3 body2Torque;
    };
    /// \}
  }
}
#endif


