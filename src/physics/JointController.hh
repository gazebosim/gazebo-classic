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
#ifndef JOINTCONTROLLER_HH
#define JOINTCONTROLLER_HH

#include <map>
#include <string>
#include "physics/PhysicsTypes.hh"
#include "transport/TransportTypes.hh"
#include "msgs/msgs.h"

namespace gazebo
{
  namespace physics
  {
    class JointController
    {
      public: JointController(ModelPtr _model);
      public: void AddJoint(JointPtr _joint);
      public: void Update();

      private: void OnJointCmd(ConstJointCmdPtr &_msg);

      private: ModelPtr model;
      private: std::map<std::string, JointPtr> joints;
      private: std::map<std::string, double> forces;
      private: transport::NodePtr node;
      private: transport::SubscriberPtr jointCmdSub;
    };
  }
}
#endif
