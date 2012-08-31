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
/* Desc: Laser Visualization Class
 * Author: Nate Koenig
 * Date: 14 Dec 2007
 */

#ifndef LASERVISUAL_HH
#define LASERVISUAL_HH

#include <string>

#include "rendering/Visual.hh"
#include "msgs/MessageTypes.hh"
#include "transport/TransportTypes.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    class DynamicLines;

    /// \brief Visualization for lasers
    class LaserVisual : public Visual
    {
      public: LaserVisual(const std::string &_name, VisualPtr _vis,
                          const std::string &_topicName);

      public: virtual ~LaserVisual();

      private: void OnScan(ConstLaserScanPtr &_msg);

      private: transport::NodePtr node;
      private: transport::SubscriberPtr laserScanSub;

      private: DynamicLines *rayFan;
    };
    /// \}
  }
}
#endif
