/*
 * Copyright 2013 Open Source Robotics Foundation
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
/* Desc: Transmitter radiation diagram visualization Class
 * Author: Carlos Agüero
 * Date: 27 Jun 2013
 */

#ifndef _TRANSMITTERVISUAL_HH_
#define _TRANSMITTERVISUAL_HH_

#include <string>

#include "gazebo/rendering/Visual.hh"
#include "gazebo/msgs/MessageTypes.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace rendering
  {
    /// \addtogroup gazebo_rendering
    /// \{

    /// \class LaserVisual LaserVisual.hh rendering/rendering.hh
    /// \brief Visualization for laser data.
    class TransmitterVisual : public Visual
    {
      /// \brief Constructor.
      /// \param[in] _name Name of the visual.
      /// \param[in] _vis Pointer to the parent Visual.
      /// \param[in] _topicName Name of the topic that has laser data.
      public: TransmitterVisual(const std::string &_name, VisualPtr _vis,
                                 const std::string &_topicName);

      /// \brief Destructor.
      public: virtual ~TransmitterVisual();

      public: void Load();

      /// Documentation inherited from parent.
      public: virtual void SetEmissive(const common::Color &_color);

      private: std::string GetTemplateSDFString();

      /// \brief Callback when laser data is received.
      private: void OnScan(ConstPosePtr &_msg);

      /// \brief Pointer to a node that handles communication.
      private: transport::NodePtr node;

      /// \brief Subscription to the laser data.
      private: transport::SubscriberPtr laserScanSub;

      /// \brief Renders the laser data.
      private: DynamicLines *rayFan;

      private: sdf::SDFPtr modelTemplateSDF;
    };
    /// \}
  }
}
#endif
