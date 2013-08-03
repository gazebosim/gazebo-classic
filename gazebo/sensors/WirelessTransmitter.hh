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
/* Desc: Wireless transmitter
 * Author: Carlos Agüero
 * Date: 24 June 2013
 */

#ifndef _WIRELESS_TRANSMITTER_HH_
#define _WIRELESS_TRANSMITTER_HH_

#include "gazebo/physics/physics.hh"
#include "gazebo/sensors/WirelessTransceiver.hh"
#include "gazebo/transport/TransportTypes.hh"

namespace gazebo
{
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{

    /// \class WirelessTransmitter WirelessTransmitter.hh sensors/sensors.hh
    /// \brief Transmitter to send wireless signals 
    class WirelessTransmitter: public WirelessTransceiver
    {
      /// \brief Constant used in the propagation model when there are no
      /// obstacles between transmitter and receiver
      public: static const double N_EMPTY;

      /// \brief Constant used in the propagation model when there are
      /// obstacles between transmitter and receiver
      public: static const double N_OBSTACLE;

      /// \brief Std desv of the Gaussian random variable used in the
      /// propagation model
      public: static const double MODEL_STD_DESV;

      /// \brief Constructor.
      public: WirelessTransmitter();

      // Documentation inherited
      protected: virtual void UpdateImpl(bool _force);

      // Documentation inherited
      public: virtual void Fini();

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName);

      /// \brief Returns the Service Set Identifier (network name).
      /// \return Service Set Identifier (network name).
      public: std::string GetESSID();

      /// \brief Returns reception frequency (MHz).
      /// \return Reception frequency (MHz).
      public: double GetFreq();

      /// \brief Returns the pose of the transmitter in world coordinate.
      /// \return Pose of object.
      public: math::Pose GetPose() const;

      /// \brief Returns the signal strength in a given world's point (dBm).
      /// \return Signal strength in a world's point (dBm).
      public: double GetSignalStrength(const math::Pose _receiver,
          const double rxGain);

      /// \brief Size of the grid used for visualization.
      private: static const double STEP;

      /// \brief The visualization shows the propagation model from a grid
      /// in which the x coordinates go from -XLIMIT to XLIMIT.
      private: static const double XLIMIT;

      /// \brief The visualization shows the propagation model from a grid
      /// in which the y coordinates go from -YLIMIT to YLIMIT.
      private: static const double YLIMIT;

      /// \brief Service Set Identifier (network name).
      private: std::string essid;

      /// \brief Reception frequency (MHz).
      protected: double freq;
    };
    /// \}
  }
}
#endif
