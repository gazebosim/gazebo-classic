/*
 * Copyright (C) 2012-2015 Open Source Robotics Foundation
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
#ifdef _WIN32
  // Ensure that Winsock2.h is included before Windows.h, which can get
  // pulled in by anybody (e.g., Boost).
  #include <Winsock2.h>
#endif

#include <ignition/math/Pose3.hh>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/sensors/SensorFactory.hh"
#include "gazebo/sensors/SensorManager.hh"
#include "gazebo/sensors/WirelessReceiver.hh"
#include "gazebo/sensors/WirelessTransmitter.hh"
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("wireless_receiver", WirelessReceiver)

/////////////////////////////////////////////////
WirelessReceiver::WirelessReceiver()
: WirelessTransceiver()
{
  this->minFreq = 2412.0;
  this->maxFreq = 2484.0;
  this->sensitivity = -90.0;
}

//////////////////////////////////////////////////
WirelessReceiver::~WirelessReceiver()
{
}

//////////////////////////////////////////////////
void WirelessReceiver::Init()
{
  WirelessTransceiver::Init();
}

/////////////////////////////////////////////////
void WirelessReceiver::Load(const std::string &_worldName)
{
  WirelessTransceiver::Load(_worldName);

  this->pub = this->node->Advertise<msgs::WirelessNodes>(this->GetTopic(), 30);
  GZ_ASSERT(this->pub != NULL,
      "wirelessReceiverSensor did not get a valid publisher pointer");

  sdf::ElementPtr transceiverElem = this->sdf->GetElement("transceiver");

  this->minFreq = transceiverElem->Get<double>("min_frequency");
  this->maxFreq = transceiverElem->Get<double>("max_frequency");
  this->sensitivity = transceiverElem->Get<double>("sensitivity");

  if (this->minFreq <= 0)
  {
    gzthrow("Wireless receiver min. frequency must be > 0. Current value is ["
      << this->minFreq << "]");
  }

  if (this->maxFreq <= 0)
  {
    gzthrow("Wireless receiver max. frequency must be > 0. Current value is ["
      << this->maxFreq << "]");
  }

  if (this->minFreq > this->maxFreq)
  {
    gzthrow("Wireless receiver min. frequency must be less or equal than max. "
        << "frequency. Current min. frequency is [" << this->minFreq <<
        "] and max frequency is [" << this->maxFreq << "]");
  }

  if (this->sensitivity >= 0)
  {
    gzthrow("Wireless receiver sensitivity must be < 0. Current value is ["
      << this->sensitivity << "]");
  }
}

//////////////////////////////////////////////////
bool WirelessReceiver::UpdateImpl(bool /*_force*/)
{
  std::string txEssid;
  msgs::WirelessNodes msg;
  double rxPower;
  double txFreq;

  this->referencePose =
      this->pose + this->parentEntity.lock()->GetWorldPose().Ign();

  ignition::math::Pose3d myPos = this->referencePose;
  Sensor_V sensors = SensorManager::Instance()->GetSensors();
  for (Sensor_V::iterator it = sensors.begin(); it != sensors.end(); ++it)
  {
    if ((*it)->GetType() == "wireless_transmitter")
    {
      boost::shared_ptr<gazebo::sensors::WirelessTransmitter> transmitter =
          boost::static_pointer_cast<WirelessTransmitter>(*it);

      txFreq = transmitter->GetFreq();
      rxPower = transmitter->SignalStrength(myPos, this->GetGain());

      // Discard if the frequency received is out of our frequency range,
      // or if the received signal strengh is lower than the sensivity
      if ((txFreq < this->GetMinFreqFiltered()) ||
          (txFreq > this->GetMaxFreqFiltered()) ||
          (rxPower < this->GetSensitivity()))
      {
        continue;
      }

      txEssid = transmitter->GetESSID();

      msgs::WirelessNode *wirelessNode = msg.add_node();
      wirelessNode->set_essid(txEssid);
      wirelessNode->set_frequency(txFreq);
      wirelessNode->set_signal_level(rxPower);
    }
  }
  if (msg.node_size() > 0)
  {
    this->pub->Publish(msg);
  }

  return true;
}

/////////////////////////////////////////////////
double WirelessReceiver::GetMinFreqFiltered() const
{
  return this->minFreq;
}

/////////////////////////////////////////////////
double WirelessReceiver::GetMaxFreqFiltered() const
{
  return this->maxFreq;
}

/////////////////////////////////////////////////
double WirelessReceiver::GetSensitivity() const
{
  return this->sensitivity;
}

//////////////////////////////////////////////////
void WirelessReceiver::Fini()
{
  WirelessTransceiver::Fini();
}
