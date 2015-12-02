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
#include "gazebo/transport/Node.hh"
#include "gazebo/transport/Publisher.hh"

#include "gazebo/sensors/WirelessReceiverPrivate.hh"
#include "gazebo/sensors/WirelessReceiver.hh"
#include "gazebo/sensors/WirelessTransmitter.hh"

using namespace gazebo;
using namespace sensors;

GZ_REGISTER_STATIC_SENSOR("wireless_receiver", WirelessReceiver)

/////////////////////////////////////////////////
WirelessReceiver::WirelessReceiver()
: WirelessTransceiver(*new WirelessReceiverPrivate),
  dataPtr(std::static_pointer_cast<WirelessReceiverPrivate>(this->dPtr))
{
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

  this->dataPtr->pub = this->dataPtr->node->Advertise<msgs::WirelessNodes>(
      this->Topic(), 30);
  GZ_ASSERT(this->dataPtr->pub != NULL,
      "wirelessReceiverSensor did not get a valid publisher pointer");

  sdf::ElementPtr transceiverElem =
    this->dataPtr->sdf->GetElement("transceiver");

  this->dataPtr->minFreq = transceiverElem->Get<double>("min_frequency");
  this->dataPtr->maxFreq = transceiverElem->Get<double>("max_frequency");
  this->dataPtr->sensitivity = transceiverElem->Get<double>("sensitivity");

  if (this->dataPtr->minFreq <= 0)
  {
    gzthrow("Wireless receiver min. frequency must be > 0. Current value is ["
      << this->dataPtr->minFreq << "]");
  }

  if (this->dataPtr->maxFreq <= 0)
  {
    gzthrow("Wireless receiver max. frequency must be > 0. Current value is ["
      << this->dataPtr->maxFreq << "]");
  }

  if (this->dataPtr->minFreq > this->dataPtr->maxFreq)
  {
    gzthrow("Wireless receiver min. frequency must be less or equal than max. "
        << "frequency. Current min. frequency is [" << this->dataPtr->minFreq <<
        "] and max frequency is [" << this->dataPtr->maxFreq << "]");
  }

  if (this->dataPtr->sensitivity >= 0)
  {
    gzthrow("Wireless receiver sensitivity must be < 0. Current value is ["
      << this->dataPtr->sensitivity << "]");
  }
}

//////////////////////////////////////////////////
bool WirelessReceiver::UpdateImpl(bool /*_force*/)
{
  std::string txEssid;
  msgs::WirelessNodes msg;
  double rxPower;
  double txFreq;

  this->dataPtr->referencePose = this->dataPtr->pose +
    this->dataPtr->parentEntity.lock()->GetWorldPose().Ign();

  ignition::math::Pose3d myPos = this->dataPtr->referencePose;
  Sensor_V sensors = SensorManager::Instance()->GetSensors();
  for (Sensor_V::iterator it = sensors.begin(); it != sensors.end(); ++it)
  {
    if ((*it)->Type() == "wireless_transmitter")
    {
      boost::shared_ptr<gazebo::sensors::WirelessTransmitter> transmitter =
          boost::static_pointer_cast<WirelessTransmitter>(*it);

      txFreq = transmitter->Freq();
      rxPower = transmitter->SignalStrength(myPos, this->Gain());

      // Discard if the frequency received is out of our frequency range,
      // or if the received signal strengh is lower than the sensivity
      if ((txFreq < this->MinFreqFiltered()) ||
          (txFreq > this->MaxFreqFiltered()) ||
          (rxPower < this->Sensitivity()))
      {
        continue;
      }

      txEssid = transmitter->ESSID();

      msgs::WirelessNode *wirelessNode = msg.add_node();
      wirelessNode->set_essid(txEssid);
      wirelessNode->set_frequency(txFreq);
      wirelessNode->set_signal_level(rxPower);
    }
  }
  if (msg.node_size() > 0)
  {
    this->dataPtr->pub->Publish(msg);
  }

  return true;
}

/////////////////////////////////////////////////
double WirelessReceiver::GetMinFreqFiltered() const
{
  return this->MinFreqFiltered();
}

/////////////////////////////////////////////////
double WirelessReceiver::MinFreqFiltered() const
{
  return this->dataPtr->minFreq;
}

/////////////////////////////////////////////////
double WirelessReceiver::GetMaxFreqFiltered() const
{
  return this->MaxFreqFiltered();
}

/////////////////////////////////////////////////
double WirelessReceiver::MaxFreqFiltered() const
{
  return this->dataPtr->maxFreq;
}

/////////////////////////////////////////////////
double WirelessReceiver::GetSensitivity() const
{
  return this->Sensitivity();
}

/////////////////////////////////////////////////
double WirelessReceiver::Sensitivity() const
{
  return this->dataPtr->sensitivity;
}

//////////////////////////////////////////////////
void WirelessReceiver::Fini()
{
  WirelessTransceiver::Fini();
}
