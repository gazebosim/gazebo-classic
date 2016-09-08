/*
 * Copyright (C) 2012-2016 Open Source Robotics Foundation
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
: WirelessTransceiver(),
  dataPtr(new WirelessReceiverPrivate)
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

  this->pub = this->node->Advertise<msgs::WirelessNodes>(
      this->Topic(), 30);
  GZ_ASSERT(this->pub != nullptr,
      "wirelessReceiverSensor did not get a valid publisher pointer");

  sdf::ElementPtr transceiverElem =
    this->sdf->GetElement("transceiver");

  this->dataPtr->minFreq = transceiverElem->Get<double>("min_frequency");
  this->dataPtr->maxFreq = transceiverElem->Get<double>("max_frequency");
  this->dataPtr->sensitivity = transceiverElem->Get<double>("sensitivity");

  if (this->dataPtr->minFreq <= 0)
  {
    gzerr << "Attempting to set a wireless receiver min frequency"
      << "of [" << this->dataPtr->minFreq << "]. Value must be > 0, "
      << "using a value of 1.0\n";
    this->dataPtr->minFreq = 1.0;
  }

  if (this->dataPtr->maxFreq <= 0)
  {
    gzerr << "Attempting to set a wireless receiver max frequency"
      << "of [" << this->dataPtr->maxFreq << "]. Value must be > 0, "
      << "using a value of 1.0\n";
    this->dataPtr->maxFreq = 1.0;
  }

  if (this->dataPtr->minFreq > this->dataPtr->maxFreq)
  {
    gzerr << "Wireless receiver min frequency[" << this->dataPtr->minFreq << "]"
      << " is greater than max frequency[" << this->dataPtr->maxFreq << "]."
      << " Swapping the values.\n";
    std::swap(this->dataPtr->maxFreq, this->dataPtr->minFreq);
  }

  if (this->dataPtr->sensitivity >= 0)
  {
    gzerr << "Attempting to set a wireless sensitivity "
      << "of [" << this->dataPtr->sensitivity << "]. Value must be > 0, "
      << "using a value of 1.0\n";
    this->dataPtr->sensitivity = 1.0;
  }
}

//////////////////////////////////////////////////
bool WirelessReceiver::UpdateImpl(const bool /*_force*/)
{
  std::string txEssid;
  msgs::WirelessNodes msg;
  double rxPower;
  double txFreq;

  this->referencePose = this->pose +
    this->parentEntity.lock()->GetWorldPose().Ign();

  ignition::math::Pose3d myPos = this->referencePose;
  Sensor_V sensors = SensorManager::Instance()->GetSensors();
  for (Sensor_V::iterator it = sensors.begin(); it != sensors.end(); ++it)
  {
    if ((*it)->Type() == "wireless_transmitter")
    {
      std::shared_ptr<gazebo::sensors::WirelessTransmitter> transmitter =
          std::static_pointer_cast<WirelessTransmitter>(*it);

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
    this->pub->Publish(msg);
  }

  return true;
}

/////////////////////////////////////////////////
double WirelessReceiver::MinFreqFiltered() const
{
  return this->dataPtr->minFreq;
}

/////////////////////////////////////////////////
double WirelessReceiver::MaxFreqFiltered() const
{
  return this->dataPtr->maxFreq;
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
