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

#include "gazebo/math/Rand.hh"
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
  // Initialization
  this->minFreq = 2412.0;
  this->maxFreq = 2484.0;
  this->sensitivity = -90.0;
}

/////////////////////////////////////////////////
void WirelessReceiver::Load(const std::string &_worldName)
{
  WirelessTransceiver::Load(_worldName);
  this->pub = this->node->Advertise<msgs::WirelessNodes>(this->GetTopic(), 30);

  this->entity = this->world->GetEntity(this->parentName);
  GZ_ASSERT(this->entity != NULL, "Unable to get the parent entity.");

  if (this->sdf->HasElement("transceiver"))
  {
    sdf::ElementPtr transElem = this->sdf->GetElement("transceiver");

    if (transElem->HasElement("min_frequency"))
    {
      this->minFreq = transElem->Get<double>("min_frequency");
    }

    if (transElem->HasElement("max_frequency"))
    {
      this->maxFreq = transElem->Get<double>("max_frequency");
    }

    if (transElem->HasElement("sensitivity"))
    {
      this->sensitivity = transElem->Get<double>("sensitivity");
    }

    this->sensitivity = transElem->Get<double>("sensitivity");
  } 
}

//////////////////////////////////////////////////
void WirelessReceiver::UpdateImpl(bool /*_force*/)
{
  if (this->pub)
  {
    std::string txEssid;
    msgs::WirelessNodes msg;
    double rxPower;
    double txFreq;

    math::Pose myPos = this->entity->GetWorldPose();
    Sensor_V sensors = SensorManager::Instance()->GetSensors();
    for (Sensor_V::iterator it = sensors.begin(); it != sensors.end(); ++it)
    {
      if ((*it)->GetType() == "wireless_transmitter")
      {
        boost::shared_ptr<gazebo::sensors::WirelessTransmitter> transmitter = 
            boost::static_pointer_cast<WirelessTransmitter>(*it);

        txFreq = transmitter->GetFreq();
        rxPower = transmitter->GetSignalStrength(myPos, this->GetGain());

        // Disgard if the frequency received is out of our frequency range,
        // or if the received signal strengh is lower than the sensivity
        if ((txFreq < this->GetLowerFreqFiltered()) ||
            (txFreq > this->GetHigherFreqFiltered()) ||
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
  }
}

/////////////////////////////////////////////////
double WirelessReceiver::GetLowerFreqFiltered() const
{
  return this->minFreq;
}

/////////////////////////////////////////////////
double WirelessReceiver::GetHigherFreqFiltered() const
{
  return this->maxFreq;
}

/////////////////////////////////////////////////
double WirelessReceiver::GetSensitivity() const
{
  return this->sensitivity;
}
