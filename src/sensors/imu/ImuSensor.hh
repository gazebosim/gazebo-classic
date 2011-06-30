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

#ifndef IMUSENSOR_HH
#define IMUSENSOR_HH

#include <vector>

#include "Sensor.hh"
#include "Body.hh"

namespace gazebo
{

  class ImuSensor: public Sensor
  {
    /// \brief Constructor
    /// \param body The IMU sensor must be attached to a body.
    public: ImuSensor(Body *body);
  
    /// \brief Destructor
    public: virtual ~ImuSensor();
  
    /// \param node The XMLConfig node
    protected: virtual void LoadChild(XMLConfigNode *node);
  
    /// \brief Save the sensor info in XML format
    protected: virtual void SaveChild(std::string &prefix, std::ostream &stream);
  
    /// Initialize the ray
    protected: virtual void InitChild();
  
    ///  Update sensed values
    protected: virtual void UpdateChild();
    
    /// Finalize the ray
    protected: virtual void FiniChild();
  
    public: Pose GetVelocity();
  
    private: Pose prevPose;
    private: Pose imuVel;
  
  };
}

#endif
