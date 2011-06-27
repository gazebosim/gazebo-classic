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
/* Desc: Base class for all sensors
 * Author: Nathan Koenig
 * Date: 25 May 2007
 */

#ifndef SENSOR_HH
#define SENSOR_HH

#include <vector>

#include "sdf/interface/sdf.h"

#include "common/Events.hh"
#include "common/Time.hh"
#include "common/Param.hh"
#include "math/Pose.hh"

namespace gazebo
{
  namespace common
  {
    class XMLConfigNode;
  }

  namespace sensors
  {
    /// \addtogroup gazebo_sensor
    /// \brief Base class for sensors
    /// \{
    
    /// \brief Base class for sensors
    class Sensor
    {
      /// \brief  Constructor
      public: Sensor();
    
      /// \brief  Destructor
      public: virtual ~Sensor();
    
      ///  \brief Load the sensor
      /// \param node XMLConfigNode pointer
      public: virtual void Load( boost::shared_ptr<sdf::SDFElement> _sdf );
  
      /// \brief Save the sensor info in XML format
      public: void Save(std::string &prefix, std::ostream &stream);
  
      /// \brief  Initialize the sensor
      public: virtual void Init();
    
      /// \brief  Update the sensor
      public: virtual void Update(bool force);
    
      /// \brief  Finalize the sensor
      public: virtual void Fini();

      /// \brief Get name 
      public: std::string GetName() const;

      /// \brief Get the type of the sensor
      public: std::string GetSensorType(){return typeName;}
  
      /// \brief Get the current pose
      public: virtual math::Pose GetPose() const;
  
      /// \brief Set whether the sensor is active or not
      public: virtual void SetActive(bool value);

      public: bool IsActive();
    
      /// \brief Load a controller for this sensor
      /// \param node XML configure parameter node
      private: void LoadController(common::XMLConfigNode *node);
    
      /// \brief Pointer to the controller of the sensor
      //
      //protected: Controller *controller;
  
      /// \brief True if active
      protected: bool active;
      protected: boost::shared_ptr<sdf::SDFElement> sdf; 

      protected: math::Pose pose;
  
      protected: std::string typeName;

      protected: std::vector<event::ConnectionPtr> connections;
    };
    /// \}
  }
}
#endif
