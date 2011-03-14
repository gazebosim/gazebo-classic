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
 * SVN: $Id$
 */

#ifndef SENSOR_HH
#define SENSOR_HH

#include "common/Param.hh"
#include "common/Pose3d.hh"
#include "Entity.hh"

namespace gazebo
{
  class XMLConfigNode;
  class Body;
  class World;
  class Simulator;
  class Controller;

  /// \addtogroup gazebo_sensor
  /// \brief Base class for sensors
  /// \{
  
  /// \brief Base class for sensors
  class Sensor : public Entity
  {
    /// \brief  Constructor
    public: Sensor(Body *body);
  
    /// \brief  Destructor
    public: virtual ~Sensor();
  
    ///  \brief Load the sensor
    /// \param node XMLConfigNode pointer
    public: virtual void Load(XMLConfigNode *node);

    /// \brief Save the sensor info in XML format
    public: void Save(std::string &prefix, std::ostream &stream);

    /// \brief Child save function
    protected: virtual void SaveChild(std::string &prefix,std::ostream &stream) {}
  
    /// \brief  Initialize the sensor
    public: void Init();
  
    /// \brief  Update the sensor
    public: void Update();
  
    /// \brief  Finalize the sensor
    public: void Fini();

    /// \brief Get the type of the sensor
    public: std::string GetSensorType(){return typeName;}

    /// \brief Get the current pose
    public: virtual Pose3d GetPose() const;

    /// \brief Get the name of the interfaces define in the sensor controller
    public: void GetInterfaceNames(std::vector<std::string>& list) const;
 
    /// \brief Set whether the sensor is active or not
    public: virtual void SetActive(bool value);
    public: bool IsActive();

    /// \brief Get the time of the last update
    public: Time GetLastUpdate();

    /// \brief  Load the child sensor
    protected: virtual void LoadChild(XMLConfigNode * /*node*/) {};
  
    /// \brief  Initialize the child
    protected: virtual void InitChild() {};
  
    /// \brief  Update the child
    protected: virtual void UpdateChild() {};
  
    /// \brief Finalize the child
    protected: virtual void FiniChild() {};
  
    /// \brief Load a controller for this sensor
    /// \param node XML configure parameter node
    private: void LoadController(XMLConfigNode *node);
  
    /// The body this sensor is attached to
    protected: Body *body;
    protected: World *world;
    protected: Simulator *simulator;

    /// \brief Pointer to the controller of the sensor
    protected: Controller *controller;

    /// \brief True if active
    protected: bool active;

    protected: ParamT<double> *updateRateP;
    protected: ParamT<bool> *alwaysActiveP;
    protected: Time updatePeriod;
    protected: Time lastUpdate;
    protected: std::string typeName;
  };
  /// \}
}
#endif
