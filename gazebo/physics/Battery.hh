/*
 * Copyright (C) 2015 Open Source Robotics Foundation
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
#ifndef _BATTERY_HH_
#define _BATTERY_HH_

#include <map>
#include <string>

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \internal
    /// \brief Private data class for Battery.
    /// This must be in the header due to templatization.
    class BatteryPrivate
    {
      /// \brief Link that contains this battery.
      public: physics::LinkPtr link;

      /// \brief Event connection.
      public: event::ConnectionPtr connection;

      /// \brief Initial voltage in volts.
      public: double initVoltage;

      /// \brief Real voltage in volts.
      public: double curVoltage;

      /// \brief Map of unique consumer ID to power loads in watts.
      public: std::map<uint32_t, double> powerLoads;

      /// \brief The function used to to update the real voltage.
      /// It takes as inputs current voltage and list of power loads.
      public: boost::function<
        double (double, const std::map<uint32_t, double> &)> updateFunc;

      /// \brief Name of the battery.
      public: std::string name;
    };

    /// \class Battery Battery.hh physics/physics.hh
    /// \brief A battery abstraction
    ///
    /// A battery is a link that acts as a battery.
    ///
    /// A plugin can access a battery with Link::GetBattery(). The default
    /// battery model is ideal: It just takes the initial voltage value as its
    /// constant voltage value. This behavior can be changed by specifying a
    /// custom Update function with Battery::SetUpdateFunc(T _updateFunc).
    /// The current voltage value can be requested at any time with
    /// Battery::GetVoltage().
    ///
    /// The battery handles a list of consumers. The function
    /// Battery::AddConsumer() creates a new consumer and returns a unique
    /// consumer identifier. The power load for a consumer can then be specified
    /// by calling
    /// Battery::SetPowerLoad(uint32_t _consumerId, double _powerLoad).
    ///
    /// The battery updates itself after each simulation iteration. It takes the
    /// power loads for each consumer and current voltage value as inputs and
    /// returns a new voltage value.
    class GZ_PHYSICS_VISIBLE Battery
    {
      /// \brief Constructor
      /// \param[in] _link The link which contains the Battery.
      public: explicit Battery(LinkPtr _link);

      /// \brief Destructor.
      public: virtual ~Battery();

      /// \brief Load the battery.
      /// \param[in] _sdf Shared point to an sdf element.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize.
      public: virtual void Init();

      /// \brief Update the parameters using new sdf values.
      /// \param[in] _sdf SDF values to update from.
      public: virtual void UpdateParameters(sdf::ElementPtr _sdf);

      /// \brief Return the name of the battery.
      public: std::string Name() const;

      /// \brief Get the parent link.
      /// \return Pointer to the parent link.
      public: LinkPtr Link() const;

      /// \brief Create a unique consumer.
      /// \return Unique consumer identifier.
      public: uint32_t AddConsumer();

      /// \brief Remove a consumer.
      /// \param[in] _consumerId Unique consumer identifier.
      public: void RemoveConsumer(uint32_t _consumerId);

      /// \brief Set consumer power load in watts.
      /// \param[in] _consumerId Unique consumer identifier.
      /// \param[in] _powerLoad Power load in watts.
      /// \return True if setting the power load consumption was successful.
      public: bool SetPowerLoad(uint32_t _consumerId, double _powerLoad);

      /// \brief Get consumer power load in watts.
      /// \param[in] _consumerId Unique consumer identifier.
      /// \param[out] _powerLoad Power load consumption in watts.
      /// \return True if getting the power load consumption was successful.
      public: bool PowerLoad(uint32_t _consumerId, double &_powerLoad) const;

      /// \brief Get list of power loads in watts.
      /// \return List of power loads in watts.
      public: const std::map<uint32_t, double>& PowerLoads() const;

      /// \brief Get the current voltage in volts.
      /// \return Voltage.
      public: double Voltage() const;

      /// \brief Setup function to update voltage.
      public: template<typename T>
              void SetUpdateFunc(T _updateFunc);

      /// \brief Initialize the list of consumers.
      protected: void InitConsumers();

      /// \brief Update the battery.
      private: void OnUpdate();

      /// \brief Update voltage using an ideal battery model.
      private: double UpdateDefault(double _voltage,
                 const std::map<uint32_t, double> &_powerLoads);

      /// \internal
      /// \brief Private data pointer.
      private: BatteryPrivate *dataPtr;
    };

    template<typename T>
    void Battery::SetUpdateFunc(T _updateFunc)
    {
      this->dataPtr->updateFunc = _updateFunc;
    }
    /// \}
  }
}
#endif
