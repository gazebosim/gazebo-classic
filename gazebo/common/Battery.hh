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
#ifndef _GAZEBO_BATTERY_HH_
#define _GAZEBO_BATTERY_HH_

#include <map>
#include <string>

#include "sdf/sdf.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    // Forward declare private data class.
    class BatteryPrivate;

    /// \addtogroup gazebo_physics
    /// \{

    /// \brief A battery abstraction
    ///
    /// The default battery model is ideal: It just takes the initial voltage
    /// value as its constant voltage value. This behavior can be changed by
    /// specifying a custom update function.
    ///
    /// The battery handles a list of consumers. It updates itself after each
    /// simulation iteration. The update function takes the power loads for each
    /// consumer and current voltage value as inputs and returns a new voltage
    /// value.
    class GZ_PHYSICS_VISIBLE Battery
    {
      /// \brief Constructor
      public: explicit Battery();

      /// \brief Destructor.
      public: virtual ~Battery();

      /// \brief Load the battery.
      /// \param[in] _sdf Shared point to an sdf element.
      public: virtual void Load(sdf::ElementPtr _sdf);

      /// \brief Initialize.
      public: virtual void Init();

      /// \brief Update the parameters using new sdf values.
      /// \param[in] _sdf SDF values to update from.
      public: virtual void UpdateParameters(const sdf::ElementPtr _sdf);

      /// \brief Return the name of the battery.
      public: std::string Name() const;

      /// \brief Create a unique consumer.
      /// \return Unique consumer identifier.
      public: uint32_t AddConsumer();

      /// \brief Remove a consumer.
      /// \param[in] _consumerId Unique consumer identifier.
      public: void RemoveConsumer(const uint32_t _consumerId);

      /// \brief Set consumer power load in watts.
      /// \param[in] _consumerId Unique consumer identifier.
      /// \param[in] _powerLoad Power load in watts.
      /// \return True if setting the power load consumption was successful.
      public: bool SetPowerLoad(const uint32_t _consumerId,
                                const double _powerLoad);

      /// \brief Get consumer power load in watts.
      /// \param[in] _consumerId Unique consumer identifier.
      /// \param[out] _powerLoad Power load consumption in watts.
      /// \return True if getting the power load consumption was successful.
      public: bool PowerLoad(const uint32_t _consumerId,
                             double &_powerLoad) const;

      /// \brief Get list of power loads in watts.
      /// \return List of power loads in watts.
      public: const std::map<uint32_t, double>& PowerLoads() const;

      /// \brief Get the real voltage in volts.
      /// \return Voltage.
      public: double Voltage() const;

      /// \brief Setup function to update voltage.
      public: template<typename T>
              void SetUpdateFunc(T _updateFunc);

      /// \brief Update the battery.
      public: void Update();

      /// \brief Initialize the list of consumers.
      protected: void InitConsumers();

      /// \brief Update voltage using an ideal battery model.
      private: double UpdateDefault(const double _voltage,
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
