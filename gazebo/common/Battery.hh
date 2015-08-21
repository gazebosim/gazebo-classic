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
#include <functional>
#include <memory>

#include "sdf/sdf.hh"
#include "gazebo/common/CommonTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    // Forward declare private data class.
    class BatteryPrivate;

    /// \addtogroup gazebo_common
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
    class GZ_COMMON_VISIBLE Battery :
      public std::enable_shared_from_this<Battery>
    {
      /// \brief Typedef the powerload map.
      /// \sa SetUpdateFunc
      public: typedef std::map<uint32_t, double> PowerLoad_M;

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
      /// \return The name of the battery.
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
      public: const PowerLoad_M &PowerLoads() const;

      /// \brief Get the real voltage in volts.
      /// \return Voltage.
      public: double Voltage() const;

      /// \brief Setup function to update voltage.
      /// \param[in] _updateFunc The update function callback that is used
      /// to modify the battery's voltage. The parameter to the update
      /// function callback is a reference to an instance of
      /// Battery::UpdateData. The update function must return the new
      /// battery voltage as a double.
      /// \sa UpdateData
      public: void SetUpdateFunc(
                  std::function<double (const BatteryPtr)> _updateFunc);

      /// \brief Update the battery. This will in turn trigger the function
      /// set using the SetUpdateFunc function.
      /// \sa SetUpdateFunc.
      public: void Update();

      /// \brief Initialize the list of consumers.
      protected: void InitConsumers();

      /// \brief Update voltage using an ideal battery model.
      /// \param[in] _data Pointer to the battery.
      /// \return New battery voltage.
      private: double UpdateDefault(const BatteryPtr _battery);

      /// \internal
      /// \brief Private data pointer.
      private: BatteryPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
