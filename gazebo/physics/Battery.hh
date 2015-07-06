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
#ifndef _BATTERY_HH_
#define _BATTERY_HH_

#include <map>
#include <string>

#include "gazebo/msgs/msgs.hh"
#include "gazebo/math/Pose.hh"
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

      /// \brief Initial power load in watts.
      public: double initPowerLoad;

      /// \brief Real voltage in volts.
      public: double curVoltage;

      /// \brief A list of power loads in watts.
      public: std::map<uint32_t, double> powerLoads;

      /// \brief The function used to to update the real voltage.
      public: boost::function<
        double (double, const std::map<uint32_t, double> &)> updateFunc;
    };

    /// \class Battery Battery.hh physics/physics.hh
    /// \brief A battery abstraction
    ///
    /// A battery is a link that acts as a battery.
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

      /// \brief Get the parent link.
      /// \return Pointer to the parent link.
      public: LinkPtr GetLink() const;

      /// \brief Create a unique consumer.
      /// \return Unique consumer identifier.
      public: uint32_t AddConsumer();

      /// \brief Remove a consumer.
      /// \param[in] _consumerId Unique consumer identifier.
      public: void RemoveConsumer(uint32_t _consumerId);

      /// \brief Set consumer power load in watts.
      /// \param[in] _consumerId Unique consumer identifier.
      /// \param[in] _powerLoad Power load in watts.
      public: void SetPowerLoad(uint32_t _consumerId, double _powerLoad);

      /// \brief Get consumer power load in watts.
      /// \param[in] _consumerId Unique consumer identifier.
      /// \return Power load in watts.
      public: double GetPowerLoad(uint32_t _consumerId) const;

      /// \brief Get list of power loads in watts.
      /// \return List of power loads in watts.
      public: const std::map<uint32_t, double>& GetPowerLoads() const;

      /// \brief Get the current voltage in volts.
      /// \return Voltage.
      public: double GetVoltage() const;

      /// \brief Setup function to update voltage.
      public: template<typename T>
              void SetUpdateFunc(T _updateFunc);

      /// \brief Initialize the list of consumers.
      private: void InitConsumers();

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
