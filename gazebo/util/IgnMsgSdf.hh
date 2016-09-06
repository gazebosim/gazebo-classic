/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_UTIL_IGNMSGSDF_HH_
#define GAZEBO_UTIL_IGNMSGSDF_HH_

#include <string>

#include <ignition/msgs/MessageTypes.hh>
#include <sdf/sdf.hh>

#include "gazebo/common/Color.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace util
  {
    /// \brief Create an ignition::msgs from an SDF element.
    /// \param[in] _sdf The sdf element.
    /// \return The new ignition::msgs object.
    template<typename T>
    GAZEBO_VISIBLE
    T Convert(const sdf::ElementPtr /*_sdf*/)
    {
      gzerr << "Invalid convertion of SDF to type["
            << typeid(T).name() << "]\n";
      return T();
    }

    /// \brief Get an ignition::msgs enum from a string.
    /// \param[in] _str The string.
    /// \return The ignition::msg enum.
    template<typename T>
    GAZEBO_VISIBLE
    T Convert(const std::string &_str)
    {
      gzerr << "Invalid convertion of string [" << _str << "] to type ["
            << typeid(T).name() << "]\n";
      return T();
    }

    /// \brief Create or update an SDF element from ignition::msgs::Plugin.
    /// \param[in] _msg Plugin messsage.
    /// \param[in] _sdf If supplied, performs an update from _msg instead of
    /// creating a new sdf element.
    /// \return The new SDF element.
    GAZEBO_VISIBLE
    sdf::ElementPtr Convert(const ignition::msgs::Plugin &_msg,
                            sdf::ElementPtr _sdf = sdf::ElementPtr());

    // Specialization of conversion from SDF to ignition message for plugins.
    template<>
    ignition::msgs::Plugin Convert(const sdf::ElementPtr _sdf);

    /// \brief Create or update an SDF element from ignition::msgs::Visual.
    /// \param[in] _msg Visual messsage.
    /// \param[in] _sdf If supplied, performs an update from _msg instead of
    /// creating a new sdf element.
    /// \return The new SDF element.
    GAZEBO_VISIBLE
    sdf::ElementPtr Convert(const ignition::msgs::Visual &_msg,
                            sdf::ElementPtr _sdf = sdf::ElementPtr());

    // Specialization of conversion from SDF to ignition message for visuals.
    template<>
    ignition::msgs::Visual Convert(const sdf::ElementPtr _sdf);

    /// \brief Create or update an SDF element from ignition::msgs::Material.
    /// \param[in] _msg Material messsage.
    /// \param[in] _sdf If supplied, performs an update from _msg instead of
    /// creating a new sdf element.
    /// \return The new SDF element.
    GAZEBO_VISIBLE
    sdf::ElementPtr Convert(const ignition::msgs::Material &_msg,
                            sdf::ElementPtr _sdf = sdf::ElementPtr());

    // Specialization of conversion from SDF to ignition message for materials.
    template<>
    ignition::msgs::Material Convert(const sdf::ElementPtr _sdf);

    /// \brief Convert an ignition::msgs::Material::ShaderType to a string.
    /// \param[in] _type An ignition::msgs::Material::ShaderType enum.
    /// \return Shader type string. Returns "unknown" if _type is unrecognized.
    GAZEBO_VISIBLE
    std::string Convert(const ignition::msgs::Material::ShaderType &_type);

    // Specialization of conversion from string to ignition message enum for
    // shader type.
    template<>
    ignition::msgs::Material::ShaderType Convert(const std::string &_str);

    /// \brief Create or update an SDF element from ignition::msgs::Geometry.
    /// \param[in] _msg Geometry messsage.
    /// \param[in] _sdf If supplied, performs an update from _msg instead of
    /// creating a new sdf element.
    /// \return The new SDF element.
    GAZEBO_VISIBLE
    sdf::ElementPtr Convert(const ignition::msgs::Geometry &_msg,
                            sdf::ElementPtr _sdf = sdf::ElementPtr());

    // Specialization of conversion from SDF to ignition message for geometries.
    template<>
    ignition::msgs::Geometry Convert(const sdf::ElementPtr _sdf);

    /// \brief Create or update an SDF element from ignition::msgs::MeshGeom.
    /// \param[in] _msg Mesh messsage.
    /// \param[in] _sdf If supplied, performs an update from _msg instead of
    /// creating a new sdf element.
    /// \return The new SDF element.
    GAZEBO_VISIBLE
    sdf::ElementPtr Convert(const ignition::msgs::MeshGeom &_msg,
                            sdf::ElementPtr _sdf = sdf::ElementPtr());

    // Specialization of conversion from SDF to ignition message for mesh
    // geometries.
    template<>
    ignition::msgs::MeshGeom Convert(const sdf::ElementPtr _sdf);

    /// \brief Create or update an SDF element from
    /// ignition::msgs::CameraSensor.
    /// \param[in] _msg Sensor messsage.
    /// \param[in] _sdf If supplied, performs an update from _msg instead of
    /// creating a new sdf element.
    /// \return The new SDF element.
    GAZEBO_VISIBLE
    sdf::ElementPtr Convert(const ignition::msgs::CameraSensor &_msg,
                            sdf::ElementPtr _sdf = sdf::ElementPtr());

    // Specialization of conversion from SDF to ignition message for sensors.
    template<>
    ignition::msgs::CameraSensor Convert(const sdf::ElementPtr _sdf);

    // Specialization of conversion from SDF to ignition message for sensors.
    template<>
    ignition::msgs::ContactSensor Convert(const sdf::ElementPtr _sdf);

    /// \brief Create or update an SDF element from ignition::msgs::GPSSensor.
    /// \param[in] _msg Sensor messsage.
    /// \param[in] _sdf If supplied, performs an update from _msg instead of
    /// creating a new sdf element.
    /// \return The new SDF element.
    GAZEBO_VISIBLE
    sdf::ElementPtr Convert(const ignition::msgs::GPSSensor &_msg,
                            sdf::ElementPtr _sdf = sdf::ElementPtr());

    // Specialization of conversion from SDF to ignition message for sensors.
    template<>
    ignition::msgs::GPSSensor Convert(const sdf::ElementPtr _sdf);

    /// \brief Create or update an SDF element from ignition::msgs::IMUSensor.
    /// \param[in] _msg Sensor messsage.
    /// \param[in] _sdf If supplied, performs an update from _msg instead of
    /// creating a new sdf element.
    /// \return The new SDF element.
    GAZEBO_VISIBLE
    sdf::ElementPtr Convert(const ignition::msgs::IMUSensor &_msg,
                            sdf::ElementPtr _sdf = sdf::ElementPtr());

    // Specialization of conversion from SDF to ignition message for sensors.
    template<>
    ignition::msgs::IMUSensor Convert(const sdf::ElementPtr _sdf);

    /// \brief Create or update an SDF element from
    /// ignition::msgs::LogicalCameraSensor.
    /// \param[in] _msg Sensor messsage.
    /// \param[in] _sdf If supplied, performs an update from _msg instead of
    /// creating a new sdf element.
    /// \return The new SDF element.
    GAZEBO_VISIBLE
    sdf::ElementPtr Convert(const ignition::msgs::LogicalCameraSensor &_msg,
                            sdf::ElementPtr _sdf = sdf::ElementPtr());

    // Specialization of conversion from SDF to ignition message for sensors.
    template<>
    ignition::msgs::LogicalCameraSensor Convert(const sdf::ElementPtr _sdf);

    // Specialization of conversion from SDF to ignition message for sensors.
    template<>
    ignition::msgs::RaySensor Convert(const sdf::ElementPtr _sdf);

    // Specialization of conversion from SDF to ignition message for sensors.
    template<>
    ignition::msgs::Sensor Convert(const sdf::ElementPtr _sdf);

    /// \brief Create or update an SDF element from ignition::msgs::SensorNoise.
    /// \param[in] _msg Sensor noise messsage.
    /// \param[in] _sdf If supplied, performs an update from _msg instead of
    /// creating a new sdf element.
    /// \return The new SDF element.
    GAZEBO_VISIBLE
    sdf::ElementPtr Convert(const ignition::msgs::SensorNoise &_msg,
                            sdf::ElementPtr _sdf = sdf::ElementPtr());

    // Specialization of conversion from SDF to ignition message for sensors.
    template<>
    ignition::msgs::SensorNoise Convert(const sdf::ElementPtr _sdf);

    /// \brief Convert a common::Color to ignition::msgs::Color.
    /// \param[in] _c The color to convert,
    /// \return An ignition::msgs::Color object,
    GAZEBO_VISIBLE
    ignition::msgs::Color Convert(const gazebo::common::Color &_c);

    /// \brief Convert an ignition::msgs::Color to a gazebo::common::Color.
    /// \param[in] _c The color to convert.
    /// \return A gazebo::common::Color object.
    GAZEBO_VISIBLE
    gazebo::common::Color Convert(const ignition::msgs::Color &_c);
  }
}
#endif
