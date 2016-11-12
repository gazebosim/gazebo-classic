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
#ifndef _GAZEBO_SENSORS_RAYSENSOR_HH_
#define _GAZEBO_SENSORS_RAYSENSOR_HH_

#include <memory>
#include <string>
#include <vector>

#include <ignition/math/Angle.hh>

#include "gazebo/sensors/Sensor.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  class OgreDynamicLines;
  class Collision;
  class MultiRayShape;

  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    // Forward declare private data class.
    class RaySensorPrivate;

    /// \addtogroup gazebo_sensors
    /// \{

    /// \class RaySensor RaySensor.hh sensors/sensors.hh
    /// \brief Sensor with one or more rays.
    ///
    /// This sensor cast rays into the world, tests for intersections, and
    /// reports the range to the nearest object.  It is used by ranging
    /// sensor models (e.g., sonars and scanning laser range finders).
    class GAZEBO_VISIBLE RaySensor: public Sensor
    {
      /// \brief Constructor
      public: RaySensor();

      /// \brief Destructor
      public: virtual ~RaySensor();

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      protected: virtual bool UpdateImpl(const bool _force);

      // Documentation inherited
      protected: virtual void Fini();

      // Documentation inherited
      public: virtual std::string Topic() const;

      /// \brief Get the minimum angle
      /// \return The minimum angle object
      public: ignition::math::Angle AngleMin() const;

      /// \brief Get the maximum angle
      /// \return the maximum angle object
      public: ignition::math::Angle AngleMax() const;

      /// \brief Get the angle in radians between each range
      /// \return Resolution of the angle
      public: double AngleResolution() const;

      /// \brief Get the minimum range
      /// \return The minimum range
      public: double RangeMin() const;

      /// \brief Get the maximum range
      /// \return The maximum range
      public: double RangeMax() const;

      /// \brief Get the range resolution
      /// \return Resolution of the range
      public: double RangeResolution() const;

      /// \brief Get the ray count
      /// \return The number of rays
      public: int RayCount() const;

      /// \brief Get the range count
      /// \return The number of ranges
      public: int RangeCount() const;

      /// \brief Get the vertical scan line count
      /// \return The number of scan lines vertically
      public: int VerticalRayCount() const;

      /// \brief Get the vertical scan line count
      /// \return The number of scan lines vertically
      public: int VerticalRangeCount() const;

      /// \brief Get the vertical scan bottom angle
      /// \return The minimum angle of the scan block
      public: ignition::math::Angle VerticalAngleMin() const;

      /// \brief Get the vertical scan line top angle
      /// \return The Maximum angle of the scan block
      public: ignition::math::Angle VerticalAngleMax() const;

      /// \brief Get the vertical angle in radians between each range
      /// \return Resolution of the angle
      public: double VerticalAngleResolution() const;

      /// \brief Get detected range for a ray.
      ///         Warning: If you are accessing all the ray data in a loop
      ///         it's possible that the Ray will update in the middle of
      ///         your access loop. This means some data will come from one
      ///         scan, and some from another scan. You can solve this
      ///         problem by using SetActive(false) <your accessor loop>
      ///         SetActive(true).
      /// \param[in] _index Index of specific ray
      /// \return Returns RangeMax for no detection.
      public: double Range(const unsigned int _index) const;

      /// \brief Get all the ranges
      /// \param[out] _ranges A vector that will contain all the range data
      public: void Ranges(std::vector<double> &_ranges) const;

      /// \brief Get detected retro (intensity) value for a ray.
      ///         Warning: If you are accessing all the ray data in a loop
      ///         it's possible that the Ray will update in the middle of
      ///         your access loop. This means some data will come from one
      ///         scan, and some from another scan. You can solve this
      ///         problem by using SetActive(false) <your accessor loop>
      ///         SetActive(true).
      /// \param[in] _index Index of specific ray
      /// \return Retro (intensity) value for ray
      public: double Retro(const unsigned int _index) const;

      /// \brief Get detected fiducial value for a ray.
      ///         Warning: If you are accessing all the ray data in a loop
      ///         it's possible that the Ray will update in the middle of
      ///         your access loop. This means some data will come from one
      ///         scan, and some from another scan. You can solve this
      ///         problem by using SetActive(false) <your accessor loop>
      ///         SetActive(true).
      /// \param[in] _index Index value of specific ray
      /// \return Fiducial value
      public: int Fiducial(const unsigned int _index) const;

      /// \brief Returns a pointer to the internal physics::MultiRayShape
      /// \return Pointer to ray shape
      public: physics::MultiRayShapePtr LaserShape() const;

      // Documentation inherited
      public: virtual bool IsActive() const;

      /// \internal
      /// \brief Private data pointer.
      private: std::unique_ptr<RaySensorPrivate> dataPtr;
    };
    /// \}
  }
}

#endif
