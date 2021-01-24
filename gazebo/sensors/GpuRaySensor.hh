/*
 * Copyright (C) 2012 Open Source Robotics Foundation
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
#ifndef _GAZEBO_SENSORS_GPURAYSENSOR_HH_
#define _GAZEBO_SENSORS_GPURAYSENSOR_HH_

#include <memory>
#include <string>
#include <vector>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>

#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    // Forward declare private data pointer.
    class GpuRaySensorPrivate;

    /// \class GpuRaySensor GpuRaySensor.hh sensors/sensors.hh
    /// \addtogroup gazebo_sensors
    /// \{

    /// \brief GPU based laser sensor
    ///
    /// This sensor cast rays into the world, tests for intersections, and
    /// reports the range to the nearest object.  It is used by ranging
    /// sensor models (e.g., sonars and scanning laser range finders).
    class GZ_SENSORS_VISIBLE GpuRaySensor: public Sensor
    {
      /// \brief Constructor
      public: GpuRaySensor();

      /// \brief Destructor
      public: virtual ~GpuRaySensor();

      /// \brief Load the sensor with SDF parameters
      /// \param[in] _sdf SDF Sensor parameters
      /// \param[in] _worldName Name of world to load from
      public: virtual void Load(const std::string &_worldName,
                                sdf::ElementPtr _sdf) override;

      /// \brief Load the sensor with default parameters
      /// \param[in] _worldName Name of world to load from
      public: virtual void Load(const std::string &_worldName) override;

      /// \brief Initialize the ray
      public: virtual void Init() override;

      // Documentation inherited
      protected: virtual bool UpdateImpl(const bool _force) override;

      /// \brief Finalize the ray
      protected: virtual void Fini();

      // Documentation inherited
      public: virtual std::string Topic() const override;

      /// \brief Returns a pointer to the internally kept rendering::GpuLaser
      /// \return Pointer to GpuLaser
      public: rendering::GpuLaserPtr LaserCamera() const;

      /// \brief Get the minimum angle
      /// \return The minimum angle
      public: ignition::math::Angle AngleMin() const;

      /// \brief Set the scan minimum angle
      /// \param[in] _angle The minimum angle
      public: void SetAngleMin(const double _angle);

      /// \brief Get the maximum angle
      /// \return the maximum angle
      public: ignition::math::Angle AngleMax() const;

      /// \brief Set the scan maximum angle
      /// \param[in] _angle The maximum angle
      public: void SetAngleMax(const double _angle);

      /// \brief Get radians between each range
      /// \return Return angle resolution
      public: double AngleResolution() const;

      /// \brief Get the minimum range
      /// \return The minimum range
      public: double RangeMin() const;

      /// \brief Get the maximum range
      /// \return The maximum range
      public: double RangeMax() const;

      /// \brief Get the range resolution
      ///      If RangeResolution is 1, the number of simulated rays is equal
      ///      to the number of returned range readings. If it's less than 1,
      ///      fewer simulated rays than actual returned range readings are
      ///      used, the results are interpolated from two nearest neighbors,
      ///      and vice versa.
      /// \return The Range Resolution
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

      /// \brief Set the vertical scan bottom angle
      /// \param[in] _angle The minimum angle of the scan block
      public: void SetVerticalAngleMin(const double _angle);

      /// \brief Get the vertical scan line top angle
      /// \return The Maximum angle of the scan block
      public: ignition::math::Angle VerticalAngleMax() const;

      /// \brief Set the vertical scan line top angle
      /// \param[in] _angle The Maximum angle of the scan block
      public: void SetVerticalAngleMax(const double _angle);

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
      public: double Range(const int _index) const;

      /// \brief Get all the ranges
      /// \param[out] _range A vector that will contain all the range data
      public: void Ranges(std::vector<double> &_ranges) const;

      /// \brief Get detected retro (intensity) value for a ray.
      ///         Warning: If you are accessing all the ray data in a loop
      ///         it's possible that the Ray will update in the middle of
      ///         your access loop. This means some data will come from one
      ///         scan, and some from another scan. You can solve this
      ///         problem by using SetActive(false) <your accessor loop>
      ///         SetActive(true).
      /// \param[in] _index Index of specific ray
      /// \return Intensity value of ray
      public: double Retro(const int _index) const;

      /// \brief Get detected fiducial value for a ray.
      ///         Warning: If you are accessing all the ray data in a loop
      ///         it's possible that the Ray will update in the middle of
      ///         your access loop. This means some data will come from one
      ///         scan, and some from another scan. You can solve this
      ///         problem by using SetActive(false) <your accessor loop>
      ///         SetActive(true).
      /// \param[in] _index Index of specific ray
      /// \return Fiducial value of ray
      public: int Fiducial(const unsigned int _index) const;

      /// \brief Gets the camera count
      /// \return Number of cameras
      public: unsigned int CameraCount() const;

      /// \brief Gets if sensor is horizontal
      /// \return True if horizontal, false if not
      public: bool IsHorizontal() const;

      /// \brief Return the ratio of horizontal ray count to vertical ray
      /// count.
      ///
      /// A ray count is the number of simulated rays. Whereas a range count
      /// is the total number of data points returned. When range count
      /// != ray count, then values are interpolated between rays.
      public: double RayCountRatio() const;

      /// \brief Return the ratio of horizontal range count to vertical
      /// range count.
      ///
      /// A ray count is the number of simulated rays. Whereas a range count
      /// is the total number of data points returned. When range count
      /// != ray count, then values are interpolated between rays.
      public: double RangeCountRatio() const;

      /// \brief Get the horizontal field of view of the laser sensor.
      /// \return The horizontal field of view of the laser sensor.
      public: double HorzFOV() const;

      /// \brief Get Cos Horz field-of-view
      /// \return 2 * atan(tan(this->hfov/2) / cos(this->vfov/2))
      public: double CosHorzFOV() const;

      /// \brief Get the vertical field-of-view.
      /// \return Vertical field of view.
      public: double VertFOV() const;

      /// \brief Get Cos Vert field-of-view
      /// \return 2 * atan(tan(this->vfov/2) / cos(this->hfov/2))
      public: double CosVertFOV() const;

      /// \brief Get (horizontal_max_angle + horizontal_min_angle) * 0.5
      /// \return (horizontal_max_angle + horizontal_min_angle) * 0.5
      public: double HorzHalfAngle() const;

      /// \brief Get (vertical_max_angle + vertical_min_angle) * 0.5
      /// \return (vertical_max_angle + vertical_min_angle) * 0.5
      public: double VertHalfAngle() const;

      /// \brief Connect to the new laser frame event.
      /// \param[in] _subscriber Event callback.
      public: event::ConnectionPtr ConnectNewLaserFrame(
        std::function<void(const float *, unsigned int, unsigned int,
        unsigned int, const std::string &)> _subscriber);

      // Documentation inherited
      public: virtual bool IsActive() const override;

      /// brief Render the camera.
      private: void Render();

      /// \brief Handle the prerenderEnded event.
      private: void PrerenderEnded();

      /// \internal
      /// \brief Private data pointer.
      private: std::unique_ptr<GpuRaySensorPrivate> dataPtr;
    };
    /// \}
  }
}
#endif
