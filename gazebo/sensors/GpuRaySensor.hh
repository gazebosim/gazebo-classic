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
/* Desc: RaySensor proximity sensor
 * Author: Mihai Emanuel Dolha
 * Date: 29 March 2012
*/

#ifndef _GPURAYSENSOR_HH_
#define _GPURAYSENSOR_HH_

#include <vector>
#include <string>
#include <boost/function.hpp>
#include <boost/thread/mutex.hpp>

#include <ignition/math/Angle.hh>
#include <ignition/math/Pose3.hh>

#include "gazebo/math/Angle.hh"
#include "gazebo/math/Pose.hh"
#include "gazebo/transport/TransportTypes.hh"
#include "gazebo/sensors/Sensor.hh"
#include "gazebo/rendering/RenderTypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    /// \class GpuRaySensor GpuRaySensor.hh sensors/sensors.hh
    /// \addtogroup gazebo_sensors
    /// \{

    /// \brief GPU based laser sensor
    ///
    /// This sensor cast rays into the world, tests for intersections, and
    /// reports the range to the nearest object.  It is used by ranging
    /// sensor models (e.g., sonars and scanning laser range finders).
    class GAZEBO_VISIBLE GpuRaySensor: public Sensor
    {
      /// \brief Constructor
      public: GpuRaySensor();

      /// \brief Destructor
      public: virtual ~GpuRaySensor();

      /// \brief Load the sensor with SDF parameters
      /// \param[in] _sdf SDF Sensor parameters
      /// \param[in] _worldName Name of world to load from
      public: virtual void Load(const std::string &_worldName,
                                sdf::ElementPtr _sdf);

      /// \brief Load the sensor with default parameters
      /// \param[in] _worldName Name of world to load from
      public: virtual void Load(const std::string &_worldName);

      /// \brief Initialize the ray
      public: virtual void Init();

      // Documentation inherited
      protected: virtual bool UpdateImpl(bool _force);

      /// \brief Finalize the ray
      protected: virtual void Fini();

      // Documentation inherited
      public: virtual std::string GetTopic() const;

      /// \brief Returns a pointer to the internally kept rendering::GpuLaser
      /// \return Pointer to GpuLaser
      public: rendering::GpuLaserPtr GetLaserCamera() const
              {return this->laserCam;}

      /// \brief Get the minimum angle
      /// \return The minimum angle
      /// \deprecated See AngleMin() function that returns an
      /// ignition::math::Angle object.
      public: math::Angle GetAngleMin() const GAZEBO_DEPRECATED(6.0);

      /// \brief Get the minimum angle
      /// \return The minimum angle
      public: ignition::math::Angle AngleMin() const;

      /// \brief Set the scan minimum angle
      /// \param[in] _angle The minimum angle
      public: void SetAngleMin(double _angle);

      /// \brief Get the maximum angle
      /// \return the maximum angle
      /// \deprecated See AngleMax() function that returns an
      /// ignition::math::Angle object.
      public: math::Angle GetAngleMax() const GAZEBO_DEPRECATED(6.0);

      /// \brief Get the maximum angle
      /// \return the maximum angle
      public: ignition::math::Angle AngleMax() const;

      /// \brief Set the scan maximum angle
      /// \param[in] _angle The maximum angle
      public: void SetAngleMax(double _angle);

      /// \brief Get radians between each range
      public: double GetAngleResolution() const;

      /// \brief Get the minimum range
      /// \return The minimum range
      public: double GetRangeMin() const;

      /// \brief Get the maximum range
      /// \return The maximum range
      public: double GetRangeMax() const;

      /// \brief Get the range resolution
      ///      If RangeResolution is 1, the number of simulated rays is equal
      ///      to the number of returned range readings. If it's less than 1,
      ///      fewer simulated rays than actual returned range readings are
      ///      used, the results are interpolated from two nearest neighbors,
      ///      and vice versa.
      /// \return The Range Resolution
      public: double GetRangeResolution() const;

      /// \brief Get the ray count
      /// \return The number of rays
      public: int GetRayCount() const;

      /// \brief Get the range count
      /// \return The number of ranges
      public: int GetRangeCount() const;

      /// \brief Get the vertical scan line count
      /// \return The number of scan lines vertically
      public: int GetVerticalRayCount() const;

      /// \brief Get the vertical scan line count
      /// \return The number of scan lines vertically
      public: int GetVerticalRangeCount() const;

      /// \brief Get the vertical scan bottom angle
      /// \return The minimum angle of the scan block
      /// \deprecated See VerticalAngleMin() function that returns an
      /// ignition::math::Angle object.
      public: math::Angle GetVerticalAngleMin() const GAZEBO_DEPRECATED(6.0);

      /// \brief Get the vertical scan bottom angle
      /// \return The minimum angle of the scan block
      public: ignition::math::Angle VerticalAngleMin() const;

      /// \brief Set the vertical scan bottom angle
      /// \param[in] _angle The minimum angle of the scan block
      public: void SetVerticalAngleMin(double _angle);

      /// \brief Get the vertical scan line top angle
      /// \return The Maximum angle of the scan block
      /// \deprecated See VerticalAngleMax() function that returns an
      /// ignition::math::Angle object.
      public: math::Angle GetVerticalAngleMax() const GAZEBO_DEPRECATED(6.0);

      /// \brief Get the vertical scan line top angle
      /// \return The Maximum angle of the scan block
      public: ignition::math::Angle VerticalAngleMax() const;

      /// \brief Set the vertical scan line top angle
      /// \param[in] _angle The Maximum angle of the scan block
      public: void SetVerticalAngleMax(double _angle);

      /// \brief Get the vertical angle in radians between each range
      /// \return Resolution of the angle
      public: double GetVerticalAngleResolution() const;

      /// \brief Get detected range for a ray.
      ///         Warning: If you are accessing all the ray data in a loop
      ///         it's possible that the Ray will update in the middle of
      ///         your access loop. This means some data will come from one
      ///         scan, and some from another scan. You can solve this
      ///         problem by using SetActive(false) <your accessor loop>
      ///         SetActive(true).
      /// \param[in] _index Index of specific ray
      /// \return Returns RangeMax for no detection.
      public: double GetRange(int _index);

      /// \brief Get all the ranges
      /// \param[out] _range A vector that will contain all the range data
      public: void GetRanges(std::vector<double> &_ranges);

      /// \brief Get detected retro (intensity) value for a ray.
      ///         Warning: If you are accessing all the ray data in a loop
      ///         it's possible that the Ray will update in the middle of
      ///         your access loop. This means some data will come from one
      ///         scan, and some from another scan. You can solve this
      ///         problem by using SetActive(false) <your accessor loop>
      ///         SetActive(true).
      /// \param[in] _index Index of specific ray
      /// \return Intensity value of ray
      public: double GetRetro(int _index) const;

      /// \brief Get detected fiducial value for a ray.
      ///         Warning: If you are accessing all the ray data in a loop
      ///         it's possible that the Ray will update in the middle of
      ///         your access loop. This means some data will come from one
      ///         scan, and some from another scan. You can solve this
      ///         problem by using SetActive(false) <your accessor loop>
      ///         SetActive(true).
      /// \param[in] _index Index of specific ray
      /// \return Fiducial value of ray
      public: int GetFiducial(int _index) const;

      /// \brief Gets the camera count
      /// \return Number of cameras
      public: unsigned int GetCameraCount() const;

      /// \brief Gets if sensor is horizontal
      /// \return True if horizontal, false if not
      public: bool IsHorizontal() const;

      /// \brief Return the ratio of horizontal ray count to vertical ray
      /// count.
      ///
      /// A ray count is the number of simulated rays. Whereas a range count
      /// is the total number of data points returned. When range count
      /// != ray count, then values are interpolated between rays.
      public: double GetRayCountRatio() const;

      /// \brief Return the ratio of horizontal range count to vertical
      /// range count.
      ///
      /// A ray count is the number of simulated rays. Whereas a range count
      /// is the total number of data points returned. When range count
      /// != ray count, then values are interpolated between rays.
      public: double GetRangeCountRatio() const;

      /// \brief Get the horizontal field of view of the laser sensor.
      /// \return The horizontal field of view of the laser sensor.
      public: double GetHorzFOV() const;

      /// \brief Get Cos Horz field-of-view
      /// \return 2 * atan(tan(this->hfov/2) / cos(this->vfov/2))
      public: double GetCosHorzFOV() const;

      /// \brief Get the vertical field-of-view.
      public: double GetVertFOV() const;

      /// \brief Get Cos Vert field-of-view
      /// \return 2 * atan(tan(this->vfov/2) / cos(this->hfov/2))
      public: double GetCosVertFOV() const;

      /// \brief Get (horizontal_max_angle + horizontal_min_angle) * 0.5
      /// \return (horizontal_max_angle + horizontal_min_angle) * 0.5
      public: double GetHorzHalfAngle() const;

      /// \brief Get (vertical_max_angle + vertical_min_angle) * 0.5
      /// \return (vertical_max_angle + vertical_min_angle) * 0.5
      public: double GetVertHalfAngle() const;

      /// \brief Connect to the new laser frame event.
      /// \param[in] _subscriber Event callback.
      public: event::ConnectionPtr ConnectNewLaserFrame(
        boost::function<void(const float *, unsigned int, unsigned int,
        unsigned int, const std::string &)> _subscriber);

      /// \brief Disconnect Laser Frame.
      /// \param[in,out] _conn Connection pointer to disconnect.
      public: void DisconnectNewLaserFrame(event::ConnectionPtr &_conn);

      // Documentation inherited
      public: virtual bool IsActive();

      /// brief Render the camera.
      private: void Render();

      /// \brief Scan SDF elementz.
      protected: sdf::ElementPtr scanElem;

      /// \brief Horizontal SDF element.
      protected: sdf::ElementPtr horzElem;

      /// \brief Vertical SDF element.
      protected: sdf::ElementPtr vertElem;

      /// \brief Range SDF element.
      protected: sdf::ElementPtr rangeElem;

      /// \brief Camera SDF element.
      protected: sdf::ElementPtr cameraElem;

      /// \brief Horizontal ray count.
      protected: unsigned int horzRayCount;

      /// \brief Vertical ray count.
      protected: unsigned int vertRayCount;

      /// \brief Horizontal range count.
      protected: unsigned int horzRangeCount;

      /// \brief Vertical range count.
      protected: unsigned int vertRangeCount;

      /// \brief Range count ratio.
      protected: double rangeCountRatio;

      /// \brief GPU laser rendering.
      private: rendering::GpuLaserPtr laserCam;

      /// \brief Mutex to protect getting ranges.
      private: boost::mutex mutex;

      /// \brief Laser message to publish data.
      private: msgs::LaserScanStamped laserMsg;

      /// \brief Parent entity of gpu ray sensor
      private: physics::EntityPtr parentEntity;

      /// \brief Publisher to publish ray sensor data
      private: transport::PublisherPtr scanPub;

      /// \brief True if the sensor was rendered.
      private: bool rendered;
    };
    /// \}
  }
}
#endif
