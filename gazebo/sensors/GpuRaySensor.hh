/*
 * Copyright 2012 Nate Koenig
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

#include "math/Angle.hh"
#include "math/Pose.hh"
#include "transport/TransportTypes.hh"
#include "sensors/Sensor.hh"
#include "rendering/RenderTypes.hh"

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
    class GpuRaySensor: public Sensor
    {
      /// \brief Constructor
      public: GpuRaySensor();

      /// \brief Destructor
      public: virtual ~GpuRaySensor();

      /// \brief Load the sensor with SDF parameters
      /// \param[in] _sdf SDF Sensor parameters
      /// \param[in] _worldName Name of world to load from
      public: virtual void Load(const std::string &_worldName,
                                sdf::ElementPtr &_sdf);

      /// \brief Load the sensor with default parameters
      /// \param[in] _worldName Name of world to load from
      public: virtual void Load(const std::string &_worldName);

      /// \brief Initialize the ray
      public: virtual void Init();

      /// \brief Update the sensor information
      /// \param[in] _force True if update is forced, false if not
      protected: virtual void UpdateImpl(bool _force);

      /// \brief Finalize the ray
      protected: virtual void Fini();

      /// \brief Returns a pointer to the internally kept rendering::GpuLaser
      /// \return Pointer to GpuLaser
      public: rendering::GpuLaserPtr GetLaserCamera() const
              {return this->laserCam;}

      /// \brief Get the minimum angle
      /// \return The minimum angle
      public: math::Angle GetAngleMin() const;

      /// \brief Set the scan minimum angle
      /// \param[in] _angle The minimum angle
      public: void SetAngleMin(double _angle);

      /// \brief Get the maximum angle
      /// \return the maximum angle
      public: math::Angle GetAngleMax() const;

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
      public: math::Angle GetVerticalAngleMin() const;

      /// \brief Set the vertical scan bottom angle
      /// \param[in] _angle The minimum angle of the scan block
      public: void SetVerticalAngleMin(double _angle);

      /// \brief Get the vertical scan line top angle
      /// \return The Maximum angle of the scan block
      public: math::Angle GetVerticalAngleMax() const;

      /// \brief Set the vertical scan line top angle
      /// \param[in] _angle The Maximum angle of the scan block
      public: void SetVerticalAngleMax(double _angle);


      /// \brief Get detected range for a ray.
      ///         Warning: If you are accessing all the ray data in a loop
      ///         it's possible that the Ray will update in the middle of
      ///         your access loop. This means some data will come from one
      ///         scan, and some from another scan. You can solve this
      ///         problem by using SetActive(false) <your accessor loop>
      ///         SetActive(true).
      /// \param[in] _index Index of specific ray
      /// \return Returns DBL_MAX for no detection.
      public: double GetRange(int _index);

      /// \brief Get all the ranges
      /// \param _range A vector that will contain all the range data
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
      public: double GetRetro(int _index);

      /// \brief Get detected fiducial value for a ray.
      ///         Warning: If you are accessing all the ray data in a loop
      ///         it's possible that the Ray will update in the middle of
      ///         your access loop. This means some data will come from one
      ///         scan, and some from another scan. You can solve this
      ///         problem by using SetActive(false) <your accessor loop>
      ///         SetActive(true).
      /// \param[in] _index Index of specific ray
      /// \return Fiducial value of ray
      public: int GetFiducial(int _index);

      /// \brief Gets the camera count
      /// \return Number of cameras
      public: unsigned int GetCameraCount();

      /// \brief Gets if sensor is horizontal
      /// \return True if horizontal, false if not
      public: bool IsHorizontal();

      /// \brief
      /// \TODO Nate fill in.
      /// I'm not sure what these what these sensor parameters refer to
      public: double Get1stRatio();

      /// \brief
      /// @todo Document me
      public: double Get2ndRatio();

      /// \brief
      /// @todo Document me
      public: double GetHFOV();

      /// \brief
      /// @todo Document me
      public: double GetCHFOV();

      /// \brief
      /// @todo Document me
      public: double GetVFOV();

      /// \brief
      /// @todo Document me
      public: double GetCVFOV();

      /// \brief
      /// @todo Document me
      public: double GetHAngle();

      /// \brief
      /// @todo Document me
      public: double GetVAngle();

      /// \brief
      /// @todo Document me
      private: void OnPose(ConstPosePtr &_msg);

      /// \brief Connect a to the add entity signal
      /// \TODO Nate do these parameters need to be specified here?
      public: event::ConnectionPtr ConnectNewLaserFrame(
        boost::function<void(const float *, unsigned int, unsigned int,
        unsigned int, const std::string &)> subscriber);

      /// \brief Disconnect Laser Frame
      /// \param Connection pointer to disconnect
      public: void DisconnectNewLaserFrame(event::ConnectionPtr &c);

      protected: math::Vector3 offset;
      protected: sdf::ElementPtr rayElem;
      protected: sdf::ElementPtr scanElem;
      protected: sdf::ElementPtr horzElem;
      protected: sdf::ElementPtr vertElem;
      protected: sdf::ElementPtr rangeElem;
      protected: sdf::ElementPtr cameraElem;

      protected: unsigned int cameraCount;

      protected: double hfov, vfov, chfov, cvfov, hang, vang;
      protected: double near, far;
      protected: unsigned int width_1st, height_1st;
      protected: unsigned int width_2nd, height_2nd;
      protected: double ratio_1st, ratio_2nd;
      protected: bool isHorizontal;

      private: rendering::GpuLaserPtr laserCam;
      private: rendering::ScenePtr scene;

      private: transport::PublisherPtr scanPub;
      private: boost::mutex *mutex;
      private: msgs::LaserScan laserMsg;
    };
    /// \}
  }
}

#endif
