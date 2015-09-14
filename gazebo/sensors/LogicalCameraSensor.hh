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
#ifndef _GAZEBO_LOGICAL_CAMERASENSOR_HH_
#define _GAZEBO_LOGICAL_CAMERASENSOR_HH_

#include <string>
#include <sdf/sdf.hh>

#include "gazebo/sensors/Sensor.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace sensors
  {
    // Forward declare private data class
    class LogicalCameraSensorPrivate;

    /// \addtogroup gazebo_sensors Sensors
    /// \{

    /// \class LogicalCameraSensor LogicalCameraSensor.hh sensors/sensors.hh
    /// \brief A camera sensor that reports locations of objects instead
    /// of rendering a scene. This camera finds models in the sensor's
    /// vicinity, and transmits information about the models on the sensor's
    /// topic.
    class GAZEBO_VISIBLE LogicalCameraSensor : public Sensor
    {
      /// \brief Constructor
      public: LogicalCameraSensor();

      /// \brief Destructor
      public: virtual ~LogicalCameraSensor();

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName,
                                sdf::ElementPtr _sdf);

      // Documentation inherited
      public: virtual void Load(const std::string &_worldName);

      // Documentation inherited
      public: virtual void Init();

      // Documentation inherited
      public: virtual std::string GetTopic() const;

      /// \brief Get the near distance. This is the distance from the
      /// frustum's vertex to the closest plane.
      /// \return Near distance.
      public: double Near() const;

      /// \brief Get the far distance. This is the distance from the
      /// frustum's vertex to the farthest plane.
      /// \return Far distance.
      public: double Far() const;

      /// \brief Get the horizontal field of view. The field of view is the
      /// angle between the frustum's vertex and the edges of the near or far
      /// plane. This value represents the horizontal angle.
      /// \return The field of view.
      public: ignition::math::Angle HorizontalFOV() const;

      /// \brief Get the aspect ratio, which is the width divided by height
      /// of the near or far planes.
      /// \return The frustum's aspect ratio.
      public: double AspectRatio() const;

      /// \brief Get the latest image. An image is an instance of
      /// msgs::LogicalCameraImage, which contains a list of detected models.
      /// \return List of detected models.
      public: msgs::LogicalCameraImage Image() const;

      // Documentation inherited
      protected: virtual bool UpdateImpl(bool _force);

      // \brief Finalize the logical camera
      protected: virtual void Fini();

      // Documentation inherited
      public: virtual bool IsActive();

      // \internal
      // \brief Private data pointer
      private: LogicalCameraSensorPrivate *dataPtr;
    };
    /// \}
  }
}
#endif
