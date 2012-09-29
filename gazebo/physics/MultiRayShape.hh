/*
 * Copyright 2011 Nate Koenig
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
#ifndef MULTIRAYGEOM_HH
#define MULTIRAYGEOM_HH

#include <vector>
#include <string>

#include "common/Exception.hh"
#include "common/Console.hh"
#include "math/Vector3.hh"
#include "math/Angle.hh"

#include "physics/Collision.hh"
#include "physics/Shape.hh"
#include "physics/RayShape.hh"

namespace gazebo
{
  namespace msgs
  {
    class Visual;
  }

  namespace physics
  {
    /// \addtogroup gazebo_physics
    /// \{

    /// \brief Laser collision contains a set of ray-collisions,
    ///        structured to simulate a laser range scanner
    class MultiRayShape : public Shape
    {
      /// \brief Constructor
      public: MultiRayShape(CollisionPtr parent);

      /// \brief Destructor
      public: virtual ~MultiRayShape();

      /// \brief Init the shape
      public: virtual void Init();

      public: void SetDisplayType(const std::string &type);

      /// \brief Get detected range for a ray.
      /// \returns Returns DBL_MAX for no detection.
      public: double GetRange(int index);

      /// \brief Get detected retro (intensity) value for a ray.
      public: double GetRetro(int index);

      /// \brief Get detected fiducial value for a ray.
      public: int GetFiducial(int index);

      /// \brief Get the minimum range
      public: double GetMinRange() const;

      /// \brief Get the maximum range
      public: double GetMaxRange() const;

      /// \brief Get the range resolution
      public: double GetResRange() const;


      /// \brief Get the horizontal sample count
      public: int GetSampleCount() const;

      /// \brief Get the range resolution
      public: double GetScanResolution() const;

      /// \brief Get the minimum angle
      public: math::Angle GetMinAngle() const;

      /// \brief Get the maximum angle
      public: math::Angle GetMaxAngle() const;




      /// \brief Get the vertical sample count
      public: int GetVerticalSampleCount() const;

      /// \brief Get the vertical range resolution
      public: double GetVerticalScanResolution() const;

      /// \brief Get the vertical min angle
      public: math::Angle GetVerticalMinAngle() const;

      /// \brief Get the vertical max angle
      public: math::Angle GetVerticalMaxAngle() const;

      /// \brief Update the collision
      public: void Update();

      public: void FillShapeMsg(msgs::Geometry &) {}
      public: virtual void ProcessMsg(const msgs::Geometry &) {}
       /// \brief Physics engine specific method for updating the rays
      protected: virtual void UpdateRays() = 0;

      /// \brief Add a ray to the collision
      protected: virtual void AddRay(const math::Vector3 &start,
                                     const math::Vector3 &end);

      /// Ray data
      protected: std::vector<RayShapePtr> rays;

      protected: math::Pose offset;
      protected: sdf::ElementPtr rayElem;
      protected: sdf::ElementPtr scanElem;
      protected: sdf::ElementPtr horzElem;
      protected: sdf::ElementPtr vertElem;
      protected: sdf::ElementPtr rangeElem;


      /// \brief Connect a to the add entity signal
      public: template<typename T>
              event::ConnectionPtr ConnectNewLaserScans(T subscriber)
              { return newLaserScans.Connect(subscriber); }
      public: void DisconnectNewLaserScans(event::ConnectionPtr &c)
              { newLaserScans.Disconnect(c); }
      protected: event::EventT<void()> newLaserScans;
    };
    /// \}
  }
}
#endif

