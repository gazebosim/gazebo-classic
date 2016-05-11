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
#ifndef _GAZEBO_PHYSICS_MULTIRAYSHAPE_HH_
#define _GAZEBO_PHYSICS_MULTIRAYSHAPE_HH_

#include <vector>
#include <string>

#include "gazebo/math/Vector3.hh"
#include "gazebo/math/Angle.hh"

#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/Shape.hh"
#include "gazebo/physics/RayShape.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    // Forward declar private data class
    class MultiRayShapePrivate;

    /// \addtogroup gazebo_physics
    /// \{

    /// \class MultiRayShape MultiRayShape.hh physics/physics.hh
    /// \brief Laser collision contains a set of ray-collisions,
    /// structured to simulate a laser range scanner.
    class GZ_PHYSICS_VISIBLE MultiRayShape : public Shape
    {
      /// \brief Constructor.
      /// \param[in] _parent Parent collision shape.
      public: explicit MultiRayShape(CollisionPtr _parent);

      /// \brief Constructor for a stand alone multiray shape. Stand alone
      /// means the multiray shape is not attached to a Collision object.
      ///
      /// Example:
      ///
      ///  gazebo::physics::MultiRayShapePtr rays =
      ///      boost::dynamic_pointer_cast<gazebo::physics::MultiRayShape>(
      ///        world->GetPhysicsEngine()->CreateShape("multiray",
      ///          gazebo::physics::CollisionPtr()));
      ///
      /// \param[in] _physicsEngine Pointer to the physics engine.
      public: explicit MultiRayShape(PhysicsEnginePtr _physicsEngine);

      /// \brief Destructor.
      public: virtual ~MultiRayShape();

      /// \brief Init the shape.
      public: virtual void Init();

      // Documentation inherited
      public: virtual void SetScale(const math::Vector3 &_scale)
              GAZEBO_DEPRECATED(7.0);

      // Documentation inherited
      public: virtual void SetScale(const ignition::math::Vector3d &_scale);

      /// \brief Get detected range for a ray.
      /// \param[in] _index Index of the ray.
      /// \returns Returns DBL_MAX for no detection.
      /// \deprecated See Range(const unsigned int) const
      public: double GetRange(unsigned int _index) GAZEBO_DEPRECATED(7.0);

      /// \brief Get detected range for a ray.
      /// \param[in] _index Index of the ray.
      /// \returns Returns IGN_DBL_INF for no detection.
      public: double Range(unsigned int _index) const;

      /// \brief Get detected retro (intensity) value for a ray.
      /// \param[in] _index Index of the ray.
      /// \return Retro value for the ray.
      /// \deprecated See Retro(const unsigned int) const
      public: double GetRetro(unsigned int _index) GAZEBO_DEPRECATED(7.0);

      /// \brief Get detected retro (intensity) value for a ray.
      /// \param[in] _index Index of the ray.
      /// \return Retro value for the ray, IGN_DBL_INF on error.
      public: double Retro(const unsigned int _index) const;

      /// \brief Get detected fiducial value for a ray.
      /// \param[in] _index Index of the ray.
      /// \return Fiducial value for the ray.
      /// \deprecated See Fiducial(const unsigned int) const
      public: int GetFiducial(unsigned int _index) GAZEBO_DEPRECATED(7.0);

      /// \brief Get detected fiducial value for a ray.
      /// \param[in] _index Index of the ray.
      /// \return Fiducial value for the ray, IGN_DBL_INF on error.
      public: int Fiducial(const unsigned int _index) const;

      /// \brief Get the minimum range.
      /// \return Minimum range of all the rays.
      /// \deprecated See MinRange() const
      public: double GetMinRange() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the minimum range.
      /// \return Minimum range of all the rays.
      public: double MinRange() const;

      /// \brief Get the maximum range.
      /// \return Maximum range of all the rays.
      /// \deprecated See MaxRange() const
      public: double GetMaxRange() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the maximum range.
      /// \return Maximum range of all the rays.
      public: double MaxRange() const;

      /// \brief Get the range resolution.
      /// \return Range resolution of all the rays.
      /// \deprecated See ResolutionRange() const
      public: double GetResRange() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the range resolution.
      /// \return Range resolution of all the rays.
      public: double ResolutionRange() const;

      /// \brief Get the horizontal sample count.
      /// \return Horizontal sample count.
      /// \deprecated See SampleCount() const
      public: int GetSampleCount() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the horizontal sample count.
      /// \return Horizontal sample count.
      public: int SampleCount() const;

      /// \brief Get the horizontal resolution.
      /// \return Horizontal resolution.
      /// \deprecated See ScanResolution() const
      public: double GetScanResolution() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the horizontal resolution.
      /// \return Horizontal resolution.
      public: double ScanResolution() const;

      /// \brief Get the minimum angle.
      /// \return Minimum angle of ray scan.
      /// \deprecated See function that returns an ignition::math object
      public: math::Angle GetMinAngle() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the minimum angle.
      /// \return Minimum angle of ray scan.
      public: ignition::math::Angle MinAngle() const;

      /// \brief Get the maximum angle.
      /// \return Maximum angle of ray scan.
      /// \deprecated See function that returns an ignition::math object
      public: math::Angle GetMaxAngle() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the maximum angle.
      /// \return Maximum angle of ray scan.
      public: ignition::math::Angle MaxAngle() const;

      /// \brief Get the vertical sample count.
      /// \return Verical sample count.
      /// \deprecated See VerticalSampleCount() const
      public: int GetVerticalSampleCount() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the vertical sample count.
      /// \return Verical sample count.
      public: int VerticalSampleCount() const;

      /// \brief Get the vertical range resolution.
      /// \return Vertical range resolution.
      /// \deprecated See VerticalScanResolution() const
      public: double GetVerticalScanResolution() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the vertical range resolution.
      /// \return Vertical range resolution.
      public: double VerticalScanResolution() const;

      /// \brief Get the vertical min angle.
      /// \return Vertical min angle.
      /// \deprecated See function that returns an ignition::math object
      public: math::Angle GetVerticalMinAngle() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the vertical min angle.
      /// \return Vertical min angle.
      public: ignition::math::Angle VerticalMinAngle() const;

      /// \brief Get the vertical max angle.
      /// \return Vertical max angle.
      /// \deprecated See function that returns an ignition::math object
      public: math::Angle GetVerticalMaxAngle() const GAZEBO_DEPRECATED(7.0);

      /// \brief Get the vertical max angle.
      /// \return Vertical max angle.
      public: ignition::math::Angle VerticalMaxAngle() const;

      /// \brief Update the ray collisions.
      public: void Update();

      /// \TODO This function is not implemented.
      /// \brief Fill a message with this shape's values.
      /// \param[out] _msg Message that contains the shape's values.
      public: void FillMsg(msgs::Geometry &_msg);

      /// \TODO This function is not implemented.
      /// \brief Update the ray based on a message.
      /// \param[in] _msg Message to update from.
      public: virtual void ProcessMsg(const msgs::Geometry &_msg);

      /// Documentation inherited
      public: virtual double ComputeVolume() const;

      /// \brief Connect a to the new laser scan signal.
      /// \param[in] _subscriber Callback function.
      /// \return The connection, which must be kept in scope.
      public: event::ConnectionPtr ConnectNewLaserScans(
                  std::function<void ()> _subscriber);

      /// \brief Disconnect from the new laser scans signal.
      /// \param[in] _conn Connection to remove.
      public: void DisconnectNewLaserScans(event::ConnectionPtr &_conn);

      /// \brief Method for updating the rays. This function is normally
      /// called automatically, such as when a laser sensor is updated.
      /// Only call this function on a standalone multiray shape.
      /// \sa explicit MultiRayShape(PhysicsEnginePtr _physicsEngine)
      public: virtual void UpdateRays() = 0;

      /// \brief Add a ray to the collision.
      /// \param[in] _start Start of the ray.
      /// \param[in] _end End of the ray.
      /// \deprecated See function that accepts ignition::math parameters.
      public: virtual void AddRay(const math::Vector3 &_start,
                     const math::Vector3 &_end) GAZEBO_DEPRECATED(7.0);

      /// \brief Add a ray to the collision.
      /// \param[in] _start Start of the ray.
      /// \param[in] _end End of the ray.
      public: virtual void AddRay(const ignition::math::Vector3d &_start,
                  const ignition::math::Vector3d &_end);

      /// \brief Set the points of a ray.
      /// \param[in] _rayIndex Index of the ray to set.
      /// \param[in] _start Start of the ray.
      /// \param[in] _end End of the ray.
      /// \return True if the ray was set. False can be returned if the
      /// _rayIndex is invalid.
      public: bool SetRay(const unsigned int _rayIndex,
                  const ignition::math::Vector3d &_start,
                  const ignition::math::Vector3d &_end);

      /// \brief Get the number of rays.
      /// \return Number of rays in this shape.
      public: unsigned int RayCount() const;

      /// \brief Get a pointer to a ray
      /// \param[in] _rayIndex index to the ray
      /// \return Pointer to the ray, or NULL on error
      /// \sa RayCount()
      public: RayShapePtr Ray(const unsigned int _rayIndex) const;

      /// \internal
      /// \brief Private data pointer.
      protected: MultiRayShapePrivate *multiRayShapeDPtr;
    };
    /// \}
  }
}
#endif
