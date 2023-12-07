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
/* Desc: Collision class
 * Author: Nate Koenig
 * Date: 13 Feb 2006
 */

#ifndef _ODECOLLISION_HH_
#define _ODECOLLISION_HH_

#include "gazebo/physics/ode/ode_inc.h"

#include "gazebo/physics/PhysicsTypes.hh"
#include "gazebo/physics/Collision.hh"
#include "gazebo/physics/ode/ODETypes.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace physics
  {
    /// \addtogroup gazebo_physics_ode
    /// \{

    /// \brief Data structure for wheel plowing parameters.
    class ODECollisionWheelPlowingParams
    {
      /// \brief Flag to disable scaling of slip parameters by the number
      /// of contact points. This behavior was added in bitbucket PR 2965,
      /// but it can interact poorly with wheels on heightmap collisions.
      /// Keep current behavior (scaling on) as default.
      /// https://osrf-migration.github.io/gazebo-gh-pages/#!/osrf/gazebo/pull-requests/2965
      public: bool disableScalingSlipByNumberOfContactPoints = false;

      /// \brief Maximum angle by which wheel contact points are rotated.
      public: ignition::math::Angle maxAngle;

      /// \brief Plowing saturation velocity: the linear wheel velocity [m/s]
      /// at which maximum plowing effect is reached.
      public: double saturationVelocity = 0.0;

      /// \brief Plowing deadband velocity: the linear wheel velocity [m/s]
      /// below which no plowing effect occurs.
      public: double deadbandVelocity = 0.0;

      /// Nonlinear slip model:
      /// The nonlinear slip model applies a multiplier to the slip compliance
      /// depending on the terrain slope at the contact point. The slip
      /// compliance multipliers are defined as a piecewise linear function of
      /// the terrain slope at the contact point in degrees. Two sets of
      /// piecewise linear segments are defined as the lower and upper
      /// nonlinear parameters. The slip compliance multiplier is 1.0 for
      /// terrain slope values in between the largest lower slope value and
      /// the smallest upper slope value.
      ///
      /// A graphical representation of upper piecewise linear segments is
      /// provided below. The horizontal axis corresponds to terrain slope in
      /// degrees and the vertical axis corresponds to the slip compliance
      /// multiplier. This example shows three piecewise linear segments
      /// connecting from 0 to 1, 1 to 2, and from 2 onwards. The piecewise
      /// linear endpoint coordinates and rates of change are encoded in
      /// vectors of the same length for which the values at each index are
      /// related:
      ///
      /// * per_degree[i]: the rate of change of the slip multiplier per
      ///   degree of change in terrain slope. In this example the final
      ///   slope is flat (per_degree[2] == 0), but that is not required.
      /// * degrees[i]: the slope in degrees above which the slip compliance
      ///   multiplier can vary according to the rate of change in
      ///   per_degree[i].
      /// * multipliers[i]: the starting value of the slip compliance
      ///   multiplier at degrees[i]. Note that multipliers[0] == 1.0 always.
      ///
      /// The degrees[i] and per_degree[i] values are specified inside
      /// <upper/> XML elements, and the corresponding multiplier[i] values
      /// are computed automatically. The <upper/> XML elements should be
      /// defined in order, such that each //upper/degree value is larger than
      /// the previous value.
      /** \verbatim
                   |                                                  .
              slip |                          per_degree[2] == 0      .
        compliance |                    2 o o o o o o o o             .
        multiplier |                  o (degrees[2], multipliers[2])  .
                   |                o                                 .
                   |              o |                                 .
                   |            o---┘ per_degree[1]                   .
                   |          o                                       .
                   |        1--------                                 .
                   |       o (degrees[1], multipliers[1])             .
                   |      o                                           .
                   |     o|                                           .
                   |    o-┘ per_degree[0]                             .
                   |   o                                              .
         ----------+--0                                               .
               1.0 |   (degrees[0], multipliers[0] == 1.0)            .
                   |                                                  .
      -------------+-------------------------- slopeDegrees
                   |

      <gz:plowing_wheel>
        <nonlinear_slip>
          <upper>
            <degree>{degrees[0]}</degree>
            <per_degree>{per_degree[0]}</per_degree>
          </upper>
          <upper>
            <degree>{degrees[1]}</degree>
            <per_degree>{per_degree[1]}</per_degree>
          </upper>
          <upper>
            <degree>{degrees[2]}</degree>
            <per_degree>{per_degree[2]}</per_degree>
          </upper>
        </nonlinear_slip>
      </gz:plowing_wheel>
      \endverbatim */
      /// The lower nonlinear parameters are defined in a similar manner inside
      /// <lower/> XML elements. Again, the elements should be defined in
      /// order, such that each //lower/degree value is larger than the
      /// previous value.

      class NonlinearSlipParams
      {
        /// \brief The pairs of slope in degrees and slip multipliers that
        /// define the piecewise linear behavior for the lower slope values.
        public: std::vector<ignition::math::Vector2d>
            lowerDegreesMultipliers{{-361.0, 1.0}};

        /// \brief The pairs of slope in degrees and slip multipliers that
        /// define the piecewise linear behavior for the upper slope values.
        public: std::vector<ignition::math::Vector2d>
            upperDegreesMultipliers{{361.0, 1.0}};

        /// \brief The rates of change in slip compliance multiplier per degree
        /// of slope in slope below the lower degrees[i] value for each
        /// piecewise linear segment.
        public: std::vector<double> lowerPerDegrees{0.0};

        /// \brief The rates of change in slip compliance multiplier per degree
        /// of slope in slope above the upper degrees[i] value for each
        /// piecewise linear segment.
        public: std::vector<double> upperPerDegrees{0.0};
      };

      /// \brief Nonlinear parameters for longitudinal wheel slip.
      public: NonlinearSlipParams longitudinalNonlinearSlipParams;

      /// \brief Nonlinear parameters for lateral wheel slip.
      public: NonlinearSlipParams lateralNonlinearSlipParams;
    };

    /// \brief Base class for all ODE collisions.
    class GZ_PHYSICS_VISIBLE ODECollision : public Collision
    {
      /// \brief Constructor.
      /// \param[in] _link Parent Link
      public: explicit ODECollision(LinkPtr _parent);

      /// \brief Destructor.
      public: virtual ~ODECollision();

      // Documentation inherited.
      public: virtual void Load(sdf::ElementPtr _sdf);

      // Documentation inherited.
      public: virtual void Fini();

      /// \brief Set the encapsulated collision object.
      /// \param[in] _collisionId ODE id of the collision object.
      /// \param[in] _placeable True to make the object movable.
      public: void SetCollision(dGeomID _collisionId, bool _placeable);

      /// \brief Return the collision id.
      /// \return The collision id.
      public: dGeomID GetCollisionId() const;

      /// \brief Get the ODE collision class.
      /// \return The ODE collision class.
      public: int GetCollisionClass() const;

      // Documentation inherited.
      public: virtual void OnPoseChange();

      // Documentation inherited.
      public: virtual void SetCategoryBits(unsigned int bits);

      // Documentation inherited.
      public: virtual void SetCollideBits(unsigned int bits);

      // Documentation inherited.
      public: virtual ignition::math::AxisAlignedBox BoundingBox() const;

      /// \brief Get the collision's space ID
      /// \return The collision's space ID
      public: dSpaceID GetSpaceId() const;

      /// \brief Set the collision's space ID
      /// \param[in] _spaceid ID of an ODE collision space.
      public: void SetSpaceId(dSpaceID _spaceid);

      /// \brief Similar to Collision::GetSurface, but provides dynamically
      ///        casted pointer to ODESurfaceParams.
      /// \return Dynamically casted pointer to ODESurfaceParams.
      public: ODESurfaceParamsPtr GetODESurface() const;

      /// \brief Parse wheel plowing parameters from a Collision SDF Element.
      /// \param[in] _sdf Collision SDF Element to parse from.
      /// \param[out] _plowing Wheel plowing parameters object to write to.
      /// \param[in] _scopedNameForErrorMessages Scoped name of collision to
      /// use in error messages. If empty, no error messages will be printed.
      /// parsed.
      /// \return True if valid wheel plowing parameters were parsed.
      private: static bool ParseWheelPlowingParams(
          sdf::ElementPtr _sdf,
          ODECollisionWheelPlowingParams &_plowing,
          const std::string &_scopedNameForErrorMessages = "");

      /// \brief Friend with ODEPhysics to allow Collide callback to call
      /// ParseWheelPlowingParams.
      private: friend class ODEPhysics;

      /// \brief Used when this is static to set the posse.
      private: void OnPoseChangeGlobal();

      /// \brief Used when this is not statis to set the posse.
      private: void OnPoseChangeRelative();

      /// \brief Empty pose change callback.
      private: void OnPoseChangeNull();

      /// \brief Collision space for this.
      protected: dSpaceID spaceId;

      /// \brief ID for the collision.
      protected: dGeomID collisionId;

      /// \brief Function used to set the pose of the ODE object.
      private: void (ODECollision::*onPoseChangeFunc)();
    };
    /// \}
  }
}
#endif
