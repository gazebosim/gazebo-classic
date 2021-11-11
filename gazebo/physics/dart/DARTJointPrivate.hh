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

#ifndef _GAZEBO_DARTJOINT_PRIVATE_HH_
#define _GAZEBO_DARTJOINT_PRIVATE_HH_

#include <map>
#include <string>
#include <vector>
#include <boost/function.hpp>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Time.hh"
#include "gazebo/physics/Joint.hh"
#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTTypes.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data class for DARTJoint
    class DARTJointPrivate
    {
      /// \brief Constructor
      public: explicit DARTJointPrivate(const DARTPhysicsPtr &_engine)
        : forceApplied {0.0, 0.0},
          forceAppliedTime(),
          dartPhysicsEngine(_engine),
          dtProperties(nullptr),
          dtJoint(nullptr),
          dtChildBodyNode(nullptr)
      {
        mDefaultValues["Axis0"] = ignition::math::Vector3d::UnitX;
        mDefaultValues["Axis1"] = ignition::math::Vector3d::UnitY;
      }

      /// \brief Default destructor
      public: ~DARTJointPrivate() = default;

      /// \brief Call all the cached setter functions and clear them
      public: void Initialize()
      {
        // dtJoint must have been set before initialization, which is
        // not very nice but necessary. The current solution also requires
        // the constructor to be called before we can set the joint, so
        // passing it into constructor is not an option either.
        // Cached functions may need a valid dtJoint already, or there will
        // be crashes with boost function binding which are not nice to debug.
        // So in general, we can consider this object invalid without a
        // dtJoint which was explicitly set before calling Initialize().
        if (!this->dtJoint)
        {
          gzerr << "No joint initialized for joint "
                << this->dtProperties->mName << std::endl;
        }
        GZ_ASSERT(this->dtJoint, "dtJoint is null pointer.\n");

        for (auto func : mFuncs)
          func();

        mFuncs.clear();

#if DART_VERSION_AT_LEAST(6, 10, 0)
        this->dtJoint->setLimitEnforcement(true);
 #else
        this->dtJoint->setPositionLimitEnforced(true);
 #endif
      }

      /// \brief Return true if DART Joint is initialized
      public: bool IsInitialized() const
      {
        return this->dtJoint;
      }

      /// \brief Cache a setter function. The cached functions will be called in
      /// Initialize().
      public: void Cache(const std::string &_key,
                         boost::function<void()> _func)
      {
        gzlog << "Attempting to set "<< _key << " to DART Joint when "
              << "it's not "
              << "initialized yet. "
              << "Please dont't call this function during Model::Load() is "
              << "being processed. "
              << "Caching this function to be called in DARTJoint::Init().\n";

        mFuncs.push_back(_func);
      }

      /// \brief Cache a setter function and a value. The cached functions will
      /// be called in Initialize(). The cached value can be obtained through
      /// GetCached().
      public: void Cache(const std::string &_key,
                         boost::function<void()> _func,
                         const boost::any &_value)
      {
        Cache(_key, _func);

        mCachedValues[_key] = _value;
      }

      /// \brief Get cached value.
      public: template <typename T>
              T GetCached(const std::string &_key) const
      {
        // Try to find cached value
        auto cachedValResult = mCachedValues.find(_key);
        if (cachedValResult != mCachedValues.end())
        {
          try
          {
            return boost::any_cast<T>(cachedValResult->second);
          }
          catch(const boost::bad_any_cast &_e)
          {
            gzerr << "GetCached(" << _key << ") error:" << _e.what() << "\n";
            return T();
          }
        }

        // Try to find predefined default value
        auto defValResult = mDefaultValues.find(_key);
        if (defValResult != mDefaultValues.end())
        {
          try
          {
            return boost::any_cast<T>(defValResult->second);
          }
          catch(const boost::bad_any_cast &_e)
          {
            gzerr << "GetCached(" << _key << ") error:" << _e.what() << "\n";
            return T();
          }
        }

        // Return the default value of the object itself
        return T();
      }

      /// \brief Cached setter functions.
      public: std::vector<boost::function<void()>> mFuncs;

      /// \brief Cached values.
      public: std::map<std::string, boost::any> mDefaultValues;

      /// \brief Default values that will be used when there is no cached value.
      public: std::map<std::string, boost::any> mCachedValues;

      /// \brief Save force applied by user
      /// This plus the joint feedback (joint contstraint forces) is the
      /// equivalent of simulated force torque sensor reading
      /// Allocate a 2 vector in case hinge2 joint is used.
      /// This is used by DART to store external force applied by the user.
      public: double forceApplied[MAX_JOINT_AXIS];

      /// \brief Save time at which force is applied by user
      /// This will let us know if it's time to clean up forceApplied.
      public: common::Time forceAppliedTime;

      /// \brief DARTPhysics engine pointer
      public: DARTPhysicsPtr dartPhysicsEngine;

      /// \brief DART Joint properties
      public: DARTJointPropPtr dtProperties;

      /// \brief DART joint pointer
      public: dart::dynamics::Joint *dtJoint;

      /// \brief DART child body node pointer
      public: dart::dynamics::BodyNode *dtChildBodyNode;
    };
  }
}
#endif
