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

#ifndef _GAZEBO_DARTLINK_PRIVATE_HH_
#define _GAZEBO_DARTLINK_PRIVATE_HH_

#include <map>
#include <vector>
#include <boost/any.hpp>
#include <boost/function.hpp>

#include "gazebo/common/Assert.hh"
#include "gazebo/common/Console.hh"
#include "gazebo/physics/dart/dart_inc.h"
#include "gazebo/physics/dart/DARTTypes.hh"

namespace gazebo
{
  namespace physics
  {
    /// \internal
    /// \brief Private data class for DARTLink
    class DARTLinkPrivate
    {
      using DARTBodyNodePropPtr =
        std::shared_ptr<dart::dynamics::BodyNode::Properties>;

      /// \brief Constructor
      public: DARTLinkPrivate()
        : dartPhysics(NULL),
          dtProperties(NULL),
          dtBodyNode(NULL),
          dartParentJoint(NULL),
          dartChildJoints {},
          isSoftBody(false),
          staticLink(false),
          dtWeldJointConst(NULL)
      {
      }

      /// \brief Default destructor
      public: ~DARTLinkPrivate()
      {
        // We don't need to delete dtBodyNode because skeletone will delete
        // dtBodyNode if it is registered to the skeletone.

        delete dtWeldJointConst;
      }

      /// \brief Call all the cached setter functions and clear them
      public: void Initialize()
      {
        GZ_ASSERT(dtBodyNode != NULL, "dtBodyNode is null pointer.\n");

        for (auto func : mCachedFuncs)
          func();
      }

      /// \brief Return true if DART BodyNode is initialized
      public: bool IsInitialized() const
      {
        return dtBodyNode != NULL;
      }

      /// \brief Cache a setter function. The cached functions will be called in
      /// Initialize().
      public: void Cache(const std::string &_key,
                         boost::function<void()> _func)
      {
        gzlog << "Attempting to set "<< _key << " to DART BodyNode when "
              << "it's not "
              << "initialized yet. "
              << "Please dont't call this function during Model::Load() is "
              << "being processed. "
              << "Caching this function to be called in DARTLink::Init().\n";

        mCachedFuncs.push_back(_func);
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
      public: std::vector<boost::function<void()>> mCachedFuncs;

      /// \brief Cached values.
      public: std::map<std::string, boost::any> mCachedValues;

      /// \brief Default values that will be used when there is no cached value.
      public: std::map<std::string, boost::any> mDefaultValues;

      /// \brief Pointer to the DART physics engine.
      public: DARTPhysicsPtr dartPhysics;

      /// \brief Pointer to the DART BodyNode properties.
      public: DARTBodyNodePropPtr dtProperties;

      /// \brief Pointer to the DART BodyNode.
      public: dart::dynamics::BodyNode *dtBodyNode;

      /// \brief Pointer to the parent joint.
      public: DARTJointPtr dartParentJoint;

      /// \brief List of pointers to the child joints.
      public: std::vector<DARTJointPtr> dartChildJoints;

      /// \brief True if this link is soft body.
      public: bool isSoftBody;

      /// \brief If true, freeze link to world (inertial) frame.
      public: bool staticLink;

      /// \brief Weld joint constraint for SetLinkStatic()
      public: dart::constraint::WeldJointConstraint *dtWeldJointConst;
    };
  }
}
#endif
