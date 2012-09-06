/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#ifndef SKELETONANIMATION_HH
#define SKELETONANIMATION_HH

#include <math/Matrix4.hh>
#include <math/Pose.hh>

#include <map>
#include <utility>
#include <string>

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common Animation
    /// \{

    /// \brief Node animation
    class NodeAnimation
    {
      public: NodeAnimation(const std::string& _name);

      public: ~NodeAnimation();

      public: void SetName(const std::string& _name);

      public: std::string GetName() const;

      public: void AddKeyFrame(const double _time, const math::Matrix4 _trans);

      public: void AddKeyFrame(const double _time, const math::Pose _pose);

      public: unsigned int GetFrameCount() const;

      public: void GetKeyFrame(const unsigned int _i, double& _time,
                      math::Matrix4& _trans) const;

      public: std::pair<double, math::Matrix4> GetKeyFrame(
                      const unsigned int _i) const;

      public: double GetLength() const;

      public: math::Matrix4 GetFrameAt(double _time, bool _loop = true) const;

      public: void Scale(const double _scale);

      public: double GetTimeAtX(const double _x) const;

      protected: std::string name;

      protected: std::map<double, math::Matrix4> keyFrames;

      protected: double length;
    };

    /// \brief Skeleton animation
    class SkeletonAnimation
    {
      public: SkeletonAnimation(const std::string& _name);

      public: ~SkeletonAnimation();

      public: void SetName(const std::string& _name);

      public: std::string GetName() const;

      public: unsigned int GetNodeCount() const;

      public: bool HasNode(const std::string& _node) const;

      public: void AddKeyFrame(const std::string& _node, const double _time,
                      const math::Matrix4 _mat);

      public: void AddKeyFrame(const std::string& _node, const double _time,
                      const math::Pose _pose);

      public: math::Matrix4 GetNodePoseAt(const std::string& _node,
                      const double _time, const bool _loop = true);

      public: std::map<std::string, math::Matrix4> GetPoseAt(const double _time,
                      const bool _loop = true) const;

      public: std::map<std::string, math::Matrix4> GetPoseAtX(const double _x,
                      const std::string& _node, const bool _loop = true) const;

      public: void Scale(const double _scale);

      public: double GetLength() const;

      protected: std::string name;

      protected: double length;

      protected: std::map<std::string, NodeAnimation*> animations;
    };
    /// \}
  }
}

#endif
