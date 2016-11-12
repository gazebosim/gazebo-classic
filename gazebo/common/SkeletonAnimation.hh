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
#ifndef _GAZEBO_SKELETONANIMATION_HH_
#define _GAZEBO_SKELETONANIMATION_HH_

#include <map>
#include <utility>
#include <string>

#include <ignition/math/Matrix4.hh>
#include <ignition/math/Pose3.hh>

#include "gazebo/util/system.hh"
#include "gazebo/common/CommonTypes.hh"

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common Animation
    /// \{

    /// \class NodeAnimation SkeletonAnimation.hh common/common.hh
    /// \brief Node animation
    class GZ_COMMON_VISIBLE NodeAnimation
    {
      /// \brief constructor
      /// \param[in] _name the name of the node
      public: NodeAnimation(const std::string &_name);

      /// \brief Destructor. It empties the key frames list
      public: ~NodeAnimation();

      /// \brief Changes the name of the animation
      /// \param[in] the new name
      public: void SetName(const std::string &_name);

      /// \brief Returns the name
      /// \return the name
      public: std::string GetName() const;

      /// \brief Adds a key frame at a specific time
      /// \param[in] _time the time of the key frame
      /// \param[in] _trans the transformation
      public: void AddKeyFrame(const double _time,
                  const ignition::math::Matrix4d &_trans);

      /// \brief Adds a key frame at a specific time
      /// \param[in] _time the time of the key frame
      /// \param[in] _pose the pose
      public: void AddKeyFrame(const double _time,
                  const ignition::math::Pose3d &_pose);

      /// \brief Returns the number of key frames.
      /// \return the count
      public: unsigned int GetFrameCount() const;

      /// \brief Finds a key frame using the index. Note the index of a key
      /// frame can change as frames are added.
      /// \param[in] _i the index
      /// \param[out] _time the time of the frame, or -1 if the index id is out
      /// of bounds
      /// \param[out] _trans the transformation for this key frame
      public: void GetKeyFrame(const unsigned int _i, double &_time,
                  ignition::math::Matrix4d &_trans) const;

      /// \brief Returns a key frame using the index. Note the index of a key
      /// frame can change as frames are added.
      /// \param[in] _i the index
      /// \return a pair that contains the time and transformation. Time is -1
      /// if the index is out of bounds
      public: std::pair<double, ignition::math::Matrix4d> KeyFrame(
                      const unsigned int _i) const;

      /// \brief Returns the duration of the animations
      /// \return the time of the last animation
      public: double GetLength() const;

      /// \brief Returns a frame transformation at a specific time
      /// if a node does not exist at that time (with tolerance of 1e-6 sec),
      /// the transformation is interpolated.
      /// \param[in] _time the time
      /// \param[in] _loop when true, the time is divided by the duration
      /// (see GetLength)
      public: ignition::math::Matrix4d FrameAt(
                  double _time, bool _loop = true) const;

      /// \brief Scales each transformation in the key frames. This only affects
      /// the translational values.
      /// \param[in] _scale the scaling factor
      public: void Scale(const double _scale);

      /// \brief Returns the time where a transformation's translational value
      /// along the X axis is equal to _x.
      /// When no transformation is found (within a tolerance of 1e-6), the time
      /// is interpolated.
      /// \param[in] _x the value along x. You must ensure that _x is within a
      /// valid range.
      public: double GetTimeAtX(const double _x) const;

      /// \brief the name of the animation
      protected: std::string name;

      /// \brief the dictionary of key frames, indexed by time
      protected: std::map<double, ignition::math::Matrix4d> keyFrames;

      /// \brief the duration of the animations (time of last key frame)
      protected: double length;
    };

    /// \brief Skeleton animation
    class GZ_COMMON_VISIBLE SkeletonAnimation
    {
      /// \brief The Constructor
      /// \param[in] _name the name of the animation
      public: SkeletonAnimation(const std::string &_name);

      /// \brief The destructor. Clears the list without destroying
      /// the animations
      public: ~SkeletonAnimation();

      /// \brief Changes the name
      /// \param[in] _name the new name
      public: void SetName(const std::string &_name);

      /// \brief Returns the name
      /// \return the name
      public: std::string GetName() const;

      /// \brief Returns the number of animation nodes
      /// \return the count
      public: unsigned int GetNodeCount() const;

      /// \brief Looks for a node with a specific name in the animations
      /// \param[in] _node the name of the node
      /// \return true if the node exits
      public: bool HasNode(const std::string &_node) const;

      /// \brief Adds or replaces a named key frame at a specific time
      /// \param[in] _node the name of the new or existing node
      /// \param[in] _time the time
      /// \param[in] _mat the key frame transformation
      public: void AddKeyFrame(const std::string &_node, const double _time,
                      const ignition::math::Matrix4d &_mat);

      /// \brief Adds or replaces a named key frame at a specific time
      /// \param[in] _node the name of the new or existing node
      /// \param[in] _time the time
      /// \param[in] _pose the key frame transformation as a
      /// ignition::math::Pose3d
      public: void AddKeyFrame(const std::string &_node, const double _time,
                      const ignition::math::Pose3d &_pose);

      /// \brief Returns the key frame transformation for a named animation at
      /// a specific time
      /// if a node does not exist at that time (with tolerance of 1e-6 sec),
      /// the transformation is interpolated.
      /// \param[in] _node the name of the animation node
      /// \param[in] _time the time
      /// \param[in] _loop when true, the time is divided by the duration
      /// (see GetLength)
      /// \return the transformation
      public: ignition::math::Matrix4d NodePoseAt(const std::string &_node,
                      const double _time, const bool _loop = true);

      /// \brief Returns a dictionary of transformations indexed by name at
      /// a specific time
      /// if a node does not exist at that specific time
      /// (with tolerance of 1e-6 sec), the transformation is interpolated.
      /// \param[in] _time the time
      /// \param[in] _loop when true, the time is divided by the duration
      /// (see GetLength)
      /// \return the transformation for every node
      public: std::map<std::string, ignition::math::Matrix4d> PoseAt(
                  const double _time, const bool _loop = true) const;

      /// \brief Returns a dictionary of transformations indexed by name where
      /// a named node transformation's translational value along the X axis is
      /// equal to _x.
      /// \param[in] _x the value along x. You must ensure that _x is within a
      /// valid range.
      /// \param[in] _node the name of the animation node
      /// \param[in] _loop when true, the time is divided by the duration
      /// (see GetLength)
      public: std::map<std::string, ignition::math::Matrix4d> PoseAtX(
                  const double _x, const std::string &_node,
                  const bool _loop = true) const;


      /// \brief Scales every animation in the animations list
      /// \param[in] _scale the scaling factor
      public: void Scale(const double _scale);

      /// \brief Returns the duration of the animations
      /// \return the duration in seconds
      public: double GetLength() const;

      /// \brief the node name
      protected: std::string name;

      /// \brief the duration of the longest animation
      protected: double length;

      /// \brief a dictionary of node animations
      protected: std::map<std::string, NodeAnimation*> animations;
    };
    /// \}
  }
}

#endif
