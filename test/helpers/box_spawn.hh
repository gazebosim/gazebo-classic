/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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

#ifndef _HELPER_BOX_SPAWN_HH_
#define _HELPER_BOX_SPAWN_HH_

#include <string>
#include <sstream>

#include "test/ServerFixture.hh"

#include "gazebo/common/Time.hh"
#include "gazebo/physics/physics.hh"

namespace gazebo
{
class BoxSpawn
{
  public: BoxSpawn() : spawnCount(0)
          {
          }

  /// \brief Class to hold parameters for spawning boxes.
  public: class BoxSpawnOptions
  {
    /// \brief Constructor.
    public: BoxSpawnOptions() : wait(common::Time(99, 0)),
              size(math::Vector3(0.1, 0.1, 0.1)), mass(1.0),
              createCollision(true), createVisual(true)
            {
            }

    /// \brief Destructor.
    public: ~BoxSpawnOptions()
            {
            }

    /// \brief Length of time to wait for model to spawn in order to return
    ///        Joint pointer.
    public: common::Time wait;

    /// \brief Initial pose for spawned model.
    public: math::Pose pose;

    /// \brief Size of box shape.
    public: math::Vector3 size;

    /// \brief Mass of box (inertia calculated based on mass and size).
    public: double mass;

    /// \brief Flag to create a collision object.
    public: bool createCollision;

    /// \brief Flag to create a visual object.
    public: bool createVisual;

    /// \brief Name to use for model if non-empty.
    public: std::string name;
  };

  /// \brief Spawn a model with a joint connecting to the world. The function
  ///        will wait for duration _wait for the model to spawn and attempt
  ///        to return a pointer to the spawned joint. This function is not
  ///        guaranteed to return a valid JointPtr, so the output should be
  ///        checked.
  /// \param[in] _opt Options for spawned model and joint.
  public: std::string BoxSpawnString(BoxSpawnOptions &_opt)
  {
    std::ostringstream modelStr;
    std::ostringstream modelName;
    if (_opt.name.empty())
    {
      modelName << "box_model" << BoxSpawn::spawnCount++;
      _opt.name = modelName.str();
    }
    else
    {
      modelName << _opt.name;
    }

    double x2 = _opt.size.x * _opt.size.x;
    double y2 = _opt.size.y * _opt.size.y;
    double z2 = _opt.size.z * _opt.size.z;

    modelStr
      << "<sdf version='" << SDF_VERSION << "'>"
      << "<model name ='" << modelName.str() << "'>"
      << "  <pose>" << _opt.pose << "</pose>"
      << "  <link name='link'>"
      << "    <inertial>"
      << "      <mass>" << _opt.mass << "</mass>"
      << "      <inertia>"
      << "        <ixx>" << _opt.mass * (y2+z2) / 12.0 << "</ixx>"
      << "        <iyy>" << _opt.mass * (x2+z2) / 12.0 << "</iyy>"
      << "        <izz>" << _opt.mass * (x2+y2) / 12.0 << "</izz>"
      << "        <ixy>0</ixy>"
      << "        <ixz>0</ixz>"
      << "        <iyz>0</iyz>"
      << "      </inertia>"
      << "    </inertial>";
    if (_opt.createCollision)
    {
      modelStr
        << "<collision name='collision'>"
        << "  <geometry>"
        << "    <box>"
        << "      <size>" << _opt.size << "</size>"
        << "    </box>"
        << "  </geometry>"
        << "</collision>";
    }
    if (_opt.createVisual)
    {
      modelStr
        << "<visual name='visual'>"
        << "  <geometry>"
        << "    <box>"
        << "      <size>" << _opt.size << "</size>"
        << "    </box>"
        << "  </geometry>"
        << "</visual>";
    }
    modelStr
      << "  </link>"
      << "</model>";

    return modelStr.str();
  }

  /// \brief Count of spawned models, used to ensure unique model names.
  private: unsigned int spawnCount;
};  // class BoxSpawn
}   // namespace gazebo
#endif
