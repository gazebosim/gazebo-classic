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

#ifndef _GAZEBO_COLLADALOADERFWD_HH_
#define _GAZEBO_COLLADALOADERFWD_HH_

#include <string>
#include <vector>
#include <map>

class TiXmlElement;

namespace gazebo
{
  namespace common
  {
    class ColladaLoader;

    /// \{

    /// \class ColladaLoader ColladaLoader.hh common/common.hh
    {



          TiXmlElement *_skelXml,
          const ignition::math::Matrix4d &_transform, Mesh *_mesh);



                                               SkeletonNode *_parent);

                                             SkeletonNode *_node);

                   const ignition::math::Matrix4d &_transform, Mesh *_mesh);

                                          const std::string &_name,
                                          const std::string &_id);

                                           const std::string &_id);

                   const ignition::math::Matrix4d &_transform);


         const ignition::math::Matrix4d &_transform,
         std::vector<ignition::math::Vector3d> &_verts,
         std::vector<ignition::math::Vector3d> &_norms);

         const ignition::math::Matrix4d &_transform,
         std::vector<ignition::math::Vector3d> &_verts,
         std::vector<ignition::math::Vector3d> &_norms,
         std::map<unsigned int, unsigned int> &_vertDup,
         std::map<unsigned int, unsigned int> &_normDup);

          const ignition::math::Matrix4d &_transform,
          std::vector<ignition::math::Vector3d> &_values,
          std::map<unsigned int, unsigned int> &_duplicates);

          const ignition::math::Matrix4d &_transform,
          std::vector<ignition::math::Vector3d> &_values,
          std::map<unsigned int, unsigned int> &_duplicates);

          std::vector<ignition::math::Vector2d> &_values,
          std::map<unsigned int, unsigned int> &_duplicates);


                                       const std::string &_type,
                                       Material *_mat);

                                   const ignition::math::Matrix4d &_transform,
                                   Mesh *_mesh);

                                   const ignition::math::Matrix4d &_transform,
                                   Mesh *_mesh);

                               const ignition::math::Matrix4d &_transform,
                               Mesh *_mesh);




      /// \internal
    };
    /// \}
  }
}
#endif
