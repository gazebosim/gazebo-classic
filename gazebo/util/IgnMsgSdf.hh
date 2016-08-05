/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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
#ifndef GAZEBO_UTIL_IGNMSGSDF_HH_
#define GAZEBO_UTIL_IGNMSGSDF_HH_

#include <string>

#include <ignition/msgs/MessageTypes.hh>
#include <sdf/sdf.hh>

#include "gazebo/common/Console.hh"
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace util
  {
    /// \brief Create an ignition::msgs::Plugin from a plugin SDF element.
    /// \param[in] _sdf The sdf element.
    /// \return The new ignition::msgs::Plugin object.
    template<typename T>
    GAZEBO_VISIBLE
    T Convert(const sdf::ElementPtr /*_sdf*/)
    {
      gzerr << "Invalid convertion of SDF to type["
            << typeid(T).name() << "]\n";
      return T();
    }

    /// \brief Create or update an SDF element from ignition::msgs::Plugin.
    /// \param[in] _msg Plugin messsage.
    /// \param[in] _sdf if supplied, performs an update from _msg instead of
    /// creating a new sdf element.
    /// \return The new SDF element.
    GAZEBO_VISIBLE
    sdf::ElementPtr Convert(const ignition::msgs::Plugin &_msg,
                            sdf::ElementPtr _sdf = sdf::ElementPtr());

    // Specialization of conversion from SDF to ignition message for plugins.
    template<>
    ignition::msgs::Plugin Convert(const sdf::ElementPtr _sdf)
    {
      ignition::msgs::Plugin result;

      if (_sdf->GetName() != "plugin")
      {
        gzerr << "Tried to convert SDF [" << _sdf->GetName() <<
            "] into [plugin]" << std::endl;
        return result;
      }

      result.set_name(_sdf->Get<std::string>("name"));
      result.set_filename(_sdf->Get<std::string>("filename"));

      std::stringstream ss;
      for (sdf::ElementPtr innerElem = _sdf->GetFirstElement();
          innerElem; innerElem = innerElem->GetNextElement(""))
      {
        ss << innerElem->ToString("");
      }
      result.set_innerxml(ss.str());

      return result;
    }
  }
}
#endif
