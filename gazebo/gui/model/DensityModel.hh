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
#ifndef _GAZEBO_DENSITY_MODEL_HH_
#define _GAZEBO_DENSITY_MODEL_HH_

#include <string>
#include <vector>

#include "gazebo/common/SingletonT.hh"

namespace gazebo
{
  namespace gui
  {
    /// \class DensityEntry DensityModel.hh
    /// \brief A model class for a single density value
    class GZ_GUI_VISIBLE DensityEntry
    {
      /// \brief Constructor
      /// \param[in] _desc Description for the entry.
      /// \param[in] _value Value for the entry.
      public: DensityEntry(const std::string &_desc, double _value);

      /// \brief Destructor
      public: virtual ~DensityEntry();

      /// \brief Returns string represention of entry
      /// \return String representation of entry.
      public: std::string ToString() const;

      /// \brief Description
      public: std::string desc;

      /// \brief Density value in kg/m^3
      public: double value;
    };

    /// \class DensityModel DensityModel.hh
    /// \brief A model class for link densities
    class GZ_GUI_VISIBLE DensityModel : public SingletonT<DensityModel>
    {
      /// \brief Constructor
      private: DensityModel();

      /// \brief Destructor
      private: virtual ~DensityModel();

      /// \brief Accessor for retrieving density entries
      /// \return List of entries.
      public: std::vector<const DensityEntry *> Entries() const;

      /// \brief Return first entry with matching desc or null if not found.
      /// \param[in] _desc Description of entry to match.
      /// \return Matching DensityEntry if found else null.
      public: const DensityEntry *EntryByDesc(const std::string &_desc) const;

      /// \brief Return first entry with matching value or null if not found.
      /// \param[in] Density value of entry to match.
      /// \return Matching DensityEntry if found else null.
      public: const DensityEntry *EntryByValue(double _value) const;

      /// \brief This is a singleton class.
      private: friend class SingletonT<DensityModel>;

      /// \brief List of density entries
      private: std::vector<const DensityEntry *> entries;
    };
  }
}
#endif
