/* Copyright (C) 2012
 *     Jonas Mellin & Zakiruz Zaman
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
 */
/* Desc: RFID Tag Manager
 * Author: Jonas Mellin & Zakiruz Zaman 
 * Date: 6th December 2011
 */

#ifndef _RFID_TAG_MANAGER__
#define _RFID_TAG_MANAGER__

#include <vector>
#include "common/SingletonT.hh"
#include "RFIDTag.hh"

namespace gazebo
{
  /// \ingroup gazebo_sensors
  /// \brief Sensors namespace
  namespace sensors
  {
    /// \addtogroup gazebo_sensors
    /// \{
    /// \class RFIDTagManager RFIDTagManager.hh sensors/sensors.hh
    /// \brief 
    /// \TODO Nate fill in
    class RFIDTagManager : public SingletonT<RFIDTagManager>
    {
      /// \brief Adds tag model
      /// \param _model Tagged model to add
      /// \TODO Nate check
      public: void AddTaggedModel(RFIDTag *_model);

      /// \brief Gets vector of tagged models
      /// \return vector of RFIDTag, the tagged models
      /// \TODO Nate check
      public: std::vector<RFIDTag*> GetTags()
              {return this->taggedModels;}

      private: std::vector<RFIDTag*> taggedModels;

      private: friend class SingletonT<RFIDTagManager>;
    };
    /// \}
  }
}
#endif
