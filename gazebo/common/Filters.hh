/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#ifndef _GAZEBO_FILTERS_HH_
#define _GAZEBO_FILTERS_HH_

#include <iostream>
#include <vector>
#include <map>

#include <boost/function.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/thread/mutex.hpp>

#include <gazebo/gazebo_config.h>
#include <gazebo/common/Time.hh>
#include <gazebo/common/CommonTypes.hh>
#include <gazebo/math/Helpers.hh>

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{

    /// \internal
    /// \brief Private data members for MovingWindowFilter class.
    /// This must be in the header due to templatization.
    template< typename T>
    class MovingWindowFilterPrivate
    {
      // \brief Constructor
      public: MovingWindowFilterPrivate();

      /// \brief For moving window smoothed velocity
      public: int velWindowSize;
      public: T velFiltered;
      public: std::vector<T> velHistory;
      public: typename std::vector<T>::iterator velIter;
    };

    //////////////////////////////////////////////////
    template<typename T>
    MovingWindowFilterPrivate<T>::MovingWindowFilterPrivate()
    {
      /// \TODO FIXME hardcoded for now
      this->velWindowSize = 4;
      this->velHistory.resize(this->velWindowSize);
      this->velIter = this->velHistory.begin();
    }

    /// \class MovingWindowFilter filters.hh common/common.hh
    /// \brief Base class for MovingWindowFilter
    template< typename T>
    class MovingWindowFilter
    {
      /// \brief Constructor
      public: MovingWindowFilter();

      /// \brief Destructor
      public: virtual ~MovingWindowFilter();

      /// \brief Update value of filter
      public: void Update(T _vel);

      /// \brief Get filtered result
      public: T Get();

      /// \brief Allow subclasses to initialize their own data pointer.
      /// \param[in] _d Reference to data pointer.
      protected: MovingWindowFilter<T>(MovingWindowFilterPrivate<T> &_d);

      /// \brief Data pointer.
      protected: MovingWindowFilterPrivate<T> *dataPtr;
    };

    //////////////////////////////////////////////////
    template<typename T>
    MovingWindowFilter<T>::MovingWindowFilter()
      : dataPtr(new MovingWindowFilterPrivate<T>())
    {
    }

    //////////////////////////////////////////////////
    template<typename T>
    MovingWindowFilter<T>::~MovingWindowFilter()
    {
      this->dataPtr->velHistory.clear();
      delete this->dataPtr;
      this->dataPtr = NULL;
    }

    //////////////////////////////////////////////////
    template<typename T>
    void MovingWindowFilter<T>::Update(T _vel)
    {
      // update avg
      double n = 1.0/static_cast<double>(this->dataPtr->velWindowSize);

      // add new data
      if (this->dataPtr->velIter == this->dataPtr->velHistory.end())
        this->dataPtr->velIter = this->dataPtr->velHistory.begin();
      (*this->dataPtr->velIter) = _vel;

      // progressive add and subtrace oldest
      // this->dataPtr->velFiltered += n*(*this->dataPtr->velIter - *old);

      // sum and avg
      this->dataPtr->velFiltered = T();
      for (typename std::vector<T>::iterator it =
        this->dataPtr->velHistory.begin();
        it != this->dataPtr->velHistory.end(); ++it)
        this->dataPtr->velFiltered += *it;
      this->dataPtr->velFiltered *= n;
    }

    //////////////////////////////////////////////////
    template<typename T>
    T MovingWindowFilter<T>::Get()
    {
      return this->dataPtr->velFiltered;
    }
    /// \}
  }
}
#endif
