/*
 * Copyright (C) 2014-2016 Open Source Robotics Foundation
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

#ifndef _GAZEBO_MOVING_WINDOW_FILTER_HH_
#define _GAZEBO_MOVING_WINDOW_FILTER_HH_

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

namespace gazebo
{
  namespace common
  {
    /// \addtogroup gazebo_common Common
    /// \{

    /// \cond
    /// \brief Private data members for MovingWindowFilter class.
    /// This must be in the header due to templatization.
    template< typename T>
    class MovingWindowFilterPrivate
    {
      // \brief Constructor
      public: MovingWindowFilterPrivate();

      /// \brief For moving window smoothed value
      public: unsigned int valWindowSize;

      /// \brief buffer history of raw values
      public: std::vector<T> valHistory;

      /// \brief iterator pointing to current value in buffer
      public: typename std::vector<T>::iterator valIter;

      /// \brief keep track of running sum
      public: T sum;

      /// \brief keep track of number of elements
      public: unsigned int samples;
    };
    /// \endcond

    //////////////////////////////////////////////////
    template<typename T>
    MovingWindowFilterPrivate<T>::MovingWindowFilterPrivate()
    {
      /// \TODO FIXME hardcoded initial value for now
      this->valWindowSize = 4;
      this->valHistory.resize(this->valWindowSize);
      this->valIter = this->valHistory.begin();
      this->sum = T();
      this->samples = 0;
    }

    /// \class MovingWindowFilter MovingWindowFilter.hh common/common.hh
    /// \brief Base class for MovingWindowFilter
    template< typename T>
    class MovingWindowFilter
    {
      /// \brief Constructor
      public: MovingWindowFilter();

      /// \brief Destructor
      public: virtual ~MovingWindowFilter();

      /// \brief Update value of filter
      /// \param[in] _val new raw value
      public: void Update(T _val);

      /// \brief Set window size
      /// \param[in] _n new desired window size
      public: void SetWindowSize(unsigned int _n);

      /// \brief Get the window size.
      /// \return The size of the moving window.
      public: unsigned int GetWindowSize() const;

      /// \brief Get whether the window has been filled.
      /// \return True if the window has been filled.
      public: bool GetWindowFilled() const;

      /// \brief Get filtered result
      /// \return latest filtered value
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
      this->dataPtr->valHistory.clear();
      delete this->dataPtr;
      this->dataPtr = nullptr;
    }

    //////////////////////////////////////////////////
    template<typename T>
    void MovingWindowFilter<T>::Update(T _val)
    {
      // update sum and sample size with incoming _val

      // keep running sum
      this->dataPtr->sum += _val;

      // shift pointer, wrap around if end has been reached.
      ++this->dataPtr->valIter;
      if (this->dataPtr->valIter == this->dataPtr->valHistory.end())
      {
        // reset iterator to beginning of queue
        this->dataPtr->valIter = this->dataPtr->valHistory.begin();
      }

      // increment sample size
      ++this->dataPtr->samples;

      if (this->dataPtr->samples > this->dataPtr->valWindowSize)
      {
        // subtract old value if buffer already filled
        this->dataPtr->sum -= (*this->dataPtr->valIter);
        // put new value into queue
        (*this->dataPtr->valIter) = _val;
        // reduce sample size
        --this->dataPtr->samples;
      }
      else
      {
        // put new value into queue
        (*this->dataPtr->valIter) = _val;
      }
    }

    //////////////////////////////////////////////////
    template<typename T>
    void MovingWindowFilter<T>::SetWindowSize(unsigned int _n)
    {
      this->dataPtr->valWindowSize = _n;
      this->dataPtr->valHistory.clear();
      this->dataPtr->valHistory.resize(this->dataPtr->valWindowSize);
      this->dataPtr->valIter = this->dataPtr->valHistory.begin();
      this->dataPtr->sum = T();
      this->dataPtr->samples = 0;
    }

    //////////////////////////////////////////////////
    template<typename T>
    unsigned int MovingWindowFilter<T>::GetWindowSize() const
    {
      return this->dataPtr->valWindowSize;
    }

    //////////////////////////////////////////////////
    template<typename T>
    bool MovingWindowFilter<T>::GetWindowFilled() const
    {
      return this->dataPtr->samples == this->dataPtr->valWindowSize;
    }

    //////////////////////////////////////////////////
    template<typename T>
    T MovingWindowFilter<T>::Get()
    {
      return this->dataPtr->sum / static_cast<double>(this->dataPtr->samples);
    }
    /// \}
  }
}
#endif
