/*
 * Copyright (C) 2017 Open Source Robotics Foundation
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
#ifndef GAZEBO_RENDERING_GPULASERDATAITERATOR_HH_
#define GAZEBO_RENDERING_GPULASERDATAITERATOR_HH_

#include <memory>

namespace gazebo
{
  namespace rendering
  {
    /// \brief struct containing info about a single ray measurement
    struct GpuLaserData
    {
      /// \brief The distance of the reading in meters
      // cppcheck-suppress unusedStructMember
      double range;

      /// \brief The intensity reading
      // cppcheck-suppress unusedStructMember
      double intensity;

      /// \brief Which plane or cone this reading belongs to [0, vRes)
      // cppcheck-suppress unusedStructMember
      unsigned int beam;

      /// \brief the index of areading in a plane or cone[0, hRes)
      // cppcheck-suppress unusedStructMember
      unsigned int reading;
    };

    /// \brief const Bidirectional iterator for laser data
    ///
    /// This class contains the information needed to access Laser Data
    /// It implements a Bidirectional Input iterator
    /// http://www.cplusplus.com/reference/iterator/BidirectionalIterator/
    /// The compiler should optimize out calls to this class
    template <typename F>
    class GpuLaserDataIterator
    {
      public: friend F;

      /// \brief Destructor
      public: ~GpuLaserDataIterator();

      /// \brief Operator ==
      /// \param[in] _rvalue The iterator on the right of the ==
      /// \return true iff the iterators point to the same reading
      public: bool operator==(const GpuLaserDataIterator &_rvalue) const;

      /// \brief Operator !=
      /// \param[in] _rvalue The iterator on the right of the !=
      /// \return true iff the iterators point to different readings
      public: bool operator!=(const GpuLaserDataIterator &_rvalue) const;

      /// \brief Dereference operator *iter
      /// \return A struct of laser data
      public: const GpuLaserData operator*() const;

      /// \brief Dereference operator iter->
      /// \return a shared pointer object at the iterator's index
      public: const std::unique_ptr<const GpuLaserData> operator->() const;

      /// \brief Advance iterator to next reading (prefix: ++it)
      /// \return reference to this pointer after advancing
      public: GpuLaserDataIterator<F> &operator++();

      /// \brief Advance this iterator (postfix: it++)
      /// \param[in] _dummy does nothing, required for postfix overload
      /// \return a copy of this iterator prior to advancing
      public: GpuLaserDataIterator<F> operator++(int _dummy);

      /// \brief Move itereator to previous (prefix: --it)
      /// \return reference to this pointer after moving
      public: GpuLaserDataIterator<F>& operator--();

      /// \brief Move itereator to previous (postfix: it--)
      /// \param[in] _dummy does nothing, required for postfix overload
      /// \return a copy of this iterator prior to moving
      public: GpuLaserDataIterator<F> operator--(int _dummy);

      /// \brief contstruct an iterator to a specified index
      protected: GpuLaserDataIterator(const unsigned int _index,
                         const float *_data, const unsigned int _skip,
                         const unsigned int _rangeOffset,
                         const unsigned int _intensityOffset,
                         const unsigned int _horizontalResolution);


      // Not using PIMPL because it has no benefit on templated classes

      /// \brief which reading is this [0, vRes * hRes)
      private: unsigned int index = 0;

      /// \brief the data being decoded
      private: const float *data = nullptr;

      /// \brief offset between consecutive readings
      private: const unsigned int skip = 0;

      /// \brief offset within a reading to range data
      private: const unsigned int rangeOffset = 0;

      /// \brief offset within a reading to intensity data
      private: const unsigned int intensityOffset = 0;

      /// \brief Number of readings in each plane or cone
      private: const unsigned int horizontalResolution = 0;
    };
  }
}

#include "GpuLaserDataIteratorImpl.hh"

#endif

