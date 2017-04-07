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


#ifndef GAZEBO_RENDERING_GPULASERDATAITERATORIMPL_HH_
#define GAZEBO_RENDERING_GPULASERDATAITERATORIMPL_HH_

namespace gazebo
{
  namespace rendering
  {
    template <typename F>
    GpuLaserDataIterator<F>::~GpuLaserDataIterator()
    {
      // Do nothing
    }

    template <typename F>
    bool GpuLaserDataIterator<F>::operator==(
        const GpuLaserDataIterator<F> &_rvalue) const
    {
      return this->index == _rvalue.index;
    }

    template <typename F>
    bool GpuLaserDataIterator<F>::operator!=(
        const GpuLaserDataIterator<F> &_rvalue) const
    {
      return this->index != _rvalue.index;
    }

    template <typename F>
    const GpuLaserData GpuLaserDataIterator<F>::operator*() const
    {
      return {
        // range
        this->data[this->index * this->skip + this->rangeOffset],
        // intensity
        this->data[this->index * this->skip + this->intensityOffset],
        // beam
        index / this->horizontalResolution,
        // reading
        index % this->horizontalResolution,
      };
    }

    template <typename F>
    const std::unique_ptr<const GpuLaserData>
      GpuLaserDataIterator<F>::operator->() const
    {
      return std::unique_ptr<const GpuLaserData>(
          new GpuLaserData(this->operator*()));
    }

    template <typename F>
    GpuLaserDataIterator<F>& GpuLaserDataIterator<F>::operator++()
    {
      ++(this->index);
      return *this;
    }

    template <typename F>
    GpuLaserDataIterator<F> GpuLaserDataIterator<F>::operator++(int /*_dummy*/)
    {
      GpuLaserDataIterator<F> copy = *this;
      this->index++;
      return copy;
    }

    template <typename F>
    GpuLaserDataIterator<F>& GpuLaserDataIterator<F>::operator--()
    {
      --(this->index);
      return *this;
    }

    template <typename F>
    GpuLaserDataIterator<F> GpuLaserDataIterator<F>::operator--(int /*_dummy*/)
    {
      GpuLaserDataIterator<F> copy = *this;
      this->index--;
      return copy;
    }

    template <typename F>
    GpuLaserDataIterator<F>::GpuLaserDataIterator(const unsigned int _index,
        const float *_data, const unsigned int _skip, unsigned int _rangeOffset,
        const unsigned int _intensityOffset,
        const unsigned int _horizontalResolution) : index(_index), data(_data),
        skip(_skip), rangeOffset(_rangeOffset),
        intensityOffset(_intensityOffset),
        horizontalResolution(_horizontalResolution)
    {
      // Do nothing
    }
  }
}

// GAZEBO_RENDERING_GPULASERDATAITERATORIMPL_HH_
#endif
