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

#ifndef GAZEBO_COMMON_WEAKBIND_HH_
#define GAZEBO_COMMON_WEAKBIND_HH_

#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>

namespace gazebo
{
  namespace common
  {
    namespace details
    {

      /// \class WeakBinder WeakBind.hh
      /// \brief Function object wrapper used by common::weakBind
      template <typename Func, typename T>
      class WeakBinder
      {

        public: using WeakPtr = boost::weak_ptr<T>;

        private: Func func;
        private: WeakPtr ptr;

        public: WeakBinder(Func _func, WeakPtr _ptr) :
            func(_func),
            ptr(_ptr)
        {}

        // Return non-void version
        public: template <typename... Args> auto operator()(Args&&... _args)
            -> typename std::enable_if<
                !std::is_void<
                  decltype(this->func(std::forward<Args>(_args)...))
                >::value,
                decltype(this->func(std::forward<Args>(_args)...))
              >::type
        {
          auto ptrLock = this->ptr.lock();
          if (ptrLock)
          {
            return this->func(std::forward<Args>(_args)...);
          }
          else
          {
            return {};
          }
        }

        // Return void version
        public: template <typename... Args> auto operator()(Args&&... _args)
          -> typename std::enable_if<
                std::is_void<
                  decltype(this->func(std::forward<Args>(_args)...))
                >::value,
                void
             >::type
        {
          auto ptrLock = this->ptr.lock();
          if (ptrLock)
          {
            this->func(std::forward<Args>(_args)...);
          }
        }

      };

      template <typename Func, typename T>
      WeakBinder<Func, T> makeWeakBinder(Func func, boost::weak_ptr<T> ptr)
      {
        return WeakBinder<Func, T>(func, ptr);
      }

    }

    /// \addtogroup gazebo_common Common
    /// \{
    /// \brief Bind parameters to a function object and return a call wrapper.
    /// This function does not keep an owning pointer to the first shared_ptr
    /// given in parameter. When the wrapper is called and if the pointer has
    /// expired, the function object is not called and no error is reported.
    /// If the pointer is valid, the wrapper takes ownership of the pointer and
    /// calls the function object.
    /// \param[in] _func Wrapped function object
    /// \param[in] _ptr Owning pointer to bind as the first argument of the
    /// function object.
    /// \param[in] _args Arguments to bind to the function object
    template <typename T, typename Func, typename... Args>
    auto weakBind(Func _func, boost::shared_ptr<T> _ptr, Args... _args)
    #if __cplusplus < 201402L
      -> decltype(details::makeWeakBinder(
            boost::bind(_func, _ptr.get(), _args...),
            boost::weak_ptr<T>(_ptr)
        ))
    #endif
    {
      return details::makeWeakBinder(
                boost::bind(_func, _ptr.get(), _args...),
                boost::weak_ptr<T>(_ptr)
      );
    }
    /// \}
  }

}

#endif
