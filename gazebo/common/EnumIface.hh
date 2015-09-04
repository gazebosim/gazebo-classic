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
#ifndef _GAZEBO_ENUMITERATOR_HH_
#define _GAZEBO_ENUMITERATOR_HH_

#include <string>
#include <vector>
#include <algorithm>
#include "gazebo/common/Assert.hh"

namespace gazebo
{
  namespace common
  {
    /// \brief A macro that allows an enum to have an iterator and string
    /// conversion.
    /// \param[in] enumType Enum type
    /// \param[in] begin Enum value that marks the beginning of the enum
    /// values.
    /// \param[in] end Enum value that marks the end of the enum values.
    /// \param[in] names A vector of strings, one for each enum value.
    /// \sa EnumIface
    /// \sa EnumIterator
    #define GZ_ENUM(enumType, begin, end, ...) \
    template<> enumType common::EnumIface<enumType>::range[] = {begin, end}; \
    template<> std::vector<std::string> common::EnumIface<enumType>::names = \
    {__VA_ARGS__};

    /// \brief Enum interface. Use this interface to convert an enum to
    /// a string, and set an enum from a string.
    template<typename T>
    class EnumIface
    {
      /// \brief Get the beginning enum value.
      /// \return Enum value that marks the beginning of the enum list.
      public: static T Begin()
      {
        return range[0];
      }

      /// \brief Get the end enum value.
      /// \return Enum value that marks the end of the enum list.
      public: static T End()
      {
        return range[1];
      }

      /// \brief Convert enum value to string.
      /// \param[in] _e Enum value to convert.
      /// \return String representation of the enum. An empty string is
      /// returned if _e is invalid, or the names for the enum have not been
      /// set.
      static std::string Str(T const &_e)
      {
        if (_e < names.size())
          return names[_e];
        else
          return "";
      }

      /// \brief Set an enum from a string. This function requires a valid
      /// string, and an array of names for the enum must exist.
      /// \param[in] _str String value to convert to enum value.
      /// \param[in] _e Enum variable to set.
      /// \sa EnumIterator
      static void Set(T &_e, const std::string &_str)
      {
        static auto begin = std::begin(names);
        static auto end = std::end(names);

        auto find = std::find(begin, end, _str);
        if (find != end)
        {
          _e = static_cast<T>(std::distance(begin, find));
        }
      }

      /// \internal
      /// \brief The beginning and end values. Do not use this directly.
      public: static T range[2];

      /// \internal
      /// \brief Array of string names for each element in the enum. Do not
      /// use this directly.
      public: static std::vector<std::string> names;
    };

    /// \brief An iterator over enum types.
    ///
    ///  Example:
    ///
    /// \verbatim
    /// enum MyType
    /// {
    ///   MY_TYPE_BEGIN = 0,
    ///   TYPE1 = MY_TYPE_BEGIN,
    ///   TYPE2 = 1,
    ///   MY_TYPE_END
    /// };
    ///
    /// GZ_ENUM(MyType, MY_TYPE_BEGIN, MY_TYPE_END,
    ///  "TYPE1",
    ///  "TYPE2",
    ///  "MY_TYPE_END")
    ///
    /// int main()
    /// {
    ///   EnumIface<MyType> i = MY_TYPE_BEGIN;
    ///   std::cout << "Type Number[" << *i << "]\n";
    ///   std::cout << "Type Name[" << EnumIface::Str(*i) << "]\n";
    ///   i++;
    ///   std::cout << "Type++ Number[" << *i << "]\n";
    ///   std::cout << "Type++ Name[" << EnumIface::Str(*i) << "]\n";
    /// }
    /// \verbatim
    template<typename Enum>
    class EnumIterator
    : std::iterator<std::bidirectional_iterator_tag, Enum>
    {
      /// \brief Constructor
      public: EnumIterator() : c(this->End())
      {
      }

      /// \brief Constructor
      /// \param[in] _c Enum value
      public: EnumIterator(const Enum _c) : c(_c)
      {
        GZ_ASSERT(this->c >= this->Begin() && this->c <= this->End(),
            "Invalid enum value in EnumIterator constructor");
      }

      /// \brief Equal operator
      /// \param[in] _c Enum value to copy
      public: EnumIterator &operator=(const Enum _c)
      {
        GZ_ASSERT(_c >= this->Begin() && _c <= this->End(),
            "Invalid operator= value in EnumIterator");
        this->c = _c;
        return *this;
      }

      /// \brief Get the beginning of the enum
      /// \return Value at the beginning of the enum list
      public: static Enum Begin()
      {
        return EnumIface<Enum>::Begin();
      }

      /// \brief Get the end of the enum
      /// \return Value at the end of the enum list
      public: static Enum End()
      {
        return EnumIface<Enum>::End();
      }

      /// \brief Prefix increment operator.
      /// \return Iterator pointing to the next value in the enum.
      public: EnumIterator &operator++()
      {
        GZ_ASSERT(this->c != this->End(), "Incrementing past end of enum");
        this->c = static_cast<Enum>(this->c + 1);
        return *this;
      }

      /// \brief Postfix increment operator.
      /// \return Iterator pointing to the next value in the enum.
      public: EnumIterator operator++(const int)
      {
        GZ_ASSERT(this->c != this->End(), "Incrementing past end of enum");
        EnumIterator cpy(*this);
        ++*this;
        return cpy;
      }

      /// \brief Prefix decrement operator
      /// \return Iterator pointing to the previous value in the enum
      public: EnumIterator &operator--()
      {
        GZ_ASSERT(this->c != this->Begin(), "decrementing beyond begin?");
        this->c = static_cast<Enum>(this->c - 1);
        return *this;
      }

      /// \brief Postfix decrement operator
      /// \return Iterator pointing to the previous value in the enum
      public: EnumIterator operator--(const int)
      {
        GZ_ASSERT(this->c != this->Begin(), "Decrementing beyond beginning.");
        EnumIterator cpy(*this);
        --*this;
        return cpy;
      }

      /// \brief Dereference operator
      /// \return Value of the iterator
      public: Enum operator*() const
      {
        GZ_ASSERT(this->c != this->End(), "Cannot dereference end iterator");
        return c;
      }

      /// \brief Get the enum value.
      /// \return Value of the iterator
      public: Enum Value() const
      {
        return this->c;
      }

      /// \brief Enum value
      /// Did not use a private data class since this should be the only
      /// member value every used.
      private: Enum c;
    };

    /// \brief Equality operator
    /// \param[in] _e1 First iterator
    /// \param[in] _e1 Second iterator
    /// \return True if the two iterators contain equal enum values.
    template<typename Enum>
    bool operator==(EnumIterator<Enum> _e1, EnumIterator<Enum> _e2)
    {
      return _e1.Value() == _e2.Value();
    }

    /// \brief Inequality operator
    /// \param[in] _e1 First iterator
    /// \param[in] _e1 Second iterator
    /// \return True if the two iterators do not contain equal enum values.
    template<typename Enum>
    bool operator!=(EnumIterator<Enum> _e1, EnumIterator<Enum> _e2)
    {
      return !(_e1 == _e2);
    }
  }
}
#endif
