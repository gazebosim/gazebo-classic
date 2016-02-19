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

#ifndef _GAZEBO_COMMON_TOKENIZER_HH_
#define _GAZEBO_COMMON_TOKENIZER_HH_

#include <memory>
#include <string>
#include <vector>
#include "gazebo/util/system.hh"

namespace gazebo
{
  namespace common
  {
    /// \brief Convenience function that splits a string based on
    /// a delimeter.
    /// \param[in] _str The string to split.
    /// \param[in] _delim Token delimiter.
    /// \return Vector of tokens.
    GZ_COMMON_VISIBLE
    std::vector<std::string> split(const std::string &_str,
                                   const std::string &_delim);

    // Forward declare private data classes.
    class TokenizerPrivate;

    /// \brief A string tokenizer class.
    /// Heavily inspired by http://www.songho.ca/misc/tokenizer/tokenizer.html
    class GZ_COMMON_VISIBLE Tokenizer
    {
      /// \brief Class constructor.
      /// \param[in] _str A string argument.
      public: Tokenizer(const std::string &_str);

      /// \brief Class destructor.
      public: ~Tokenizer();

      /// \brief Splits the internal string into tokens.
      /// \param[in] _delim Token delimiter.
      /// \return Vector of tokens.
      public: std::vector<std::string> Split(const std::string &_delim);

      /// \brief Get the next token.
      /// \param[in] _delim Token delimiter.
      /// \return The token or empty string if there are no more tokens.
      private: std::string NextToken(const std::string &_delim);

      /// \brief Consume a delimiter if we are currently pointing to it.
      /// \param[in] _delim Token delimiter.
      private: void SkipDelimiter(const std::string &_delim);

      /// \brief Return if a delimiter starts at a given position
      /// \param[in] _delim Token delimiter.
      /// \param[in] _currPos Position to check.
      /// \return True when a delimiter starts at _pos or false otherwise.
      private: bool IsDelimiter(const std::string &_delim,
                                const size_t _pos) const;

      /// \internal
      /// \brief Pointer to private data.
      private: std::unique_ptr<TokenizerPrivate> dataPtr;
    };
  }
}
#endif
