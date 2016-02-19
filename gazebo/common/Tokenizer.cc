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

#include <string>
#include <vector>

#include "gazebo/common/Tokenizer.hh"

using namespace gazebo;
using namespace common;

namespace gazebo
{
  namespace common
  {
    /// \internal
    /// \brief Tokenizer private data.
    class TokenizerPrivate
    {
      /// \brief Input string.
      public: std::string buffer;

      /// \brief Current position under check.
      public: size_t currPos;
    };
  }
}

/////////////////////////////////////////////////
std::vector<std::string> common::split(const std::string &_str,
                               const std::string &_delim)
{
  return Tokenizer(_str).Split(_delim);
}

/////////////////////////////////////////////////
Tokenizer::Tokenizer(const std::string &_str)
  : dataPtr(new TokenizerPrivate())
{
  this->dataPtr->buffer = _str;
  this->dataPtr->currPos = 0;
}

/////////////////////////////////////////////////
Tokenizer::~Tokenizer()
{
}

/////////////////////////////////////////////////
std::vector<std::string> Tokenizer::Split(const std::string &_delim)
{
  std::vector<std::string> tokens;
  std::string token;

  this->dataPtr->currPos = 0;
  while ((token = this->NextToken(_delim)) != "")
    tokens.push_back(token);

  return tokens;
}

/////////////////////////////////////////////////
std::string Tokenizer::NextToken(const std::string &_delim)
{
  std::string token;
  this->SkipDelimiter(_delim);

  // Append each char to token string until it meets delimiter.
  while (this->dataPtr->currPos < this->dataPtr->buffer.size() &&
         !this->IsDelimiter(_delim, this->dataPtr->currPos))
  {
    token += this->dataPtr->buffer.at(this->dataPtr->currPos);
    ++this->dataPtr->currPos;
  }

  return token;
}

/////////////////////////////////////////////////
void Tokenizer::SkipDelimiter(const std::string &_delim)
{
  while (this->dataPtr->currPos < this->dataPtr->buffer.size() &&
         this->IsDelimiter(_delim, this->dataPtr->currPos))
  {
    this->dataPtr->currPos += _delim.size();
  }
}

/////////////////////////////////////////////////
bool Tokenizer::IsDelimiter(const std::string &_delim,
    const size_t _pos) const
{
  return this->dataPtr->buffer.find(_delim, _pos) == _pos;
}
