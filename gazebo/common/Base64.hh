/*
 * Copyright (C) 2012-2014 Open Source Robotics Foundation
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
#ifndef _BASE_64_HH_
#define _BASE_64_HH_

#include <string>

/// \brief Encode a binary string into base 64.
/// \param[in] _bytesToEncode String of bytes to encode.
/// \param[in] _len Length of _bytesToEncode.
/// \param[out] _result Based64 string is appended to this string.
void Base64Encode(const char *_bytesToEncode, unsigned int _len,
    std::string &_result);


/// \brief Decode a base64 string.
/// \param[in] _encodedString A base 64 encoded string.
/// \return The decoded string.
std::string Base64Decode(const std::string &_encodedString);
#endif
