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
#ifndef _SDF_CONVERTER_HH_
#define _SDF_CONVERTER_HH_

#include <tinyxml.h>
#include <string>

namespace sdf
{
  /// \brief Convert from one version of SDF to another
  class Converter
  {
    /// \brief Convert SDF to the specified version.
    /// \param[in] _doc SDF xml doc
    /// \param[in] _toVersion Version number in string format
    /// \param[in] _quiet False to be more verbose
    public: static bool Convert(TiXmlDocument *_doc,
                                const std::string &_toVersion,
                                bool _quiet = false);

    /// \cond
    /// This is an internal function.
    /// \brief Generic convert function that converts the SDF based on the
    /// given Convert file.
    /// \param[in] _doc SDF xml doc
    /// \param[in] _convertDoc Convert xml doc
    public: static void Convert(TiXmlDocument *_doc,
        TiXmlDocument *_convertDoc);
    /// \endcond

    private: static void ConvertImpl(TiXmlElement *_elem,
                                     TiXmlElement *_convert);

    /// \brief Rename an element or attribute.
    /// \param[in] _elem The element to be renamed, or the element which
    /// has the attribute to be renamed.
    /// \param[in] _renameElem A 'convert' element that describes the rename
    /// operation.
    private: static void Rename(TiXmlElement *_elem,
                                     TiXmlElement *_renameElem);

    /// \brief Move an element or attribute within a common ancestor element.
    /// \param[in] _elem Ancestor element of the element or attribute to
    /// be moved.
    /// \param[in] _moveElem A 'convert' element that describes the move
    /// operation.
    private: static void Move(TiXmlElement *_elem,
                                     TiXmlElement *_moveElem);

    private: static const char *GetValue(const char *_valueElem,
                                         const char *_valueAttr,
                                         TiXmlElement *_elem);

    private: static void CheckDeprecation(TiXmlElement *_elem,
                                          TiXmlElement *_convert);
  };
}
#endif
