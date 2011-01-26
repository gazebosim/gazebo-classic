/*
 * Copyright 2011 Nate Koenig & Andrew Howard
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
#ifndef PARAMSNOTEBOOK_HH
#define PARAMSNOTEBOOK_HH

#include <wx/dialog.h>


namespace gazebo
{
  class ParamsNotebook : public wxDialog
  {
    /// \brief Constructor
    public: ParamsNotebook( wxWindow *parent );

    /// \brief Destructor
    public: virtual ~ParamsNotebook();

    /// \brief On done event
    public: void OnDone(wxCommandEvent &event);

    /// \brief On apply event
    public: void OnApply(wxCommandEvent &event);

    private: wxNotebook *notebook;
  };
}
#endif
