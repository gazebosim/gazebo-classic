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
/////////////////////////////////////////////////////////////////////////////
// Name:        xh_propgrid.h
// Purpose:     XML resource handler for wxPropertyGrid
// Author:      Jaakko Salli
// Modified by:
// Created:     May-16-2007
// RCS-ID:      $Id:
// Copyright:   (c) Jaakko Salli
// Licence:     wxWindows license
/////////////////////////////////////////////////////////////////////////////

#ifndef _WX_XH_PROPGRID_H_
#define _WX_XH_PROPGRID_H_

/*

  NOTE: relevant source file, xh_propgrid.cpp is *not* included in the
    wxPropertyGrid library (to prevent xrc-lib dependency). To use this
    code, you will need to separately add src/xh_propgrid.cpp to your
    application.

*/

#include "wx/xrc/xmlres.h"

#include "gui/propgrid/propgrid.h"
#include "gui/propgrid/manager.h"

#if wxUSE_XRC && wxCHECK_VERSION(2,8,0)

class wxPropertyGridXmlHandler : public wxXmlResourceHandler
{
    friend class wxPropertyGridXrcPopulator;
    DECLARE_DYNAMIC_CLASS(wxPropertyGridXmlHandler)

public:
    wxPropertyGridXmlHandler();
    virtual wxObject *DoCreateResource();
    virtual bool CanHandle(wxXmlNode *node);

    void InitPopulator();
    void PopulatePage( wxPropertyGridState* state );
    void DonePopulator();

    void HandlePropertyGridParams();

private:
    wxPropertyGridManager*      m_manager;
    wxPropertyGrid*             m_pg;
    wxPropertyGridPopulator*    m_populator;
};

#endif // wxUSE_XRC && wxCHECK_VERSION(2,8,0)

#endif // _WX_XH_PROPGRID_H_
