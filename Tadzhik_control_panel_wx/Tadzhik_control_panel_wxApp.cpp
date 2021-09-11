/***************************************************************
 * Name:      Tadzhik_control_panel_wxApp.cpp
 * Purpose:   Code for Application Class
 * Author:    Me ()
 * Created:   2018-05-13
 * Copyright: Me ()
 * License:
 **************************************************************/

#include "wx_pch.h"
#include "Tadzhik_control_panel_wxApp.h"

//(*AppHeaders
#include "Tadzhik_control_panel_wxMain.h"
#include <wx/image.h>
//*)

IMPLEMENT_APP(Tadzhik_control_panel_wxApp);

bool Tadzhik_control_panel_wxApp::OnInit()
{
    //(*AppInitialize
    bool wxsOK = true;
    wxInitAllImageHandlers();
    if ( wxsOK )
    {
    	Tadzhik_control_panel_wxFrame* Frame = new Tadzhik_control_panel_wxFrame(0);
    	Frame->Show();
    	SetTopWindow(Frame);
    }
    //*)
    return wxsOK;

}
